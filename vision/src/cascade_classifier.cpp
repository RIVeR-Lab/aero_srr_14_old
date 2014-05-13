#include <vision/cascade_classifier.h>
#include <boost/foreach.hpp>
#include <boost/assign/list_of.hpp>

namespace vision{
  using namespace message_filters::sync_policies;

  CascadeClassifier::CascadeClassifier(ros::NodeHandle nh, ros::NodeHandle pnh): it_(nh), show_windows_(false){
    scale_factor_ = 1.05;
    min_neighbors_ = 5;
    min_size_ = cv::Size(50, 50);
    max_size_ = cv::Size(150, 150);
    robot_frame_ = "aero/base_footprint";


#ifdef USE_GPU
    block_matcher_.ndisp = 128;
    block_matcher_.winSize = 15;
#else
    block_matcher_.state->preFilterSize       = 9;
    block_matcher_.state->preFilterCap        = 31;
    block_matcher_.state->SADWindowSize       = 15;
    block_matcher_.state->minDisparity        = 0;
    block_matcher_.state->numberOfDisparities = 128;
    block_matcher_.state->uniquenessRatio     = 15;
    block_matcher_.state->textureThreshold    = 10;
    block_matcher_.state->speckleWindowSize   = 100;
    block_matcher_.state->speckleRange        = 4;
#endif



    std::string cascade_path = "/home/aero/srr/src/aero_srr_14/vision/cascadeTraining4bHookdata/cascade.xml";

    pnh.getParam("show_windows", show_windows_);
    pnh.getParam("cascade_path", cascade_path);



#ifdef USE_GPU
    ROS_INFO("Using GPU Detector");
#else
    ROS_INFO("Using CPU Detector");
#endif



    image_synchronizer_.reset(new ImageSynchronizer(ImageSyncPolicy(6), sub_l_image_, sub_r_image_, sub_l_info_, sub_r_info_));
    image_synchronizer_->registerCallback(boost::bind(&CascadeClassifier::imageCb,
						      this, _1, _2, _3, _4));


    disparity_pub_ = nh.advertise<stereo_msgs::DisparityImage>("disparity", 5);

    object_detection_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("object_detection", 5);
    object_detection_disparity_pub_ = it_.advertise("object_detection/disparity", 1);
    object_detection_image_pub_ = it_.advertise("object_detection/image", 1);

    image_transport::TransportHints hints("raw", ros::TransportHints(), pnh);
    sub_l_image_.subscribe(it_, "left/image_rect", 1, hints);
    sub_r_image_.subscribe(it_, "right/image_rect", 1, hints);

    sub_l_info_.subscribe(nh, "left/camera_info", 1);
    sub_r_info_.subscribe(nh, "right/camera_info", 1);

    


    if (!cascade_classifier_.load(cascade_path)) {
      ROS_ERROR("Error loading cascade classifier: %s", cascade_path.c_str());
    }

    if(show_windows_){
      cv::namedWindow("image");
      cv::namedWindow("disparity");
    }
  }

  CascadeClassifier::~CascadeClassifier(){
    if(show_windows_){
      cv::destroyWindow("image");
      cv::destroyWindow("disparity");
    }
  }


  void CascadeClassifier::imageCb(const ImageConstPtr& l_image_msg,
				  const ImageConstPtr& r_image_msg,
				  const CameraInfoConstPtr& l_info_msg,
				  const CameraInfoConstPtr& r_info_msg){
    ROS_INFO("Got Matching Images delay=%fs", (ros::Time::now()-l_image_msg->header.stamp).toSec());

    stereo_model_.fromCameraInfo(l_info_msg, r_info_msg);
    cv_bridge::CvImageConstPtr l_image_ptr;
    cv_bridge::CvImageConstPtr r_image_ptr;
    try{
      l_image_ptr = cv_bridge::toCvShare(l_image_msg, sensor_msgs::image_encodings::MONO8);
      r_image_ptr = cv_bridge::toCvShare(r_image_msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    //create disparity message
    DisparityImagePtr disp_msg = boost::make_shared<DisparityImage>();
    disp_msg->header         = l_image_msg->header;
    disp_msg->image.header   = l_image_msg->header;
    disp_msg->image.height   = l_image_msg->height;
    disp_msg->image.width    = l_image_msg->width;
    disp_msg->image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    disp_msg->image.step     = disp_msg->image.width * sizeof(float);
    disp_msg->image.data.resize(disp_msg->image.height * disp_msg->image.step);
    disp_msg->f = stereo_model_.right().fx();
    disp_msg->T = stereo_model_.baseline();

    // Compute window of (potentially) valid disparities
    int border   = window_size() / 2;
    int left   = num_disparities() + min_disparity() + border - 1;
    int wtf = (min_disparity() >= 0) ? border + min_disparity() : std::max(border, -min_disparity());
    int right  = disp_msg->image.width - 1 - wtf;
    int top    = border;
    int bottom = disp_msg->image.height - 1 - border;
    disp_msg->valid_window.x_offset = left;
    disp_msg->valid_window.y_offset = top;
    disp_msg->valid_window.width    = right - left;
    disp_msg->valid_window.height   = bottom - top;

    // Disparity search range
    disp_msg->min_disparity = min_disparity();
    disp_msg->max_disparity = min_disparity() + num_disparities() - 1;
    disp_msg->delta_d = 1.0 / 16; // OpenCV uses 16 disparities per pixel

    cv::Mat_<float> d_image(disp_msg->image.height, disp_msg->image.width,
                             reinterpret_cast<float*>(&disp_msg->image.data[0]),
                             disp_msg->image.step);
    InternalMat l_image(l_image_ptr->image);
    InternalMat r_image(r_image_ptr->image);

    compute_disparity(l_image, r_image, d_image);
    disparity_pub_.publish(disp_msg);

    cv::Mat l_feedback_image;
    //cv::convertScaleAbs(l_image_ptr->image, l_feedback_image, 100, 0.0);
    cv::cvtColor(l_image_ptr->image, l_feedback_image, CV_GRAY2RGB);

    cv::Mat d_feedback_image;
    cv::convertScaleAbs(d_image, d_feedback_image, 100, 0.0);
    cv::cvtColor(d_feedback_image, d_feedback_image, CV_GRAY2RGB);

    detect(l_image_msg->header.frame_id, l_image_msg->header.stamp, l_image, d_image, l_feedback_image, d_feedback_image);


    cv_bridge::CvImage disparity_msg;
    disparity_msg.header  = l_image_msg->header;
    disparity_msg.encoding = sensor_msgs::image_encodings::BGR8;
    disparity_msg.image    = d_feedback_image;
    object_detection_disparity_pub_.publish(disparity_msg.toImageMsg());
    cv_bridge::CvImage image_msg;
    image_msg.header  = l_image_msg->header;
    image_msg.encoding = sensor_msgs::image_encodings::BGR8;
    image_msg.image    = l_feedback_image;
    object_detection_image_pub_.publish(image_msg.toImageMsg());

  }

  void CascadeClassifier::compute_disparity(const InternalMat& l_image, const InternalMat& r_image, cv::Mat& d_image){
    ros::Time disp_start = ros::Time::now();

#ifdef USE_GPU
    cv::gpu::GpuMat gpu_d_image;
    block_matcher_(l_image, r_image, gpu_d_image);
    gpu_d_image.download(d_image);
#else
    block_matcher_(l_image, r_image, d_image, CV_32F);
#endif

    double cx_l = stereo_model_.left().cx();
    double cx_r = stereo_model_.right().cx();
    if (cx_l != cx_r)
      cv::subtract(d_image, cv::Scalar(cx_l - cx_r), d_image);

    ROS_INFO("Disparity took %fs", (ros::Time::now()-disp_start).toSec());
  }


  void CascadeClassifier::detect(const std::string& l_camera_frame, const ros::Time& time, const InternalMat& l_image, const cv::Mat& d_image, cv::Mat& l_feedback_image, cv::Mat& d_feedback_image){

    std::vector<cv::Rect> detections;

    ros::Time classifier_start = ros::Time::now();
#ifdef USE_GPU
    cv::gpu::GpuMat gpu_detections_mat;
    int num_detections = cascade_classifier_.detectMultiScale(l_image, gpu_detections_mat, scale_factor_, min_neighbors_, min_size_);
    cv::Mat detections_mat;
    gpu_detections_mat.colRange(0, num_detections).download(detections_mat);
    cv::Rect* detections_rects = detections_mat.ptr<cv::Rect>();
    for(int i = 0; i < num_detections; ++i)
      detections.push_back(detections_rects[i]);
#else
    cascade_classifier_.detectMultiScale(l_image, detections, scale_factor_, min_neighbors_, 0, min_size_, max_size_);
#endif

    ros::Time post_start = ros::Time::now();
    {
      BOOST_FOREACH(cv::Rect detection, detections){
	cv::Point2d detection_center(detection.x + detection.width / 2,
			 detection.y + detection.height / 2);


	int disparity_y_offset = 15;
	int disparity_width = 30;
	int disparity_height = 40;

	cv::Point2d disparity_center(detection_center.x,
			 detection_center.y+disparity_y_offset);

	float disp_val = average_disparity(d_image, disparity_center, disparity_width, disparity_height);
	if(disp_val<0){
          ROS_WARN("No disparity for detection: In left camera at (%d, %d)", (int)detection_center.x, (int)detection_center.y);
	  cv::ellipse(l_feedback_image, detection_center,
		      cv::Size(detection.width / 2, detection.height / 2),
		      0, 0, 360, cv::Scalar(0, 0, 255), 2, 8, 0);
	  cv::rectangle(d_feedback_image,
			cv::Point(disparity_center.x - disparity_width / 2,
				  disparity_center.y - disparity_height / 2),
			cv::Point(disparity_center.x + disparity_width / 2,
				  disparity_center.y + disparity_height / 2),
			cv::Scalar(0, 0, 255), 2);
	  continue;
	}

	cv::ellipse(l_feedback_image, detection_center,
		    cv::Size(detection.width / 2, detection.height / 2),
		    0, 0, 360, cv::Scalar(255, 0, 0), 2, 8, 0);

	cv::rectangle(d_feedback_image,
		      cv::Point(disparity_center.x - disparity_width / 2,
                                disparity_center.y - disparity_height / 2),
		      cv::Point(disparity_center.x + disparity_width / 2,
                                disparity_center.y + disparity_height / 2),
				cv::Scalar(255, 0, 0), 2);
	cv::rectangle(l_feedback_image,
		      cv::Point(disparity_center.x - disparity_width / 2,
                                disparity_center.y - disparity_height / 2),
		      cv::Point(disparity_center.x + disparity_width / 2,
                                disparity_center.y + disparity_height / 2),
				cv::Scalar(255, 0, 0), 1);


	cv::Point3d projected_position;
	stereo_model_.projectDisparityTo3d(detection_center, disp_val, projected_position);


	tf::Point position_tf(projected_position.x, projected_position.y, projected_position.z);

	geometry_msgs::PointStamped camera_point, robot_point;
	camera_point.header.frame_id = l_camera_frame;
	camera_point.header.stamp = time;
	tf::pointTFToMsg(position_tf, camera_point.point);

        ROS_INFO("Got detection: In left camera at (%d, %d), disp = %f (%f m)", (int)detection_center.x, (int)detection_center.y, disp_val, projected_position.z);

	geometry_msgs::PoseWithCovarianceStamped msg;
	msg.header = camera_point.header;
	msg.pose.pose.position = camera_point.point;
	msg.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
	msg.pose.covariance = boost::assign::list_of(0.2) (0)   (0)  (0)  (0)  (0)
						      (0)  (0.2)  (0)  (0)  (0)  (0)
						      (0)   (0)  (0.3) (0)  (0)  (0)
						      (0)   (0)   (0)  (0)  (0)  (0)
						      (0)   (0)   (0)  (0)  (0)  (0)
						      (0)   (0)   (0)  (0)  (0)  (0) ;
	object_detection_pub_.publish(msg);

      }

    }
    ros::Time publish_start = ros::Time::now();

    if(show_windows_){
      static boost::mutex display_mutex;
      boost::lock_guard<boost::mutex> lock(display_mutex);
      cv::imshow("image", l_image);
      cv::imshow("disparity", d_image);
      cv::waitKey(3);
    }

    ROS_INFO("Classification done,  class: %fs, post: %fs, publish: %fs", (post_start-classifier_start).toSec(), (publish_start-post_start).toSec(), (ros::Time::now()-publish_start).toSec());
  }

  float CascadeClassifier::average_disparity(const cv::Mat& disp, const cv::Point2d& pt, int width, int height) {
    int startx = pt.x - width/2;
    int starty = pt.y - height/2;
    int ctr = 0;
    float sum = 0.0;
    for (int i = 0; i < width; i++) {
      for (int j = 0; j < height; j++) {
	float value = disp.at<float>(starty + i, startx + j);
	if (value > 0.0){
	  sum = sum + value;
	  ctr++;
	}
      }
    }
    if(ctr == 0)
      return -1;
    return sum / (float) ctr;
  }




}
