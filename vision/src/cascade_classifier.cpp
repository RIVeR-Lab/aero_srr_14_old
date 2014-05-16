#include <vision/cascade_classifier.h>
#include <boost/foreach.hpp>
#include <boost/assign/list_of.hpp>

namespace vision{
  using namespace message_filters::sync_policies;

  CascadeClassifier::CascadeClassifier(ros::NodeHandle nh, ros::NodeHandle pnh): it_(nh), stereo_model_init_(false), show_windows_(false){
    scale_factor_ = 1.05;
    min_neighbors_ = 5;
    min_size_ = cv::Size(50, 50);
    max_size_ = cv::Size(150, 150);
    robot_frame_ = "aero/base_footprint";
    object_radius_ = 0.03;

    std::string cascade_path = "/home/aero/srr/src/aero_srr_14/vision/cascadeTraining4bHookdata/cascade.xml";

    pnh.getParam("show_windows", show_windows_);
    pnh.getParam("cascade_path", cascade_path);



#ifdef USE_GPU
    ROS_INFO("Using GPU Detector");
#else
    ROS_INFO("Using CPU Detector");
#endif



    image_synchronizer_.reset(new ImageSynchronizer(ImageSyncPolicy(10), sub_l_image_, sub_d_image_));
    image_synchronizer_->registerCallback(boost::bind(&CascadeClassifier::imageCb,
					    this, _1, _2));

    info_synchronizer_.reset(new InfoSynchronizer(InfoSyncPolicy(6), sub_l_info_, sub_r_info_));
    info_synchronizer_->registerCallback(boost::bind(&CascadeClassifier::infoCb,
					    this, _1, _2));

    object_location_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("object_detection", 5);
    disparity_pub_ = it_.advertise("object_detection/disparity", 1);
    image_pub_ = it_.advertise("object_detection/image", 1);

    image_transport::TransportHints hints("raw", ros::TransportHints(), pnh);
    sub_l_image_.subscribe(it_, "left/image_rect", 1, hints);
    sub_d_image_.subscribe(nh, "disparity", 1);

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
				  const DisparityImageConstPtr& d_image_msg){
    ROS_INFO("Got Matching Image and Disparity delay=%fs", (ros::Time::now()-l_image_msg->header.stamp).toSec());
    cv_bridge::CvImagePtr l_image_ptr;
    cv_bridge::CvImagePtr d_image_ptr;
    try{
      l_image_ptr = cv_bridge::toCvCopy(l_image_msg, sensor_msgs::image_encodings::BGR8);
      d_image_ptr = cv_bridge::toCvCopy(d_image_msg->image, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    detect_and_publish(l_image_msg->header.frame_id, l_image_msg->header.stamp, l_image_ptr->image, d_image_ptr->image);
  }

  void CascadeClassifier::infoCb(const CameraInfoConstPtr& l_info_msg,
				 const CameraInfoConstPtr& r_info_msg){
    
    boost::lock_guard<boost::mutex> lock(stereo_model_mutex_);
    stereo_model_.fromCameraInfo(l_info_msg, r_info_msg);
    stereo_model_init_ = true;
  }

  void CascadeClassifier::detect_and_publish(const std::string& l_camera_frame, const ros::Time& time, cv::Mat& l_mat, cv::Mat& d_image){
    ros::Time pre_start = ros::Time::now();
    cv::Mat image_gray;
    cv::cvtColor(l_mat, image_gray, CV_RGB2GRAY);
    cv::equalizeHist(image_gray, image_gray);
#ifdef USE_GPU
    cv::gpu::GpuMat image_gray_gpu(image_gray);
#endif

    cv::Mat disparity_color;
    cv::convertScaleAbs(d_image, disparity_color, 100, 0.0);
    cv::cvtColor(disparity_color, disparity_color, CV_GRAY2RGB);

    std::vector<cv::Rect> detections;

    ros::Time classifier_start = ros::Time::now();
#ifdef USE_GPU
    cv::gpu::GpuMat gpu_detections_mat;
    int num_detections = cascade_classifier_.detectMultiScale(image_gray_gpu, gpu_detections_mat, scale_factor_, min_neighbors_, min_size_);
    cv::Mat detections_mat;
    gpu_detections_mat.colRange(0, num_detections).download(detections_mat);
    cv::Rect* detections_rects = detections_mat.ptr<cv::Rect>();
    for(int i = 0; i < num_detections; ++i)
      detections.push_back(detections_rects[i]);
#else
    cascade_classifier_.detectMultiScale(image_gray, detections, scale_factor_, min_neighbors_, 0, min_size_, max_size_);
#endif

    ros::Time post_start = ros::Time::now();
    {
      boost::lock_guard<boost::mutex> lock(stereo_model_mutex_);
      if(!stereo_model_init_)
	return;
      BOOST_FOREACH(cv::Rect detection, detections){
	cv::Point2d detection_center(detection.x + detection.width / 2,
			 detection.y + detection.height / 2);

	cv::Point2d disparity_center(detection_center.x,
			 detection_center.y+40);

	float disp_val = average_disparity(d_image, detection_center, 30, 50);
	if(disp_val<0){
          ROS_WARN("No disparity for detection: In left camera at (%d, %d)", (int)detection_center.x, (int)detection_center.y);
	  cv::ellipse(l_mat, detection_center,
		      cv::Size(detection.width / 2, detection.height / 2),
		      0, 0, 360, cv::Scalar(0, 0, 255), 2, 8, 0);
	  continue;
	}

	cv::ellipse(l_mat, detection_center,
		    cv::Size(detection.width / 2, detection.height / 2),
		    0, 0, 360, cv::Scalar(255, 0, 0), 2, 8, 0);

	cv::rectangle(disparity_color,
		      cv::Point(detection_center.x - 30 / 2,
                                detection_center.y - 50 / 2),
		      cv::Point(detection_center.x + 30 / 2,
                                detection_center.y + 50 / 2),
		      cv::Scalar(255, 0, 0), 2);


	cv::Point3d projected_position;
	stereo_model_.projectDisparityTo3d(detection_center, disp_val, projected_position);
	projected_position.z += object_radius_;

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
	object_location_pub_.publish(msg);

      }

    }
    ros::Time publish_start = ros::Time::now();

    cv_bridge::CvImage disparity_msg;
    disparity_msg.header.stamp   = time;
    disparity_msg.header.frame_id   = l_camera_frame;
    disparity_msg.encoding = sensor_msgs::image_encodings::BGR8;
    disparity_msg.image    = disparity_color;
    disparity_pub_.publish(disparity_msg.toImageMsg());
    cv_bridge::CvImage image_msg;
    image_msg.header.stamp   = time;
    image_msg.header.frame_id   = l_camera_frame;
    image_msg.encoding = sensor_msgs::image_encodings::BGR8;
    image_msg.image    = l_mat;
    image_pub_.publish(image_msg.toImageMsg());

    if(show_windows_){
      static boost::mutex display_mutex;
      boost::lock_guard<boost::mutex> lock(display_mutex);
      cv::imshow("image", l_mat);
      cv::imshow("disparity", disparity_color);
      cv::waitKey(3);
    }

    ROS_INFO("Classification done,  pre: %fs, class: %fs, post: %fs, publish: %fs", (classifier_start-pre_start).toSec(), (post_start-classifier_start).toSec(), (publish_start-post_start).toSec(), (ros::Time::now()-publish_start).toSec());
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
