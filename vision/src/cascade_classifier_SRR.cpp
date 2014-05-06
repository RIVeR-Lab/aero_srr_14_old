#include <cascade_classifier_SRR.h>


using namespace std;
using namespace cv;
using namespace sensor_msgs;
using namespace vision;

cascade_classifier_node::cascade_classifier_node(): it_(nh_), m_LeftCameraView("Top_Left_Rectified"), m_leftCbTrig(false), m_rightCbTrig(false), m_stereoModelNotConfigured(true), CascadeManager(.25,.15,.05,.5,2.0), AeroManager(1.5, .20, .10,.25,.25),  pnh_("~"), CV_Windows_enabled(true)
    {
        // Subscribe to input video feed and publish output video feed
     //   m_image_sub_left = it_.subscribe("camera/image", 1, &cascade_classifier_node::m_imageCb, this);
	m_image_sub_left = it_.subscribeCamera("left_image",20, &cascade_classifier_node::m_imageCb, this);
	m_image_sub_right = it_.subscribeCamera("right_image",20, &cascade_classifier_node::m_imageCbRight, this);
        m_disp_sub.subscribe(nh_,"/aero/lower_stereo/disparity", 3);
	m_disp_sub.registerCallback(&cascade_classifier_node::m_dispCb,this);
       // m_image_pub_left = it_.advertise("/stereo/left/image_rect_small", 1);
	ObjLocationPub = nh_.advertise<vision::ObjectLocationMsg>(
"ObjectPose", 2);
ObjLocationPubWorld = nh_.advertise<vision::ObjectLocationMsg>(
"ObjectPoseWorld", 2);
		disp_timer = nh_.createTimer(ros::Duration(1 / 20),
&cascade_classifier_node::m_computeDisparityCb, this);
        cv::namedWindow(m_LeftCameraView);
 std::string cascade_path_WHA = "/home/aero/SRR_Training/HOOK/cascadeTraining4bHookdata/cascade.xml";
pnh_.getParam("cascade_path_WHA", cascade_path_WHA);
pnh_.getParam("CV_Windows_enabled", CV_Windows_enabled);
        if (!cascade_WHA.load(cascade_path_WHA)) {
            printf("--(!)Error loading\n");
        }
    }

 cascade_classifier_node::~cascade_classifier_node()
{
     cv::destroyWindow(m_LeftCameraView); 
}

void cascade_classifier_node::m_computeDisparityCb(const ros::TimerEvent& event)
{
	if(m_leftCbTrig && m_rightCbTrig)
{
m_configStereoModel();
		m_computeDisparityPoint();	
}
}
void cascade_classifier_node::m_detectAndDisplay(const cv_bridge::CvImagePtr& cv_ptr, cv::Mat& frame, const char* WINDOW)
    {
        std::vector<cv::Rect> WHA_faces;
        std::vector<std::vector<cv::Rect> > Detections;
        static cv::Mat frame_gray;
        frame = cv_ptr->image;
        int HORIZON = 660;
	

        cv::cvtColor(frame, frame_gray, CV_RGB2GRAY);
        cv::equalizeHist(frame_gray, frame_gray);

        cascade_WHA.detectMultiScale(frame_gray, WHA_faces, 1.01, 5, 0,
                    cv::Size(50, 50), cv::Size(150, 150));

        for (int i = 0; i < WHA_faces.size(); i++)
        {
            cv::Point center(WHA_faces[i].x + WHA_faces[i].width / 2,
                             WHA_faces[i].y + WHA_faces[i].height / 2);


            cv::ellipse(frame, center,
                        cv::Size(WHA_faces[i].width / 2, WHA_faces[i].height / 2),
                        0, 0, 360, cv::Scalar(255, 0, 0), 2, 8, 0);
            cv::rectangle(frame,
                          cv::Point(center.x - WHA_faces[i].width / 2,
                                center.y - WHA_faces[i].height / 2),
                          cv::Point(center.x + WHA_faces[i].width / 2,
                                center.y + WHA_faces[i].height / 2),
                          cv::Scalar(255, 0, 0));

		DetectionPtr_t newDetection(new Detection_t());
		newDetection->first.first = center.x;
		newDetection->first.second = center.y;
		newDetection->second = WHA;
		detection_list_.push_back(newDetection);
        }
	
        cv::line(frame,cv::Point2d(0,HORIZON),cv::Point2d(frame_gray.cols,HORIZON),cv::Scalar(0,255,0));
       
if(CV_Windows_enabled){ 
cv::imshow(m_LeftCameraView, frame);
       	 cv::waitKey(3);
}
    }
void cascade_classifier_node::m_configStereoModel()
{
	this->stereo_model.fromCameraInfo(this->left_info, this->right_info);
	m_stereoModelNotConfigured = false;
}

float cascade_classifier_node::nNdisp(const cv::Point2d& pt, const cv::Mat& disp) {
int window = 10;
int startx = pt.x - window/2;
int starty = pt.y;
int ctr = 0;
float sum = 0.0;
for (int i = 0; i < window; i++) {
	for (int j = 0; j < window; j++) {
		float value = disp.at<float>(starty + i, startx + j);
		if (value > 0.0)
		{
			sum = sum + value;
			ctr ++;
		}
	}
}
// ROS_INFO_STREAM("CTR pts = " << ctr);
	if(ctr == 0)
		return 0.0;
	return sum / (float) ctr;
}

void cascade_classifier_node::m_computeDisparityPoint()
    {
	
   	if(!m_disp_ptr)
		return;
   	cv::Mat disp_frame;
	disp_frame = m_disp_ptr->image;
	geometry_msgs::PointStamped camera_point, world_point, robot_point;

	for (int i = 0; i < (int) detection_list_.size(); i++) {
		Point2d obj_centroid(detection_list_.at(i)->first.first,
			detection_list_.at(i)->first.second);
		Point3d obj_3d;
		float disp_val_pre_filter = disp_frame.at<float>(obj_centroid.y, obj_centroid.x);
	
		float disp_val = nNdisp(obj_centroid, disp_frame);
cout << "Disparity value at " << obj_centroid.x <<","<< obj_centroid.y<< " is " << disp_val <<endl;
	this->stereo_model.projectDisparityTo3d(obj_centroid, disp_val,
obj_3d);
	tf::Point detection(obj_3d.x, obj_3d.y, obj_3d.z);
cout << "3D val is " << obj_3d.x <<","<< obj_3d.y<< " is " << obj_3d.z <<endl;
	tf::pointTFToMsg(detection, camera_point.point);
	ros::Time tZero(0);
	camera_point.header.frame_id = "aero/lower_stereo/optical_frame";
	camera_point.header.stamp = left_image.header.stamp;

	robot_point.header.frame_id = "aero/base_footprint";
	robot_point.header.stamp = left_image.header.stamp;

	world_point.header.frame_id = "aero/odom";
	world_point.header.stamp = left_image.header.stamp;

	try
	{
		optimus_prime.waitForTransform("aero/odom",
		camera_point.header.frame_id, camera_point.header.stamp,
		ros::Duration(.50));
		optimus_prime.transformPoint("aero/odom", camera_point, world_point);
		optimus_prime.transformPoint("aero/base_footprint",camera_point,robot_point);
	}
	catch(std::exception& e)
	{
	ROS_ERROR_STREAM(e.what());
	}
	tf::Point robot_rel_detection;
	tf::pointMsgToTF(world_point.point, detection);
	tf::pointMsgToTF(robot_point.point, robot_rel_detection);

	CascadeManager.addDetection(detection, detection_list_.at(i)->second);
	AeroManager.addAndReplaceDetection(robot_rel_detection, WHA);
	}

	tf::Point detection;
	tf::Point robot_rel_det;
	detection_list_.clear();
	CascadeManager.shrink();
	AeroManager.shrink();
	double confidence, rr_conf;
	object_type type, rr_type;
	if(CascadeManager.getDetection(detection, type, confidence))
	{
			cout << "I Got A Detection: " << endl << "X:" << detection.getX()
<< ", Y: " << detection.getY() << ", Z: " << detection.getZ()
<< ", " << confidence
<< std::endl;
ObjectLocationMsg msg;
	msg.header.frame_id = "aero/odom";
msg.header.stamp = left_image.header.stamp;
msg.pose.header.frame_id = "aero/odom";
msg.pose.header.stamp = left_image.header.stamp;
buildMsg(detection, msg.pose);
ObjLocationPubWorld.publish(msg);
	}
	if(AeroManager.getDetection(robot_rel_det, rr_type, rr_conf)) {
ObjectLocationMsg msg;
msg.header.frame_id = "aero/base_footprint";
msg.header.stamp = ros::Time::now();
msg.pose.header.frame_id = "aero/base_footprint";
msg.pose.header.stamp = ros::Time::now();
buildMsg(robot_rel_det, msg.pose);
ObjLocationPub.publish(msg);
ROS_WARN_STREAM("Sent obj pose msg");
}
    }


void cascade_classifier_node::buildMsg(const tf::Point& point,
geometry_msgs::PoseStamped& msg) const {
tf::pointTFToMsg(point, msg.pose.position);
tf::Quaternion q;
q.setRPY(0, 0, 0);
q.normalize();
tf::quaternionTFToMsg(q, msg.pose.orientation);
}

void cascade_classifier_node::m_dispCb(const stereo_msgs::DisparityImage::ConstPtr& msg)
    {
        	
   
            static cv::Mat frame;
	    
            try
            {
                m_disp_ptr = cv_bridge::toCvCopy(msg->image, sensor_msgs::image_encodings::TYPE_32FC1);
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }

	if(detection_list_.size() >1)
	m_computeDisparityPoint();
    }

void cascade_classifier_node::m_imageCb(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info)
    {
	m_leftCbTrig = true;
        	left_info = *cam_info;
		 left_image = *msg;
            cv_bridge::CvImagePtr cv_ptr;
            static cv::Mat frame;

            try
            {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }
	   	
		
	if(!m_stereoModelNotConfigured)
               m_detectAndDisplay(cv_ptr,frame, m_LeftCameraView);

    }



void cascade_classifier_node::m_imageCbRight(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info)
    {
	m_rightCbTrig = true;
	    right_info = *cam_info;	
            cv_bridge::CvImagePtr cv_ptr;
            static cv::Mat frame;

            try
            {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }

	


    }

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cascade_classifier_SRR");
    cascade_classifier_node ic;
    ros::spin();
    return 0;
}
