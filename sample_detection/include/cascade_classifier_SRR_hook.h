
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <time.h>


static const std::string OPENCV_WINDOW_RIGHT = "Top_Right_Rectified";
//static const std::string cascade_path_WHA = "/home/aero/ObjectDetection/PuckCropped/cascadeTraining/cascade.xml";
//static const std::string cascade_path_WHA = "/home/aero/Pictures/SRR_Training_Dataset/hook_wha/hook_data/cascade.xml";
//static const std::string cascade_path_WHA = "/home/aero/Pictures/SRR_Training_Dataset/hook_wha/hook_new_data/cascade.xml"; // Good
//static const std::string cascade_path_WHA = "/home/aero/Pictures/SRR_Training_Dataset/hook_wha/hook_lbp_data/cascade.xml";
//static const std::string cascade_path_WHA = "/home/aero/Pictures/SRR_Training_Dataset/hook_wha/training2/hook_data_10s_20_20/cascade.xml";

//static const std::string cascade_path_WHA = "/home/aero/Pictures/SRR_Training_Dataset/hook_wha/Training_3/training_3_hook_data/cascade.xml";

//static const std::string cascade_path_WHA = "/home/aero/training4_data/cascade.xml";
//static const std::string cascade_path_WHA = "/home/aero/training4_hog/cascade.xml";

//static const std::string cascade_path_WHA = "/home/aero/training4_hog_995/cascade.xml";
//static const std::string cascade_path_WHA = "/home/aero/training4_hog_999/cascade.xml";


static const std::string cascade_path_WHA = "/home/aero/Pictures/SRR_Training_Dataset/hook_wha/training_4/training4_hook_data/cascade.xml";



class cascade_classifier_node
{

public:
	 cascade_classifier_node();
     ~cascade_classifier_node();
private:
     //ROS related vars.
     ros::NodeHandle nh_;
     ros::NodeHandle pnh_;
     image_transport::ImageTransport it_;
     image_transport::Subscriber m_m_image_sub_left;
     image_transport::Publisher  m_m_image_pub_left;

	
    // CV related vars
	cv::CascadeClassifier cascade_WHA;
    char* m_LeftCameraView;
	
	// Private methods
	void m_detectAndDisplay(const cv_bridge::CvImagePtr& cv_ptr, cv::Mat& frame, const char* WINDOW);
	void m_imageCb(const sensor_msgs::ImageConstPtr& msg);
	

};
