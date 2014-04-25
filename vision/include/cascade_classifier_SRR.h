
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <time.h>
#include <image_geometry/stereo_camera_model.h>
#include <stereo_msgs/DisparityImage.h>


static const std::string OPENCV_WINDOW_RIGHT = "Top_Right_Rectified";
static const std::string cascade_path_WHA = "/home/aero/SRR_Training/HOOK/cascadeTraining/cascade.xml";


class cascade_classifier_node
{

public:
	 cascade_classifier_node();
     ~cascade_classifier_node();
private:
     //ROS related vars.
     ros::NodeHandle nh_;
     image_transport::ImageTransport it_;
     image_transport::CameraSubscriber m_image_sub_left;
     image_transport::CameraSubscriber m_image_sub_right;
     //image_transport::Subscriber m_disp_sub;
     image_transport::Publisher  m_image_pub_left;
	sensor_msgs::CameraInfo left_info;
	sensor_msgs::CameraInfo right_info;
     	cv_bridge::CvImagePtr m_disp_ptr;
message_filters::Subscriber<stereo_msgs::DisparityImage> m_disp_sub; 

	
    // CV related vars
	cv::CascadeClassifier cascade_WHA;
    char* m_LeftCameraView;
    	image_geometry::StereoCameraModel stereo_model;

	// other vars
	bool m_leftCbTrig;
	bool m_rightCbTrig;
	bool m_stereoModelNotConfigured;
	
	// Private methods
	void m_detectAndDisplay(const cv_bridge::CvImagePtr& cv_ptr, cv::Mat& frame, const char* WINDOW);
	void m_dispCb(const stereo_msgs::DisparityImage::ConstPtr& msg);
	void m_imageCb(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info);
	void m_imageCbRight(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info);
	void m_computeDisparityPoint(double x, double y);
	void m_configStereoModel();


	

};
