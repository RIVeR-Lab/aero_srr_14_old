
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <time.h>


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
     image_transport::Subscriber m_image_sub_left;
     image_transport::Publisher  m_image_pub_left;

	
    // CV related vars
	cv::CascadeClassifier cascade_WHA;
    char* m_LeftCameraView;
	
	// Private methods
	void m_detectAndDisplay(const cv_bridge::CvImagePtr& cv_ptr, cv::Mat& frame, const char* WINDOW);
	void m_imageCb(const sensor_msgs::ImageConstPtr& msg);
	

};
