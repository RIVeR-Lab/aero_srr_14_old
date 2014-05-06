
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
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <detection_manager.h>
#include <classifierTypes.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <vision/ObjectLocationMsg.h>


static const std::string OPENCV_WINDOW_RIGHT = "Top_Right_Rectified";
//static const std::string cascade_path_WHA = "/home/aero/SRR_Training/HOOK/cascadeTraining/cascade.xml";
//static const std::string cascade_path_WHA = "/home/aero/SRR_Training/HOOK/cascadeTraining20Hog/cascade.xml";
static const std::string cascade_path_WHA = "/home/aero/SRR_Training/HOOK/cascadeTraining4bHookdata/cascade.xml";

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
	tf::TransformListener optimus_prime;
	sensor_msgs::Image left_image;
	ros::Publisher ObjLocationPub, ObjLocationPubWorld;
	ros::Timer disp_timer;

message_filters::Subscriber<stereo_msgs::DisparityImage> m_disp_sub; 

	
   	 // CV related vars
	cv::CascadeClassifier cascade_WHA;
    	char* m_LeftCameraView;
    	image_geometry::StereoCameraModel stereo_model;
	cv::Mat m_disp_frame;
	
	// other vars
	bool m_leftCbTrig;
	bool m_rightCbTrig;
	bool m_stereoModelNotConfigured;


	// Manager
												 vision::DetectionManager CascadeManager, AeroManager;
	typedef std::pair<int, int> PixPoint_t;
	typedef std::pair<PixPoint_t, vision::object_type> Detection_t;
	typedef boost::shared_ptr<Detection_t> DetectionPtr_t;
	std::vector<DetectionPtr_t> detection_list_;

	
	// Private methods
	
	/**
	* @author Samir Zutshi
	* @brief Uses the trained cascade xml file to scan the image and track the detections
	*/
	void m_detectAndDisplay(const cv_bridge::CvImagePtr& cv_ptr, cv::Mat& frame, const char* WINDOW);
	
	/**
	* @author Samir Zutshi
	* @brief Callback for the disparity image
	*/
	void m_dispCb(const stereo_msgs::DisparityImage::ConstPtr& msg);
	
	/**
	* @author Samir Zutshi
	* @brief Callback for the left camera
	*/
	void m_imageCb(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info);
	
	/**
	* @author Samir Zutshi
	* @brief Callback for the right camera
	*/
	void m_imageCbRight(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info);
	
	/**
	* @author Samir Zutshi
	* @brief Computes the disparity at the detection point and adds it to manager.
	*/
	void m_computeDisparityPoint();
	
	/**
	* @author Samir Zutshi
	* @brief Configures the stereo camera model for 3d calculations.
	*/
	void m_configStereoModel();
	
	/**
	* @author Samir Zutshi
	* @brief builds a object location message for the publisher to send out. 
	*/
	void buildMsg(const tf::Point& point, geometry_msgs::PoseStamped& msg) const;

	/**
	* @author Samir Zutshi
	* @brief Averages the disparities in the area to get a proper value.
	*/
	float nNdisp(const cv::Point2d& pt, const cv::Mat& disp);

	/**
	* @author Samir Zutshi
	* @brief Callback for the disparity calculation
	*/
	void m_computeDisparityCb(const ros::TimerEvent& event);


	

};
