#include <cascade_classifier_SRR.h>

using namespace std;
using namespace cv;
using namespace sensor_msgs;

cascade_classifier_node::cascade_classifier_node(): it_(nh_), m_LeftCameraView("Top_Left_Rectified"), m_leftCbTrig(false), m_rightCbTrig(false), m_stereoModelNotConfigured(true)
    {
        // Subscribe to input video feed and publish output video feed
     //   m_image_sub_left = it_.subscribe("camera/image", 1, &cascade_classifier_node::m_imageCb, this);
	m_image_sub_left = it_.subscribeCamera("/stereo/left/image_raw",1, &cascade_classifier_node::m_imageCb, this);
	m_image_sub_right = it_.subscribeCamera("/stereo/right/image_raw",1, &cascade_classifier_node::m_imageCbRight, this);
        m_disp_sub = it_.subscribe("/stereo/disparity", 1, &cascade_classifier_node::m_dispCb, this);
       // m_image_pub_left = it_.advertise("/stereo/left/image_rect_small", 1);
        cv::namedWindow(m_LeftCameraView);
    }

 cascade_classifier_node::~cascade_classifier_node()
{
     cv::destroyWindow(m_LeftCameraView); 
}

void cascade_classifier_node::m_detectAndDisplay(const cv_bridge::CvImagePtr& cv_ptr, cv::Mat& frame, const char* WINDOW)
    {
        std::vector<cv::Rect> WHA_faces;
        std::vector<std::vector<cv::Rect> > Detections;
        static cv::Mat frame_gray;
        frame = cv_ptr->image;
        int HORIZON = 660;

        if (!cascade_WHA.load(cascade_path_WHA)) {
            printf("--(!)Error loading\n");
        }

        cv::cvtColor(frame, frame_gray, CV_RGB2GRAY);
        cv::equalizeHist(frame_gray, frame_gray);

        cascade_WHA.detectMultiScale(frame_gray, WHA_faces, 1.1, 155, 0,
                    cv::Size(40, 40), cv::Size(100, 100));

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

		m_computeDisparityPoint(center.x, center.y);
        }
        cv::line(frame,cv::Point2d(0,HORIZON),cv::Point2d(frame_gray.cols,HORIZON),cv::Scalar(0,255,0));
        cv::imshow(m_LeftCameraView, frame);
        cv::waitKey(3);

    }
void cascade_classifier_node::m_configStereoModel()
{
	this->stereo_model.fromCameraInfo(this->left_info, this->right_info);
}
void cascade_classifier_node::m_computeDisparityPoint(double x, double y)
    {
   	
   	cv::Mat disp_frame;
	disp_frame = m_disp_ptr->image;
	Point2d obj_centroid(x,y);
	Point3d obj_3d;
	float disp_val = disp_frame.at<float>(obj_centroid.y, obj_centroid.x);
	this->stereo_model.projectDisparityTo3d(obj_centroid, disp_val,
obj_3d);

	cout << "Detection at "<< obj_3d.x << "," << obj_3d.y << "," << obj_3d.z <<endl;

	

    }

void cascade_classifier_node::m_dispCb(const stereo_msgs::DisparityImage::ConstPtr& msg)
    {
        	
   
            static cv::Mat frame;
	    
            try
            {
                m_disp_ptr = cv_bridge::toCvCopy(msg->image, sensor_msgs::image_encodings::BGR8);
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }


    }

void cascade_classifier_node::m_imageCb(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info)
    {
	m_leftCbTrig = true;
        	left_info = *cam_info;
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
		if(m_leftCbTrig && m_rightCbTrig && m_stereoModelNotConfigured)
		{
			m_stereoModelNotConfigured = false;
			m_configStereoModel();
		}	


            // Update GUI Window
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
