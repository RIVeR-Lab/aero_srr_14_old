#include <cascade_classifier_SRR_hook.h>

using namespace std;
using namespace cv;
using namespace sensor_msgs;

cascade_classifier_node::cascade_classifier_node(): it_(nh_), m_LeftCameraView("Top_Left_Rectified")
    {
        // Subscribe to input video feed and publish output video feed
       // m_m_image_sub_left = it_.subscribe("/stereo/left/image_raw", 1, &cascade_classifier_node::m_imageCb, this);

        m_m_image_sub_left = it_.subscribe("/aero/lower_stereo/left/image_raw", 1, &cascade_classifier_node::m_imageCb, this);

        //m_m_image_pub_left = it_.advertise("/stereo/left/image_rect_small", 1);

        cv::namedWindow(m_LeftCameraView);
    }

 cascade_classifier_node::~cascade_classifier_node()
{
     cv::destroyWindow(m_LeftCameraView); printf("des");
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

        cascade_WHA.detectMultiScale(frame_gray, WHA_faces, 1.05, 10, 0,
                    cv::Size(10, 10), cv::Size(150, 150));                      // 1.05, 20, max size 150

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
        }
        cv::line(frame,cv::Point2d(0,HORIZON),cv::Point2d(frame_gray.cols,HORIZON),cv::Scalar(0,255,0));
        cv::imshow(m_LeftCameraView, frame);
        cv::waitKey(3);

    }

void cascade_classifier_node::m_imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
// i++;
// if (i>10)
//{
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



            // Update GUI Window
            m_detectAndDisplay(cv_ptr,frame, m_LeftCameraView);
//}

    }


//};



int main(int argc, char** argv)
{
    ros::init(argc, argv, "cascade_classifier_SRR");
    cascade_classifier_node ic;
    ros::spin();
    return 0;
}
