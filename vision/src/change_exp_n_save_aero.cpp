#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <boost/filesystem.hpp>
#include <time.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
//using boost::filesystem;
#include <ctime>

#include <boost/random.hpp>

static const std::string OPENCV_WINDOW_LEFT = "Top_Left_RandomExposure";
//static const std::string OPENCV_WINDOW_RIGHT = "Top_Right_Rectified";


class change_exp_n_save_aero
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_left;
    image_transport::Publisher image_pub_left;

    image_transport::Subscriber image_sub_right;
    image_transport::Publisher image_pub_right;



int i;
public:
    change_exp_n_save_aero()
        : it_(nh_),i(0)
    {


 // Subscribe to input video feed and publish output video feed

        image_sub_left = it_.subscribe("aero/lower_stereo/left/image_raw", 1, &change_exp_n_save_aero::imageCb, this);
        image_pub_left = it_.advertise("/stereo/left/image_rect_small", 1);
        cv::namedWindow(OPENCV_WINDOW_LEFT);



    }

    ~change_exp_n_save_aero()
    {
        cv::destroyWindow(OPENCV_WINDOW_LEFT);

    }

 double Random_do()
    {
        printf("here\n");
      const double rangeMin = 0.01;  // 0.0008;
      const double rangeMax = 0.04; // 0.004;
      typedef boost::uniform_real<> NumberDistribution;
      typedef boost::mt19937 RandomNumberGenerator;
      typedef boost::variate_generator<RandomNumberGenerator&, NumberDistribution> Generator;

      NumberDistribution distribution(rangeMin, rangeMax);
      RandomNumberGenerator generator;
      Generator numberGenerator(generator, distribution);
      generator.seed(std::time(0)); // seed with the current time

      printf("%f\n",numberGenerator());

      return numberGenerator();

    }


 void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        i++;
        if (i>10)
        {

            dynamic_reconfigure::ReconfigureRequest srv_req;
            dynamic_reconfigure::ReconfigureResponse srv_resp;
            dynamic_reconfigure::DoubleParameter double_param;
            dynamic_reconfigure::BoolParameter bool_param;
            dynamic_reconfigure::Config conf;

            bool_param.name= "auto_exposure";
            bool_param.value = false;
            conf.bools.push_back(bool_param);

            double_param.name = "exposure";
            double_param.value = Random_do();
            conf.doubles.push_back(double_param);

            srv_req.config= conf;
            ros::service::call("/aero/upper_stereo/left_camera", srv_req, srv_resp);

            cv_bridge::CvImagePtr cv_ptr;

            try
            {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }


            time_t now;
            time(&now);
            cv::imwrite(std::string("/home/aero/Pictures/output_of_change_exp_n_save_aero/")+ctime(&now)+".jpg",cv_ptr->image);

            cv::imshow(OPENCV_WINDOW_LEFT, cv_ptr->image);
            cv::waitKey(3);
            i=0;


            // Output modified video stream
            //image_pub_left.publish(cv_ptr_clone->toImageMsg());
        }
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "change_exp_n_save_aero");
    change_exp_n_save_aero ic;
    ros::spin();
    return 0;
}
