#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image", 1);

  cv::Mat image;
    image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR); 
  cv_bridge::CvImage image2pub(std_msgs::Header(), "bgr8", image);	

  sensor_msgs::ImagePtr msg = image2pub.toImageMsg();
  
  ros::Rate loop_rate(5);
printf("before while");
  while (nh.ok()) {
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}

