#include <ros/ros.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

int main(int argc, char **argv){
  ros::init(argc, argv, "zero_twist_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ros::Publisher twist_pub = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("zero_twist", 1);

  geometry_msgs::TwistWithCovarianceStamped twist;
  twist.header.frame_id = "aero/sample_detection";
  ros::Rate rate(10);
  while(ros::ok()){
    twist.header.stamp = ros::Time::now();
    twist_pub.publish(twist);
    ros::spinOnce();
    rate.sleep();
  }
}
