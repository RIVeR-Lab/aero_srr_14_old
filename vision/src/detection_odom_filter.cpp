#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

class DetectionOdomFilter{
private:
  tf::TransformBroadcaster tf_pub;
  tf::TransformListener tf_sub;
  ros::Subscriber detection_odom_sub;
  ros::Publisher detection_pub;
  ros::Timer tf_timer;
  geometry_msgs::Pose pose;

  double covar_limit;
  std::string frame_id;
  std::string child_frame_id;
  bool invert_tf;
public:
  DetectionOdomFilter(ros::NodeHandle nh, ros::NodeHandle pnh){
    covar_limit = 0.2;
    frame_id = "frame";
    child_frame_id = "child";
    invert_tf = false;
    
    pnh.getParam("covar_limit", covar_limit);
    pnh.getParam("frame_id", frame_id);
    pnh.getParam("child_frame_id", child_frame_id);
    pnh.getParam("invert_tf", invert_tf);

    detection_pub = nh.advertise<geometry_msgs::PoseStamped>("detection", 1);
    pose.orientation = tf::createQuaternionMsgFromYaw(0);

    detection_odom_sub = nh.subscribe("detection_odom", 1, &DetectionOdomFilter::detection_cb, this);
    tf_timer = nh.createTimer(ros::Duration(0.05), &DetectionOdomFilter::timerCb, this);
  }
  void detection_cb(const nav_msgs::Odometry::ConstPtr& msg){
    double max_covar = *std::max_element(msg->pose.covariance.begin(), msg->pose.covariance.end());
    if(max_covar < covar_limit){
      pose = msg->pose.pose;
      geometry_msgs::PoseStamped pose_stamped;
      pose_stamped.pose = pose;
      pose_stamped.header = msg->header;
      detection_pub.publish(pose_stamped);
    }
  }
  void timerCb(const ros::TimerEvent& e){
    geometry_msgs::TransformStamped stamped_transform;
    stamped_transform.header.frame_id = frame_id;
    stamped_transform.header.stamp = ros::Time::now();
    stamped_transform.child_frame_id = child_frame_id;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
    transform.setRotation(tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w));
    if(invert_tf)
      tf::transformTFToMsg(transform.inverse(), stamped_transform.transform);
    else
      tf::transformTFToMsg(transform, stamped_transform.transform);
    tf_pub.sendTransform(stamped_transform);
  }

};


int main(int argc, char **argv){
  ros::init(argc, argv, "detection_odom_filter");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  DetectionOdomFilter odom_filter(nh, pnh);

  ros::spin();
}
