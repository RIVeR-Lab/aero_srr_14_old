#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>

class BeaconTransformUpdater{
private:
  tf::TransformBroadcaster tf_pub;
  tf::TransformListener tf_sub;
  ros::Subscriber filtered_beacon_sub;
  ros::Timer tf_timer;
  geometry_msgs::Pose beacon_pose;
public:
  BeaconTransformUpdater(ros::NodeHandle nh, ros::NodeHandle pnh){
    filtered_beacon_sub = nh.subscribe("beacon_estimate/filtered", 1, &BeaconTransformUpdater::beacon_estimate_cb, this);
    tf_timer = nh.createTimer(ros::Duration(0.05), &BeaconTransformUpdater::timerCb, this);
    beacon_pose.orientation = tf::createQuaternionMsgFromYaw(0);
  }
  void beacon_estimate_cb(const nav_msgs::Odometry::ConstPtr& msg){
    double max_covar = *std::max_element(msg->pose.covariance.begin(), msg->pose.covariance.end());
    if(max_covar < 0.2){
      beacon_pose = msg->pose.pose;
    }
  }
  void timerCb(const ros::TimerEvent& e){
    geometry_msgs::TransformStamped stamped_transform;
    stamped_transform.header.frame_id = "beacon";
    stamped_transform.header.stamp = ros::Time::now();
    stamped_transform.child_frame_id = "aero/odom";
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(beacon_pose.position.x, beacon_pose.position.y, beacon_pose.position.z));
    transform.setRotation(tf::Quaternion(beacon_pose.orientation.x, beacon_pose.orientation.y, beacon_pose.orientation.z, beacon_pose.orientation.w));
    tf::transformTFToMsg(transform.inverse(), stamped_transform.transform);
    tf_pub.sendTransform(stamped_transform);
  }

};


int main(int argc, char **argv){
  ros::init(argc, argv, "beacon_transform_updater");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  BeaconTransformUpdater transform_updater(nh, pnh);

  ros::spin();
}
