#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <ros/ros.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <geometry_msgs/PoseArray.h>
#include <XmlRpcException.h>
#include <boost/foreach.hpp>

void tagCb(const apriltags_ros::AprilTagDetectionArray::ConstPtr& msg);
std::map<int, tf::Transform> parse_tag_locations(XmlRpc::XmlRpcValue& tag_descriptions);
void timerCb(const ros::TimerEvent& e);

static std::map<int, tf::Transform> tag_locations;
int main(int argc, char **argv){
  ros::init(argc, argv, "beacon_detector");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");


  XmlRpc::XmlRpcValue tag_locations_xml;
  if(!pnh.getParam("tag_locations", tag_locations_xml)){
    ROS_WARN("No tag locations specified");
  }
  else{
    try{
      tag_locations = parse_tag_locations(tag_locations_xml);
    } catch(XmlRpc::XmlRpcException e){
      ROS_ERROR_STREAM("Error loading tag locations: "<<e.getMessage());
    }
  }

  ros::Timer tf_timer = nh.createTimer(ros::Duration(0.1), &timerCb);
  ros::Subscriber detection_sub = nh.subscribe("tag_detections", 1, &tagCb);
  ros::spin();
}


void timerCb(const ros::TimerEvent& e){
  static tf::TransformBroadcaster tf_pub;
  typedef std::pair<int, tf::Transform> tag_location_pair;
  BOOST_FOREACH(tag_location_pair tag_location_pair, tag_locations){
    int id = tag_location_pair.first;
    tf::Transform tag_transform = tag_location_pair.second;
    geometry_msgs::TransformStamped stamped_transform;
    stamped_transform.header.frame_id = "beacon";
    stamped_transform.header.stamp = ros::Time::now() + ros::Duration(0.25);
    std::stringstream frame_name_stream;
    frame_name_stream << "beacon_tag_" << id;
    stamped_transform.child_frame_id = frame_name_stream.str();
    tf::transformTFToMsg(tag_transform, stamped_transform.transform);
    tf_pub.sendTransform(stamped_transform);
  }
}

void tagCb(const apriltags_ros::AprilTagDetectionArray::ConstPtr& msg){
  static ros::NodeHandle nh;
  static ros::Publisher pub = nh.advertise<geometry_msgs::PoseArray>("beacon_estamates", 1);
  if(msg->detections.size()>0){
  geometry_msgs::PoseArray pose_array;
  pose_array.header = msg->detections[0].pose.header;
  BOOST_FOREACH(apriltags_ros::AprilTagDetection detection, msg->detections){
    std::map<int, tf::Transform>::const_iterator transform_itr = tag_locations.find(detection.id);
    if(transform_itr != tag_locations.end()){
      tf::Transform transform = transform_itr->second;
      tf::Pose pose;
      tf::poseMsgToTF(detection.pose.pose, pose);
      tf::Pose result = pose * transform.inverse();
      geometry_msgs::Pose result_pose;
      tf::poseTFToMsg(result, result_pose);
      pose_array.poses.push_back(result_pose);
      ROS_INFO("Got detection %d, (%f, %f, %f)", detection.id, result.getOrigin().y(), result.getOrigin().y(), result.getOrigin().z());
    }
  }
  pub.publish(pose_array);
}
}


std::map<int, tf::Transform> parse_tag_locations(XmlRpc::XmlRpcValue& tag_descriptions){
  std::map<int, tf::Transform> descriptions;
  ROS_ASSERT(tag_descriptions.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int32_t i = 0; i < tag_descriptions.size(); ++i) {
    XmlRpc::XmlRpcValue& tag_description = tag_descriptions[i];
    ROS_ASSERT(tag_description.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    ROS_ASSERT(tag_description["id"].getType() == XmlRpc::XmlRpcValue::TypeInt);
    ROS_ASSERT(tag_description["x"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    ROS_ASSERT(tag_description["y"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    ROS_ASSERT(tag_description["z"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    ROS_ASSERT(tag_description["roll"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    ROS_ASSERT(tag_description["pitch"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    ROS_ASSERT(tag_description["yaw"].getType() == XmlRpc::XmlRpcValue::TypeDouble);

    int id = (int)tag_description["id"];
    double x = (double)tag_description["x"];
    double y = (double)tag_description["y"];
    double z = (double)tag_description["z"];
    double roll = (double)tag_description["roll"];
    double pitch = (double)tag_description["pitch"];
    double yaw = (double)tag_description["yah"];


    tf::Transform tag_transform(tf::createQuaternionFromRPY(roll, pitch, yaw), tf::Vector3(x, y, z));

    ROS_INFO_STREAM("Loaded tag location: "<<id);
    descriptions.insert(std::make_pair(id, tag_transform));
  }
  return descriptions;
}
