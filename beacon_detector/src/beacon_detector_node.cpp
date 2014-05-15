#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <ros/ros.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <XmlRpcException.h>
#include <boost/foreach.hpp>
#include <boost/assign/list_of.hpp>


class BeaconDetector{
private:
  typedef std::pair<int, tf::Transform> tag_location_pair;
  std::map<int, tf::Transform> tag_locations;
  std::map<int, ros::Publisher> tag_pubs;
  ros::Timer tf_timer;
  ros::Subscriber detection_sub;
  
  tf::TransformBroadcaster tf_pub;
public:
  BeaconDetector(ros::NodeHandle nh, ros::NodeHandle pnh){
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


    BOOST_FOREACH(tag_location_pair tag_location_pair, tag_locations){
      int id = tag_location_pair.first;
      std::stringstream topic_name_ss;
      topic_name_ss << "beacon_estimate/tag_" << id;
      tag_pubs[id] = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(topic_name_ss.str(), 1);
    }

    tf_timer = nh.createTimer(ros::Duration(0.1), &BeaconDetector::timerCb, this);
    detection_sub = nh.subscribe("tag_detections", 1, &BeaconDetector::tagCb, this);
  }
private:
  void timerCb(const ros::TimerEvent& e){
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
    if(msg->detections.size()>0){
      BOOST_FOREACH(apriltags_ros::AprilTagDetection detection, msg->detections){
	std::map<int, tf::Transform>::const_iterator transform_itr = tag_locations.find(detection.id);
	if(transform_itr != tag_locations.end()){
	  tf::Transform transform = transform_itr->second;
	  tf::Pose pose;
	  tf::poseMsgToTF(detection.pose.pose, pose);
	  tf::Pose result = pose * transform.inverse();
	  geometry_msgs::PoseWithCovarianceStamped result_pose;
	  tf::poseTFToMsg(result, result_pose.pose.pose);
	  result_pose.header = detection.pose.header;
	  double var = 0.2;
	  result_pose.pose.covariance = boost::assign::list_of(var) (0)   (0)  (0)  (0)  (0)
				 		    	       (0) (var)  (0)  (0)  (0)  (0)
				 		    	       (0)  (0)  (var) (0)  (0)  (0)
				 		    	       (0)  (0)   (0) (var) (0)  (0)
				 		    	       (0)  (0)   (0)  (0) (var) (0)
				 		    	       (0)  (0)   (0)  (0)  (0) (var);
	  tag_pubs[detection.id].publish(result_pose);
	  ROS_INFO("Got detection %d, (%f, %f, %f)", detection.id, result.getOrigin().y(), result.getOrigin().y(), result.getOrigin().z());
	}
      }
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
      double yaw = (double)tag_description["yaw"];


      tf::Transform tag_transform(tf::createQuaternionFromRPY(roll, pitch, yaw), tf::Vector3(x, y, z));

      ROS_INFO_STREAM("Loaded tag location: "<<id);
      descriptions.insert(std::make_pair(id, tag_transform));
    }
    return descriptions;
  }

};

int main(int argc, char **argv){
  ros::init(argc, argv, "beacon_detector");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  BeaconDetector detector(nh, pnh);

  ros::spin();
}
