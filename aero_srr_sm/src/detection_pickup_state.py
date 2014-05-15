#!/usr/bin/env python

import roslib; roslib.load_manifest('aero_srr_sm')
import rospy
import smach
from arm_state_util import *
import actionlib
from jaco_msgs.msg import *
import tf
import moveit_commander
import moveit_msgs.msg

class DetectionPickupState(smach.State):
    def __init__(self, robot, jaco_group):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'], input_keys=['detection_msg'])
        self.jaco_group = jaco_group
        self.robot = robot
        self.tf_listener = tf.TransformListener()
            

    def execute(self, userdata):
        rospy.loginfo('Executing pickup state')
        print('Got detection ', str(userdata['detection_msg']) )

        object_location = PoseStamped()
        object_location.pose = userdata['detection_msg'].pose.pose;
        object_location.header = userdata['detection_msg'].header;
        print('Got location ', str(object_location) )
        rospy.loginfo('planning in %s', self.jaco_group.get_planning_frame())
        try:
            self.tf_listener.waitForTransform(self.jaco_group.get_planning_frame(), object_location.header.frame_id, object_location.header.stamp, rospy.Duration(1))
        
            object_location_planning = self.tf_listener.transformPose(self.jaco_group.get_planning_frame(), object_location)
            object_goal_pose = Pose()
            object_goal_pose.position = object_location_planning.pose.position
            object_goal_pose.position.z = object_goal_pose.position.z + 0.50
            angle = tf.transformations.quaternion_from_euler(0, 0, 0);
            object_goal_pose.orientation = Quaternion(x = angle[0],
                                     y = angle[1],
                                     z = angle[2],
                                     w = angle[3])

            print('Got goal in planning frame ', str(object_goal_pose) )

            print self.robot.get_current_state()
            self.jaco_group.set_pose_target(object_goal_pose)

            rospy.loginfo('Creating arm plan')
            plan = self.jaco_group.plan()

            if not plan.joint_trajectory.header.frame_id:
                rospy.logerr('Arm planning failed')
                return 'aborted'
            print('Got plan ', str(plan) )
            rospy.loginfo('Executing arm plan')

            self.jaco_group.execute(plan)

            object_goal_pose.position.z = object_goal_pose.position.z - 0.15

            self.jaco_group.set_pose_target(object_goal_pose)

            rospy.loginfo('Creating arm plan')
            plan = self.jaco_group.plan()

            if not plan.joint_trajectory.header.frame_id:
                rospy.logerr('Arm planning failed')
                return 'aborted'
            print('Got plan ', str(plan) )
            rospy.loginfo('Executing arm plan')

            self.jaco_group.execute(plan)

        except tf.Exception:
            return 'aborted'
        return 'succeeded'
        
