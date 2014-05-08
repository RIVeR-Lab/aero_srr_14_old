#!/usr/bin/env python

import roslib; roslib.load_manifest('aero_srr_sm')
import rospy
import smach
from move_state_util import *
import actionlib
from move_base_msgs.msg import *
import tf

class DetectionDriveState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'], input_keys=['detection_msg'])
        self.client = actionlib.SimpleActionClient('/aero/move_base', MoveBaseAction)
        self.tf_listener = tf.TransformListener()
        
            

    def execute(self, userdata):
        rospy.loginfo('Executing nav to detection state')
        print('Got detection ', str(userdata['detection_msg']) )

        self.client.wait_for_server()

        object_location = userdata['detection_msg'].pose;
        print('Got location ', str(object_location) )

        try:
            self.tf_listener.waitForTransform('aero/base_footprint', object_location.header.frame_id, object_location.header.stamp, rospy.Duration(1))
        
            object_location_base = self.tf_listener.transformPose('aero/base_footprint', object_location)
            object_position = object_location_base.pose.position

            goal=create_move_goal('aero/base_footprint', object_location_base.header.stamp, object_position.x-0.75, object_position.y-0.2, 0);
            self.client.send_goal(goal)

            self.client.wait_for_result()
            if self.client.get_state() == GoalStatus.SUCCEEDED:
                return 'succeeded';
            if self.client.get_state() == GoalStatus.PREEMPTED:
                return 'preempted'
        except tf.Exception:
            return 'aborted'
        return 'aborted'
        
