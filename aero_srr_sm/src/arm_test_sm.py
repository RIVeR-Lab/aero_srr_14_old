#!/usr/bin/env python

import roslib; roslib.load_manifest('aero_srr_sm')
import rospy
import smach
import smach_ros
from pprint import pprint
from actionlib import *
from actionlib.msg import *
from std_msgs.msg import *
from geometry_msgs.msg import *
from move_base_msgs.msg import *
import math
import tf2_ros
import tf

from fake_state import *
from move_state_util import *
from arm_state_util import *


# main
def main():
    rospy.init_node('arm_test_sm')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded', 'failed'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('INIT', FakeState(),
                               transitions={'succeeded':'TRAJECTORY_A',
                                            'aborted':'failed',
                                            'preempted':'failed'})

        trajectory_a = [create_arm_pose('jaco_api_origin', -0.35, -0.2, 0.4, -math.pi/2, -math.pi/2, 0),
                        create_arm_pose('jaco_api_origin', -0.35, 0.1, 0.4, -math.pi/2, 0, 0),
                        create_arm_fingers(create_arm_pose('jaco_api_origin', -0.35, 0.1, 0.04, -math.pi/2, 0, 0), 1, 1, 1),
                        create_arm_pose('jaco_api_origin', -0.35, 0.2, 0.04, -math.pi/2, 0, 0),
                        create_arm_fingers(create_arm_pose('jaco_api_origin', -0.35, 0.2, 0.04, -math.pi/2, 0, 0), 60, 60, 60),
                        create_arm_pose('jaco_api_origin', -0.35, 0.2, 0.4, -math.pi/2, 0, 0)]
        smach.StateMachine.add('TRAJECTORY_A',
                               create_arm_trajectory_state(trajectory_a),
                               transitions={'succeeded':'succeeded',
                                            'aborted':'failed',
                                            'preempted':'failed'})


    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()

    #rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
