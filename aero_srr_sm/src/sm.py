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
from vision.msg import *
import math
import tf2_ros
import tf

from fake_state import *
from detection_pickup_state import *
from move_state_util import *
from arm_state_util import *


def child_term_cb(outcome_map):
    if outcome_map['WAIT_FOR_DETECTION'] == 'invalid':
        return True
    elif outcome_map['WAIT_DETECT_FAKE'] == 'succeeded':
        return True
    elif outcome_map['WAIT_DETECT_FAKE'] == 'preempted':
        return True
    elif outcome_map['WAIT_DETECT_FAKE'] == 'aborted':
        return True
    else:
        return False

def out_cb(outcome_map):
    if outcome_map['WAIT_FOR_DETECTION'] == 'invalid':
        return 'succeeded'
    elif outcome_map['WAIT_DETECT_FAKE'] == 'succeeded':
        return 'failed'
    elif outcome_map['WAIT_DETECT_FAKE'] == 'preempted':
        return 'failed'
    elif outcome_map['WAIT_DETECT_FAKE'] == 'aborted':
        return 'failed'
    else:
        return 'failed'
    
def monitor_cb(ud, msg):
    ud['detection_msg'] = msg
    return False




# main
def main():
    rospy.init_node('aero_srr_sm')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded', 'failed'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('WAIT_FOR_IMU',
                               smach_ros.MonitorState("/aero/imu/is_calibrated", Bool, lambda ud, msg: not msg.data),
        #becomes invalid when callback is false
                               transitions={'invalid':'LEAVE_PLATFORM',
                                            'valid':'failed',
                                            'preempted':'failed'})

        smach.StateMachine.add('LEAVE_PLATFORM',
                               create_move_state(5, 0, 0),
                               transitions={'succeeded':'MOVE_TOWARDS_PRECACHE',
                                            'aborted':'failed',
                                            'preempted':'failed'})
        smach.StateMachine.add('MOVE_TOWARDS_PRECACHE',
                               create_move_state(10, 0, 0),
                               transitions={'succeeded':'SEARCH_FOR_PRECACHE',
                                            'aborted':'failed',
                                            'preempted':'failed'})

        
        drive_detect_concurrence = smach.Concurrence(outcomes=['succeeded', 'failed'],
                                                    default_outcome='failed',
                                                    output_keys=['detection_msg'],
                                                    child_termination_cb=child_term_cb,
                                                    outcome_cb=out_cb)
        with drive_detect_concurrence:
                smach.Concurrence.add('WAIT_FOR_DETECTION',
                                      smach_ros.MonitorState("/aero/lower_detection", ObjectLocationMsg, monitor_cb, output_keys = ['detection_msg']))
                smach.Concurrence.add('DRIVE_WHILE_DETECTING', create_move_state(20, 0, 0))
        smach.StateMachine.add('SEARCH_FOR_PRECACHE', drive_detect_concurrence,
                               transitions={'succeeded':'PICKUP_PRECACHE',
                                            'failed':'failed'})
        
        smach.StateMachine.add('PICKUP_PRECACHE', DetectionPickupState(),
                               transitions={'succeeded':'NAV_TO_PLATFORM',
                                            'aborted':'failed',
                                            'preempted':'failed'})

        smach.StateMachine.add('NAV_TO_PLATFORM',
                               create_move_state(5, 0, 0),
                               transitions={'succeeded':'NAV_ONTO_PLATFORM',
                                            'aborted':'failed',
                                            'preempted':'failed'})
        smach.StateMachine.add('NAV_ONTO_PLATFORM',
                               create_move_state(0, 0, 0),
                               transitions={'succeeded':'MOVE_TOWARDS_PRECACHE',
                                            'aborted':'failed',
                                            'preempted':'failed'})

    sis = smach_ros.IntrospectionServer('aero_srr_sm', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
