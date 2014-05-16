#!/usr/bin/env python

import roslib; roslib.load_manifest('aero_srr_sm')
import rospy
import smach
import smach_ros
from pprint import pprint
from actionlib import *
from actionlib.msg import *
from std_msgs.msg import *
from nav_msgs.msg import *
from geometry_msgs.msg import *
from move_base_msgs.msg import *
import math
import tf2_ros
import tf

from fake_state import *
from simple_publisher_state import *
from detection_pickup_state import *
from detection_drive_state import *
from move_state_util import *
from arm_state_util import *
from detection_state_util import *


def child_term_cb(outcome_map):
    if outcome_map['WAIT_FOR_DETECTION'] == 'invalid':
        return True
    elif outcome_map['DRIVE_WHILE_DETECTING'] == 'succeeded':
        return True
    elif outcome_map['DRIVE_WHILE_DETECTING'] == 'preempted':
        return True
    elif outcome_map['DRIVE_WHILE_DETECTING'] == 'aborted':
        return True
    else:
        return False

def out_cb(outcome_map):
    if outcome_map['WAIT_FOR_DETECTION'] == 'invalid':
        return 'succeeded'
    elif outcome_map['DRIVE_WHILE_DETECTING'] == 'succeeded':
        return 'failed'
    elif outcome_map['DRIVE_WHILE_DETECTING'] == 'preempted':
        return 'failed'
    elif outcome_map['DRIVE_WHILE_DETECTING'] == 'aborted':
        return 'failed'
    else:
        return 'failed'
    
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
                               transitions={'invalid':'WAIT_FOR_START',
                                            'valid':'failed',
                                            'preempted':'failed'})

        smach.StateMachine.add('WAIT_FOR_START', FakeState(),
                               transitions={'succeeded':'SHUTTER_LASER',
                                            'aborted':'failed',
                                            'preempted':'failed'})

        smach.StateMachine.add('SHUTTER_LASER', SimplePublisherState('/aero/laser_shutter', Bool, Bool(True)),
                               transitions={'succeeded':'LEAVE_PLATFORM'})
        smach.StateMachine.add('LEAVE_PLATFORM',
                               create_move_state(2, 0, 0),
                               transitions={'succeeded':'UNSHUTTER_LASER',
                                            'aborted':'failed',
                                            'preempted':'failed'})
        smach.StateMachine.add('UNSHUTTER_LASER', SimplePublisherState('/aero/laser_shutter', Bool, Bool(False)),
                               transitions={'succeeded':'MOVE_TOWARDS_PRECACHE'})
        smach.StateMachine.add('MOVE_TOWARDS_PRECACHE',
                               create_move_state(3, 0, 0),
                               transitions={'succeeded':'SEARCH_FOR_PRECACHE',
                                            'aborted':'failed',
                                            'preempted':'failed'})

        
        drive_detect_concurrence = smach.Concurrence(outcomes=['succeeded', 'failed'],
                                                    default_outcome='failed',
                                                    output_keys=['detection_msg'],
                                                    child_termination_cb=child_term_cb,
                                                    outcome_cb=out_cb)
        with drive_detect_concurrence:
                smach.Concurrence.add('WAIT_FOR_DETECTION', create_detect_state())
                smach.Concurrence.add('DRIVE_WHILE_DETECTING', create_move_state(10, 0, 0))
        smach.StateMachine.add('SEARCH_FOR_PRECACHE', drive_detect_concurrence,
                               transitions={'succeeded':'A',
                                            'failed':'failed'})

        smach.StateMachine.add('A', FakeState(),
                               transitions={'succeeded':'B',
                                            'aborted':'failed',
                                            'preempted':'failed'})
        smach.StateMachine.add('B', create_detect_state(),
                               transitions={'invalid':'NAV_NEAR_PRECACHE',
                                            'valid':'failed',
                                            'preempted':'failed'})

        smach.StateMachine.add('NAV_NEAR_PRECACHE', DetectionDriveState(-1.5, -0.2),
                               transitions={'succeeded':'DETECT_WHEN_NEAR',
                                            'aborted':'failed',
                                            'preempted':'failed'})

        smach.StateMachine.add('DETECT_WHEN_NEAR', create_detect_state(),
                               transitions={'invalid':'NAV_TO_PRECACHE',
                                            'valid':'failed',
                                            'preempted':'failed'})

        smach.StateMachine.add('NAV_TO_PRECACHE', DetectionDriveState(-0.7, -0.2),
                               transitions={'succeeded':'WAIT_FOR_DETECTION_AFTER_NAV',
                                            'aborted':'failed',
                                            'preempted':'failed'})
        
        smach.StateMachine.add('WAIT_FOR_DETECTION_AFTER_NAV', create_detect_state(),
                               transitions={'invalid':'PICKUP_PRECACHE',
                                            'valid':'failed',
                                            'preempted':'failed'})
        #smach.StateMachine.add('PICKUP_PRECACHE', FakeState(),
        #                       transitions={'succeeded':'NAV_TO_PLATFORM',
        #                                    'aborted':'failed',
        #                                    'preempted':'failed'})
        smach.StateMachine.add('PICKUP_PRECACHE', DetectionPickupState(),
                               transitions={'succeeded':'succeeded',
##                               transitions={'succeeded':'NAV_TO_PLATFORM',
                                            'aborted':'failed',
                                            'preempted':'failed'})

        #smach.StateMachine.add('NAV_TO_PLATFORM',
        #                       create_move_state(2, 0, 0),
        #                       transitions={'succeeded':'NAV_ONTO_PLATFORM',
        #                                    'aborted':'failed',
        #                                    'preempted':'failed'})
        #smach.StateMachine.add('NAV_ONTO_PLATFORM',
        #                       create_move_state(0, 0, 0),
        #                       transitions={'succeeded':'MOVE_TOWARDS_PRECACHE',
        #                                    'aborted':'failed',
        #                                    'preempted':'failed'})

    sis = smach_ros.IntrospectionServer('aero_srr_sm', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
