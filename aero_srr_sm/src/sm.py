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
from srr_search.msg import *
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
from sensor_state_util import *
from drive_home_state import *


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
                               transitions={'invalid':'SHUTTER_LASER',
                                            'valid':'WAIT_FOR_IMU',
                                            'preempted':'WAIT_FOR_IMU'})

        smach.StateMachine.add('SHUTTER_LASER', create_shutter_laser_state(),
                               transitions={'succeeded':'WAIT_FOR_START', 'failed': 'WAIT_FOR_START'})

        smach.StateMachine.add('WAIT_FOR_START', FakeState(),
                               transitions={'succeeded':'STOW_ARM',
                                            'aborted':'WAIT_FOR_START',
                                            'preempted':'WAIT_FOR_START'})

        smach.StateMachine.add('STOW_ARM', ArmStowState(),
                               transitions={'succeeded':'LEAVE_PLATFORM',
                                            'aborted':'LEAVE_PLATFORM',
                                            'preempted':'STOW_ARM'})

        smach.StateMachine.add('LEAVE_PLATFORM',
                               create_move_state(4, 0, 0),
                               transitions={'succeeded':'UNSHUTTER_LASER',
                                            'aborted':'LEAVE_PLATFORM',
                                            'preempted':'LEAVE_PLATFORM'})
        smach.StateMachine.add('UNSHUTTER_LASER', create_unshutter_laser_state(),
                               transitions={'succeeded':'MOVE_TOWARDS_PRECACHE'})
        smach.StateMachine.add('MOVE_TOWARDS_PRECACHE',
                               create_move_state(25, 0, 0),
                               transitions={'succeeded':'SEARCH_FOR_PRECACHE',
                                            'aborted':'MOVE_TOWARDS_PRECACHE',
                                            'preempted':'MOVE_TOWARDS_PRECACHE'})

        
        drive_detect_concurrence = smach.Concurrence(outcomes=['succeeded', 'failed'],
                                                    default_outcome='failed',
                                                    output_keys=['detection_msg'],
                                                    child_termination_cb=child_term_cb,
                                                    outcome_cb=out_cb)

        spiral_goal = SpiralSearchGoal()
        with drive_detect_concurrence:
                smach.Concurrence.add('WAIT_FOR_DETECTION', create_detect_state())
                smach.Concurrence.add('DRIVE_WHILE_DETECTING', create_spiral_search_state(25, 0, 0, 10, 2, 2, math.pi/3))
        smach.StateMachine.add('SEARCH_FOR_PRECACHE', drive_detect_concurrence,
                               transitions={'succeeded':'SHUTTER_LASER_FOR_PICKUP',
                                            'failed':'SEARCH_FOR_PRECACHE'})

        smach.StateMachine.add('SHUTTER_LASER_FOR_PICKUP', SimplePublisherState('/aero/laser_shutter', Bool, Bool(True), True, ['detection_msg']),
                               transitions={'succeeded':'CHECK_NEAR_PRECACHE'})


        smach.StateMachine.add('WAIT_BEFORE_DETECT', DelayState(2.0),
                               transitions={'succeeded':'DETECT',
                                            'aborted':'DETECT'})

        smach.StateMachine.add('DETECT', create_detect_state(),
                               transitions={'invalid':'CHECK_NEAR_PRECACHE',
                                            'valid':'SEARCH_FOR_PRECACHE',
                                            'preempted':'SEARCH_FOR_PRECACHE'})

        smach.StateMachine.add('CHECK_NEAR_PRECACHE', CheckNearPrecacheState(),
                               transitions={'far':'NAV_NEAR_PRECACHE',
                                            'near':'NAV_CLOSE_PRECACHE',
                                            'close':'PICKUP_PRECACHE',
                                            'aborted':'DETECT'})

        smach.StateMachine.add('NAV_NEAR_PRECACHE', DetectionDriveState(-1.5, -0.2),
                               transitions={'succeeded':'WAIT_BEFORE_DETECT',
                                            'aborted':'SEARCH_FOR_PRECACHE',
                                            'preempted':'DETECT'})

        smach.StateMachine.add('NAV_CLOSE_PRECACHE', DetectionDriveState(-0.7, -0.2),
                               transitions={'succeeded':'WAIT_BEFORE_DETECT',
                                            'aborted':'SEARCH_FOR_PRECACHE',
                                            'preempted':'DETECT'})

        smach.StateMachine.add('PICKUP_PRECACHE', DetectionPickupState(),
                               transitions={'succeeded':'UNSHUTTER_LASER_AFTER_PICKUP',
                                            'aborted':'DETECT',
                                            'preempted':'DETECT'})

        smach.StateMachine.add('UNSHUTTER_LASER_AFTER_PICKUP', SimplePublisherState('/aero/laser_shutter', Bool, Bool(False), True),
                               transitions={'succeeded':'RETURN_TO_START'})

        smach.StateMachine.add('RETURN_TO_START',
                               create_drive_home_state(),
                               transitions={'succeeded':'succeeded',
                                            'aborted':'RETURN_TO_START',
                                            'preempted':'RETURN_TO_START'})

    sis = smach_ros.IntrospectionServer('aero_srr_sm', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
