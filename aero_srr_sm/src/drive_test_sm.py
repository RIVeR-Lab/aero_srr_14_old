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
from srr_search.msg import *
import math
import tf2_ros
import tf

from fake_state import *
from move_state_util import *


# main
def main():
    rospy.init_node('drive_test_sm')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded', 'failed'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('INIT', FakeState(),
                               transitions={'succeeded':'SPIRAL',
                                            'aborted':'failed',
                                            'preempted':'failed'})

        smach.StateMachine.add('SPIRAL',
                               create_spiral_search_state(0, 0, 0, 10, 2, 2, math.pi/6),
                               transitions={'succeeded':'INIT',
                                            'aborted':'failed',
                                            'preempted':'failed'})


    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
