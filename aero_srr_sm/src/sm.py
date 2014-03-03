#!/usr/bin/env python

import roslib; roslib.load_manifest('aero_srr_sm')
import rospy
import smach
import smach_ros
from pprint import pprint

# define state Foo
class FakeState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'])

    def execute(self, userdata):
        rospy.loginfo('Executing fake state ')
        outcome = ''
        while outcome not in self.get_registered_outcomes():
            print 'enter a possible outcome: '+str(self.get_registered_outcomes())
            outcome = raw_input('Enter state outcome: ');
            if outcome == 's':
                outcome = 'succeeded';
            if outcome == 'a':
                outcome = 'aborted';
            if outcome == 'p':
                outcome = 'preempted';
        return outcome
        



# main
def main():
    rospy.init_node('aero_srr_sm')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded', 'failed'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('INIT', FakeState(),
                               transitions={'succeeded':'LEAVE_PLATFORM',
                                            'aborted':'failed',
                                            'preempted':'failed'})
        smach.StateMachine.add('LEAVE_PLATFORM', FakeState(),
                               transitions={'succeeded':'SEARCH_FOR_PRECACHE',
                                            'aborted':'failed',
                                            'preempted':'failed'})
        smach.StateMachine.add('SEARCH_FOR_PRECACHE', FakeState(),
                               transitions={'succeeded':'NAV_TO_PRECACHE',
                                            'aborted':'SEARCH_FOR_PRECACHE',
                                            'preempted':'failed'})
        smach.StateMachine.add('NAV_TO_PRECACHE', FakeState(),
                               transitions={'succeeded':'PICKUP_PRECACHE',
                                            'aborted':'SEARCH_FOR_PRECACHE',
                                            'preempted':'failed'})

        pickup_sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
        with pickup_sm:
            smach.StateMachine.add('NAV_TO_PICKUP_POSITION', FakeState(),
                                   transitions={'succeeded':'POSITION_ARM_FOR_GRAB',
                                                'aborted':'aborted',
                                                'preempted':'preempted'})
            smach.StateMachine.add('POSITION_ARM_FOR_GRAB', FakeState(),
                                   transitions={'succeeded':'GRAB',
                                                'aborted':'aborted',
                                                'preempted':'preempted'})
            smach.StateMachine.add('GRAB', FakeState(),
                                   transitions={'succeeded':'PICKUP_SAMPLE',
                                                'aborted':'POSITION_ARM_FOR_GRAB',
                                                'preempted':'preempted'})
            smach.StateMachine.add('PICKUP_SAMPLE', FakeState(),
                                   transitions={'succeeded':'STORE_SAMPLE',
                                                'aborted':'PICKUP_SAMPLE',
                                                'preempted':'preempted'})
            smach.StateMachine.add('STORE_SAMPLE', FakeState(),
                                   transitions={'succeeded':'succeeded',
                                                'aborted':'STORE_SAMPLE',
                                                'preempted':'preempted'})
        smach.StateMachine.add('PICKUP_PRECACHE', pickup_sm,
                               transitions={'succeeded':'NAV_TO_PLATFORM',
                                            'aborted':'SEARCH_FOR_PRECACHE',
                                            'preempted':'failed'})

        smach.StateMachine.add('NAV_TO_PLATFORM', FakeState(),
                               transitions={'succeeded':'NAV_ONTO_PLATFORM',
                                            'aborted':'NAV_TO_PLATFORM',
                                            'preempted':'failed'})
        smach.StateMachine.add('NAV_ONTO_PLATFORM', FakeState(),
                               transitions={'succeeded':'succeeded',
                                            'aborted':'NAV_TO_PLATFORM',
                                            'preempted':'failed'})

    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
