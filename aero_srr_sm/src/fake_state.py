#!/usr/bin/env python

import roslib; roslib.load_manifest('aero_srr_sm')
import rospy
import smach

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
        
