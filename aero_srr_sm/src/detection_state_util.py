#!/usr/bin/env python

import roslib; roslib.load_manifest('aero_srr_sm')
import rospy
import smach
from arm_state_util import *
import actionlib
from jaco_msgs.msg import *
from jaco_msgs.srv import *
import tf
from geometry_msgs.msg import *
from nav_msgs.msg import *

def detect_monitor_cb(ud, msg):
    #print msg
    ud['detection_msg'] = msg
    return False

def create_detect_state():
    return smach_ros.MonitorState("/aero/lower_stereo/object_detection/filtered", PoseStamped, detect_monitor_cb, output_keys = ['detection_msg'])
