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


def create_arm_state(x, y, z, roll, pitch, yaw):
    goal_angle = tf.transformations.quaternion_from_euler(roll, pitch, yaw);
    goal=MoveBaseGoal(
        target_pose=PoseStamped(
            header = Header(frame_id="aero/odom"),
            pose   = Pose(
                position = Point(x = x, y = y, z = z),
                orientation = Quaternion(x = goal_angle[0],
                                         y = goal_angle[1],
                                         z = goal_angle[2],
                                         w = goal_angle[3]),
                )
        ));
    return smach_ros.SimpleActionState('/aero/move_base', MoveBaseAction, goal)

