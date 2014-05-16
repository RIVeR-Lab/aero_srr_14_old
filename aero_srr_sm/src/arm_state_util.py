import roslib; roslib.load_manifest('aero_srr_sm')
import rospy
import smach
import smach_ros
from pprint import pprint
from actionlib import *
from actionlib.msg import *
from std_msgs.msg import *
from geometry_msgs.msg import *
from jaco_msgs.msg import *
import math
import tf2_ros
import tf


def create_arm_pose_from_pose_stamped(pose):
    return TrajectoryPoint(
        position_type = TrajectoryPoint.POSITION_TYPE_CARTESIAN_POSITION,
        position = pose,
        hand_mode = TrajectoryPoint.HAND_MODE_NO_MOVE
        )
def create_arm_api_pose(x, y, z, roll, pitch, yaw):
    return create_arm_pose('jaco_api_origin', rospy.get_rostime(), x, y, z, roll, pitch, yaw)
def create_arm_pose(frame, stamp, x, y, z, roll, pitch, yaw):
    angle = tf.transformations.quaternion_from_euler(roll, pitch, yaw);
    pose = PoseStamped(
        header = Header(frame_id=frame, stamp=stamp),
        pose   = Pose(
            position = Point(x = x, y = y, z = z),
            orientation = Quaternion(x = angle[0],
                                     y = angle[1],
                                     z = angle[2],
                                     w = angle[3]),
            )
    );
    return create_arm_pose_from_pose_stamped(pose)

def create_arm_delay(delay):
    return TrajectoryPoint(
        position_type = TrajectoryPoint.POSITION_TYPE_TIME_DELAY,
        delay = delay,
        hand_mode = TrajectoryPoint.HAND_MODE_NO_MOVE
        )

def create_arm_fingers(base, f1, f2, f3):
    base.fingers = FingerPosition(Finger_1 = f1, Finger_2 = f2, Finger_3 = f3)
    base.hand_mode = TrajectoryPoint.HAND_MODE_POSITION
    return base

def create_arm_trajectory_state(trajectory):
    goal=TrajectoryGoal(trajectory);
    return smach_ros.SimpleActionState('/aero/jaco/arm_trajectory', TrajectoryAction, goal)

