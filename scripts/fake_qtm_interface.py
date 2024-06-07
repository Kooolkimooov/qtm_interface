#!/usr/bin/python3

# roslaunch qtm_interface qtm_interface.launch

"""
    Streaming 6Dof from QTM
"""

import rospy
from geometry_msgs.msg import PoseStamped
import numpy as np
from scipy.spatial.transform import Rotation as R


def fake_packets():  

    pose_publisher = rospy.Publisher("fake_6dof_pose", PoseStamped, queue_size=2)
    rate = rospy.Rate(100)

    robot_pose = PoseStamped()

    # Linear position          
    robot_pose.pose.position.x = 1.
    robot_pose.pose.position.y = 1.
    robot_pose.pose.position.z = 1.

    matrix = np.eye(3)
    matrix[1, 1] = -1
    matrix[2, 2] = -1

    quat = R.from_matrix(np.array(matrix).reshape((3,3))).as_quat()

    # Angular position
    robot_pose.pose.orientation.x = quat[0]
    robot_pose.pose.orientation.y = quat[1]
    robot_pose.pose.orientation.z = quat[2]
    robot_pose.pose.orientation.w = quat[3]

    robot_pose.header.frame_id = "map"
    framenumber = 0

    while not rospy.is_shutdown():

        robot_pose.header.seq = framenumber
        framenumber+=1

        pose_publisher.publish(robot_pose)
        rate.sleep()

if __name__ == "__main__":
    qtm_node = rospy.init_node("qtm")
    fake_packets()