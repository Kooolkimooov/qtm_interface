#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist
from qtm_interface import BODY_NAME

#Definition of the class node.
class send_robot_command(object):
    def __init__(self) -> None:
        #Initialization of the node
        rospy.init_node("send_robot_command")

        #Creation of the publisher
        self.publisher = rospy.Publisher("robot_topic", Twist, queue_size=10)

        #Definition of the time rate in hz
        self.rate = rospy.Rate(100)

    def loginfo(self,arg):
        rospy.loginfo(arg)
    
    def send_twist_to_robot(self,Vx,Vy,Vz,Wx,Wy,Wz):
        data = Twist()

        #Linear Velocity
        data.linear.x = Vx
        data.linear.y = Vy
        data.linear.z = Vz

        #Angulare Velocity
        data.angular.x = Wx
        data.angular.y = Wy
        data.angular.z = Wz

        self.publisher.publish(data)




def main():
    try:
        #Creation of a node
        node = send_robot_command()

        rospy.loginfo('Hello, ROS1!!!')

        while not rospy.is_shutdown():
            node.send_twist_to_robot(2,0,0,0,0,0)
            node.rate.sleep()

    except rospy.ROSInternalException:
        pass


if __name__=="__main__":
    main()