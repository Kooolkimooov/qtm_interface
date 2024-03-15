#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist
from qtm_interface import BODY_NAME
import keyboard

#Definition of the class node.
class send_robot_command(object):
    def __init__(self) -> None:
        self.data = Twist()
        #Initialization of the node
        rospy.init_node("send_robot_command")

        #Creation of the publisher
        self.publisher = rospy.Publisher("mobile_base/commands/velocity", Twist, queue_size=10)

        #Definition of the time rate in hz
        self.rate = rospy.Rate(100)

    def loginfo(self,arg):
        rospy.loginfo(arg)
    
    def send_twist_to_robot(self,Vx,Vy,Vz,Wx,Wy,Wz):
        

        #Linear Velocity
        self.data.linear.x = Vx
        self.data.linear.y = Vy
        self.data.linear.z = Vz

        #Angulare Velocity
        self.data.angular.x = Wx
        self.data.angular.y = Wy
        self.data.angular.z = Wz

        self.publisher.publish(self.data)

            
    def on_pessed(self,event):
        if keyboard.KeyboardEvent.event_type == "up":
            self.send_twist_to_robot(0.5,0,0,0,0,0)
        
        if keyboard.KeyboardEvent.event_type == "down":
            self.send_twist_to_robot(-0.5,0,0,0,0,0)
        
        if keyboard.KeyboardEvent.event_type == "left":
            self.send_twist_to_robot(0,0,0,0,0,0.5)

        if keyboard.KeyboardEvent.event_type == "right":
            self.send_twist_to_robot(0,0,0,0,0,-0.5)

    
    def keyboardControl(self): 
        keyboard.hook(self.on_pessed)



def main():
    try:
        #Creation of a node
        node = send_robot_command()

        rospy.loginfo('Hello, ROS1!!!')

        while not rospy.is_shutdown():
            node.send_twist_to_robot(0,0,0,0,0,0)
            node.rate.sleep()
            #keyboard.hook(node.on_pessed)

    except rospy.ROSInternalException:
        pass


if __name__=="__main__":
    main()