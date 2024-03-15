#!/usr/bin/python3

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoystickControl:
    def __init__(self):
        # Creating Twist message
        self.twist = Twist()
        self.Vx = 0.5
        self.Wz = 0.5

        rospy.init_node('joystick_control')
        rospy.Subscriber("joy", Joy, self.joy_callback)
        self.cmd_vel_pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(100)  # 100 Hz

    def joy_callback(self, data):
        # Assuming axes[1] is forward/backward control and axes[0] is left/right control
        linear_vel = data.axes[1]*self.Vx  # Forward/backward
        angular_vel = data.axes[0]*self.Wz  # Left/right

        if (data.buttons[5]==1):
            linear_vel = data.axes[1]*1.5   # Forward/backward
            angular_vel = data.axes[0]*1    # Left/right

        self.twist.linear.x = linear_vel
        self.twist.angular.z = angular_vel

        # Publishing Twist message
        #self.cmd_vel_pub.publish(self.twist)

    def run(self):
        rospy.spin()

    def send_twist_to_robot(self):
        self.cmd_vel_pub.publish(self.twist)

if __name__ == '__main__':
    try:
        joystick_control = JoystickControl()

        while not rospy.is_shutdown():
            joystick_control.send_twist_to_robot()
            joystick_control.rate.sleep()
    except rospy.ROSInterruptException:
        pass
