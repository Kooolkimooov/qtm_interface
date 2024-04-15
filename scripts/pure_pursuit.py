#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import math
from scipy.interpolate import CubicSpline
import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist

#Topic d'Acquisition
topic_name = "qtm/turtle/6dof_pose"


# def interpolation(waitpoints):

#     # Time intervall
#     t = np.linspace(-1,1,1000)

#     # indexs of points in time t intervall
#     step = round(len(t)/len(waitpoints[:,0]))
#     indexes = list(range(0,len(t),step))

#     # Interpolation object (Boundary conditions) of X(t) and Y(t)
#     x_spline = CubicSpline(t[indexes],np.array(waitpoints[:,0]), bc_type="natural")
#     y_spline = CubicSpline(t[indexes],np.array(waitpoints[:,1]), bc_type="natural")

#     # x(t) and y(t) functions
#     x = x_spline(t)
#     y = y_spline(t)

#     return x, y

def interpolation(waypoint_list, interpolation_number):
        waypoint_list = np.array(waypoint_list)
        # paramètre intermédiaire :
        t = np.linspace(0, 1, len(waypoint_list))
        tt = np.linspace(0, 1, interpolation_number)
        y_t = CubicSpline(t, waypoint_list[:, 1], bc_type="natural")
        y_tt = y_t(tt)
        x_t = CubicSpline(t, waypoint_list[:, 0], bc_type="natural")
        x_tt = x_t(tt)

        return x_tt, y_tt


########################################################################################################
# Pure pursuit algorithm
########################################################################################################

# Parameters
k = 0.0015  # look forward gain
Lfc = 0.2  # [m] look-ahead distance
Kp = 0.01  # speed proportional gain
dt = 0.01  # [s] time tick


class State:

    def __init__(self, v):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.v = v

        self.emergency = False

        # ROS Features
        rospy.init_node('path_follower')
        rospy.Subscriber(topic_name, Pose, self.acquistion)
        self.cmd_vel_pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)
        self.rate = rospy.Rate(100)  # 100 Hz

    def acquistion(self,pose):
        if (math.isnan((pose.position.x) or math.isnan(pose.position.y) or math.isnan(pose.orientation.z))):
           self.emergency = True
           
        else:
           self.x = pose.position.x/1000
           self.y = pose.position.y/1000
           self.yaw = pose.orientation.z * np.pi/180

    def update(self, a, delta):
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.v * math.tan(delta) * dt
        self.v += a * dt
        
        twist = Twist()
        twist.linear.x = self.v
        twist.angular.z = delta

        #publish self.v and delta
        self.cmd_vel_pub.publish(twist)

    def calc_distance(self, point_x, point_y):
        dx = self.x - point_x
        dy = self.y - point_y
        return math.hypot(dx, dy)

def proportional_control(target, current):
    a = Kp * (target - current)
    return a


class TargetCourse:

    def __init__(self, cx, cy):
        self.cx = cx
        self.cy = cy
        self.old_nearest_point_index = 0

    def search_target_index(self, state):

        # To speed up nearest point search, doing it at only first time.
        if self.old_nearest_point_index is None:
            # search nearest point index
            dx = [state.x - icx for icx in self.cx]
            dy = [state.y - icy for icy in self.cy]
            d = np.hypot(dx, dy)
            ind = np.argmin(d)
            self.old_nearest_point_index = ind
        else:
            ind = self.old_nearest_point_index
            distance_this_index = state.calc_distance(self.cx[ind],self.cy[ind])
            
            while True:
                distance_next_index = state.calc_distance(self.cx[ind + 1],self.cy[ind + 1])
                if distance_this_index < distance_next_index:
                    break
                ind = ind + 1 if (ind + 1) < len(self.cx) else ind
                distance_this_index = distance_next_index
            self.old_nearest_point_index = ind

        Lf = k * state.v + Lfc  # update look ahead distance

        # search look ahead target point index
        while Lf > state.calc_distance(self.cx[ind], self.cy[ind]):
            if (ind + 1) >= len(self.cx):
                break  # not exceed goal
            ind += 1

        return ind, Lf


def pure_pursuit_steer_control(state, trajectory, t_ind):
    ind, Lf = trajectory.search_target_index(state)

    if t_ind >= ind:
        ind = t_ind

    if ind < len(trajectory.cx):
        tx = trajectory.cx[ind]
        ty = trajectory.cy[ind]
    else:  # toward goal
        tx = trajectory.cx[-1]
        ty = trajectory.cy[-1]
        ind = len(trajectory.cx) - 1

    alpha = math.atan2(ty - state.y, tx - state.x) - state.yaw

    delta = math.atan2(2.0 * math.sin(alpha) / Lf, 1.0)

    return delta, ind



########################################################################################################



def main():
    # Trajectory
    # waitpoints = np.array([[-1, 0], [0.9, 0.9], [0.8, -0.8], [0.9, 0.6], [0, 0.7], [0, 0]])

    waitpoints = np.array([[0, 0],[-0.91, 1.2], [0.36, 1.12], [-1.13, 0.38], [-0.45, -0.80], [0.7, -0.9], [0, 0]])
    cx, cy = interpolation(waitpoints, 10000)

    target_course = TargetCourse(cx, cy)
    
    # initial state
    state = State(v=0.0)
    target_speed = 1

    target_ind, _ = target_course.search_target_index(state)

    while not rospy.is_shutdown():
        if(not state.emergency):
            # Calc control input
            ai = proportional_control(target_speed, state.v)
            di, target_ind = pure_pursuit_steer_control(state, target_course, target_ind)
            state.update(ai, di)  # Control vehicle

            state.rate.sleep()






if __name__=="__main__":
    main()