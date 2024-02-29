#!/usr/bin/python3

from qtm_interface import robot_position

#Linear Velocity
Px = robot_position.position.x
Py = robot_position.position.y
Pz = robot_position.position.z

#Angulare Velocity
Rx = robot_position.orientation.x
Ry = robot_position.orientation.y
Rz = robot_position.orientation.z