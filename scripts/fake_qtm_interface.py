#!/usr/bin/python3

from argparse import ArgumentParser

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, Wrench

from fake_qtm_interface_simulator.bluerov import Bluerov
from fake_qtm_interface_simulator.runge_kutta import runge_kutta_4
from fake_qtm_interface_simulator.simulator import Simulator
from fake_qtm_interface_simulator.turtlebot import Turtlebot

ROBOT_TYPE = [ 'bluerov', 'turtlebot' ]


def fake_packets( args ):

  assert (len( args.robot_type ) == len( args.robot_name ) and len( args.robot_type ) == len(
      args.robot_command_topic
      )), (f"inconsistent number of arguments: each simulated robot must be defined with a -rt, -rn and -rc argument:\n"
           f"--robot-type:          {args.robot_type}\n"
           f"--robot-name:          {args.robot_name}\n"
           f"--robot-command-topic: {args.robot_command_topic} \n")

  print( args )
  fake_robots = { }
  rate = rospy.Rate( 100 )

  for type, name, command in zip( args.robot_type, args.robot_name, args.robot_command_topic ):
    simulator = Simulator()

    if type == 'bluerov':
      simulator = Bluerov()
    elif type == 'turtlebot':
      simulator = Turtlebot()

    state = simulator.initial_state()
    pose = PoseStamped()
    pose.header.frame_id = "map"

    def callback( payload ):
      fake_robots[ name ][ 'actuation' ] = simulator.actuation_from_ros( payload )

    fake_robots[ name ] = {
        'simulator': simulator,
        'state'    : state,
        'actuation': np.zeros( simulator.actuation_size ),
        'publisher': rospy.Publisher( name, PoseStamped, queue_size = 2 ),
        'listener' : rospy.Subscriber( '/' + command, Wrench, callback ),
        'pose'     : pose
        }

  while not rospy.is_shutdown():
    for name, robot in fake_robots.items():
      robot[ 'state' ] = runge_kutta_4( robot[ 'simulator' ], 0.01, robot[ 'state' ], robot[ 'actuation' ] )
      robot[ 'pose' ].pose = robot[ 'simulator' ].rospose_from_state( robot[ 'state' ] )
      robot[ 'pose' ].header.seq += 1
      robot[ 'publisher' ].publish( robot[ 'pose' ] )

    rate.sleep()


if __name__ == "__main__":
  qtm_node = rospy.init_node( "qtm" )

  parser = ArgumentParser()
  parser.add_argument(
      "--robot-type",
      "-t",
      required = True,
      choices = ROBOT_TYPE,
      type = str,
      action = "append",
      help = f"the type of robot to be simulated, must be one of {ROBOT_TYPE} ; must be accompanied by -n and -c "
             f"values"
      )
  parser.add_argument(
      "--robot-name",
      "-n",
      required = True,
      type = str,
      action = "append",
      help = "the unique name of the robot to be simulated, will be used as a topic name : /qtm/<robot-name>; must be "
             "accompanied by -t and -c values"
      )
  parser.add_argument(
      "--robot-command-topic",
      "-c",
      required = True,
      type = str,
      action = "append",
      help = "the command topic name for the robot, will be used to subscribe to command topic : "
             "/<robot-command-topic>; "
             "must be accompanied by -t and -n values"
      )

  fake_packets( parser.parse_known_args()[ 0 ] )
