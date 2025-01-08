#!/usr/bin/python3

from argparse import ArgumentParser, Namespace

import numpy as np

import rospy
from fake_qtm_interface_simulator.bluerov import Bluerov
from fake_qtm_interface_simulator.runge_kutta import runge_kutta_4
from fake_qtm_interface_simulator.simulator import Simulator
from fake_qtm_interface_simulator.turtlebot import Turtlebot
from geometry_msgs.msg import PoseStamped

ROBOT_TYPE = [ 'bluerov', 'turtlebot' ]


def parse_arguments() -> Namespace:
  parser = ArgumentParser()

  parser.add_argument(
      "--water-surface-depth",
      "-w",
      default = 0.,
      type = float,
      help = "default=0.0; the z coordinate of the water level to be simulated"
      )
  parser.add_argument(
      "--robot-type",
      "-t",
      required = True,
      choices = ROBOT_TYPE,
      type = str,
      action = "append",
      help = f"required - the type of robot to be simulated, must be one of {ROBOT_TYPE}; must be accompanied by -n "
             f"and -c values"
      )
  parser.add_argument(
      "--robot-name",
      "-n",
      required = True,
      type = str,
      action = "append",
      help = "required - the unique name of the robot to be simulated, will be used as a topic name : "
             "/qtm/<ROBOT-NAME>; must be accompanied by -t and -c values"
      )
  parser.add_argument(
      "--robot-command-topic",
      "-c",
      required = True,
      type = str,
      action = "append",
      help = "required - the command topic name for the robot, will be used to subscribe to command topic : "
             "/<ROBOT-COMMAND-TOPIC>; must be accompanied by -t and -n values"
      )
  return parser.parse_known_args()[ 0 ]


def fake_packets( args: Namespace ):

  assert (len( args.robot_type ) == len( args.robot_name ) and len( args.robot_type ) == len(
      args.robot_command_topic
      )), (f"inconsistent number of arguments: each simulated robot must be defined with a -rt, -rn and -rc argument:\n"
           f"--robot-type:          {args.robot_type}\n"
           f"--robot-name:          {args.robot_name}\n"
           f"--robot-command-topic: {args.robot_command_topic} \n")

  fake_robots = { }
  rate = rospy.Rate( 100. )

  rospy.loginfo( f"Simulating water surface depth at {args.water_surface_depth}" )
  for type, name, command in zip( args.robot_type, args.robot_name, args.robot_command_topic ):
    rospy.loginfo( f"simulating {type} on topic /qtm/{name} with input from /{command}" )

    simulator = Simulator()

    if type == 'bluerov':
      simulator = Bluerov( water_surface_depth = args.water_surface_depth )
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
        'publisher': rospy.Publisher( f'pose/{name}', PoseStamped, queue_size = 2 ),
        'listener' : rospy.Subscriber( f'/{command}', simulator.get_command_type(), callback ),
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
  rospy.init_node( "qtm" )
  parsed_arguments = parse_arguments()
  fake_packets( parsed_arguments )
