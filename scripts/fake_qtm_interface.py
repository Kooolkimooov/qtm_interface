#!/usr/bin/python3

from argparse import ArgumentParser, Namespace, RawDescriptionHelpFormatter

from numpy import zeros

import rospy
from geometry_msgs.msg import PoseStamped
from pympc.models.dynamics.bluerov import Bluerov
from pympc.models.dynamics.turtlebot import Turtlebot
from pympc.models.model import runge_kutta_4
from pympc.models.ros_interface.base_interface import BaseInterface
from pympc.models.ros_interface.bluerov import BluerovROSInterface
from pympc.models.ros_interface.turtlebot import TurtlebotROSInterface
from rostopic import publish_message

ROBOT_TYPE = [ 'bluerov', 'turtlebot' ]


def parse_arguments() -> Namespace:
  parser = ArgumentParser( formatter_class = RawDescriptionHelpFormatter )

  parser.description = (f"Simulate an arbitrary amount of robots of type {ROBOT_TYPE} while taking in commands from  "
                        f"given topic and publishing the resulting pose to a given topic inside the /qtm namespace "
                        f"to simulate a robot, this program needs the following arguments:\n"
                        f"--robot-type\n--robot-name\n--robot-command-topic\n"
                        f"These arguments may be specified multiple times to simulate multiple robots but their "
                        f"number of appearance must be equal\nexample usage:\n"
                        f"$ rosrun qtm_interface fake_qtm_interface.py --robot-type bluerov --robot-name br "
                        f"--robot-command-topic /br/mavros/rc/override\n"
                        f"$ rosrun qtm_interface fake_qtm_interface.py -w 2 -t bluerov -n br1 -c "
                        f"/br1/mavros/rc/override -t bluerov -n br2 -c /br2/mavros/rc/override")

  parser.add_argument(
      "--robot-name",
      "-n",
      required = True,
      type = str,
      action = "append",
      help = "required; the unique name of the robot to be simulated, will be used as a topic name : "
             "/qtm/pose/<ROBOT-NAME>; must be accompanied by -t and -c values"
      )

  parser.add_argument(
      "--robot-type",
      "-t",
      required = True,
      choices = ROBOT_TYPE,
      type = str,
      action = "append",
      help = f"required; the type of robot to be simulated, must be one of {ROBOT_TYPE}; must be accompanied by -n "
             f"and -c values"
      )

  parser.add_argument(
      "--robot-command-topic",
      "-c",
      required = True,
      type = str,
      action = "append",
      help = "required; the command topic name for the robot, will be used to subscribe to command topic : "
             "/<ROBOT-COMMAND-TOPIC>; must be accompanied by -t and -n values"
      )

  parser.add_argument(
      "--water-surface-depth",
      "-w",
      required = False,
      type = float,
      action = "store",
      default = 0.,
      help = "default=0.0; the z coordinate of the water level to be simulated; only used with robot type bluerov"
      )

  return parser.parse_known_args()[ 0 ]


def publish_fake_packets( args: Namespace ):

  assert (len( args.robot_type ) == len( args.robot_name ) and len( args.robot_type ) == len(
      args.robot_command_topic
      )), (f"inconsistent number of arguments: each simulated robot must be defined with a -rt, -rn and -rc argument:\n"
           f"--robot-type:          {args.robot_type}\n"
           f"--robot-name:          {args.robot_name}\n"
           f"--robot-command-topic: {args.robot_command_topic} \n")

  if 'bluerov' in args.robot_type:
    rospy.loginfo( f"Simulating water surface depth at {args.water_surface_depth}" )

  fake_robots = { }

  for type, name, command in zip( args.robot_type, args.robot_name, args.robot_command_topic ):
    rospy.loginfo( f"simulating {type} on topic /qtm/pose/{name} with input from /{command}" )

    simulator = Bluerov
    interface = BaseInterface

    if type == 'bluerov':
      simulator = Bluerov( water_surface_depth = args.water_surface_depth, reference_frame = 'ENU' )
      interface = BluerovROSInterface
    elif type == 'turtlebot':
      simulator = Turtlebot()
      interface = TurtlebotROSInterface
    else:
      raise ValueError

    state = interface.initial_state
    pose = PoseStamped()
    pose.header.frame_id = "map"

    def callback( payload ):
      fake_robots[ name ][ 'actuation' ] = interface.actuation_from_ros_actuation( payload )

    fake_robots[ name ] = {
        'simulator': simulator,
        'interface': interface,
        'state'    : state,
        'actuation': zeros( simulator.actuation_size ),
        'publisher': rospy.Publisher( f'pose/{name}', PoseStamped, queue_size = 10 ),
        'listener' : rospy.Subscriber( f'/{command}', interface.command_type, callback ),
        'pose'     : pose
        }

  rate = rospy.Rate( 100. )

  while not rospy.is_shutdown():
    for robot in fake_robots.values():
      robot[ 'state' ] = runge_kutta_4( robot[ 'simulator' ], 0.01, robot[ 'state' ], robot[ 'actuation' ] )
      robot[ 'pose' ].pose = robot[ 'interface' ].ros_pose_from_state( robot[ 'state' ] )
      robot[ 'pose' ].header.seq += 1
      robot[ 'publisher' ].publish( robot[ 'pose' ] )

    rate.sleep()


if __name__ == "__main__":
  parsed_arguments = parse_arguments()
  rospy.init_node( "qtm" )
  publish_fake_packets( parsed_arguments )
