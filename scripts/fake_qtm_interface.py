#!/usr/bin/python3

from argparse import ArgumentParser, Namespace, RawDescriptionHelpFormatter
from copy import deepcopy

from numpy import array, eye, zeros

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from pympc.models.dynamics.bluerov import Bluerov
from pympc.models.dynamics.dynamics import Dynamics
from pympc.models.dynamics.turtlebot import Turtlebot
from pympc.models.model import Model
from pympc.models.ros_interface.bluerov import BluerovROSInterface
from pympc.models.ros_interface.interface import Interface
from pympc.models.ros_interface.turtlebot import TurtlebotROSInterface

ROBOT_TYPE = [ 'bluerov', 'turtlebot' ]


def parse_arguments() -> Namespace:
  parser = ArgumentParser( formatter_class = RawDescriptionHelpFormatter )

  parser.description = (f"Simulate an arbitrary amount of robots of type {ROBOT_TYPE} while taking in commands from "
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
             "/qtm/<ROBOT-NAME>; must be accompanied by -t and -c values"
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
      "--robot-initial-position",
      "-i",
      required = False,
      type = float,
      nargs = 6,
      action = "append",
      default = [ ],
      help = "default origin; the initial position of the robot to be simulated,"
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


class Simulation( Model ):
  def __init__( self, robot_type, robot_name, robot_command_topic, water_surface_depth, initial_position: list ):

    self.robot_type = robot_type
    self.robot_name = robot_name
    self.robot_command_topic = robot_command_topic

    dynamics = Dynamics()
    self.interface = Interface()

    if robot_type == 'bluerov':
      dynamics = Bluerov( water_surface_depth = water_surface_depth, reference_frame = 'ENU' )
      self.interface = BluerovROSInterface()
    elif robot_type == 'turtlebot':
      dynamics = Turtlebot()
      self.interface = TurtlebotROSInterface()
    else:
      raise ValueError

    initial_state = zeros( (dynamics.state_size,) )
    initial_state[ :dynamics.state_size // 2 ] = self.interface.pose_from_ros_pose(
        self.interface.ros_pose_from_state( array( initial_position + [ 0.0 ] * 6 ) )
        )

    self.model = Model( dynamics, 0.01, deepcopy( initial_state ), zeros( dynamics.actuation_size ) )

    self.pose = PoseWithCovarianceStamped()
    self.pose.header.frame_id = "map"
    self.publisher = rospy.Publisher( f'{self.robot_name}', PoseWithCovarianceStamped, queue_size = 10 )
    self.listener = rospy.Subscriber( f'/{self.robot_command_topic}', self.interface.command_type, self.callback )

    rospy.loginfo(
        f"simulating {robot_type} on topic {self.publisher.resolved_name} with input from {self.listener.resolved_name}"
        )

  def callback( self, payload ):
    self.model.actuation = self.interface.actuation_from_ros_actuation( payload )


def publish_fake_packets( args: Namespace ):

  if len( args.robot_initial_position ) == 0:
    args.robot_initial_position = [ [ 0., 0., 0., 0., 0., 0. ] for _ in range( len( args.robot_type ) ) ]

  assert (len( args.robot_type ) == len( args.robot_name ) and len( args.robot_type ) == len(
      args.robot_command_topic
      ) and len( args.robot_type ) == len( args.robot_initial_position )), (
      f"inconsistent number of arguments: each simulated robot must be defined with a -t, -n and -c argument. "
      f"and if an initial position is given, it must be given for all robots\n"
      f"--robot-type:             {args.robot_type}\n"
      f"--robot-name:             {args.robot_name}\n"
      f"--robot-command-topic:    {args.robot_command_topic} \n"
      f"--robot-initial-position: {args.robot_initial_position} \n")

  if 'bluerov' in args.robot_type:
    rospy.loginfo( f"simulating water surface depth at {args.water_surface_depth}" )

  fake_robots = [ ]

  for robot_type, robot_name, robot_command_topic, robot_initial_position in zip(
      args.robot_type, args.robot_name, args.robot_command_topic, args.robot_initial_position
      ):
    fake_robots += [ Simulation(
        robot_type, robot_name, robot_command_topic, args.water_surface_depth, robot_initial_position
        ) ]

  rate = rospy.Rate( 100. )

  while not rospy.is_shutdown():
    for robot in fake_robots:
      robot.model.step()
      robot.pose.pose.pose = robot.interface.ros_pose_from_state( robot.model.state )
      robot.pose.header.seq += 1
      robot.pose.pose.covariance = (0.01 * eye( 6, dtype = float )).flatten().tolist()
      robot.pose.header.stamp = rospy.Time.now()
      robot.publisher.publish( robot.pose )

    rate.sleep()


if __name__ == "__main__":
  parsed_arguments = parse_arguments()
  rospy.init_node( "qtm" )
  publish_fake_packets( parsed_arguments )
