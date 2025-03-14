#!/usr/bin/python3

import asyncio
import xml.etree.ElementTree as ET
from argparse import ArgumentParser, Namespace
from signal import SIGINT, signal

import qtm_rt
from numpy import array, isnan, eye
from scipy.spatial.transform import Rotation

import rospy
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from visualization_msgs.msg import Marker


def sigint_handler( *args ):
  rospy.loginfo( "SIGINT received" )
  rospy.signal_shutdown( "SIGINT received" )


signal( SIGINT, sigint_handler )


def parse_arguments() -> Namespace:
  parser = ArgumentParser()

  parser.add_argument(
      "--robot-name",
      "-n",
      required = True,
      type = str,
      action = "append",
      help = "required; the unique name of the robot to be tracked, will be used as a topic name : "
             "/qtm/<ROBOT-NAME>, may be repeated to track multiple robots"
      )
  parser.add_argument(
      "--qtm-ip",
      "-i",
      required = True,
      type = str,
      action = "store",
      help = "required; the ip address of the computer running QTM"
      )
  parser.add_argument(
      "--qtm-password",
      "-k",
      required = False,
      type = str,
      action = "store",
      default = "password",
      help = "default=password; the password to use to take control of QTM"
      )
  parser.add_argument(
      "--qtm-rt-version",
      "-v",
      required = False,
      type = str,
      action = "store",
      default = "1.21",
      help = "default=1.21; the version of the QTM RT protocol to use"
      )
  parser.add_argument(
      "--qtm-port",
      "-p",
      required = False,
      type = int,
      action = "store",
      default = 22223,
      help = "default=22223; the port number to use for the QTM RT protocol"
      )
  parser.add_argument(
      "--from-file",
      "-f",
      required = False,
      type = bool,
      action = "store",
      default = False,
      help = "default=False; wether to stream from file; WARNING: file must be already openend in QTM"
      )
  parser.add_argument(
      "--save-on-qtm",
      "-s",
      required = False,
      type = bool,
      action = "store",
      default = False,
      help = "default=False; wether to save the capture on the QTM computer on shutdown"
      )
  parser.add_argument(
      "--track-unlabelled",
      "-u",
      required = False,
      type = bool,
      action = "store",
      default = False,
      help = "default=False; wether to track unlabelled markers, will be used as a topic name : "
             "/qtm/untracked_markers"
      )
  return parser.parse_known_args()[ 0 ]


def create_body_index( xml_string ):
  """ Extract a name to index dictionary from 6dof settings xml """
  xml = ET.fromstring( xml_string )

  body_to_index = { }
  for index, body in enumerate( xml.findall( "*/Body/Name" ) ):
    body_to_index[ body.text.strip() ] = index

  return body_to_index


async def shutdown( args: Namespace, connection: qtm_rt.QRTConnection ):
  rospy.loginfo( "stopping QTMRT ..." )
  async with qtm_rt.TakeControl( connection, args.qtm_password ):

    await connection.stop()
    
    if args.save_on_qtm:
      try: 
        # TODO: implement option to save as 
        rospy.loginfo( "saving file on QTM ..." )
        await connection.save()

      except: pass

  rospy.loginfo("done.")


async def main( args: Namespace ):

  rospy.loginfo( f"trying to connect to {args.qtm_ip}:{args.qtm_port} QRTv{args.qtm_rt_version} ..." )
  # Connect to qtm
  connection = await qtm_rt.connect( args.qtm_ip, args.qtm_port, args.qtm_rt_version )
  # Connection failed?
  if connection is None:
    rospy.logerr( "failed to connect." )
    exit()
  else:
    rospy.loginfo( "connection successful." )

  # Take control of qtm, context manager will automatically release control after scope end
  async with qtm_rt.TakeControl( connection, args.qtm_password ):


    if not args.from_file:
      await connection.new()
      await connection.await_event(qtm_rt.QRTEvent.EventConnected)

    
    await connection.start( rtfromfile = args.from_file )

  robots = { }
  components = [ "6dres" ] + ([ "3dnolabels" ] if args.track_unlabelled else [ ])
  rospy.loginfo( f'tracking components: {components}' )

  params = await connection.get_parameters( parameters = [ "6d" ] )
  body_index = create_body_index( params )
  wanted_body_index = { name: body_index[ name ] for name in args.robot_name if name in body_index.keys() }
  rospy.loginfo( f'tracking bodies: {list( wanted_body_index.keys() )}' )

  if len( wanted_body_index ) != len( args.robot_name ):
    rospy.logwarn(
        f"not all wanted robots present in qtm frame:\nwanted: {args.robot_name}; tracked: {wanted_body_index}"
        )

  if args.track_unlabelled:
    unlabelled_publisher = rospy.Publisher( "unlabelled_markers", Marker, queue_size = 10 )
    rospy.loginfo( 'publishing unlabelled markers on topic /qtm/unlabelled_markers' )
    unlabelled_msg = Marker()
    unlabelled_msg.header.frame_id = "map"
    unlabelled_msg.type = Marker.SPHERE_LIST
    unlabelled_msg.color.r = 1.
    unlabelled_msg.color.g = 1.
    unlabelled_msg.color.b = 1.
    unlabelled_msg.color.a = 1.
    unlabelled_msg.scale.x = .1
    unlabelled_msg.scale.y = .1
    unlabelled_msg.scale.z = .1

  for name in args.robot_name:
    robots[ name ] = rospy.Publisher( f'{name}', PoseWithCovarianceStamped, queue_size = 10 )
    rospy.loginfo( f'publishing {name} on topic /qtm/{name}' )

  pose = PoseWithCovarianceStamped()
  pose.header.frame_id = "map"

  rate = rospy.Rate( 100. )
  while not rospy.is_shutdown():
    packet = await connection.get_current_frame( components = components )

    if args.track_unlabelled:
      _, markers = packet.get_3d_markers_no_label()
      unlabelled_msg.header.seq = packet.framenumber
      unlabelled_msg.points = [ ]
      for marker in markers:
        unlabelled_msg.points += [ Point( marker.x / 1000., marker.y / 1000, marker.z / 1000. ) ]
      unlabelled_publisher.publish( unlabelled_msg )

    _, bodies = packet.get_6d_residual()
    for name, publisher in robots.items():
      position, rotation_matrix, residual = bodies[ wanted_body_index[ name ] ]

      if any(isnan(position)): continue

      quaternion = Rotation.from_matrix( array( rotation_matrix ).reshape( (3, 3) ).T ).as_quat()

      pose.pose.covariance = (residual * eye(6, dtype=float) / 1000.).flatten().tolist()
      
      pose.pose.pose.position.x = position.x / 1000.
      pose.pose.pose.position.y = position.y / 1000.
      pose.pose.pose.position.z = position.z / 1000.
      pose.pose.pose.orientation.x = quaternion[ 0 ]
      pose.pose.pose.orientation.y = quaternion[ 1 ]
      pose.pose.pose.orientation.z = quaternion[ 2 ]
      pose.pose.pose.orientation.w = quaternion[ 3 ]

      pose.header.seq = packet.framenumber
      pose.header.stamp = rospy.Time.now()
      publisher.publish( pose )
      rate.sleep()

  return connection


if __name__ == "__main__":
  args = parse_arguments()
  qtm_node = rospy.init_node( "qtm" )
  connection = asyncio.get_event_loop().run_until_complete( asyncio.ensure_future( main( args ) ) )
  asyncio.get_event_loop().run_until_complete( asyncio.ensure_future( shutdown( args, connection ) ) )
