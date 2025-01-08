#!/usr/bin/python3

"""
    Streaming 6Dof from QTM
"""

import asyncio
import xml.etree.ElementTree as ET

import qtm
import rospy
from geometry_msgs.msg import PoseStamped
from numpy import array
from scipy.spatial.transform import Rotation as R

QTM_IP = "10.0.1.69"  # IP du pc qtm
PORT = 22223  # port that qtm listens to
VERSION = "1.21"  # version of the rt protocol
COMPONENTS = [ "3dnolabels", "6d" ]  # type of data to stream
BODY_NAME = "turtle_bot_0"  # nom du rigid body
PASSWORD = "password"  # qtm password for remote control
REAL_TIME = True  # if the script should start playback (false) or recording (true)


def create_body_index( xml_string ):
  """ Extract a name to index dictionary from 6dof settings xml """
  xml = ET.fromstring( xml_string )

  body_to_index = { }
  for index, body in enumerate( xml.findall( "*/Body/Name" ) ):
    body_to_index[ body.text.strip() ] = index

  return body_to_index


def body_enabled_count( xml_string ):
  xml = ET.fromstring( xml_string )
  return sum( enabled.text == "true" for enabled in xml.findall( "*/Body/Enabled" ) )


async def shutdown( connection: qtm.QRTConnection ):
  rospy.loginfo( "waiting for end of capture..." )
  if REAL_TIME:
    event = await connection.await_event( qtm.QRTEvent.EventCaptureStopped, timeout = 3600 )
  else:
    event = await connection.await_event( qtm.QRTEvent.EventRTfromFileStopped, timeout = 3600 )

  if event:
    rospy.loginfo( "shutting down..." )
    async with qtm.TakeControl( connection, PASSWORD ):
      await connection.stream_frames_stop()
      rospy.loginfo( "stream stopped." )
    asyncio.get_event_loop().stop()
    rospy.loginfo( "loop stopped." )


async def main():
  global REAL_TIME
  rospy.loginfo( "trying to connect to " + QTM_IP + ":" + str( PORT ) + " v" + VERSION )
  # Connect to qtm
  connection = await qtm.connect( QTM_IP, PORT, VERSION )

  # Connection failed?
  if connection is None:
    rospy.logerr( "failed to connect." )
    quit()
  else:
    rospy.loginfo( "connection successful." )

  # Take control of qtm, context manager will automatically release control after scope end
  async with qtm.TakeControl( connection, PASSWORD ):

    if REAL_TIME:
      rospy.logwarn( f"please start recording on qtm machine ({QTM_IP})..." )
      await connection.new()
      # await connection.await_event(qtm.QRTEvent.EventCaptureStarted)
      rospy.loginfo( "recording started." )
    else:
      rospy.loginfo( "starting playback..." )
      await connection.start( rtfromfile = True )
      rospy.loginfo( "playback started." )

  rospy.loginfo( "getting scene parameters..." )
  xml_string = await connection.get_parameters( parameters = [ "6d" ] )
  body_index = create_body_index( xml_string )
  wanted_index = body_index[ BODY_NAME ]

  def on_packet( packet: qtm.QRTPacket ):

    _, bodies = packet.get_6d()
    pos, rot = bodies[ wanted_index ]

    robot_pose = PoseStamped()

    # Linear position
    robot_pose.pose.position.x = pos.x / 1000.
    robot_pose.pose.position.y = pos.y / 1000.
    robot_pose.pose.position.z = pos.z / 1000.

    quat = R.from_matrix( array( rot.matrix ).reshape( (3, 3) ) ).as_quat()

    # Angular position
    robot_pose.pose.orientation.x = quat[ 0 ]
    robot_pose.pose.orientation.y = quat[ 1 ]
    robot_pose.pose.orientation.z = quat[ 2 ]
    robot_pose.pose.orientation.w = quat[ 3 ]

    robot_pose.header.seq = packet.framenumber
    robot_pose.header.frame_id = "map"

    pose_publisher.publish( robot_pose )

  rospy.loginfo( f"starting streaming frames for 6dof {BODY_NAME}..." )
  await connection.stream_frames( components = COMPONENTS, on_packet = on_packet )

  asyncio.ensure_future( shutdown( connection ) )


if __name__ == "__main__":

  qtm_node = rospy.init_node( "qtm" )
  pose_publisher = rospy.Publisher( f"{BODY_NAME}/6dof_pose", PoseStamped, queue_size = 2 )

  asyncio.ensure_future( main() )
  asyncio.get_event_loop().run_forever()
