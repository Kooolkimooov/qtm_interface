#!/usr/bin/python3

# roslaunch qtm_interface qtm_interface.launch

"""
    Streaming 6Dof from QTM
"""

import rospy
from geometry_msgs.msg import Pose

import asyncio
import xml.etree.ElementTree as ET
import signal
from scipy.spatial.transform import Rotation as R
import numpy as np

import qtm

QTM_IP = "192.168.1.3"                  # IP du pc qtm
PORT = 22223                            # port that qtm listens to
VERSION = "1.21"                        # version of the rt protocol
COMPONENTS = ["3dnolabels", "6deuler"]  # type of data to stream
BODY_NAME = "turtle"                    # nom du rigid body
PASSWORD = "password"                   # qtm password for remote control
REAL_TIME = True                        # if the script should start playback (false) or recording (true)

def create_body_index(xml_string):
    """ Extract a name to index dictionary from 6dof settings xml """
    xml = ET.fromstring(xml_string)

    body_to_index = {}
    for index, body in enumerate(xml.findall("*/Body/Name")):
        body_to_index[body.text.strip()] = index

    return body_to_index

def body_enabled_count(xml_string):
    xml = ET.fromstring(xml_string)
    return sum(enabled.text == "true" for enabled in xml.findall("*/Body/Enabled"))


async def shutdown(connection: qtm.QRTConnection):
    rospy.loginfo("waiting for end of capture...")
    event = await connection.await_event(qtm.QRTEvent.EventCaptureStopped, timeout=3600)
    if event == qtm.QRTEvent.EventCaptureStopped:
        rospy.loginfo("shutting down...")
        async with qtm.TakeControl(connection, PASSWORD):
            await connection.stream_frames_stop()
            rospy.loginfo("stream stopped.")
        asyncio.get_event_loop().stop()
        rospy.loginfo("loop stopped.")

async def main():
    global REAL_TIME
    rospy.loginfo("trying to connect...")
    # Connect to qtm
    connection = await qtm.connect(QTM_IP, PORT, VERSION, timeout=1) 

    # Connection failed?
    if connection is None:
        rospy.logerr("failed to connect.")
        return
    else:
        rospy.loginfo("connection successful.")

    # Take control of qtm, context manager will automatically release control after scope end
    async with qtm.TakeControl(connection, PASSWORD):

        if REAL_TIME:
            rospy.logwarn(f"please start recording on qtm machine ({QTM_IP})...")
            await connection.new()
            await connection.await_event(qtm.QRTEvent.EventCaptureStarted)
            rospy.loginfo("recording started.")
        else:
            rospy.loginfo("starting playback...")
            await connection.start(rtfromfile=True)
            rospy.loginfo("playback started.")

    rospy.loginfo("getting scene parameters...")
    xml_string = await connection.get_parameters(parameters=["6d"])
    body_index = create_body_index(xml_string)
    wanted_index = body_index[BODY_NAME] 

    def on_packet(packet: qtm.QRTPacket):  
        _, bodies = packet.get_6d_euler()
        pos, rot = bodies[wanted_index] 

        robot_pose = Pose()

        # Linear position          
        robot_pose.position.x = pos.x      
        robot_pose.position.y = pos.y
        robot_pose.position.z = pos.z

        # Angular position
        robot_pose.orientation.x = rot.a1
        robot_pose.orientation.y = rot.a2
        robot_pose.orientation.z = rot.a3

        pose_publisher.publish(robot_pose)

    
    rospy.loginfo(f"starting streaming frames for 6dof {BODY_NAME}...")
    await connection.stream_frames(components=COMPONENTS, on_packet=on_packet)

    asyncio.ensure_future(shutdown(connection))

if __name__ == "__main__":
    qtm_node = rospy.init_node("qtm")
    pose_publisher = rospy.Publisher(f"qtm/{BODY_NAME}/6dof_pose", Pose, queue_size=2)
    
    asyncio.ensure_future(main())
    asyncio.get_event_loop().run_forever()