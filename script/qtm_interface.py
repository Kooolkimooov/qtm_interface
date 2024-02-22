#!/usr/bin/python3

# roslaunch qtm_interface qtm_interface.launch

"""
    Streaming 6Dof from QTM
"""

import rospy
from geometry_msgs.msg import Twist

import asyncio
import xml.etree.ElementTree as ET
import signal
from scipy.spatial.transform import Rotation as R
import numpy as np

import qtm

QTM_IP = "192.168.1.4"          # IP du pc qtm
BODY_NAME = "BriceRigidBody"    # nom du rigid body
connection = None

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

async def main():
    global connection
    # Connect to qtm
    connection = await qtm.connect(QTM_IP)

    # Connection failed?
    if connection is None:
        rospy.ERROR("Failed to connect")
        return

    # Take control of qtm, context manager will automatically release control after scope end
    async with qtm.TakeControl(connection, "password"):

        realtime = False

        if realtime:
            # Start new realtime
            await connection.new()
        else:
            # Load qtm file
            # await connection.load(QTM_FILE)

            # start rtfromfile
            await connection.start(rtfromfile=True)

    # Get 6dof settings from qtm
    xml_string = await connection.get_parameters(parameters=["6d"])
    body_index = create_body_index(xml_string)

    def on_packet(packet):
        _, bodies = packet.get_6d()
        wanted_index = body_index[BODY_NAME] 
        pos, rot = bodies[wanted_index]           
        pose = Twist()
        pose.linear.x = pos.x      
        pose.linear.y = pos.y
        pose.linear.z = pos.z
        r = R.from_matrix(np.array(rot).reshape((3, 3))).as_euler('xyz')
        # print(pose)
        pose.angular.x = r[0]
        pose.angular.y = r[1]
        pose.angular.z = r[2]
        pub_pose.publish(pose)

    # Start streaming frames
    await connection.stream_frames(components=["6d"], on_packet=on_packet)
    
    
def stop(*args, **kwargs):
    # Stop streaming
    print("stopping streaming")
    asyncio.get_event_loop().stop()

if __name__ == "__main__":
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    qtm_node = rospy.init_node("qtm")
    pub_pose = rospy.Publisher(f"qtm/{BODY_NAME}/6dof_pose", Twist , queue_size=10)
    # Run our asynchronous function until complete
    asyncio.ensure_future(main())
    asyncio.get_event_loop().run_forever()