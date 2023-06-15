#!/usr/bin/python3

"""
    Streaming 6Dof from QTM
"""

import rospy
from geometry_msgs.msg import Point, Twist

import asyncio
import xml.etree.ElementTree as ET
import signal
from scipy.spatial.transform import Rotation as R
import numpy as np

import qtm

QTM_FILE = "L_dynamique6y200dis1_0026.qtm"
wanted_body = "BriceRigidBody"
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
    connection = await qtm.connect("192.168.1.2")

    # Connection failed?
    if connection is None:
        print("Failed to connect")
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

    print("{} of {} 6DoF bodies enabled".format(body_enabled_count(xml_string), len(body_index)))

    def on_packet(packet):
        _, bodies = packet.get_6d()
        # print(
        #     "Framenumber: {} - Body count: {}".format(
        #         packet.framenumber, info.body_count
        #     )
        # )

        # if wanted_body is not None and wanted_body in body_index:
            # Extract one specific body
            # wanted_index = body_index[wanted_body]
            # position, rotation = bodies[wanted_index]
            # print("{} - Pos: {} - Rot: {}".format(wanted_body, position, rotation))
        # else:
            # Print all bodies
            # for position, rotation in bodies:
                # print("Pos: {} - Rot: {}".format(position, rotation))
                
        pos, rot = bodies[0]           
        pose = Twist()
        pose.linear.x = pos.x      
        pose.linear.y = pos.y
        pose.linear.z = pos.z
        r = R.from_matrix(np.array(rot).reshape((3, 3)))
        # print(pose)
        r = r.as_euler('xyz')
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
    rospy.init_node("qtm")
    pub_pose = rospy.Publisher("qtm/6dof_pose", Twist , queue_size=10)
    # Run our asynchronous function until complete
    asyncio.ensure_future(main())
    asyncio.get_event_loop().run_forever()