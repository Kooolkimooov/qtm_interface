#!/usr/bin/python3

import rospy  # ROS pour Python
from geometry_msgs.msg import PoseStamped  # Message pour publier les poses
import asyncio  # Gestion des tâches asynchrones
import xml.etree.ElementTree as ET  # Parsing XML
from numpy import array  # Manipulation de matrices et vecteurs
from scipy.spatial.transform import Rotation as R  # Matrices de rotation à quaternion
import qtm  # Bibliothèque pour QTM

# Paramètres de connexion
QTM_IP = "10.0.1.69"
PORT = 22223
VERSION = "1.21"
COMPONENTS = ["6d"]
PASSWORD = "password"
REAL_TIME = True
NUM_ROBOTS = 2
ROBOT_PREFIX = "turtle_bot_"  # Préfixe des noms des robots

# Fonction pour créer un index associant les noms des robots à leurs indices
def create_body_index(xml_string):
    """ Associe chaque nom de rigid body à son index """
    xml = ET.fromstring(xml_string)
    body_to_index = {}
    for index, body in enumerate(xml.findall("*/Body/Name")):
        body_to_index[body.text.strip()] = index
    return body_to_index

# Fonction principale de streaming avec un seul on_packet
async def main():
    global REAL_TIME

    # Connexion à QTM
    rospy.loginfo(f"Connecting to {QTM_IP}:{PORT}...")
    connection = await qtm.connect(QTM_IP, PORT, VERSION)

    if connection is None:
        rospy.logerr("Failed to connect to QTM.")
        quit()

    rospy.loginfo("Connected to QTM.")

    # Prendre le contrôle de QTM
    async with qtm.TakeControl(connection, PASSWORD):
        if REAL_TIME:
            rospy.loginfo("Waiting for QTM recording to start...")
            await connection.new()
            await connection.await_event(qtm.QRTEvent.EventCaptureStarted)
        else:
            rospy.loginfo("Starting playback...")
            await connection.start(rtfromfile=True)

    # Récupérer les paramètres des rigid bodies
    rospy.loginfo("Getting rigid body parameters...")
    xml_string = await connection.get_parameters(parameters=["6d"])
    body_index = create_body_index(xml_string)

    # Préparer les publishers pour chaque robot
    publishers = []
    for i in range(1, NUM_ROBOTS + 1):
        body_name = f"{ROBOT_PREFIX}{i}"
        if body_name not in body_index:
            rospy.logerr(f"Body name {body_name} not found in QTM!")
            continue
        topic_name = f"/{ROBOT_PREFIX}{i}/6dof_pose"
        publishers.append(rospy.Publisher(topic_name, PoseStamped, queue_size=10))

    # Vérifier qu'on a bien des publishers
    if not publishers:
        rospy.logerr("No valid robots found. Exiting...")
        quit()

    # Fonction unique pour gérer tous les robots
    def on_packet(packet: qtm.QRTPacket):
        _, bodies = packet.get_6d()
        for i, publisher in enumerate(publishers):
            # Vérifie que l'index existe pour éviter des erreurs
            if i >= len(bodies):
                rospy.logwarn(f"Index {i} not found in packet. Skipping...")
                continue

            pos, rot = bodies[i]
            pose = PoseStamped()

            # Position en mètres
            pose.pose.position.x = pos.x / 1000.0
            pose.pose.position.y = pos.y / 1000.0
            pose.pose.position.z = pos.z / 1000.0

            # Orientation en quaternions
            quat = R.from_matrix(array(rot.matrix).reshape((3, 3))).as_quat()
            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]

            # Métadonnées
            pose.header.seq = packet.framenumber
            pose.header.frame_id = "map"

            # Publie sur le topic correspondant
            publisher.publish(pose)

    # Démarrer le streaming des frames
    rospy.loginfo("Starting streaming...")
    await connection.stream_frames(components=COMPONENTS, on_packet=on_packet)

if __name__ == "__main__":
    rospy.init_node("qtm_multi_robot")
    asyncio.ensure_future(main())
    asyncio.get_event_loop().run_forever()


