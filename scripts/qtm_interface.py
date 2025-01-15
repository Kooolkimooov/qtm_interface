#!/usr/bin/python3

# Importation des bibliotheques necessaires
import rospy  # ROS pour Python
from geometry_msgs.msg import PoseStamped  # Message de type PoseStamped pour publier les poses
import asyncio  # Gestion asynchrone des taches
import xml.etree.ElementTree as ET  # Parsing des fichiers XML
from numpy import array  # Manipulation de matrices et vecteurs
from scipy.spatial.transform import Rotation as R  # Conversion de matrices de rotation en quaternions
import qtm  # Bibliotheque pour interagir avec Qualisys Track Manager (QTM)

# Parametres de connexion à QTM
QTM_IP = "10.0.1.69"  # Adresse IP de QTM
PORT = 22223  # Port utilise par QTM
VERSION = "1.21"  # Version du protocole de communication RT
COMPONENTS = ["6d"]  # Type de donnees a streamer (6 degres de liberte : position + orientation)
PASSWORD = "password"  # Mot de passe pour controler QTM a distance
REAL_TIME = True  # Indique si l execution est en temps reel
NUM_ROBOTS = 1  # Nombre de robots a gerer (modifiable pour adapter le code)
ROBOT_PREFIX = "turtle_bot_" #Prefixe pour nommer les robots dynamiquement

# Fonction pour extraire l index des rigid bodies a partir d une chaine XML
def create_body_index(xml_string):
    """ Génère un dictionnaire associant le nom d'un rigid body à son index """
    xml = ET.fromstring(xml_string)

    body_to_index = {}
    for index, body in enumerate(xml.findall("*/Body/Name")):  # Parcourt les noms des rigid bodies
        body_to_index[body.text.strip()] = index

    return body_to_index

# Fonction pour arreter le streaming et fermer la connexion à QTM
async def shutdown(connection: qtm.QRTConnection):
    """ Arrete le streaming des donnees et ferme proprement la connexion """
    rospy.loginfo("Waiting for end of capture...")
    if REAL_TIME:
        # Attend que l evenement CaptureStopped se produise
        await connection.await_event(qtm.QRTEvent.EventCaptureStopped, timeout=3600)
    else:
        # Attend que la lecture en temps reel a partir d un fichier se termine
        await connection.await_event(qtm.QRTEvent.EventRTfromFileStopped, timeout=3600)

    # Arret du streaming
    rospy.loginfo("Shutting down...")
    async with qtm.TakeControl(connection, PASSWORD):  # Prend le controle de QTM pour arreter le streaming
        await connection.stream_frames_stop()
        rospy.loginfo("Stream stopped.")
    asyncio.get_event_loop().stop()
    rospy.loginfo("Loop stopped.")

# Fonction pour streamer les donnees de position et d orientation d un robot specifique
async def stream_robot_pose(connection, body_name, body_index, pose_publisher):
    """ Streame les donnees de position et d orientation (6DoF) pour un robot donne """
    def on_packet(packet: qtm.QRTPacket):
        # Recupere la position et la rotation pour le robot avec l index donne
        _, bodies = packet.get_6d()
        pos, rot = bodies[body_index]

    quat = R.from_matrix( array( rot.matrix ).reshape( (3, 3) ) ).as_quat()

        # Convertit la position lineaire en metres
        robot_pose.pose.position.x = pos.x / 1000.0
        robot_pose.pose.position.y = pos.y / 1000.0
        robot_pose.pose.position.z = pos.z / 1000.0

        # Convertit la rotation en quaternion
        quat = R.from_matrix(array(rot.matrix).reshape((3, 3)).T).as_quat()
        robot_pose.pose.orientation.x = quat[0]
        robot_pose.pose.orientation.y = quat[1]
        robot_pose.pose.orientation.z = quat[2]
        robot_pose.pose.orientation.w = quat[3]

        # Ajoute des metadonnees au message
        robot_pose.header.seq = packet.framenumber
        robot_pose.header.frame_id = "map"

        # Publie les donnees sur le topic ROS
        pose_publisher.publish(robot_pose)

    # Configure QTM pour streamer les donnees en continu
    rospy.loginfo(f"Starting streaming frames for 6DoF {body_name}...")
    await connection.stream_frames(components=COMPONENTS, on_packet=on_packet)

# Fonction principale pour gerer la connexion à QTM et le streaming pour tous les robots
async def main():
    global REAL_TIME

    # Connexion a QTM
    rospy.loginfo(f"Trying to connect to {QTM_IP}:{PORT} v{VERSION}...")
    connection = await qtm.connect(QTM_IP, PORT, VERSION)

    if connection is None:
        rospy.logerr("Failed to connect.")
        quit()
    else:
        rospy.loginfo("Connection successful.")

    # Prend le controle de QTM pour configurer le streaming
    async with qtm.TakeControl(connection, PASSWORD):
        if REAL_TIME:
            rospy.logwarn(f"Please start recording on QTM machine ({QTM_IP})...")
            await connection.new()
            await connection.await_event(qtm.QRTEvent.EventCaptureStarted) 
            rospy.loginfo("Recording started.")
        else:
            rospy.loginfo("Starting playback...")
            await connection.start(rtfromfile=True)
            rospy.loginfo("Playback started.")

    # Recupere les parametres de la scene QTM
    rospy.loginfo("Getting scene parameters...")
    xml_string = await connection.get_parameters(parameters=["6d"])
    body_index = create_body_index(xml_string)

    # Configure les publishers et les taches pour chaque robot
    publishers = {}
    tasks = []
    for i in range(0, NUM_ROBOTS ):  # Boucle pour gerer chaque robot
        body_name = f"{ROBOT_PREFIX}{i}"  # Genere dynamiquement le nom du rigid body
        if body_name not in body_index:
            rospy.logerr(f"Body name {body_name} not found in QTM!")
            continue
        index = body_index[body_name]
        topic_name = f"{ROBOT_PREFIX}{i}/6dof_pose"  # Topic ROS pour publier les poses
        publishers[body_name] = rospy.Publisher(topic_name, PoseStamped, queue_size=10)
        # Ajoute une tache asynchrone pour streamer les donnees de ce robot
        tasks.append(stream_robot_pose(connection, body_name, index, publishers[body_name]))

    # Execute toutes les taches de streaming en parallele
    await asyncio.gather(*tasks)

    # Ajoute une tache pour gerer l arret du streaming
    asyncio.ensure_future(shutdown(connection))

# Point d entree principal
if __name__ == "__main__":
    rospy.init_node("qtm_multi_robot")  # Initialise le noeud ROS
    asyncio.ensure_future(main())  # Demarre la tache principale
    asyncio.get_event_loop().run_forever()  # Boucle principale pour gerer les taches asynchrones
