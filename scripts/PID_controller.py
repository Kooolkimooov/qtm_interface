#!/usr/bin/python3
#=================================== Import =============================================
import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
import numpy as np
from scipy.integrate import odeint
from threading import Thread
from math import isnan

#Topic d'Acquisition
topic_name = "qtm/turtle/6dof_pose"



#================================= Déclarations =========================================

############################################ Peut être modifié ######################################################
#, [0.8, 0.8], [0.01, 0.01], [-0.5, -0.2], [0.8, -0.8], [-0.4, -0.4]]

class Controller:

    def __init__(self) -> None:

        self.acquisition_flag=False

        self.waypoints = np.array([[1, 1], [1, 1], [-1, -0.7], [1, -0.7], [0, 0], [-2, 0]])


        self.theta_tb = 0
        self.tb_center = np.array([0.7, 0.7])

        self.waypoints_incr = 0
        self.waypoint = self.waypoints[self.waypoints_incr]

        self.X0 = np.array([self.tb_center[0], self.tb_center[1], self.theta_tb])

        self.Xout = self.X0.reshape(1, -1)
        self.Trajectory = self.Xout

        self.MaxAngularSpeed = np.deg2rad(70)
        self.MaxAngularAccel = np.deg2rad(360)
        self.MaxLinearSpeed = 1.5
        self.MaxLinearAccel = 1

        self.linearSpeed = 0
        self.angularSpeed = 0

        self.linearAccel = 0
        self.angularAccel = 0

        self.epsilon_angle = np.deg2rad(0.5)
        self.epsilon_dist = 0.03

        self.TB_state = "rotating"
        self.idle_delay = 0

        self.emergency = False

        self.delta_t = 1/100

        self.dist_remain = 0
        self.theta_remain = 0

        rospy.init_node('PID_controller')
        rospy.Subscriber(topic_name, Pose, self.acquistion)
        self.cmd_vel_pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)
        self.rate = rospy.Rate(100)  # 100 Hz

        rospy.loginfo("Initialisation of the node PID_controller")

    def acquistion(self,pose):
        twist = Twist()
        if (isnan((pose.position.x) or isnan(pose.position.y) or isnan(pose.orientation.z))):
            self.emergency = True
            self.tracking_lost()
            
        else:
            self.tb_center[0] = pose.position.x/1000
            self.tb_center[1] = pose.position.y/1000
            self.theta_tb = pose.orientation.z * np.pi/180
            self.acquisition_flag=True
            if self.emergency:
                for t in range(100000):
                    twist.linear.x = 0
                    self.cmd_vel_pub.publish(twist)
                for t in range(100000):
                    twist.linear.x = -0.3
                    self.cmd_vel_pub.publish(twist)
                self.tracking_back()
                self.emergency = False
    
    def tracking_back(self):
        self.waypoints[self.waypoints_incr, :] = np.array([.01, .01])
        self.TB_state = "rotating"

    def tracking_lost(self):
        twist =  Twist()
        twist.linear.x = -0.3
        self.cmd_vel_pub.publish(twist)


    #================================== Fonctions ============================================
    # Fonctions de calculs
    def dynamic_model(self,_, X, u):
        # Calcul de la dynamique du TurtleBot
        linear = np.array([np.cos(X[2]), np.sin(X[2]), 0])
        angular = np.array([0, 0, 1])
        dx = linear * u[0] + angular * u[1]
        return dx

    def ModuloByAngle(self,tb_center, waypoint, theta_tb):
        # Pour déterminer l'angle le plus court à parcourir entre la direction du robot
        # et celle du waypoint

        vector_tbw = np.array([waypoint[0] - tb_center[0], waypoint[1] - tb_center[1]])
        vector_tb = np.array([np.cos(theta_tb), np.sin(theta_tb)])
        theta_r = np.arccos(np.dot(vector_tb, vector_tbw) / (np.linalg.norm(vector_tb) * np.linalg.norm(vector_tbw)))

        # Pour déterminer de quel côté est le waypoint par rapport au robot
        vect_product = np.cross(np.concatenate((vector_tb, [0])), np.concatenate((vector_tbw, [0])))

        if vect_product[2] < 0:
            theta_r = -theta_r

        # Normalisation de la différence d'angle dans la plage [-pi, pi] radians
        if theta_r <= -np.pi:
            theta_r = theta_r + 2 * np.pi
        elif theta_r > np.pi:
            theta_r = theta_r - 2 * np.pi

        return theta_r


    def DistancePointToSegment(self,A, angle, P):
        # Projection du waypoint sur la vraie direction du robot dû à l'erreur dynamique

        # Calcul du vecteur de direction à partir de l'angle
        direction = np.array([np.cos(angle), np.sin(angle)])

        # Calcul du vecteur entre A et P
        AP = P - A

        # Calcul de la projection de Pp sur la direction
        t = np.dot(AP, direction) / np.linalg.norm(direction) ** 2
        pp = A + t * direction

        # Calcul de la distance minimale entre P et pp
        dist = np.linalg.norm(P - pp)

        return dist, pp

    # Algorithmes de génération de commandes (angulaire et linéaire)

    def rotate_tb(self,tb_center, waypoint, theta_tb, MaxAngularSpeed, MaxAngularAccel, angularSpeed, epsilon):
        # Génère la consigne trapézoïdale en vitesse angulaire
        rospy.loginfo(f"rotating {theta_tb} {tb_center} {waypoint}")
        self.theta_remain = self.ModuloByAngle(tb_center, waypoint, theta_tb)
        theta_stop = ((angularSpeed**2) / (2 * MaxAngularAccel))*6  # Voir démonstration

        bool_val = False

        if self.theta_remain > 0:  # L'angle restant à parcourir est supérieur à 0
            if angularSpeed < 0:  # La vitesse angulaire est négative (anormale)
                angularAccel = -MaxAngularAccel  # On freine
            else:  # La vitesse est positive, ce qui est normal
                if self.theta_remain > theta_stop:
                    if angularSpeed < MaxAngularSpeed:
                        angularAccel = MaxAngularAccel  # On accélère
                    else:
                        angularAccel = 0  # On maintient la vitesse
                else:
                    angularAccel = -MaxAngularAccel  # On freine
        else:  # L'angle restant à parcourir est inférieur à 0
            if angularSpeed > 0:  # La vitesse angulaire est positive (anormale)
                angularAccel = -MaxAngularAccel  # On freine
            else:  # La vitesse est négative, ce qui est normal
                if abs(self.theta_remain) > abs(theta_stop):
                    if angularSpeed > -MaxAngularSpeed:
                        angularAccel = -MaxAngularAccel  # On accélère négativement
                    else:
                        angularAccel = 0  # On maintient la vitesse
                else:
                    angularAccel = MaxAngularAccel  # On freine négativement

        if abs(self.theta_remain) < epsilon:
            bool_val = True  # On a fini

        return angularAccel, bool_val


    def linear_tb(self,waypoint, tb_center, theta_tb, MaxLinearSpeed, MaxLinearAccel, linearSpeed, epsilon):

        rospy.loginfo(f"Translating {theta_tb} {tb_center} {waypoint}")
        # Génère la consigne trapézoïdale en vitesse linéaire (algo très proche de la consigne en vitesse angulaire)

        projected_dist, projected_point = self.DistancePointToSegment(tb_center, theta_tb, waypoint)

        self.dist_remain = np.linalg.norm(projected_point - tb_center)  # Distance entre le centre du robot et le projeté du waypoint

        # On regarde si le waypoint est devant ou derrière le robot avec angle_error
        vecteur_tb = np.array([np.cos(theta_tb), np.sin(theta_tb)])
        vecteur_TBW = np.array([waypoint[0] - tb_center[0], waypoint[1] - tb_center[1]])

        angle_error = np.arccos(np.dot(vecteur_tb, vecteur_TBW) / (np.linalg.norm(vecteur_tb) * np.linalg.norm(vecteur_TBW)))

        dist_stop = (linearSpeed**2 / (2 * MaxLinearAccel))*6  # Voir démonstration
        linearSpeed = 0
        bool_val = False

        if (-np.pi / 2 <= angle_error <= np.pi / 2):  # Le waypoint est devant le robot
            if linearSpeed < 0:  # Le robot recule (anormal)
                linearAccel = -MaxLinearSpeed  # On freine
            else:  # Le robot avance, ce qui est normal
                if self.dist_remain > dist_stop:
                    if linearSpeed < MaxLinearSpeed:
                        linearAccel = MaxLinearAccel  # On accélère
                    else:
                        linearAccel = 0  # On maintient la vitesse
                else:
                    linearAccel = -MaxLinearAccel  # On freine
        else:  # Le waypoint est derrière le robot
            if linearSpeed > 0:  # Le robot avance (anormal)
                linearAccel = -MaxLinearSpeed  # On freine
            else:
                if self.dist_remain > dist_stop:  # Le robot avance (anormal)
                    if abs(linearSpeed) < MaxLinearSpeed:
                        linearAccel = -MaxLinearAccel  # On accélère négativement
                    else:
                        linearAccel = 0  # On maintient la vitesse
                else:
                    linearAccel = MaxLinearAccel # On freine négativement

        if self.dist_remain < epsilon:
            bool_val = True  # On a fini

        return linearAccel, bool_val

#================================== Boucle de Simulation ============================================



def main():
    controller = Controller()
    twist = Twist()

    while not rospy.is_shutdown():
        if(controller.acquisition_flag):
            # Machine à états
            if controller.TB_state == "rotating":

                controller.angularAccel, bool_val = controller.rotate_tb(controller.tb_center, controller.waypoint, controller.theta_tb, controller.MaxAngularSpeed, controller.MaxAngularAccel, controller.angularSpeed, controller.epsilon_angle)
                controller.angularSpeed += controller.angularAccel * controller.delta_t
                controller.linearSpeed = 0


                if bool_val:
                    controller.TB_state = "moving"

                # rospy.loginfo(f"TB_state : {controller.TB_state}\n")
                # rospy.loginfo(f" Centre du robot : {controller.tb_center}\n Waypoint : {controller.waypoint} \n\n\n")
                # rospy.loginfo(f"Angle restant : {controller.ModuloByAngle(controller.tb_center, controller.waypoint, controller.theta_tb)*180/np.pi}\n Angle stop : {((controller.angularSpeed**2) / (2 * controller.MaxAngularAccel))*180/np.pi} \n\n\n")
                # rospy.loginfo(f"Vitesse angulaire : {controller.angularSpeed}\n Accel angulaire : {controller.angularAccel} \n\n\n")
                # rospy.loginfo(f"Epsilon angle : {controller.epsilon_angle*180/np.pi}\n\n\n\n\n")

            elif controller.TB_state == "moving":
                dist_error = controller.dist_remain

                controller.linearAccel, bool_val = controller.linear_tb(controller.waypoint, controller.tb_center, controller.theta_tb, controller.MaxLinearSpeed, controller.MaxLinearAccel, controller.linearSpeed, controller.epsilon_dist)
                controller.linearAccel = controller.linearAccel
                controller.angularSpeed = 0
                controller.linearSpeed += controller.linearAccel * controller.delta_t

                #controller.linearSpeed = controller.linearSpeed * ())
                

                



                #projected_dist, projected_point = controller.DistancePointToSegment(controller.tb_center, controller.theta_tb, controller.waypoint)

                # rospy.loginfo(f"TB_state : {controller.TB_state}\n")
                # rospy.loginfo(f"Projeté du point : {projected_dist}\n Waypoint : {controller.waypoint} \n\n\n")
                # rospy.loginfo(f"Centre du robot : {controller.tb_center}") 
                # rospy.loginfo(f"Distance restante : {np.linalg.norm(projected_point - controller.tb_center)}\n Distance stop : {(controller.linearSpeed**2 / (2 * controller.MaxLinearAccel))*2} \n\n\n")
                # rospy.loginfo(f"Vitesse linéaire : {controller.linearSpeed}\n Accel linéaire : {controller.linearAccel} \n\n\n")
                # rospy.loginfo(f"Epsilon dist : {controller.epsilon_dist}\n\n\n\n\n")

                if bool_val:
                    controller.TB_state = "idle"

            elif controller.TB_state == "idle":
                controller.waypoints_incr += 1
                controller.angularAccel = 0
                controller.linearAccel = 0
                controller.angularSpeed = 0
                controller.linearSpeed = 0
                if controller.waypoints_incr < len(controller.waypoints):
                    controller.waypoint = controller.waypoints[controller.waypoints_incr]
                    controller.TB_state = "rotating"
                else:
                        controller.TB_state = "idle"


            twist.linear.x = controller.linearSpeed
            twist.angular.z = controller.angularSpeed

            if not controller.emergency:
                controller.cmd_vel_pub.publish(twist)

            controller.rate.sleep()





if __name__=="__main__":
    main()