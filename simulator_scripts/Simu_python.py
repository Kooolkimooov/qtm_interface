#!/usr/bin/env python

#=================================== Import =============================================
import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import time
import json
#================================= Déclarations =========================================
f = open('user_settings.json')
user_data = json.load(f)
print(user_data)

def get_user_parameters(data):
    # récupération données phase de rotation
    MaxAngularAccel = np.deg2rad(data['rotation_phase']['Max_Angular_Accel'])
    MaxAngularSpeed = np.deg2rad(data['rotation_phase']['Max_Angular_Speed'])
    epsilon_angle = np.deg2rad(data['rotation_phase']['Epsilon_angle'])

    # récupération données phase de translation
    MaxLinearAccel = data['linear_phase']['Max_Linear_Accel']
    MaxLinearSpeed = data['linear_phase']['Max_Linear_Speed']
    epsilon_dist = data['linear_phase']['Epsilon_dist']

    # récupération de la liste de waypoints
    waypoints = data['waypoints_list']['Waypoints_list']

    return MaxAngularAccel, MaxAngularSpeed, epsilon_angle, MaxLinearAccel, MaxLinearSpeed, epsilon_dist, waypoints

get_user_parameters(user_data)

# Paramètres de simulation
Fq = 100
Delta_t = 1 / Fq
end_time = 150
Trajectory_Time = 0

theta_tb = np.deg2rad(45)
tb_center = np.array([0, 0])

MaxAngularAccel, MaxAngularSpeed, epsilon_angle, MaxLinearAccel, MaxLinearSpeed, epsilon_dist, waypoints = get_user_parameters(user_data)
waypoints.pop(0) #on enlève la position initiale du robot
waypoints_incr = 0
waypoint = waypoints[waypoints_incr]

X0 = np.array([tb_center[0], tb_center[1], theta_tb])

Xout = X0.reshape(1, -1)
Trajectory = Xout

linearSpeed = 0
angularSpeed = 0

linearAccel = 0
angularAccel = 0

TB_state = "rotating"
idle_delay = 0
#############################################################################################################

# Initialisation du graphique Matplotlib en mode interactif
plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111)
ax.set_xlabel('Position en X')
ax.set_ylabel('Position en Y')
ax.set_title('Trajectoire du TurtleBot')
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
robot_circle = Circle((X0[0], X0[1]), 0.1, color='blue', alpha=0.5)
ax.add_patch(robot_circle)
hPlot, = ax.plot(X0[0], X0[1], 'g')
cible, = ax.plot(waypoint[0], waypoint[1], 'ro')
turtle = ax.quiver(X0[0], X0[1], 0.1 * np.cos(X0[2]), 0.1 * np.sin(X0[2]), color='r', scale=10, width=0.005)
affich_coord = ax.text(0, 7.7, f'X: {X0[0]}, Y: {X0[1]}, Theta: {np.rad2deg(X0[2])}')
affich_consigne = ax.text(2.2, 7, 'Lin: None, Rot: None')




#================================== Fonctions ============================================
# Fonctions de calculs
def kinematic_model(_, X, u):
    # Calcul de la dynamique du TurtleBot
    linear = np.array([np.cos(X[2]), np.sin(X[2]), 0])
    angular = np.array([0, 0, 1])
    dx = linear * u[0] + angular * u[1]
    return dx

def ModuloByAngle(tb_center, waypoint, theta_tb):
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


def DistancePointToSegment(A, angle, P):
    # Projection du waypoint sur la vraie direction du robot dû à l'erreur dynamique

    # Calcul du vecteur de direction à partir de l'angle
    direction = np.array([np.cos(angle), np.sin(angle)])

    # Calcul du vecteur entre A et P
    AP = P - A

    # Calcul de la projection Pp sur la direction
    t = np.dot(AP, direction) / np.linalg.norm(direction) ** 2
    pp = A + t * direction

    # Calcul de la distance minimale entre P et pp
    dist = np.linalg.norm(P - pp)

    return dist, pp

# Algorithmes de génération de commandes (angulaire et linéaire)

def rotate_tb(TB_center, waypoint, theta_tb, MaxAngularSpeed, MaxAngularAccel, angularSpeed, epsilon):
    # Génère la consigne trapézoïdale en vitesse angulaire

    theta_remain = ModuloByAngle(TB_center, waypoint, theta_tb)
    theta_stop = ((angularSpeed**2) / (2 * MaxAngularAccel))*10  # Voir démonstration

    bool_val = False

    if theta_remain > 0:  # L'angle restant à parcourir est supérieur à 0
        if angularSpeed < 0:  # La vitesse angulaire est négative (anormale)
            angularAccel = -MaxAngularAccel  # On freine
        else:  # La vitesse est positive, ce qui est normal
            if theta_remain > theta_stop:
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
            if abs(theta_remain) > abs(theta_stop):
                if angularSpeed > -MaxAngularSpeed:
                    angularAccel = -MaxAngularAccel  # On accélère négativement
                else:
                    angularAccel = 0  # On maintient la vitesse
            else:
                angularAccel = MaxAngularAccel  # On freine négativement

    if abs(theta_remain) < epsilon:
        bool_val = True  # On a fini

    return angularAccel, bool_val


def linear_tb(waypoint, TB_center, theta_tb, MaxLinearSpeed, MaxLinearAccel, linearSpeed, epsilon):
    # Génère la consigne trapézoïdale en vitesse linéaire (algo très proche de la consigne en vitesse angulaire)

    projected_dist, projected_point = DistancePointToSegment(TB_center, theta_tb, waypoint)

    dist_remain = np.linalg.norm(projected_point - TB_center)  # Distance entre le centre du robot et le projeté du waypoint

    # On regarde si le waypoint est devant ou derrière le robot avec angle_error
    vecteur_tb = np.array([np.cos(theta_tb), np.sin(theta_tb)])
    vecteur_TBW = np.array([waypoint[0] - TB_center[0], waypoint[1] - TB_center[1]])

    angle_error = np.arccos(np.dot(vecteur_tb, vecteur_TBW) / (np.linalg.norm(vecteur_tb) * np.linalg.norm(vecteur_TBW)))

    dist_stop = (linearSpeed**2 / (2 * MaxLinearAccel))*10  # Voir démonstration
    linearSpeed = 0
    bool_val = False

    if (-np.pi / 2 <= angle_error <= np.pi / 2):  # Le waypoint est devant le robot
        if linearSpeed < 0:  # Le robot recule (anormal)
            linearAccel = -MaxLinearSpeed  # On freine
        else:  # Le robot avance, ce qui est normal
            if dist_remain > dist_stop:
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
            if dist_remain > dist_stop:  # Le robot avance (anormal)
                if abs(linearSpeed) < MaxLinearSpeed:
                    linearAccel = -MaxLinearAccel  # On accélère négativement
                else:
                    linearAccel = 0  # On maintient la vitesse
            else:
                linearAccel = MaxLinearAccel  # On freine négativement

    if dist_remain < epsilon:
        bool_val = True  # On a fini

    return linearAccel, bool_val

#================================== Boucle de Simulation ============================================
i = 0
while waypoints_incr < len(waypoints):
    start_time = time.time()
    # Machine à états
    if TB_state == "rotating":
        angularAccel, bool_val = rotate_tb(tb_center, waypoint, theta_tb, MaxAngularSpeed, MaxAngularAccel, angularSpeed, epsilon_angle)
        angularAccel = angularAccel
        angularSpeed += angularAccel * Delta_t
        linearSpeed = 0
        if bool_val:
            TB_state = "moving"
    elif TB_state == "moving":
        linearAccel, bool_val = linear_tb(waypoint, tb_center, theta_tb, MaxLinearSpeed, MaxLinearAccel, linearSpeed, epsilon_dist)
        linearAccel = linearAccel
        angularSpeed = 0
        linearSpeed += linearAccel * Delta_t
        if bool_val:
            TB_state = "idle"
    elif TB_state == "idle":
        waypoints_incr += 1
        angularAccel = 0
        linearAccel = 0
        angularSpeed = 0
        linearSpeed = 0
        if waypoints_incr < len(waypoints):
            waypoint = waypoints[waypoints_incr]
            cible.set_data(waypoint[0], waypoint[1])
            TB_state = "rotating"
        else:
            TB_state = "idle"

    u = [linearSpeed, angularSpeed]
    t_eval = [0, Delta_t]
    Xout = odeint(kinematic_model(), Xout[-1], t_eval, args=(u,), tfirst=True)
    X0 = Xout[-1]
    theta_tb = X0[2]
    tb_center = X0[:2]
    Trajectory = np.append(Trajectory, Xout[1:], axis=0)

    if i % 20 == 0:
        hPlot.set_data(Trajectory[:, 0], Trajectory[:, 1])
        turtle.set_offsets((X0[0], X0[1]))
        robot_circle.center = (X0[0], X0[1])
        turtle.set_UVC(0.1 * np.cos(X0[2]), 0.1 * np.sin(X0[2]))
        affich_coord.set_text(f'X: {X0[0]}, Y: {X0[1]}, Theta: {np.rad2deg(X0[2])}')
        affich_consigne.set_text(f'Lin: {u[0]:.2f} m/s, Rot: {np.rad2deg(u[1]):.2f} deg/s')
        plt.pause(0.001)

    i += 1
    elapsed_time = time.time() - start_time
    print(elapsed_time)
plt.ioff()
plt.show()
