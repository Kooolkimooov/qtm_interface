# Notice d'utilisation, Liaison Qualisys et ROS, Interface utilisateur

## Interfaçage Qualisys et ROS

Deux ordinateurs et un routeur sont nécessaires pour mettre en place le pilotage du robot à partir du système de motion capture Qualisys et de ROS.

- Un ordinateur qui utilise Windows sur lequel est installé le logiciel Qualisys
- Un ordinateur qui utilise une distribution Linux sur lequel on peut utiliser l'environnement ROS
- Un routeur qui permet de relier les deux ordinateurs et le robot à un même réseau

### Utilisation de Qualisys

Dans un premier temps, il faut préparer l'aire de pilotage du robot en positionnant les caméras pour qu'au moins 3 d'entre elles puissent détecter le robot quelle que soit sa position dans l'aire. Il est aussi nécessaire de monter le repère absolu et de le placer si possible au milieu de la surface.

Ensuite, il faut lancer une acquisition sur le logiciel Qualisys, l'annuler et lancer une calibration avec la baguette de calibration appropriée. Il faut que la précision soit inférieure à 3 mm pour que les conditions soient optimales pour avoir un bon retour de la position du robot.

Pour créer un Rigid Body du robot, il faut positionner des boules réfléchissantes en limitant le plus possible toute symétrie dans leur positionnement. En faisant une acquisition et en sélectionnant les points correspondants au robot sur le logiciel, il est alors possible de créer son Rigid Body. Un repère local sera associé au Rigid Body ; celui-ci peut être modifié dans les propriétés pour correspondre au repère absolu.

Une fois le Rigid Body créé, Qualisys va détecter la position du centre du robot à tout instant quand une acquisition est lancée. Les programmes que nous avons conçus sont faits pour récupérer ces données et les utiliser pour asservir le système en position.

### Procédure d'utilisation du système

Après avoir effectué une calibration de l'aire de pilotage.

Voici la liste exhaustive des actions à effectuer pour lancer l'expérience :
- Vérifier que les caméras sont bien allumées et que les ordinateurs sont connectés au réseau du routeur.
- **PC Qualisys** : Lancer une calibration 
  - `Capture > Calibrate`
- **PC Qualisys** : Fermer toute simulation Qualisys sur l'application
- **PC ROS** : Établir la connexion entre l'ordinateur et le robot (master ros sur un terminal et commande de connexion sur un autre)
  - `ssh turtle@'addresse_ip_turtleblot'`
  - `mot de passe: turtle`
- **PC ROS** : lancer ROS Master sur turtlebot
  - `roscore`
- **PC ROS** : démarrer le turtlebot
  - `roslaunch turtlebot_bringup minimal.launch`
- **PC ROS** : Lancer le nœud interface.py et rentrer les paramètres de pilotage, le mode de pilotage ainsi que les waypoints de la trajectoire. Après avoir appuyé sur le bouton "Exporter", le nœud se fermera et un fichier .json contenant les paramètres sera créé (ou écrasé)
  - `rosrun qtm_interface interface`
- **PC ROS** : Lancer le nœud qtm_interface.py
  - `rosrun qtm_interface qtm_interface`
- **PC Qualisys** : Une demande d'acquisition s'est normalement lancée automatiquement, il suffit d'appuyer sur le bouton start (rond rouge) et donner un nom à l'acquisition (si cela est déjà fait le nom sera gardé avec un numéro qui sera incrémenté pour chaque acquisition). La position du robot est maintenant captée et publiée sur un topic ROS.
- **PC ROS** : Lancer le nœud linear_controller.py ou pure_pursuit.py , le topic ROS est utilisé pour obtenir les positions, le fichier .json est lu pour utiliser les paramètres utilisateur, le robot suit la trajectoire demandée avec le mode demandé.
  - `rosrun qtm_interface linear_controller` ou `rosrun qtm_interface pure_pursuit` 
- **PC ROS** : Arrêter le nœud PID_controller.py pour arrêter le pilotage.
- **PC Qualisys** : Arrêter l'acquisition.   

## Utilisation de l'interface

Pour utiliser cette interface, vous trouverez deux panneaux distincts :

1. **Panneau de Commande :**
   - Configurez le mode de commande, les vitesses, les accélérations maximales du robot (particulièrement utile en mode linéaire).
   - Définissez les valeurs acceptables d'erreur de position par rapport à la trajectoire.
   - Modes disponibles : 
     - Interpolation linéaire entre les waypoints (rotation puis translation).
     - Interpolation par spline cubique entre les waypoints.
     - Commande par modèle prédictif (visualisation des waypoints uniquement).

2. **Panneau de Visualisation :**
   - Attention, vous devez impérativement choisir le mode de commande et remplir les entrées textuelles et appuyer sur le bouton **"Mise à jour"** avant de créer votre trajectoire.
   - Saisissez la position initiale du robot.
   - Définissez les sommets du polygone représentant la surface de pilotage.
   - Indiquez la trajectoire souhaitée.
   - Visualisez la trajectoire et mettez-la à jour de manière intuitive.
   - Accédez aux détails des waypoints et de leurs coordonnées.
   - En cas d'erreur dans la trajectoire, utilisez le bouton **"Nettoyer"** pour recommencer.
   - Pour initialiser la construction d'une trajectoire, saisissez les données nécessaires dans les champs textuels sous le graphique, puis appuyez sur le bouton **"Mise à jour"**.

Une fois que vous avez configuré les paramètres dans les deux panneaux, appuyez sur "Exporter" pour générer le fichier de données et fermer l'interface.

**Vous êtes maintenant prêt à lancer l'asservissement en position !**

## Détail du fonctionnement des nœuds ROS

#### Noeud qtm_interface.py

Ce script Python fonctionne comme un nœud ROS pour interagir avec le logiciel Qualisys Track Manager (QTM) afin de diffuser des données 6Dof (position et orientation) d'un objet (rigid body) vers ROS.

**Étapes clés :**

1. **Connexion à QTM** :
   - Le nœud tente d'établir une connexion avec QTM sur l'adresse IP spécifiée (`QTM_IP`) et

 le port (`PORT`).

2. **Gestion du Contrôle** :
   - Le nœud prend le contrôle de QTM en utilisant le mot de passe spécifié (`PASSWORD`).

3. **Lancement de l'Acquisition** :
   - Si `REAL_TIME` est activé, le nœud attend que l'enregistrement démarre sur QTM.
   - Sinon, il lance la lecture d'un enregistrement existant sur QTM.

4. **Streaming des Données** :
   - Le nœud commence à diffuser les données de positionnement et d'orientation du rigid body spécifié (`BODY_NAME`) en utilisant les composants définis (`COMPONENTS`), tels que les positions 3D sans libellés et les angles d'Euler 6Dof.

5. **Traitement des Paquets de Données** :
   - Chaque paquet de données reçu de QTM est analysé pour extraire la position et l'orientation du rigid body spécifié.
   - Ces informations sont ensuite publiées sur le topic ROS spécifié (`qtm/{BODY_NAME}/6dof_pose`).

6. **Arrêt Contrôlé** :
   - Lorsque le nœud est arrêté ou termine son exécution, il interrompt la diffusion des données et libère le contrôle de QTM.

#### Noeud linear_controller.py

Ce script Python implémente un contrôleur PID pour commander un robot (TurtleBot) en utilisant les données de position et d'orientation fournies par un système de motion capture (QTM).

**Étapes clés :**

1. **Initialisation du Contrôleur** :
   - Le contrôleur est initialisé avec des paramètres tels que les waypoints à suivre, les vitesses maximales autorisées, et les seuils d'erreur.

2. **Acquisition des Données** :
   - Les données de position et d'orientation du robot sont acquises à partir du topic `qtm/turtle/6dof_pose`.

3. **Machine à États** :
   - Le contrôleur fonctionne selon un modèle à plusieurs états : 
     - **"rotating"** : Le robot ajuste son orientation pour faire face au prochain waypoint.
     - **"moving"** : Le robot se déplace vers le waypoint suivant.
     - **"idle"** : Le robot attend ou passe au waypoint suivant.

4. **Algorithmes de Commande** :
   - **`rotate_tb`** : Génère une commande de rotation pour orienter le robot vers le prochain waypoint.
   - **`linear_tb`** : Génère une commande de translation pour déplacer le robot vers le prochain waypoint.

5. **Boucle Principale** :
   - La boucle principale du script met à jour les commandes de vitesse du robot en fonction de son état actuel et des waypoints à atteindre.

6. **Publication des Commandes** :
   - Les commandes de vitesse calculées sont publiées sur le topic `mobile_base/commands/velocity` pour contrôler effectivement le robot.
