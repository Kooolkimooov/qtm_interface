import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
import numpy as np
import json
from scipy.interpolate import CubicSpline
class JSON :

    def __init__(self):
        self.path = "./user_settings.json"

    def json_constructor(self):
        data = {
            "mode_de_commande":{
                "Commande_linéaire": interface.linear.get(),
                "Commande_cubique": interface.cubique.get(),
                "Commande_predictive": interface.predictif.get()
            },
            "rotation_phase": {
                "Max_Angular_Accel": interface.max_angular_accel_slider.get(),
                "Max_Angular_Speed": interface.max_angular_speed_slider.get(),
                "Epsilon_angle": float(interface.epsilon_angular_entry.get())
            },
            "linear_phase": {
                "Max_Linear_Accel": interface.max_linear_accel_slider.get(),
                "Max_Linear_Speed": interface.max_linear_speed_slider.get(),
                "Epsilon_dist": float(interface.epsilon_linear_entry.get())
            },
            "waypoints_list": {
                "Waypoints_list": interface.waypoints
            }
        }

        with open(self.path, "w") as f:
            json.dump(data, f, indent=2)
        interface.root.destroy()


json_file = JSON()

class HelpWindow:

    def __init__(self, parent, markdown_file):
        self.parent = parent
        self.markdown_file = markdown_file

        # Créer la fenêtre d'aide
        self.window = tk.Toplevel(parent)
        self.window.title("Aide")

        # Créer un widget Text pour afficher le contenu Markdown
        self.text_area = tk.Text(self.window, wrap=tk.WORD)
        self.text_area.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Créer une scrollbar pour la zone de texte
        self.scrollbar = tk.Scrollbar(self.window, command=self.text_area.yview)
        self.scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.text_area.config(yscrollcommand=self.scrollbar.set)

        # Charger le contenu du fichier Markdown dans la zone de texte
        self.load_markdown_content()

    def load_markdown_content(self):
        with open(self.markdown_file, "r", encoding="utf-8") as file:
            markdown_content = file.read()
        self.text_area.insert(tk.END, markdown_content)



class Interface :
    def show_helper(self):
        path_to_md_file = "./Notice.md"
        help_window = HelpWindow(self.root, path_to_md_file)

    def clear(self):
        # Effacer la liste des waypoints
        self.waypoints.clear()

        # Effacer les éléments du tableau des waypoints
        for widget in self.waypoints_table.winfo_children():
            widget.destroy()

        # Effacer les éléments tracés sur le graphe
        self.ax_interface.clear()
        self.ax_interface.set_xlim(-2, 2)
        self.ax_interface.set_ylim(-2, 2)
        self.ax_interface.axhline(0, color='black', linewidth=0.5)
        self.ax_interface.axvline(0, color='black', linewidth=0.5)
        self.ax_interface.set_aspect('equal', adjustable='box')
        self.ax_interface.set_title('Trajectoire')
        self.ax_interface.set_xlabel('X')
        self.ax_interface.set_ylabel('Y')

        # Mettre à jour le graphe
        self.canvas.draw()


    # suivi souris sur le graphe
    def motion(self, event):
        x, y = event.xdata, event.ydata
        if x is not None and y is not None:
            # Calculer les coordonnées de la souris par rapport à la fenêtre principale
            graph_x, graph_y = self.canvas.get_tk_widget().winfo_rootx(), self.canvas.get_tk_widget().winfo_rooty()   # coordonnées du graphique par rapport à la fenêtre d'application
            x_window = self.root.winfo_pointerx() - graph_x + 40
            y_window = self.root.winfo_pointery() - graph_y + 75
            # Déplacer le label coord_mouse aux nouvelles coordonnées
            self.coord_mouse.place(x=x_window, y=y_window)
            # Mettre à jour le texte du label avec les coordonnées de la souris
            self.coord_mouse.config(text=f"X={x:.2f}, Y={y:.2f}")
        else:
            self.coord_mouse.place_forget()  # Cacher le label si la souris est en dehors du graphe


    # constructeur de trajectoire
    def init_tableANDgraph(self):
        x0 = float(self.TB_center_x_entry.get())
        y0 = float(self.TB_center_y_entry.get())

        coin1_x = float(self.qualisys_x_entry_1.get())
        coin1_y = float(self.qualisys_y_entry_1.get())

        coin2_x = float(self.qualisys_x_entry_2.get())
        coin2_y = float(self.qualisys_y_entry_2.get())

        coin3_x = float(self.qualisys_x_entry_3.get())
        coin3_y = float(self.qualisys_y_entry_3.get())

        coin4_x = float(self.qualisys_x_entry_4.get())
        coin4_y = float(self.qualisys_y_entry_4.get())

        self.waypoints = [[x0, y0]]

        # Créer l'en-tête de la table
        tk.Label(self.waypoints_table, text=" id ", borderwidth=1, relief="solid").grid(row=0, column=0, sticky="nsew")
        tk.Label(self.waypoints_table, text=" X ", borderwidth=1, relief="solid").grid(row=0, column=1, sticky="nsew")
        tk.Label(self.waypoints_table, text=" Y ", borderwidth=1, relief="solid").grid(row=0, column=2, sticky="nsew")

        tk.Label(self.waypoints_table, text=" Init ", borderwidth=1, relief="solid").grid(row=1, column=0,sticky="nsew")
        tk.Label(self.waypoints_table, text=f" {x0} ", borderwidth=1, relief="solid").grid(row=1, column=1, sticky="nsew")
        tk.Label(self.waypoints_table, text=f" {y0} ", borderwidth=1, relief="solid").grid(row=1, column=2, sticky="nsew")

        self.ax_interface.plot(x0, y0, marker='x', color='black')
        self.ax_interface.plot(coin1_x, coin1_y, marker='o', color="green")
        self.ax_interface.plot(coin2_x, coin2_y, marker='o', color="green")
        self.ax_interface.plot(coin3_x, coin3_y, marker='o', color="green")
        self.ax_interface.plot(coin4_x, coin4_y, marker='o', color="green")

        polygon_coords = [(coin1_x, coin1_y), (coin2_x, coin2_y), (coin3_x, coin3_y), (coin4_x, coin4_y)]

        # Calcul du centre du polygone
        cx = np.mean([polygon_coords[0] for coords in polygon_coords])
        cy = np.mean([polygon_coords[1] for coords in polygon_coords])

        # Trier les sommets en fonction de l'angle polaire par rapport au centre du polygone
        sorted_corners = sorted(polygon_coords, key=lambda coords: np.arctan2(coords[1] - cy, coords[0] - cx))

        surface = plt.Polygon(sorted_corners, closed=True, facecolor=(0.564, 0.933, 0.564), edgecolor=(0.564, 0.933, 0.564))
        self.ax_interface.add_patch(surface)

        # Mettre à jour le graphe
        self.canvas.draw()

    def update_checkbox(self, event):
        if event == "linear" and self.linear.get() == 1:
            self.cubique.set(0)
            self.predictif.set(0)
        elif event == "cubique" and self.cubique.get() == 1:
            self.linear.set(0)
            self.predictif.set(0)
        elif event == "predictif" and self.predictif.get() == 1:
            self.linear.set(0)
            self.cubique.set(0)

    def cubic_spline_trajectory_constructor(self, waypoint_list, interpolation_number):
        waypoint_list = np.array(waypoint_list)
        # paramètre intermédiaire :
        t = np.linspace(0, 1, len(waypoint_list))
        tt = np.linspace(0, 1, interpolation_number)
        y_t = CubicSpline(t, waypoint_list[:, 1], bc_type="natural")
        y_tt = y_t(tt)
        x_t = CubicSpline(t, waypoint_list[:, 0], bc_type="natural")
        x_tt = x_t(tt)
        return x_tt, y_tt

    def trajectory_constructor(self, event):
            x, y = event.xdata, event.ydata
            self.waypoints.append([x, y])
            self.ax_interface.plot(x, y, marker='x', color='red')
            if len(self.waypoints) > 1:
                prev_x, prev_y = self.waypoints[-2]
                if self.linear.get():
                    self.ax_interface.plot([prev_x, x], [prev_y, y], color='orange')
                    self.ax_interface.text(prev_x + 0.05, prev_y + 0.05, str(len(self.waypoints)-1), color='blue')  # Afficher le numéro du waypoint
                elif self.cubique.get():
                    x_tt, y_tt = self.cubic_spline_trajectory_constructor(self.waypoints, 1000)
                    lines = self.ax_interface.lines[:]
                    for line in lines:
                        if line.get_color() == 'orange':
                            line.remove()
                    self.ax_interface.plot(x_tt, y_tt, color='orange')
                    self.ax_interface.text(prev_x + 0.05, prev_y + 0.05, str(len(self.waypoints) - 1), color='blue')  # Afficher le numéro du waypoint
                else:
                    self.ax_interface.text(prev_x + 0.05, prev_y + 0.05, str(len(self.waypoints)-1), color='blue')  # Afficher le numéro du waypoint
                # Ajouter chaque ligne de waypoints à la table
                for idx, point in enumerate(self.waypoints):
                    if idx == 0:
                        pass
                    else:
                        tk.Label(self.waypoints_table, text=f" {idx + 1} ", borderwidth=1, relief="solid").grid(row=idx + 1, column=0, sticky="nsew")
                        tk.Label(self.waypoints_table, text=f" {point[0]:.2f} ", borderwidth=1, relief="solid").grid( row=idx + 1, column=1, sticky="nsew")
                        tk.Label(self.waypoints_table, text=f" {point[1]:.2f} ", borderwidth=1, relief="solid").grid(row=idx + 1, column=2, sticky="nsew")
            self.canvas.draw_idle()

    def __init__(self):
        self.waypoints = []

        # ---------------------------------- Création de la fenêtre principale
        self.root = tk.Tk()
        self.root.title("Interface_TurtleBot")
        self.root.geometry("800x800")

        #----------------------------------- Création d'un menu déroulant

        # Créer un menu principal
        self.menu = tk.Menu(self.root)
        self.root.config(menu=self.menu)

        # Créer un sous-menu "Help" dans le menu principal
        self.help_menu = tk.Menu(self.menu, tearoff=False)
        self.help_menu.add_command(label="Aide", command=self.show_helper)

        # Ajouter le sous-menu "Help" au menu principal
        self.menu.add_cascade(label="Aide", menu=self.help_menu)

        # ---------------------------------- Création de la PanedWindow
        self.paned_window = tk.PanedWindow(orient="horizontal")

        # ---------------------------------- Création des deux volets
        self.side_param = tk.Frame(self.paned_window, bg="lightgrey", pady=10)
        self.side_trajectory = tk.Frame(self.paned_window, bg="lightgrey", pady=10)
        self.paned_window.add(self.side_param, width=200)
        self.paned_window.add(self.side_trajectory, width=600)

        #----------------------------------- Mode de commande
        self.mode_commande_container = tk.Frame(self.side_param, bg="lightblue", width=200)
        self.mode_commande_label = tk.Label(self.side_param, bg="lightblue", text="Mode de commande", font=("Arial", 12, "bold"))
        self.mode_commande_container_label = tk.Frame(self.mode_commande_container, bg="lightblue")
        self.mode_commande_container_checkbox = tk.Frame(self.mode_commande_container, bg="lightblue")

        self.spline_lineaire_label = tk.Label(self.mode_commande_container_label, font=("Arial", 10, "bold") , text="Controle Linéaire : ", bg="lightblue")
        self.spline_cubique_label = tk.Label(self.mode_commande_container_label, font=("Arial", 10, "bold") ,text="Controle Cubique : ", bg="lightblue")
        self.predictiv_control_label = tk.Label(self.mode_commande_container_label, font=("Arial", 10, "bold") ,text="Controle Prédictif : ", bg="lightblue")

        # ---------------------------------- Mode de commande
        self.linear = tk.IntVar()
        self.cubique = tk.IntVar()
        self.predictif = tk.IntVar()

        self.spline_lineaire = tk.Checkbutton(self.mode_commande_container_checkbox, bg="lightblue", variable=self.linear, command=lambda: self.update_checkbox("linear"))
        self.spline_cubique = tk.Checkbutton(self.mode_commande_container_checkbox, bg="lightblue", variable=self.cubique, command=lambda: self.update_checkbox("cubique"))
        self.predictiv_control = tk.Checkbutton(self.mode_commande_container_checkbox, bg="lightblue", variable=self.predictif, command=lambda: self.update_checkbox("predictif"))

        # ---------------------------------- Conteneur pour la phase de rotation
        self.rotation_container = tk.Frame(self.side_param, bg="lightblue", width=200)
        self.max_angular_accel_slider = tk.Scale(self.rotation_container, from_=0, to=100, resolution=0.1, length=150,orient="horizontal")
        self.max_angular_accel_slider.set(5)  # Valeur par défaut
        self.max_angular_speed_slider = tk.Scale(self.rotation_container, from_=0, to=100, resolution=1, length=150, orient="horizontal")
        self.max_angular_speed_slider.set(20)  # Valeur par défaut
        self.epsilon_angular_entry = tk.Entry(self.rotation_container)

        # ---------------------------------- Conteneur pour la phase linéaire
        self.linear_container = tk.Frame(self.side_param, bg="lightblue", width=200)
        self.max_linear_accel_slider = tk.Scale(self.linear_container, from_=0, to=1, resolution=0.01, length=150, orient="horizontal")
        self.max_linear_accel_slider.set(0.1)  # Valeur par défaut
        self.max_linear_speed_slider = tk.Scale(self.linear_container, from_=0, to=1, resolution=0.05, length=150, orient="horizontal")
        self.max_linear_speed_slider.set(0.1)  # Valeur par défaut
        self.epsilon_linear_entry = tk.Entry(self.linear_container)

        # ---------------------------------- Ajout du graphe
        self.graph_container = tk.Frame(self.side_trajectory, bg="lightblue", width=500)

        self.fig_interface = plt.Figure(figsize=(4, 4))
        self.ax_interface = self.fig_interface.add_subplot(111)
        self.ax_interface.set_xlim(-2, 2)
        self.ax_interface.set_ylim(-2, 2)
        self.ax_interface.axhline(0, color='black', linewidth=0.5)
        self.ax_interface.axvline(0, color='black', linewidth=0.5)
        self.ax_interface.set_aspect('equal', adjustable='box')
        self.ax_interface.set_title('Trajectoire')
        self.ax_interface.set_xlabel('X')
        self.ax_interface.set_ylabel('Y')

        self.fig_interface.patch.set_facecolor('lightblue')
        self.canvas = FigureCanvasTkAgg(self.fig_interface, master=self.graph_container)
        self.canvas.draw()



        self.coord_mouse = tk.Label(self.side_trajectory, text="", font=("Arial", 10))
        self.coord_mouse.place(x=0, y=0)

        # ---------------------------------- Tableau de waypoints
        self.waypoints_table_frame = tk.Frame(self.graph_container, bg="lightblue")
        self.waypoints_table = tk.Frame(self.waypoints_table_frame)


        self.all_entry_container = tk.Frame(self.side_trajectory, bg="lightgrey")
        # ----------------------------------- TB initial center entry
        self.TB_center_coord_container = tk.Frame(self.all_entry_container, bg="lightblue")
        self.TB_center_coord_container_label = tk.Label(self.TB_center_coord_container, text="Position initiale", font=("Arial", 12, "bold"), bg="lightblue")

        self.TB_coord_x_container = tk.Frame(self.TB_center_coord_container, bg="lightblue")
        self.TB_center_x_label = tk.Label(self.TB_coord_x_container,text="X", font=("Arial", 10), bg="lightblue")
        self.TB_center_x_entry = tk.Entry(self.TB_coord_x_container, width=5)

        self.TB_coord_y_container = tk.Frame(self.TB_center_coord_container, bg="lightblue")
        self.TB_center_y_label = tk.Label(self.TB_coord_y_container, text="Y", font=("Arial", 10), bg="lightblue")
        self.TB_center_y_entry = tk.Entry(self.TB_coord_y_container, width=5)

        # ----------------------------------- Qualysis era
        self.qualisys_era_container = tk.Frame(self.all_entry_container, bg="lightblue")
        self.qualisys_era_container_label = tk.Label(self.qualisys_era_container, text="Surface calibrée", font=("Arial", 12, "bold"), bg="lightblue")

        self.qualisys_era_x_frame = tk.Frame(self.qualisys_era_container, bg="lightblue")
        self.qualisys_era_x_label = tk.Label(self.qualisys_era_x_frame, text="X", font=("Arial", 10), bg="lightblue")

        self.qualisys_era_label_frame = tk.Frame(self.qualisys_era_container, bg="lightblue")
        self.qualisys_era_label_1 = tk.Label(self.qualisys_era_label_frame, text="Coin 1", font=("Arial", 10), bg="lightblue")
        self.qualisys_era_label_2 = tk.Label(self.qualisys_era_label_frame, text="Coin 2", font=("Arial", 10), bg="lightblue")
        self.qualisys_era_label_3 = tk.Label(self.qualisys_era_label_frame, text="Coin 3", font=("Arial", 10), bg="lightblue")
        self.qualisys_era_label_4 = tk.Label(self.qualisys_era_label_frame, text="Coin 4", font=("Arial", 10), bg="lightblue")

        self.qualisys_x_entry_1 = tk.Entry(self.qualisys_era_x_frame, width=5)
        self.qualisys_x_entry_2 = tk.Entry(self.qualisys_era_x_frame, width=5)
        self.qualisys_x_entry_3 = tk.Entry(self.qualisys_era_x_frame, width=5)
        self.qualisys_x_entry_4 = tk.Entry(self.qualisys_era_x_frame, width=5)

        self.qualisys_era_y_frame = tk.Frame(self.qualisys_era_container, bg="lightblue")
        self.qualisys_era_y_label = tk.Label(self.qualisys_era_y_frame, text="Y", font=("Arial", 10), bg="lightblue")
        self.qualisys_y_entry_1 = tk.Entry(self.qualisys_era_y_frame, width=5)
        self.qualisys_y_entry_2 = tk.Entry(self.qualisys_era_y_frame, width=5)
        self.qualisys_y_entry_3 = tk.Entry(self.qualisys_era_y_frame, width=5)
        self.qualisys_y_entry_4 = tk.Entry(self.qualisys_era_y_frame, width=5)

        self.button_container_frame = tk.Frame(self.side_trajectory, bg="lightblue")

        #----------------------------------- Update
        self.button_update_frame = tk.Frame(self.button_container_frame, bg="lightblue")
        self.update_button = tk.Button(self.button_update_frame, text="Mise à jour", width=20, command=self.init_tableANDgraph)


        # ---------------------------------- Button clear
        self.button_clear_frame = tk.Frame(self.button_container_frame, bg="lightblue")
        self.clear_button = tk.Button(self.button_clear_frame, text="Nettoyer", width=20, command=self.clear)

        #----------------------------------- Button publish
        self.button_publish_frame = tk.Frame(self.button_container_frame, bg="lightblue")
        self.button_publish = tk.Button(self.button_publish_frame, text="Exporter", width=20, command=json_file.json_constructor)

    def window_creation(self):
        self.paned_window.pack(expand=True, fill="both")

        tk.Label(self.side_param, text="Commande", font=("Arial", 16, "bold"), bg="lightgrey").pack(pady=(0, 30))

        tk.Label(self.side_trajectory, text="Visualisation", font=("Arial", 16, "bold"), bg="lightgrey").pack(pady=(0, 20), fill="x")

        self.mode_commande_label.pack(fill="x", pady=(0, 5))
        self.mode_commande_container.pack(pady=(0, 10), fill="x")
        self.mode_commande_container_label.pack(padx=(5, 5), fill="x", side="left")
        self.mode_commande_container_checkbox.pack(padx=(5, 5), fill="x", side="left")

        self.spline_lineaire_label.pack()
        self.spline_cubique_label.pack()
        self.predictiv_control_label.pack()

        self.spline_lineaire.pack()
        self.spline_cubique.pack()
        self.predictiv_control.pack()
        self.rotation_container.pack(pady=(0, 40), fill="x")
        tk.Label(self.rotation_container, text="Phase de rotation", font=("Arial", 12, "bold"), bg="lightblue").pack()
        tk.Label(self.rotation_container, text="Accélération angulaire maximale", font=("Arial", 10), bg="lightblue").pack()
        self.max_angular_accel_slider.pack(pady=(0, 5))
        tk.Label(self.rotation_container, text="Vitesse angulaire maximale", font=("Arial", 10), bg="lightblue").pack()
        self.max_angular_speed_slider.pack(pady=(0, 5))
        tk.Label(self.rotation_container, text="Epsilon angulaire", font=("Arial", 10)).pack()
        tk.Label(self.linear_container, text="Phase linéaire", font=("Arial", 12, "bold"), bg="lightblue").pack()
        tk.Label(self.linear_container, text="Accélération linéaire maximale", font=("Arial", 10), bg="lightblue").pack()
        self.epsilon_angular_entry.pack(pady=(0, 20))
        self.linear_container.pack(pady=(0, 40), fill="x")
        self.max_linear_accel_slider.pack(pady=(0, 5))
        tk.Label(self.linear_container, text="Vitesse linéaire maximale", font=("Arial", 10), bg="lightblue").pack()
        self.max_linear_speed_slider.pack(pady=(0, 5))
        tk.Label(self.linear_container, text="Epsilon linéaire", font=("Arial", 10), bg="lightblue").pack()
        self.graph_container.pack(fill="x", pady=(10, 0))

        self.waypoints_table_frame.pack(side="left", fill="y", pady=(10, 10), padx=(10, 10))
        tk.Label(self.waypoints_table_frame, text="Waypoints", font=("Arial", 10, "bold"), bg="lightblue").pack()
        self.waypoints_table.pack(side="left", fill="both", expand=True)
        self.epsilon_linear_entry.pack(pady=(0, 20))
        self.canvas.get_tk_widget().pack(side=tk.LEFT, fill=tk.BOTH, padx=(20, 0))

        self.all_entry_container.pack(pady=(10, 10), side="top")
        self.TB_center_coord_container.pack(pady=(10, 0), side="left", padx=(5, 5), fill="x")
        self.TB_center_coord_container_label.pack(pady=(5, 0), side="top")

        self.TB_coord_x_container.pack(padx=(5, 5) ,pady=(5, 5), side="left")
        self.TB_center_x_label.pack(pady=(5, 0), padx=(5, 5), side="top")
        self.TB_center_x_entry.pack(padx=(5, 5), pady=(5, 5), side="top")

        self.TB_coord_y_container.pack(padx=(5, 5), pady=(5, 5), side="left")
        self.TB_center_y_label.pack(pady=(5, 0), padx=(5, 5), side="top")
        self.TB_center_y_entry.pack(padx=(5, 5), pady=(5, 5), side="top")

        self.qualisys_era_container.pack(pady=(10, 0), side="left", padx=(5, 5), fill="x")
        self.qualisys_era_container_label.pack(pady=(5, 0), padx=(5, 5), side="top")
        self.qualisys_era_label_frame.pack(padx=(5,5), pady=(5, 5), side="left")
        self.qualisys_era_label_1.pack(padx=(5, 5), pady=(35, 4), side="top")
        self.qualisys_era_label_2.pack(padx=(5, 5), pady=(4, 4), side="top")
        self.qualisys_era_label_3.pack(padx=(5, 5), pady=(4, 4), side="top")
        self.qualisys_era_label_4.pack(padx=(5, 5), pady=(4, 4), side="top")

        self.qualisys_era_x_frame.pack(padx=(5,5), pady=(5, 5), side="left")
        self.qualisys_era_x_label.pack(pady=(5, 5), padx=(5, 5), side="top")
        self.qualisys_x_entry_1.pack(padx=(5, 5), pady=(5, 5), side="top")
        self.qualisys_x_entry_2.pack(padx=(5, 5), pady=(5, 5), side="top")
        self.qualisys_x_entry_3.pack(padx=(5, 5), pady=(5, 5), side="top")
        self.qualisys_x_entry_4.pack(padx=(5, 5), pady=(5, 5), side="top")

        self.qualisys_era_y_frame.pack(padx=(5, 5), pady=(5, 5), side="left")
        self.qualisys_era_y_label.pack(pady=(5, 0), padx=(5, 5), side="top")
        self.qualisys_y_entry_1.pack(padx=(5, 5), pady=(5, 5), side="top")
        self.qualisys_y_entry_2.pack(padx=(5, 5), pady=(5, 5), side="top")
        self.qualisys_y_entry_3.pack(padx=(5, 5), pady=(5, 5), side="top")
        self.qualisys_y_entry_4.pack(padx=(5, 5), pady=(5, 5), side="top")


        self.button_container_frame.pack(pady=(10, 10), side="top")

        self.button_clear_frame.pack(pady=(10, 10), side="left", padx=(10, 10))
        self.clear_button.pack()

        self.button_update_frame.pack(pady=(10, 10), side="left", padx=(10, 10))
        self.update_button.pack()

        self.button_publish_frame.pack(pady=(10,10), side="left", padx=(10, 10))
        self.button_publish.pack()



interface = Interface()



def main():
    # Affichage de l'interface utilisateur
    interface.window_creation()
    interface.canvas.mpl_connect('motion_notify_event', interface.motion)
    interface.canvas.mpl_connect('button_press_event', interface.trajectory_constructor)

    # Lancement de la boucle principale de l'interface graphique
    interface.root.mainloop()

# Vérification si ce fichier est exécuté en tant que script principal
if __name__ == "__main__":
    main()