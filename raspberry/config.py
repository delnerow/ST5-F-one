# Tous les paramètres du robot au même endroit.
# Les valeurs marquées (À CALIBRER) doivent être mesurées sur la vraie piste.

# ---------------------------------------------------------------- réseau
SERVER_IP = "192.168.137.1"      # surchargé par argv[1]
ROBOT_ID = "bot001"              # surchargé par argv[2]
SERVER_PORT = 5005
REQUEST_TIMEOUT_MS = 2000        # pas de réponse du serveur -> on recrée la socket
MISSION_POLL_S = 1.0             # fréquence de demande de mission
KEY_POLL_S = 0.5                 # fréquence de lecture des touches pendant la mission
STOP_KEYS = ('q', 's', 'r')      # touches d'arrêt reçues depuis control.py

# ---------------------------------------------------------------- matériel
ARDUINO_PORT = '/dev/ttyACM0'
LARGEUR, HAUTEUR = 320, 240
CAMERA_HFLIP, CAMERA_VFLIP = 1, 1

# ---------------------------------------------------------------- suivi de ligne

VITESSE = 100                     # consigne de vitesse, entre -255 et 255
KP = 50
VITESSE_DEMARRAGE = 100          # (À RÉGLER) consigne pendant le coup de démarrage
                                
TEMPS_DEMARRAGE = 0.5            # s
ZONE_MORTE = 0.08                         

# ---------------------------------------------------------------- intersections
VITESSE_LENTE = 45
VITESSE_ROTATION = 120
DISTANCE_DETECTION = 1
TEMPS_FINAL = 1
TEMPS_ROTATION = {              # secondes (minimum, maximum) à calibrer à VITESSE_ROTATION
    "gauche": {
        90: (0.5, 1.0),
        180: (1.0, 2.0),
    },
    "droite": {
        90: (0.5, 1.0),
        180: (1.0, 2.0),
    },
}
VITESSE_BALAYAGE_ROTATION = 45  # consigne de rotation après le temps maximum, jusqu'à retrouver la ligne
IMAGES_ALIGNEES_ROTATION = 3    # nombre d'images centrées consécutives avant de finir le virage
ERREUR_ALIGNEE = 0.15
IMAGES_SANS_INTER = 10

# ---------------------------------------------------------------- obstacle : attente puis demi-tour
ATTENTE_OBSTACLE_S = 3.0         # on attend jusqu'à 3 s que l'obstacle parte avant de faire demi-tour
MESURES_LIBRES = 3               # obstacle parti si l'Arduino répond "libre" (commande S) 3 fois de suite
ERREUR_LIGNE_QUITTEE = 0.5       # demi-tour : ligne de départ quittée quand |erreur| > 0.5 (ou plus vue)
TEMPS_MAX_DEMI_TOUR = 10.0       # s : si le demi-tour n'a pas retrouvé de ligne, on abandonne
DISTANCE_PROCHE_INTERSECTION = 0.25  # (À CALIBRER) m : obstacle à moins de 0.25 m du dernier carrefour
# ---------------------------------------------------------------- grille
GRID_W, GRID_H = 4, 4            # nombre d'intersections en x et en y (la mission peut l'écraser)
CELL_M = 1.0                     # (À CALIBRER) distance entre deux intersections voisines, en m

# ---------------------------------------------------------------- odométrie / localisation
# GetSpeed() renvoie des ticks/s (front descendant de la voie B : 11 ticks/tr moteur,
# réducteur 1:21,3 -> 234,3 ticks/tr roue). Roue Ø65 mm : pi x 0,065 / 234,3 = 8,72e-4 m/tick.
ENCODER_SCALE = 8.72e-4          # (À CALIBRER) m/tick : 3 m poussé à la main / nb moyen de ticks
ROUES_HS = ("FL",)              # roues à encodeur HS (cf. copieDe dans parameters.h) : ignorées par calibration.py
ENCODER_SIGN = 1                 # signe déjà corrigé par encSign dans parameters.h de l'Arduino
APPROACH_OFFSET_M = 0.10         # (À CALIBRER) distance restant à parcourir entre l'événement
                                 # "intersection vue" et le centre du carrefour (phase AVANCE)
MAX_K_ERROR = 0.7                # écart max (en cases) entre l'encodeur et un nœud candidat

# ---------------------------------------------------------------- obstacles (ultrasons)
OBSTACLE_MIN_DISTANCE_CM = 25
OBSTACLE_MAX_DISTANCE_CM = 200
OBSTACLE_CONFIRMATION_MEASURES = 10
OBSTACLE_FIRST_INTERSECTION_MAX_CM = 140
OBSTACLE_SECOND_INTERSECTION_MIN_CM = 160
LOG_PERIOD_S = 1.0
ARDUINO_OBSTACLE_STOP_CM = 25    # (À RÉGLER) arrêt d'urgence géré par l'Arduino (commande D).
                                 # 0 = désactivé. Activé, un capteur ultrason muet bloque
                                 # aussi l'avance (cause 'muet').
