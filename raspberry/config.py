# Tous les paramètres du robot au même endroit.
# Les valeurs marquées (À CALIBRER) doivent être mesurées sur la vraie piste.

# ---------------------------------------------------------------- réseau
SERVER_IP = "192.168.0.101"      # surchargé par argv[1]
ROBOT_ID = "bot001"              # surchargé par argv[2]
SERVER_PORT = 5005
REQUEST_TIMEOUT_MS = 2000        # pas de réponse du serveur -> on recrée la socket
MISSION_POLL_S = 1.0             # fréquence de demande de mission
KEY_POLL_S = 0.5                 # fréquence de lecture des touches pendant la mission
STOP_KEYS = ('q', 's')           # touche envoyée par control.py = arrêt d'urgence
DEBUG_CAPTURE_INTERVAL_S = 2.0   # période entre deux captures en mode debug

# ---------------------------------------------------------------- matériel
ARDUINO_PORT = '/dev/ttyACM0'
LARGEUR, HAUTEUR = 320, 240
CAMERA_HFLIP, CAMERA_VFLIP = 1, 1

# ---------------------------------------------------------------- suivi de ligne
VITESSE = 70                     # PWM, entre -255 et 255
KP = 35
ZONE_MORTE = 0.08

# ---------------------------------------------------------------- intersections
VITESSE_LENTE = 45
VITESSE_ROTATION = 40
DISTANCE_DETECTION = 1
TEMPS_FINAL = 0.5
TEMPS_MIN_ROTATION = 0.5
ERREUR_ALIGNEE = 0.15
IMAGES_SANS_INTER = 10
# ---------------------------------------------------------------- grille
GRID_W, GRID_H = 4, 4            # nombre d'intersections en x et en y (la mission peut l'écraser)
CELL_M = 1.0                     # (À CALIBRER) distance entre deux intersections voisines, en m

# ---------------------------------------------------------------- odométrie / localisation
ENCODER_SCALE = 1.0              # (À CALIBRER) unité de GetSpeed() -> m/s
ENCODER_SIGN = 1                 # mettre -1 si GetSpeed() est négatif en marche avant
APPROACH_OFFSET_M = 0.10         # (À CALIBRER) distance restant à parcourir entre l'événement
                                 # "intersection vue" et le centre du carrefour (phase AVANCE)
MAX_K_ERROR = 0.7                # écart max (en cases) entre l'encodeur et un nœud candidat
LOG_PERIOD_S = 1.0
