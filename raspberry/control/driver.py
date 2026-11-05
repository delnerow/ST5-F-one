# Conduite : suivi de ligne + machine à états des intersections (repris de suivi_grille.py).
# Différence : le Driver ne décide plus de la direction. step() renvoie des événements,
# et c'est la mission (missions/execution.py) qui répond avec choose().
#
# États : SUIVI -> APPROCHE -> ATTENTE -> AVANCE -> (TOURNE) -> SUIVI
#                                                \-> ARRET (si choose('stop'))
#         n'importe quel état -> ARRET (touche d'arrêt ou arrêt de sécurité de l'Arduino)
#
# Événements renvoyés par step() (ou None) :
#   ("INTERSECTION", ['gauche', 'devant', ...])  robot arrêté aprés avoir depassé l'intersection en CAMERA, en attente de choose()
#   ("CENTERED", None)                           robot au centre du carrefour
#   ("TURN_DONE", None)                          rotation terminée, ligne retrouvée
#   ("STOPPED", None)                            arrêt demandé par choose('stop') effectué
#   ("ABORTED", None)                            arrêt d'urgence (voir cause_arret)
#
# Arrêt de sécurité de l'Arduino : l'acquittement d'une commande moteur commence par
# WD (watchdog), OB (obstacle) ou MU (capteur ultrason muet). Le Driver passe alors en
# ARRET de façon verrouillée, mémorise la cause dans cause_arret et lève stop_event,
# même si l'Arduino libère ensuite l'obstacle.

import time

import config as cfg
from hardware.serial_protocol import ArduinoStop,retrCmdi,sendCmdi
from perception.image_processing import traitement, find_line, find_intersection

DIRECTIONS = ['gauche', 'devant', 'droite']


class Driver:
    def __init__(self, car, camera, stop_event=None):
        self.car, self.camera = car, camera
        self.stop_event = stop_event
        self.etat = "SUIVI"
        self.choix = None
        self.sens = None              # 'gauche' ou 'droite' pendant une rotation
        self.quarts = 0               # nombre de quarts de tour restants
        self.debut = 0.0
        self.votes = {d: 0 for d in DIRECTIONS}
        self.nb_images = 0
        self.integrale = 0.0          # erreur accumulée (terme I)
        self.erreur_prec = None       # erreur de l'image précédente (terme D)
        self.t_prec = 0.0
        self.sans_inter = 0
        self.detecter_intersections = True   # False sur le 8 : suivi de ligne pur
        self.cause_arret = None       # 'watchdog', 'obstacle', 'muet' ou None (opérateur)
        self.obstacle = False         # True quand l'Arduino a répondu "OB" à une commande moteurs
        self.obstacle_confirme = False  # obstacle resté plus de ATTENTE_OBSTACLE_S : pas la peine de réattendre
        self.ligne_quittee = False    # demi-tour : la ligne de départ a-t-elle été quittée ?
    @property
    def straight(self):
        """Vrai quand le robot roule droit (l'odométrie n'a de sens que dans ce cas)."""
        return self.etat in ("SUIVI", "APPROCHE", "AVANCE")

    # -- moteurs ---------------------------------------------------------------
    def moteurs(self, gauche, droite):
        if self.stop_event is not None and self.stop_event.is_set():
            gauche = droite = 0
            self.etat = "ARRET"
        g = int(max(-255, min(255, gauche)))
        d = int(max(-255, min(255, droite)))
        # ordre dans serial_link : avant droit, avant gauche, arrière droit, arrière gauche
        try:
            sendCmdi(self.car.serialport, b'C', d, g, d, g)
        except ArduinoStop as e:
            if e.cause == 'obstacle':
                    # L'Arduino bloque la marche avant mais accepte la rotation sur place :
                    # pas d'arrêt définitif, la mission gère (attente puis demi-tour)
                self.obstacle = True
            else:
                self._arret_arduino(e.cause)

    def _arret_arduino(self, cause):
        """Arrêt de sécurité signalé par l'Arduino : verrouille ARRET et prévient la mission."""
        deja_arrete = self.stop_event is not None and self.stop_event.is_set()
        if self.cause_arret is None and not deja_arrete:
            self.cause_arret = cause
            print(f"Arrêt de sécurité Arduino : {cause}")
        self.etat = "ARRET"
        if self.stop_event is not None:
            self.stop_event.set()

    def stop(self):
        self.moteurs(0, 0)

    # -- commandes venant de la mission ---------------------------------------------
    def choose(self, choix):
        """À appeler après un événement INTERSECTION : 'gauche', 'devant', 'droite' ou 'stop'."""
        assert self.etat == "ATTENTE", "choose() hors d'une intersection"
        self.choix = choix
        if choix in ('gauche', 'droite'):
            self.sens = choix
        self.etat, self.debut = "AVANCE", time.perf_counter()    # le temps repart après le choix

    def turn_in_place(self, sens, quarts=1):
        """Pivote sur place (au départ de la mission). quarts=2 pour faire demi-tour."""
        if self.stop_event is not None and self.stop_event.is_set():
            self.stop()
            self.etat = "ARRET"
            return
        self.sens, self.quarts = sens, quarts
        self.etat, self.debut = "TOURNE", time.perf_counter()

    def _abort_if_requested(self):
        if self.stop_event is not None and self.stop_event.is_set():
            if self.etat != "ARRET":
                self.stop()
                self.etat = "ARRET"
            return True
        return False

    def demarrer(self):
        """Coup de démarrage : plus de puissance pendant TEMPS_DEMARRAGE pour vaincre les frottements."""
        self.moteurs(cfg.VITESSE_DEMARRAGE, cfg.VITESSE_DEMARRAGE)
        time.sleep(cfg.TEMPS_DEMARRAGE)

    # -- boucle ------------------------------------------------------------------
    def _suivre_ligne(self, ligne, vitesse):
        if ligne is None:               # ligne perdue : l'Arduino garde la dernière consigne
            return
        cx, _ = ligne
        erreur = (cx - cfg.LARGEUR / 2) / (cfg.LARGEUR / 2)
        if abs(erreur) < cfg.ZONE_MORTE:
            erreur = 0
        elif erreur > 0 :
            erreur = erreur - cfg.ZONE_MORTE/2
        elif erreur < 0 :
            erreur = erreur  + cfg.ZONE_MORTE/2
        correction = cfg.KP * erreur
        self.moteurs(vitesse + correction, vitesse - correction)

    def step(self):
        if self._abort_if_requested():
            return ("ABORTED", None)
        event = self._step()
        # un arrêt de sécurité a pu survenir pendant une commande moteur de ce tour :
        # il l'emporte sur tout événement (et sur un changement d'état) de ce tour
        if self._abort_if_requested():
            return ("ABORTED", None)
        return event

    def _step(self):
        if self.etat in ("ATTENTE", "ARRET"):
            time.sleep(0.02)
            return None

        array = self.camera.capture_array("main")
        if self._abort_if_requested():
            return ("ABORTED", None)
        t = time.perf_counter()
        closed_mask, h = traitement(array)
        ligne = find_line(closed_mask, h)
        if self._abort_if_requested():
            return ("ABORTED", None)

        if self.etat == "SUIVI":
            self._suivre_ligne(ligne, cfg.VITESSE)
            inter = find_intersection(closed_mask, h) if self.detecter_intersections else None
            if inter is not None and inter['distance'] < cfg.DISTANCE_DETECTION:
                self.votes = {d: 0 for d in DIRECTIONS}
                self.nb_images = 0
                self.sans_inter = 0
                self.etat = "APPROCHE"

        elif self.etat == "APPROCHE":
            self._suivre_ligne(ligne, cfg.VITESSE_LENTE)
            inter = find_intersection(closed_mask, h)
            if inter is not None:
                self.nb_images += 1
                for d in self.votes:
                    self.votes[d] += inter[d]
                self.sans_inter = 0
            else:
                self.sans_inter += 1
            if self.sans_inter >= cfg.IMAGES_SANS_INTER:
                branches = [d for d in DIRECTIONS if self.votes[d] > self.nb_images / 2]
                if not branches:
                    self.etat = "SUIVI"
                else:
                    self.stop()
                    self.etat = "ATTENTE"
                    return ("INTERSECTION", branches)

        elif self.etat == "AVANCE":
            self.moteurs(cfg.VITESSE_LENTE, cfg.VITESSE_LENTE)
            if t - self.debut > cfg.TEMPS_FINAL: #temp d'offset heuristic
                if self.choix == 'stop':
                    self.stop()
                    self.etat = "ARRET"
                    return ("STOPPED", None)
                if self.choix == 'devant':
                    self.etat = "SUIVI"
                else:
                    self.etat, self.debut, self.quarts = "TOURNE", t, 1
                return ("CENTERED", None)

        elif self.etat == "TOURNE":
            if self.sens == 'gauche':
                self.moteurs(-cfg.VITESSE_ROTATION, cfg.VITESSE_ROTATION)
            else:
                self.moteurs(cfg.VITESSE_ROTATION, -cfg.VITESSE_ROTATION)
            if t - self.debut > cfg.TEMPS_MIN_ROTATION and ligne is not None: # on ne veut pas identifier la meme ligne que celle qu'on vient de quitter
                cx, _ = ligne
                if abs((cx - cfg.LARGEUR / 2) / (cfg.LARGEUR / 2)) < cfg.ERREUR_ALIGNEE:
                    self.quarts -= 1
                    if self.quarts > 0:
                        self.debut = t          # quart de tour suivant : on quitte d'abord la ligne
                    else:
                        self.integrale, self.erreur_prec = 0.0, None
                        self.etat = "SUIVI"
                        return ("TURN_DONE", None)
        return None
