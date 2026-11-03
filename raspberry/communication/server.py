# Communication avec le serveur (ZMQ REQ/REP, comme robot.py) + description de la mission.
#
# Protocole ajouté :
#   control -> serveur : {"cmd": "mission", "mission": {...}}
#   robot   -> serveur : {"cmd": "mission"}  ->  {"mission": {...}} ou {"mission": None}
#
# Missions :
#   {"type": "huit"}
#   {"type": "grille", "start": [x, y], "heading": "N", "goal": [x, y], "grid_size": [w, h]}   (grid_size optionnel)

import threading
import time

import zmq

import config as cfg
from missions.model import Mission


class ServerLink:
    def __init__(self, ip, robot_id):
        self.ip, self.id = ip, robot_id
        self.ctx = zmq.Context()
        self.sock = None
        self.stop_event = threading.Event()     # levé quand control.py envoie une touche d'arrêt
        self._closing = threading.Event()
        self._thread = None
        self._open()

    # -- bas niveau --------------------------------------------------------
    def _open(self):
        if self.sock is not None:
            self.sock.close(linger=0)
        self.sock = self.ctx.socket(zmq.REQ)
        self.sock.connect(f"tcp://{self.ip}:{cfg.SERVER_PORT}")

    def request(self, content):
        """Envoie un message, renvoie la réponse, ou None si le serveur ne répond pas."""
        msg = {"from": self.id}
        msg.update(content)
        self.sock.send_pyobj(msg)
        if self.sock.poll(cfg.REQUEST_TIMEOUT_MS):
            return self.sock.recv_pyobj()
        self._open()          # une socket REQ qui attend une réponse est bloquée : on la recrée
        return None

    # -- étapes du démarrage -------------------------------------------------
    def register(self):
        print(f"Connexion au serveur {self.ip}:{cfg.SERVER_PORT} comme {self.id} ...")
        while self.request({"cmd": "log"}) is None:
            print("  serveur injoignable, nouvel essai")
            time.sleep(1)
        print("Enregistré auprès du serveur.")

    def wait_for_mission(self):
        print("En attente d'une mission ...")
        while True:
            reply = self.request({"cmd": "mission"})
            if reply and reply.get("mission"):
                try:
                    mission = Mission.from_dict(reply["mission"])
                    print("Mission reçue :", mission)
                    return mission
                except (KeyError, ValueError, TypeError) as e:
                    print("Mission invalide, ignorée :", e)
            time.sleep(cfg.MISSION_POLL_S)

    # -- pendant la mission --------------------------------------------------
    def start_key_listener(self):
        """Thread qui lit les touches de control.py. Après cet appel, seul ce thread
        utilise la socket (une socket ZMQ ne se partage pas entre threads)."""
        def loop():
            while not self._closing.is_set():
                reply = self.request({"cmd": "key"})
                if reply and reply.get("key") in cfg.STOP_KEYS:
                    print("Touche d'arrêt reçue.")
                    self.stop_event.set()
                self._closing.wait(cfg.KEY_POLL_S)
        self._thread = threading.Thread(target=loop, daemon=True)
        self._thread.start()

    def close(self):
        self._closing.set()
        if self._thread is not None:
            self._thread.join(timeout=cfg.REQUEST_TIMEOUT_MS / 1000 + 1)
        self.sock.close(linger=0)
        self.ctx.term()
