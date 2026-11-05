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
from queue import Empty, Queue

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
        self._updates = Queue()
        self._update_id = 0
        self._update_id_lock = threading.Lock()
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
            pending_updates = []
            while True:
                # lu AVANT de vider la file : les messages mis en file juste avant
                # close() (cause d'un arrêt d'urgence, fin de mission) partent encore
                closing = self._closing.is_set()
                while True:
                    try:
                        pending_updates.append(self._updates.get_nowait())
                    except Empty:
                        break

                reply = self.request({"cmd": "key", "updates": pending_updates})
                if reply is not None:
                    pending_updates.clear()
                if reply and reply.get("key") in cfg.STOP_KEYS:
                    print("Touche d'arrêt reçue.")
                    self.stop_event.set()
                if closing:
                    break
                self._closing.wait(cfg.KEY_POLL_S)
        self._thread = threading.Thread(target=loop, daemon=True)
        self._thread.start()

    def send_position(self, **position):
        """Queue a position update for transmission by the socket-owning listener thread."""
        self._queue_update({"type": "position", "timestamp": time.time(), **position})

    def send_event(self, event, **details):
        """Queue a mission/state event for transmission by the listener thread."""
        self._queue_update({
            "type": "event",
            "timestamp": time.time(),
            "event": event,
            **details,
        })

    def _queue_update(self, update):
        with self._update_id_lock:
            self._update_id += 1
            update["id"] = self._update_id
        self._updates.put(update)

    def close(self):
        self._closing.set()
        if self._thread is not None:
            # un dernier envoi (au plus REQUEST_TIMEOUT_MS) peut être en cours
            self._thread.join(timeout=2 * cfg.REQUEST_TIMEOUT_MS / 1000 + 1)
        self.sock.close(linger=0)
        self.ctx.term()
