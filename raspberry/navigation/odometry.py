# Distance parcourue en ligne droite depuis la dernière remise à zéro (en mètres).
# Utilise car.GetSpeed() (commande 'P' de l'Arduino) intégré dans le temps.
#
# Si le .ino propose un compteur de ticks (retrCmdl dans hardware/serial_protocol.py), c'est plus précis :
# il suffit alors de réécrire update() en lisant les ticks, le reste du code ne change pas.

import time

import config as cfg


class Odometry:
    def __init__(self, car):
        self.car = car
        self.distance = 0.0
        self._t = time.perf_counter()

    def reset(self):
        self.distance = 0.0
        self._t = time.perf_counter()

    def update(self, active=True):
        """À appeler à chaque tour de boucle. active=False pendant les rotations sur place
        (la distance ne doit pas bouger) : on avance seulement l'horloge."""
        now = time.perf_counter()
        dt, self._t = now - self._t, now
        if not active:
            return self.distance
        try:
            speeds = self.car.GetSpeed()                  # 4 vitesses (une par roue)
        except Exception:                                 # lecture série incomplète : on ignore ce tour
            self.car.serialport.reset_input_buffer()
            return self.distance
        v = sum(speeds) / 4 * cfg.ENCODER_SIGN * cfg.ENCODER_SCALE     # m/s, moyenne des 4 roues
        self.distance += v * dt
        return self.distance
