# Ouverture de l'Arduino et de la caméra.

import os
os.environ["LIBCAMERA_LOG_LEVELS"] = "3"       # avant l'import de picamera2

from picamera2 import Picamera2
from libcamera import Transform

import config as cfg
from hardware.arduino import arduino as ard


def open_arduino():
    arduino = ard(cfg.ARDUINO_PORT)
    if not arduino.connect():
        raise RuntimeError("L'Arduino ne répond pas")
    # la protection est remise à 0 à chaque connexion : on la réactive ici
    arduino.SetObstacleProtection(cfg.ARDUINO_OBSTACLE_STOP_CM)
    return arduino


def open_camera():
    camera = Picamera2()
    config = camera.create_preview_configuration(
        {"size": (cfg.LARGEUR, cfg.HAUTEUR), "format": 'RGB888'},
        transform=Transform(hflip=cfg.CAMERA_HFLIP, vflip=cfg.CAMERA_VFLIP), buffer_count=1)
    camera.configure(config)
    camera.start()
    return camera
