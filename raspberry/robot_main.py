# Point d'entrée du robot :  python3 robot_main.py [ip_serveur] [id_robot]
#
#   1. connexion au serveur
#   2. attente de la mission (huit, grille ou debug caméra)
#   3. initialisation du matériel nécessaire à la mission
#   4. exécution de la mission
#   5. arrêt propre

import sys

import config as cfg
from communication.server import ServerLink
from hardware.serial_protocol import ArduinoStop


def main():
    ip = sys.argv[1] if len(sys.argv) > 1 else cfg.SERVER_IP
    robot_id = sys.argv[2] if len(sys.argv) > 2 else cfg.ROBOT_ID

    link = ServerLink(ip, robot_id)
    link.register()
    mission = link.wait_for_mission()
    link.start_key_listener()                 # arrêt de mission depuis control.py ('r')

    # imports ici : picamera2 / cv2 ne sont chargés qu'une fois la mission connue
    arduino = camera = None
    try:
        if mission.kind == 'debug':
            from hardware.devices import open_camera
            from missions.debug import run_debug

            camera = open_camera()
            run_debug(camera, link.stop_event, link)
        else:
            from control.driver import Driver
            from hardware.devices import open_arduino
            from missions.execution import run_grille, run_huit
            from navigation.odometry import Odometry

            arduino = open_arduino()
            from hardware.devices import open_camera

            camera = open_camera()
            drv, odo = Driver(arduino, camera, link.stop_event), Odometry(arduino)
            run = run_grille if mission.kind == 'grille' else run_huit
            ok = run(mission, drv, odo, link)
            print("Mission accomplie." if ok else "Mission interrompue.")
            drv.stop()
    except KeyboardInterrupt:
        print("Arrêt demandé")
    except (ArduinoStop, TimeoutError) as e:
        # ArduinoStop est normalement géré par le Driver ; TimeoutError = Arduino qui
        # n'acquitte plus (câble, alimentation) : c'est aussi un arrêt d'urgence.
        cause = getattr(e, "cause", "timeout_serie")
        print(f"Arrêt d'urgence : {cause}")
        link.stop_event.set()
        link.send_event("emergency_stop", cause=cause)
    finally:
        try:
            if arduino is not None:
                try:
                    arduino.Stop()
                except TimeoutError:
                    pass                  # Arduino muet : on tente quand même la déconnexion
                finally:
                    arduino.disconnect()
        finally:
            try:
                if camera is not None:
                    camera.stop()
            finally:
                link.close()


if __name__ == "__main__":
    main()
