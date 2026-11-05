#########################################################################
# CENTRALESUPELEC : ST5 Integration teaching
#
# Human-machine interface and control client
#   This program is used as human-machine interface
#   by sending messages to the robots via the server
#
#########################################################################
# Authors : Philippe Benabes & Koen de Turck
# Modifications by Morgan Roger & Erwan Libessart
# TBD : message content could explicitly include origin 
#########################################################################

import zmq

server_ip = "192.168.137.1"
verbose_mode = False
key_to_exit_program = b'q'
DEFAULT_GRID_SIZE = (4, 4)


def main():
    server_socket = connect_to(server_ip)
    register_msg = {"cmd": "log"}       # add header indicating origin ?
    send_message(server_socket, register_msg)
    

    print("Welcome to control.py")
    print("Press 'r' to abort the current mission, or 'q' to quit.")
    cmd_char = ''
    while cmd_char != 'q':
        input_str = input("Command (m = mission, r = abort mission, q = quit): ")
        if input_str != '':
            cmd_char = input_str[0]
            if cmd_char == 'm':             # nouvelle mission
                try:
                    mission = ask_mission()
                    if mission is not None:
                        reply = send_message(
                            server_socket,
                            {"cmd": "mission", "mission": mission},
                        )
                        if reply and "error" in reply:
                            print(f"Mission refusée par le serveur : {reply['error']}")
                except (ValueError, RuntimeError) as error:
                    print(f"Mission non envoyée : {error}")
            elif cmd_char != 'q':
                msg = {"cmd": "key", "key": cmd_char}       # add header indicating origin ?
                send_message(server_socket, msg)


def ask_mission():
    kind = input("Mission (g = grille, h = huit, d = debug caméra) : ").strip().lower()
    if kind.startswith('h'):
        return {"type": "huit"}
    if kind.startswith('d'):
        return {"type": "debug"}
    grid_size_input = input(
        f"Taille de la grille largeur hauteur [{DEFAULT_GRID_SIZE[0]} {DEFAULT_GRID_SIZE[1]}] : "
    ).strip()
    grid_size = (
        DEFAULT_GRID_SIZE
        if not grid_size_input
        else tuple(int(value) for value in grid_size_input.split())
    )
    if len(grid_size) != 2 or any(value < 1 for value in grid_size):
        raise ValueError("la taille doit contenir deux entiers positifs")
    sx, sy = map(int, input("Origine x y : ").split())
    heading = input("Cap de départ (N/E/S/W) : ").strip().upper()
    gx, gy = map(int, input("Destination x y : ").split())
    if heading not in ("N", "E", "S", "W"):
        raise ValueError("cap initial invalide : utiliser N, E, S ou W")
    for label, (x, y) in (("origine", (sx, sy)), ("destination", (gx, gy))):
        if not (0 <= x < grid_size[0] and 0 <= y < grid_size[1]):
            raise ValueError(f"{label} {(x, y)} hors de la grille {grid_size}")
    mission = {
        "type": "grille",
        "start": [sx, sy],
        "heading": heading,
        "goal": [gx, gy],
        "grid_size": list(grid_size),
        "blocked_nodes": [],
        "blocked_edges": [],
    }
    answer = input("Ajouter des obstacles connus ? (o/n, n par défaut) : ").strip().lower()
    if answer in ("o", "oui", "y", "yes"):
        obstacles = select_obstacles((sx, sy), (gx, gy), grid_size, heading)
        if obstacles is None:
            print("Sélection annulée.")
            return None
        mission.update(obstacles)
    elif answer not in ("", "n", "non", "no"):
        raise ValueError("répondre par oui ou non à la question sur les obstacles")
    return mission


def select_obstacles(start, goal, grid_size, heading):
    import tkinter as tk

    from obstacle_editor import ObstacleEditor

    try:
        root = tk.Tk()
    except tk.TclError as error:
        raise RuntimeError(f"impossible d’ouvrir l’éditeur de carte : {error}") from error
    try:
        editor = ObstacleEditor(root, start, goal, grid_size, heading)
        root.mainloop()
        return editor.result
    except tk.TclError as error:
        raise RuntimeError(f"erreur dans l’éditeur de carte : {error}") from error
    finally:
        root.destroy()


def connect_to(ip):
    ctx = zmq.Context()
    sock = ctx.socket(zmq.REQ)
    address = "tcp://{}:5005".format(ip)
    sock.connect(address)
    
    return sock

def send_message(sock, content):
    msg = {"from": "control"}       # header indicating origin
    msg.update(content)
    sock.send_pyobj(msg)
    reply = sock.recv_pyobj()
    if verbose_mode:
        print(reply)
    
    return reply


if __name__ == "__main__":
    main()
