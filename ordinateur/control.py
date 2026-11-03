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


def main():
    server_socket = connect_to(server_ip)
    register_msg = {"cmd": "log"}       # add header indicating origin ?
    send_message(server_socket, register_msg)
    

    print("Welcome to control.py")
    print("Press enter to validate your command and send it to the server")
    cmd_char = ''
    while cmd_char != 'q':
        input_str = input("Enter your command (press 'q' to exit): ")
        if input_str != '':
            cmd_char = input_str[0]
            if cmd_char == 'm':             # nouvelle mission
                try:
                    send_message(server_socket, {"cmd": "mission", "mission": ask_mission()})
                except ValueError:
                    print("Saisie invalide, mission non envoyée.")
            elif cmd_char != 'q':
                msg = {"cmd": "key", "key": cmd_char}       # add header indicating origin ?
                send_message(server_socket, msg)


def ask_mission():
    kind = input("Mission (g = grille, h = huit, d = debug caméra) : ").strip().lower()
    if kind.startswith('h'):
        return {"type": "huit"}
    if kind.startswith('d'):
        return {"type": "debug"}
    sx, sy = map(int, input("Origine x y : ").split())
    heading = input("Cap de départ (N/E/S/W) : ").strip().upper()
    gx, gy = map(int, input("Destination x y : ").split())
    return {"type": "grille", "start": [sx, sy], "heading": heading, "goal": [gx, gy]}


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
