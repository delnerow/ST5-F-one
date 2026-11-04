#########################################################################
# CENTRALESUPELEC : ST5 Integration teaching
#
# Communication server
#   This program receives messages from all clients (control and robots).
#   It forwards messages from the control to all robots.
#   Each robot filters out messages that are not meant for it.
#
#########################################################################
# Authors : Philippe Benabes & Koen de Turck
# Modifications by Morgan Roger & Erwan Libessart
# TBD : nodes is not declared as global
# TBD : replace 'key' with 'last_key_hit' ?
#########################################################################

import zmq
import sys

server_ip = "192.168.137.1"
verbose_mode = False
nodes = list()
#global key # last key hit ?
key = '' 
mission = None          # dernière mission envoyée par control.py
delivered = set()      # robots qui ont déjà reçu cette mission


def main():
    repsock = create_connection_interface(server_ip)

    print("Welcome to server.py")
    print("Waiting for nodes to register...")
    while True:
        msg = repsock.recv_pyobj()
        reply = process_msg(msg)
        repsock.send_pyobj(reply)
        if verbose_mode:
            print("received: {}".format(msg))
            print("reply: {}".format(reply))


def process_msg(msg):
    global key, mission
    default_reply = {"all": "is fine"}

    msg_type = {
      "new node registering": ((msg["cmd"] == "log") and msg["from"] not in nodes),
      "known node registering": ((msg["cmd"] == "log") and msg["from"] in nodes),
      "from control": (msg["from"] == "control"),
      "key request from robot": ((msg["from"] != "control") and (msg["cmd"] == "key")),
      "mission request from robot": ((msg["from"] != "control") and (msg["cmd"] == "mission"))
               }

    if msg_type["new node registering"]:
        print("new node signing in, adding {} to nodes".format(msg["from"]))
        nodes.append(msg["from"])
        if verbose_mode:
            print(nodes)
        reply = default_reply

    elif msg_type["known node registering"]:
        print("node {} already known".format(msg["from"]))
        if verbose_mode:
            print(nodes)
        reply = {"already": "registered"}


    elif msg_type["from control"]:
        print("from control")
        if verbose_mode:
            print(msg)
        if (msg["cmd"] == "key"): 
            key = msg["key"]
            print(key)
        if (msg["cmd"] == "mission"):
            mission = msg["mission"]
            delivered.clear()
            print("new mission: {}".format(mission))
        reply = default_reply

    elif msg_type["key request from robot"]:
        print("key request from {}".format(msg["from"]))
        for update in msg.get("updates", []):
            update_type = update.get("type")
            if update_type == "position":
                print(
                    "[{}] position: ({:.2f}, {:.2f}) node={} cap={} "
                    "distance={:.2f} m confiance={}".format(
                        msg["from"],
                        update.get("x", 0.0),
                        update.get("y", 0.0),
                        update.get("candidate_node", update.get("node")),
                        update.get("heading", "?"),
                        update.get("distance_m", 0.0),
                        update.get("confidence", "?"),
                    )
                )
            elif update_type == "event":
                print("[{}] événement: {}".format(msg["from"], update))
        reply = {"key": key}
        key = ''

    elif msg_type["mission request from robot"]:
        if mission is not None and msg["from"] not in delivered:
            delivered.add(msg["from"])
            print("mission sent to {}".format(msg["from"]))
            reply = {"mission": mission}
        else:
            reply = {"mission": None}

    else:
        print("invalid message")
        reply = {"message": "is invalid"}

    if verbose_mode:
        print(reply)
    return reply

def create_connection_interface(ip):
    ctx = zmq.Context()
    repsock = ctx.socket(zmq.REP)
    repaddr = "tcp://{}:5005".format(ip)
    repsock.bind(repaddr)
    
    return repsock


if __name__ == "__main__":
    main()
