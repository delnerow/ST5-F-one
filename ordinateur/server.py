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

import queue
import sys
import threading
import time

import zmq

server_ip = "192.168.137.1"
verbose_mode = False
nodes = list()
#global key # last key hit ?
key = '' 
mission = None          # dernière mission envoyée par control.py
delivered = set()      # robots qui ont déjà reçu cette mission
ui_updates = queue.Queue()
server_stopping = threading.Event()
last_position_log = {}


def main():
    print("Welcome to server.py")
    print("Waiting for nodes to register...")
    import tkinter as tk
    from tkinter import messagebox

    from grid_preview import GridPreview

    root = tk.Tk()
    preview = GridPreview(root)

    def update_preview():
        preview_changed = False
        try:
            while True:
                update = ui_updates.get_nowait()
                kind = update[0]
                if kind == "mission_submitted":
                    preview.hide()
                elif kind == "robot_update":
                    _, robot_id, data = update
                    if data.get("type") == "position":
                        preview.update_position(robot_id, data, redraw=False)
                        preview_changed = True
                    elif data.get("type") == "event":
                        if data.get("event") == "mission_started" and data.get("mission") == "grille":
                            try:
                                preview.show_mission(
                                    data["start"],
                                    data["goal"],
                                    data["grid_size"],
                                    data["heading"],
                                )
                            except (KeyError, TypeError, ValueError) as error:
                                print(f"Invalid grid preview mission data: {error}")
                        else:
                            preview.show_event(data)
                elif kind == "server_error":
                    print(f"Server stopped: {update[1]}", file=sys.stderr)
                    messagebox.showerror("Server error", str(update[1]))
                    close_server()
                    return
        except queue.Empty:
            pass
        if preview_changed:
            preview.refresh()
        if root.winfo_exists():
            root.after(50, update_preview)

    def run_server():
        try:
            serve()
        except Exception as error:
            ui_updates.put(("server_error", error))

    def close_server():
        server_stopping.set()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", close_server)
    threading.Thread(target=run_server, daemon=True).start()
    root.after(50, update_preview)
    root.mainloop()


def serve():
    repsock = create_connection_interface(server_ip)
    try:
        while not server_stopping.is_set():
            if not repsock.poll(100):
                continue
            msg = repsock.recv_pyobj()
            reply = process_msg(msg)
            repsock.send_pyobj(reply)
            if verbose_mode:
                print("received: {}".format(msg))
                print("reply: {}".format(reply))
    finally:
        repsock.close(linger=0)


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
            ui_updates.put(("mission_submitted", mission))
            print("new mission: {}".format(mission))
        reply = default_reply

    elif msg_type["key request from robot"]:
        print("key request from {}".format(msg["from"]))
        for update in msg.get("updates", []):
            update_type = update.get("type")
            if update_type == "position":
                now = time.monotonic()
                if now - last_position_log.get(msg["from"], 0.0) >= 1.0:
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
                    last_position_log[msg["from"]] = now
                ui_updates.put(("robot_update", msg["from"], update))
            elif update_type == "event":
                ui_updates.put(("robot_update", msg["from"], update))
                if update.get("event") == "debug_image_timing":
                    print(
                        "[{}] debug image: capture={:.2f} ms, prétraitement={:.2f} ms, "
                        "ligne={:.2f} ms, intersection={:.2f} ms, traitement total={:.2f} ms".format(
                            msg["from"],
                            update.get("capture_ms", 0.0),
                            update.get("preprocessing_ms", 0.0),
                            update.get("line_detection_ms", 0.0),
                            update.get("intersection_detection_ms", 0.0),
                            update.get("processing_ms", 0.0),
                        )
                    )
                else:
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
