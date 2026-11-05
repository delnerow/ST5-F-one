# Exécution des missions. Chaque fonction renvoie True si la mission est accomplie.

import time

import config as cfg
from navigation.grid_map import HEADINGS, GridMap, heading_after, left_of, relative, right_of, step
from navigation.localization import Localizer
from navigation.path_planning import plan_path


def _wait_event(drv, link, wanted):
    """Fait tourner le driver jusqu'à l'événement voulu (ou jusqu'à l'arrêt d'urgence)."""
    while not link.stop_event.is_set():
        ev = _step_and_report(drv, link)
        if ev and ev[0] == wanted:
            return True
    return False


def _step_and_report(drv, link):
    previous_state = drv.etat
    event = drv.step()
    _report_state_transition(drv, link, previous_state)
    return event


def _report_state_transition(drv, link, previous_state):
    if drv.etat != previous_state:
        link.send_event(
            "driver_state",
            previous_state=previous_state,
            state=drv.etat,
        )


def _choose_and_report(drv, link, choice):
    previous_state = drv.etat
    drv.choose(choice)
    _report_state_transition(drv, link, previous_state)
    
def _attendre_obstacle(drv, link):
    """Robot arrêté devant un obstacle : attend jusqu'à ATTENTE_OBSTACLE_S qu'il parte.
    Renvoie True si l'Arduino répond "libre" MESURES_LIBRES fois de suite."""
    debut, libres = time.perf_counter(), 0
    while time.perf_counter() - debut < cfg.ATTENTE_OBSTACLE_S and not link.stop_event.is_set():
        _, etat = drv.etat_ultrason()
        libres = libres + 1 if etat == 0 else 0
        if libres >= cfg.MESURES_LIBRES:
            return True
        time.sleep(0.1)
    return False
    
def _urgence(drv, link):
          """Obstacle signalé par l'Arduino : le robot attend jusqu'à ATTENTE_OBSTACLE_S ;
      s'il part, le robot reprend, sinon il fait demi-tour."""
    drv.stop()
    print("ARRÊT (obstacle)")
    link.send_event("emergency_stop", state=drv.etat, cause="obstacle")
    debut_pause = time.perf_counter()
    libre = False if drv.obstacle_confirme else _attendre_obstacle(drv, link)
    if link.stop_event.is_set():
        return "echec"
    if libre:
        drv.obstacle = False
        drv.debut += time.perf_counter() - debut_pause   # une manœuvre en cours reprend là où elle en était
        print("  obstacle parti : reprise")
        link.send_event("obstacle_cleared")
        return "reprise"
    print("  obstacle toujours là")
    drv.obstacle_confirme = True
    if drv.etat not in ("SUIVI", "APPROCHE"):
        if drv.etat == "AVANCE":
            drv.debut = -1e9          # l'avance vers le centre du carrefour se termine tout de suite
            print("  au carrefour : l'avance vers le centre s'arrête ici")
        else:
            drv.debut += time.perf_counter() - debut_pause
            print("  à une intersection : pas de demi-tour, la manœuvre reprend")
        drv.obstacle = False
        return "manoeuvre"
    previous_state = drv.etat
    drv.obstacle_confirme = False
    drv.demi_tour()
    _report_state_transition(drv, link, previous_state)
    while not link.stop_event.is_set():
        ev = _step_and_report(drv, link)
        if ev and ev[0] == "UTURN_DONE":
            drv.obstacle = False      # les "OB" reçus pendant la rotation ne comptent pas
            print("  demi-tour terminé, reprise")
            link.send_event("uturn_completed")
            return "demi_tour"
        if ev and ev[0] == "UTURN_FAILED":
            print("  demi-tour impossible : ligne non retrouvée")
            link.send_event("uturn_failed")
            return "echec"
    return "echec"

def _interrupt(drv, link, mission):
    """Arrête les moteurs et signale l'interruption avec sa cause."""
    drv.stop()
    cause = drv.cause_arret or "operateur"     # 'watchdog', 'obstacle', 'muet' ou 'operateur'
    print(f"Mission interrompue ({cause}).")
    link.send_event("mission_interrupted", mission=mission, cause=cause)


def _plan_from(grid, loc, goal):
    return plan_path(grid, loc.candidate_node, loc.heading, goal, allow_uturn=False)


def _decide(grid, loc, branches, goal, route, route_index, route_revision):
    """Suit le plan courant; recalcule seulement si la carte change ou si le plan diverge."""
    for relative_heading in ('gauche', 'devant', 'droite'):
        absolute_heading = heading_after(loc.heading, relative_heading)
        if relative_heading in branches:
            grid.add_branch(
                loc.candidate_node,
                absolute_heading,
            )
        else:
            grid.remove_branch(
                loc.candidate_node,
                absolute_heading,
            )

    if route_revision != grid.revision:
        route = _plan_from(grid, loc, goal)
        route_index = 0
        route_revision = grid.revision
    else:
        next_index = next(
            (i for i in range(route_index + 1, len(route))
             if route[i][0] == loc.candidate_node),
            None,
        )
        if next_index is None:
            # La localisation a quitté le chemin mémorisé : repartir de l'état estimé.
            route = _plan_from(grid, loc, goal)
            route_index = 0
            route_revision = grid.revision
        else:
            route_index = next_index

    out_h = route[route_index][1]
    if out_h is None:
        return None, route, route_index, route_revision
    rel = relative(loc.heading, out_h)
    if rel not in branches:
        print(f"  la branche planifiée {out_h} n'est pas visible après mise à jour de la carte")
        return None, route, route_index, route_revision
    return rel, route, route_index, route_revision


def run_grille(mission, drv, odo, link):
    grid = GridMap(*mission.grid_size)
    loc = Localizer(grid, mission.start, mission.heading)
    goal = mission.goal
    link.send_event(
        "mission_started",
        mission="grille",
        start=mission.start,
        heading=mission.heading,
        goal=goal,
        grid_size=mission.grid_size,
    )
    link.send_event("driver_state", state=drv.etat)
    link.send_position(
        x=float(loc.node[0]),
        y=float(loc.node[1]),
        node=loc.node,
        heading=loc.heading,
        distance_m=0.0,
        confidence=loc.confidence,
    )
    if loc.node == goal:
        print("Déjà à destination.")
        link.send_event("mission_completed", mission="grille", result="already_at_goal")
        return True

    route = plan_path(grid, loc.node, loc.heading, goal)    # ValueError si impossible
    route_index = 0
    route_revision = grid.revision
    print("Itinéraire :", " -> ".join(f"{n}" for n, _ in route))

    # 1. Orientation initiale : pivoter si le premier mouvement n'est pas "devant"
    first_h = route[0][1]
    rel = relative(loc.heading, first_h)
    if rel != 'devant':
        if rel == 'derriere':
            previous_state = drv.etat
            drv.turn_in_place('droite', 2)
        else:
            previous_state = drv.etat
            drv.turn_in_place(rel, 1)
        _report_state_transition(drv, link, previous_state)
        if not _wait_event(drv, link, "TURN_DONE"):
            _interrupt(drv, link, "grille")
            return False
        loc.heading = first_h
        link.send_event("heading_changed", heading=loc.heading)
    odo.reset()

    # 2. Boucle principale
    last_log = 0.0
    bloc_en_attente = None          # route à bloquer après un demi-tour (appliqué au prochain centre)
    while not link.stop_event.is_set():
        if drv.obstacle:
            if _urgence(drv, link) == "echec":
                 _interrupt(drv, link, "grille")
                return False
            odo.update(False)       # le temps passé à l'arrêt ne compte pas comme distance
            continue
        event = _step_and_report(drv, link)
        odo.update(drv.straight)

        now = time.perf_counter()
        x, y = loc.estimate(odo.distance)
        link.send_position(
            x=x,
            y=y,
            node=loc.node,
            candidate_node=loc.candidate_node,
            heading=loc.heading,
            distance_m=odo.distance,
            confidence=loc.confidence,
        )
        if now - last_log > cfg.LOG_PERIOD_S:
            print(f"pos ~ ({x:.2f}, {y:.2f})  cap {loc.heading}  dist {odo.distance:.2f} m  conf {loc.confidence}")
            last_log = now

        if event is None:
            continue
        kind, data = event

        if kind == "INTERSECTION":
            fix = loc.seen_intersection(data, odo.distance)
            link.send_event(
                "INTERSECTION",
                node=fix.node,
                branches=sorted(fix.observed),
                matched=fix.matched,
                skipped=fix.skipped,
            )
            flag = "OK" if fix.matched else "DOUTE"
            print(f"Intersection {fix.node} [{flag}] vu={sorted(fix.observed)} prévu={sorted(fix.expected)}"
                  f"  (cases sautées : {fix.skipped})")
            if loc.candidate_node == goal:
                print("Destination atteinte.")
                _choose_and_report(drv, link, 'stop')       # avance au centre puis s'arrête
                continue
            try:
                choix, route, route_index, route_revision = _decide(
                    grid, loc, data, goal, route, route_index, route_revision
                )
            except ValueError:
                choix = None
            if choix is None:
                print("Aucun itinéraire possible depuis ici : arrêt.")
                _choose_and_report(drv, link, 'stop')
                continue
            print(f"  -> {choix}")
            loc.on_turn(choix)
            link.send_event("direction_chosen", direction=choix, heading=loc.heading)
            _choose_and_report(drv, link, choix)

        elif kind == "CENTERED":
            loc.centered_on_intersection()
            odo.reset()                                     # centre du carrefour : distance = 0
            link.send_position(
                x=float(loc.node[0]),
                y=float(loc.node[1]),
                node=loc.node,
                candidate_node=loc.candidate_node,
                heading=loc.heading,
                distance_m=0.0,
                confidence=loc.confidence,
            )
            link.send_event(
                "localization_confirmed",
                node=loc.node,
                heading=loc.heading,
                confidence=loc.confidence,
            )

        elif kind == "TURN_DONE":
            odo.reset()                                     # centre du carrefour : distance = 0
            link.send_event("turn_completed", heading=loc.heading)

        elif kind == "STOPPED":
            loc.centered_on_intersection()
            completed = loc.node == goal
            link.send_event(
                "mission_completed" if completed else "mission_stopped",
                mission="grille",
                node=loc.node,
                goal=goal,
            )
            return completed

    _interrupt(drv, link, "grille")
    return False


def run_huit(mission, drv, odo, link):
    """Parcours en 8 : on suit la ligne et on traverse le croisement tout droit."""
    drv.detecter_intersections = False      # sur le 8 : pas de croisement à gérer, suivi de ligne pur
    link.send_event("mission_started", mission="huit")
    link.send_event("driver_state", state=drv.etat)
    drv.demarrer()
    if link.urgence_event.is_set() or drv.obstacle:
        if _urgence(drv, link) == "echec":
            break
        continue
    while not link.stop_event.is_set():
        event = _step_and_report(drv, link)
        if event and event[0] == "INTERSECTION":
            branches = event[1]
            link.send_event("INTERSECTION", branches=branches)
            _choose_and_report(
                drv, link, 'devant' if 'devant' in branches else branches[0]
            )
    _interrupt(drv, link, "huit")
    return drv.cause_arret is None          # arrêt opérateur = fin normale du 8
