"""Mission data and validation."""

from dataclasses import dataclass
from typing import Optional, Tuple

import config as cfg
from navigation.grid_map import HEADINGS, step


@dataclass
class Mission:
    kind: str
    start: Optional[Tuple[int, int]] = None
    heading: Optional[str] = None
    goal: Optional[Tuple[int, int]] = None
    grid_size: Tuple[int, int] = (cfg.GRID_W, cfg.GRID_H)
    blocked_nodes: Tuple[Tuple[int, int], ...] = ()
    blocked_edges: Tuple[Tuple[Tuple[int, int], Tuple[int, int]], ...] = ()
    initial_route: Optional[Tuple[Tuple[Tuple[int, int], Optional[str]], ...]] = None

    @classmethod
    def from_dict(cls, d):
        kind = d.get("type")
        if kind in ("huit", "debug"):
            return cls(kind=kind)
        if kind == "grille":
            start = _parse_node(d["start"], "départ")
            goal = _parse_node(d["goal"], "destination")
            heading = str(d["heading"]).upper()
            size = tuple(int(v) for v in d.get("grid_size", (cfg.GRID_W, cfg.GRID_H)))
            if len(size) != 2 or any(value < 1 for value in size):
                raise ValueError(f"taille de grille invalide : {size!r}")
            if heading not in HEADINGS:
                raise ValueError(f"cap inconnu : {heading!r} (attendu : N, E, S ou W)")
            for name, (x, y) in (("départ", start), ("destination", goal)):
                if not (0 <= x < size[0] and 0 <= y < size[1]):
                    raise ValueError(f"{name} {(x, y)} hors de la grille {size}")
            blocked_nodes = tuple(
                _parse_node(node, "obstacle") for node in d.get("blocked_nodes", ())
            )
            for node in blocked_nodes:
                _validate_in_bounds(node, size, "obstacle")

            blocked_edges = []
            for edge in d.get("blocked_edges", ()):
                if not isinstance(edge, (list, tuple)) or len(edge) != 2:
                    raise ValueError(f"segment bloqué invalide : {edge!r}")
                first, second = (_parse_node(node, "segment bloqué") for node in edge)
                _validate_in_bounds(first, size, "segment bloqué")
                _validate_in_bounds(second, size, "segment bloqué")
                if abs(first[0] - second[0]) + abs(first[1] - second[1]) != 1:
                    raise ValueError(f"les extrémités du segment ne sont pas voisines : {edge!r}")
                blocked_edges.append((first, second))

            initial_route = None
            if "initial_route" in d:
                initial_route = _parse_initial_route(d["initial_route"], start, goal, size)
                blocked_node_set = set(blocked_nodes)
                blocked_edge_set = {frozenset(edge) for edge in blocked_edges}
                for (node, route_heading), (next_node, _) in zip(initial_route, initial_route[1:]):
                    if node in blocked_node_set or next_node in blocked_node_set:
                        raise ValueError(f"itinéraire initial traversant un nœud bloqué : {node} -> {next_node}")
                    if frozenset((node, next_node)) in blocked_edge_set:
                        raise ValueError(f"itinéraire initial traversant un segment bloqué : {node} -> {next_node}")
                    if route_heading is None or step(node, route_heading) != next_node:
                        raise ValueError(f"itinéraire initial invalide entre {node} et {next_node}")

            return cls(
                kind="grille",
                start=start,
                heading=heading,
                goal=goal,
                grid_size=size,
                blocked_nodes=blocked_nodes,
                blocked_edges=tuple(blocked_edges),
                initial_route=initial_route,
            )
        raise ValueError(f"type de mission inconnu : {kind!r}")


def _parse_node(value, label):
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise ValueError(f"coordonnées {label} invalides : {value!r}")
    try:
        return int(value[0]), int(value[1])
    except (TypeError, ValueError) as error:
        raise ValueError(f"coordonnées {label} invalides : {value!r}") from error


def _validate_in_bounds(node, size, label):
    if not (0 <= node[0] < size[0] and 0 <= node[1] < size[1]):
        raise ValueError(f"{label} {node} hors de la grille {size}")


def _parse_initial_route(value, start, goal, size):
    if not isinstance(value, (list, tuple)) or not value:
        raise ValueError("itinéraire initial vide ou invalide")

    route = []
    for entry in value:
        if not isinstance(entry, (list, tuple)) or len(entry) != 2:
            raise ValueError(f"étape d'itinéraire invalide : {entry!r}")
        node = _parse_node(entry[0], "itinéraire initial")
        _validate_in_bounds(node, size, "itinéraire initial")
        heading = entry[1]
        if heading is not None:
            heading = str(heading).upper()
            if heading not in HEADINGS:
                raise ValueError(f"cap invalide dans l'itinéraire initial : {heading!r}")
        route.append((node, heading))

    if route[0][0] != start or route[-1] != (goal, None):
        raise ValueError("l'itinéraire initial doit partir du départ et terminer à destination")
    if any(heading is None for _, heading in route[:-1]):
        raise ValueError("seule la dernière étape peut avoir un cap nul")
    return tuple(route)
