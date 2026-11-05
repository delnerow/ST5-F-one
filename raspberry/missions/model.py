"""Mission data and validation."""

from dataclasses import dataclass
from typing import Optional, Tuple

import config as cfg
from navigation.grid_map import HEADINGS


@dataclass
class Mission:
    kind: str
    start: Optional[Tuple[int, int]] = None
    heading: Optional[str] = None
    goal: Optional[Tuple[int, int]] = None
    grid_size: Tuple[int, int] = (cfg.GRID_W, cfg.GRID_H)
    blocked_nodes: Tuple[Tuple[int, int], ...] = ()
    blocked_edges: Tuple[Tuple[Tuple[int, int], Tuple[int, int]], ...] = ()

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

            return cls(
                kind="grille",
                start=start,
                heading=heading,
                goal=goal,
                grid_size=size,
                blocked_nodes=blocked_nodes,
                blocked_edges=tuple(blocked_edges),
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
