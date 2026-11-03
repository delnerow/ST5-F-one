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

    @classmethod
    def from_dict(cls, d):
        kind = d.get("type")
        if kind == "huit":
            return cls(kind="huit")
        if kind == "grille":
            start = tuple(int(v) for v in d["start"])
            goal = tuple(int(v) for v in d["goal"])
            heading = str(d["heading"]).upper()
            size = tuple(int(v) for v in d.get("grid_size", (cfg.GRID_W, cfg.GRID_H)))
            if heading not in HEADINGS:
                raise ValueError(f"cap inconnu : {heading!r} (attendu : N, E, S ou W)")
            for name, (x, y) in (("départ", start), ("destination", goal)):
                if not (0 <= x < size[0] and 0 <= y < size[1]):
                    raise ValueError(f"{name} {(x, y)} hors de la grille {size}")
            return cls(kind="grille", start=start, heading=heading, goal=goal, grid_size=size)
        raise ValueError(f"type de mission inconnu : {kind!r}")
