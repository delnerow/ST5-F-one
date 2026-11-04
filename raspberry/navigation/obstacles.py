"""Ultrasonic obstacle confirmation and grid invalidation."""

from collections import namedtuple

import config as cfg
from navigation.grid_map import HEADINGS, step

ObstacleEvent = namedtuple(
    "ObstacleEvent",
    "target obstacle_distance_cm encoder_distance_cm total_distance_cm",
)


class ObstacleTracker:
    """Confirm an obstacle from consecutive valid ultrasonic measurements."""

    CLEAR = "CLEAR"
    CONFIRMING = "CONFIRMING"
    ACTIVE = "ACTIVE"

    def __init__(self):
        self.state = self.CLEAR
        self._measurements = 0

    def update(self, obstacle_distance_cm, encoder_distance_m):
        """Return a single classified event after five consecutive valid samples.

        Pass None when no obstacle is detected. Encoder distance is measured from
        the last confirmed intersection and is expressed in meters.
        """
        if obstacle_distance_cm is None:
            self._reset()
            return None

        if not cfg.OBSTACLE_MIN_DISTANCE_CM < obstacle_distance_cm < cfg.OBSTACLE_MAX_DISTANCE_CM:
            self._reset()
            return None
        if encoder_distance_m < 0:
            raise ValueError("encoder_distance_m must be non-negative")
        if self.state == self.ACTIVE:
            return None

        self._measurements += 1
        self.state = self.CONFIRMING
        if self._measurements < cfg.OBSTACLE_CONFIRMATION_MEASURES:
            return None

        self.state = self.ACTIVE
        encoder_distance_cm = encoder_distance_m * 100
        total_distance_cm = encoder_distance_cm + obstacle_distance_cm
        if total_distance_cm < cfg.OBSTACLE_FIRST_INTERSECTION_MAX_CM:
            target = "first_intersection"
        elif total_distance_cm > cfg.OBSTACLE_SECOND_INTERSECTION_MIN_CM:
            target = "second_intersection"
        else:
            target = "connecting_segment"

        return ObstacleEvent(
            target,
            obstacle_distance_cm,
            encoder_distance_cm,
            total_distance_cm,
        )

    def _reset(self):
        self.state = self.CLEAR
        self._measurements = 0


def apply_obstacle_event(grid, origin, heading, event):
    """Block the confirmed target without modifying intersection topology.

    Returns the physical edges newly blocked. ``origin`` is the last centered
    intersection, and ``heading`` is the robot's current grid heading.
    """
    if heading not in HEADINGS:
        raise ValueError(f"unknown heading: {heading!r}")

    first = step(origin, heading)
    if event.target == "first_intersection":
        if not grid.in_bounds(first):
            return ()
        return grid.block_intersection(first)

    second = step(origin, heading, 2)
    if event.target == "second_intersection":
        if not grid.in_bounds(second):
            return ()
        return grid.block_intersection(second)

    if event.target == "connecting_segment":
        if not grid.in_bounds(first) or not grid.in_bounds(second):
            return ()
        edge = frozenset((first, second))
        return (edge,) if grid.block(first, heading) else ()

    raise ValueError(f"unknown obstacle target: {event.target!r}")
