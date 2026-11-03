"""Calcul d'itinéraires sur une GridMap."""

from collections import deque

from navigation.grid_map import HEADINGS, left_of, right_of, step, turns_between


def plan_path(grid, start, heading, goal, allow_uturn=True):
    """Renvoie un plus court itinéraire sous forme de (nœud, cap de départ).

    Le dernier élément est (goal, None). Les égalités favorisent de continuer
    tout droit, puis de tourner à gauche ou à droite.
    """
    if start == goal:
        return [(start, None)]

    initial_headings = (heading, left_of(heading), right_of(heading))
    if allow_uturn:
        initial_headings += (HEADINGS[(HEADINGS.index(heading) + 2) % 4],)

    queue = deque()
    previous = {}
    for initial_heading in initial_headings:
        state = (start, initial_heading)
        if grid.can_move(start, initial_heading) and state not in previous:
            previous[state] = None
            queue.append(state)

    while queue:
        node, current_heading = queue.popleft()
        next_node = step(node, current_heading)
        if next_node == goal:
            route = []
            state = (node, current_heading)
            while state is not None:
                route.append(state)
                state = previous[state]
            route.reverse()
            route.append((goal, None))
            return route

        for next_heading in (current_heading, left_of(current_heading), right_of(current_heading)):
            if not grid.can_move(next_node, next_heading):
                continue
            next_state = (next_node, next_heading)
            if next_state not in previous:
                previous[next_state] = (node, current_heading)
                queue.append(next_state)

    raise ValueError(f"aucun itinéraire de {start} vers {goal}")
