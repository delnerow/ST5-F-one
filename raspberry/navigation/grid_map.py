# Carte de la grille : nœuds = intersections (x, y), N = +y, E = +x.
# Les coordonnées des nœuds sont des tuples (x, y).

HEADINGS = ('N', 'E', 'S', 'W')
DELTA = {'N': (0, 1), 'E': (1, 0), 'S': (0, -1), 'W': (-1, 0)}


def left_of(h): #Quel direction est à gauche du voiture?
    return HEADINGS[(HEADINGS.index(h) - 1) % 4]


def right_of(h): #Quel direction est à droite du voiture?
    return HEADINGS[(HEADINGS.index(h) + 1) % 4]


def turns_between(a, b): # Nombre de tours à effectuer de a à b
    d = (HEADINGS.index(b) - HEADINGS.index(a)) % 4
    return min(d, 4 - d)


def relative(heading, target): # Quel direction absolu est le target vu depuis un robot orienté vers heading
    """Cap absolu 'target' vu depuis un robot orienté vers 'heading'."""
    return ('devant', 'droite', 'derriere', 'gauche')[(HEADINGS.index(target) - HEADINGS.index(heading)) % 4]


def heading_after(heading, rel): # Quel cap absolu est vu depuis un robot orienté vers heading
    return {'devant': heading, 'gauche': left_of(heading), 'droite': right_of(heading)}[rel]


def step(node, heading, k=1): # Calculer la position suivante d'un nœud dans une direction donnée
    dx, dy = DELTA[heading]
    return (node[0] + dx * k, node[1] + dy * k)


class GridMap:
    def __init__(self, width, height, blocked_edges=(), physical_edges=None):
        self.w, self.h = width, height
        if physical_edges is None:
            physical_edges = [
                ((x, y), step((x, y), heading))
                for x in range(width)
                for y in range(height)
                for heading in HEADINGS
                if 0 <= step((x, y), heading)[0] < width
                and 0 <= step((x, y), heading)[1] < height
            ]
        self.physical_edges = {frozenset(edge) for edge in physical_edges}
        self.blocked = {frozenset(edge) for edge in blocked_edges}
        self._revision = 0

    @property
    def revision(self):
        """Numéro incrémenté à chaque changement de topologie ou de passage."""
        return self._revision

    def in_bounds(self, node):
        return 0 <= node[0] < self.w and 0 <= node[1] < self.h

    def can_move(self, node, heading):
        nxt = step(node, heading)
        edge = frozenset((node, nxt))
        return self.in_bounds(nxt) and edge in self.physical_edges and edge not in self.blocked

    def branches(self, node):
        """Branches physiques au nœud, même si un segment est bloqué."""
        return {
            heading for heading in HEADINGS
            if self.in_bounds(step(node, heading))
            and frozenset((node, step(node, heading))) in self.physical_edges
        }

    def intersection_type(self, node):
        """Type de l'intersection au nœud : X, T, L, straight, dead_end ou isolated."""
        branches = self.branches(node)
        if len(branches) == 4:
            return 'X'
        if len(branches) == 3:
            return 'T'
        if len(branches) == 2:
            if branches in ({'N', 'S'}, {'E', 'W'}):
                return 'straight'
            return 'L'
        if len(branches) == 1:
            return 'dead_end'
        return 'isolated'

    @property
    def intersection_types(self):
        """Types des intersections, indexés par leur position (x, y)."""
        return {
            (x, y): self.intersection_type((x, y))
            for x in range(self.w)
            for y in range(self.h)
        }

    def block(self, node, heading):
        """Marque comme bloqué un segment physique, sans changer la topologie."""
        edge = frozenset((node, step(node, heading)))
        if edge in self.physical_edges and edge not in self.blocked:
            self.blocked.add(edge)
            self._revision += 1

    def add_branch(self, node, heading):
        """Ajoute un segment physique découvert, et invalide les plans existants."""
        nxt = step(node, heading)
        if self.in_bounds(nxt):
            edge = frozenset((node, nxt))
            if edge not in self.physical_edges:
                self.physical_edges.add(edge)
                self._revision += 1

    def remove_branch(self, node, heading):
        """Retire un segment physiquement absent, d'après la géométrie observée."""
        edge = frozenset((node, step(node, heading)))
        if edge in self.physical_edges:
            self.physical_edges.remove(edge)
            self._revision += 1
        self.blocked.discard(edge)
