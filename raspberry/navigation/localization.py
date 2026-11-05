# Localisation sur la grille.
#
# Principe :
#   - On connaît le dernier nœud confirmé (self.node) et le cap (self.heading).
#   - Entre deux intersections, l'ENCODEUR donne la distance parcourue -> position estimée
#     le long de l'arête (sert à suivre le robot en continu et à compter les cases).
#   - Quand la caméra détecte une intersection, on calcule un nœud candidat sans
#     remplacer le dernier nœud confirmé.
#   - Quand le robot est centré sur l'intersection :
#       * le nœud candidat devient le nœud confirmé ;
#       * l'odomètre est remis à zéro par la mission.
#   - À chaque intersection vue par la CAMERA :
#       * k = distance / CELL_M (arrondi) = nombre de cases franchies depuis le dernier nœud
#         (k > 1 si la caméra a raté une intersection) ;
#       * les branches observées (gauche/devant/droite) sont comparées à celles que la carte
#         prévoit au nœud candidat : très informatif sur les bords et les coins (une branche
#         manque), inutile à l'intérieur de la grille où tous les nœuds se ressemblent ;
#       * on prépare une correction de position, confirmée une fois au centre.

from collections import namedtuple

import config as cfg
from navigation.grid_map import DELTA, heading_after, relative, step

Fix = namedtuple("Fix", "node skipped matched expected observed")


class Localizer:
    def __init__(self, grid, node, heading):
        self.grid = grid
        self.node = node                # dernier nœud confirmé
        self.heading = heading          # cap actuel : 'N', 'E', 'S', 'W'
        self.confidence = 1.0           # 1 = position confirmée par la carte, 0.5 = doute
        self._pending_node = None
        self._pending_confidence = None
        self._pending_origin = None
        self._returning_to_node = False
        self._return_heading = None
        self._return_distance = 0.0

    @property
    def candidate_node(self):
        """Nœud estimé à l'intersection détectée, ou dernier nœud confirmé."""
        return self.node if self._pending_node is None else self._pending_node

    @property
    def returning_to_node(self):
        return self._returning_to_node

    # -- utilitaires ----------------------------------------------------------
    def _max_cells(self, node=None, heading=None):
        node = self.node if node is None else node
        heading = self.heading if heading is None else heading
        n = 0
        while self.grid.can_move(node, heading):
            node = step(node, heading)
            n += 1
        return n

    def _expected(self, k):
        """Branches relatives (sans 'derriere') prévues par la carte au nœud k cases devant."""
        node = step(self.node, self.heading, k)
        return {relative(self.heading, h) for h in self.grid.branches(node)} - {'derriere'}

    # -- position continue (pour AFFICHAGE / suivi) ---------------------------------
    def estimate(self, dist_m):
        if self._returning_to_node:
            remaining_m = max(0.0, self._return_distance - dist_m)
            cells = remaining_m / cfg.CELL_M
            dx, dy = DELTA[self._return_heading]
            return (self.node[0] + dx * cells, self.node[1] + dy * cells)
        if self._pending_origin is None:
            node, heading = self.node, self.heading
        else:
            node, heading = self._pending_origin
        cells = min(max(dist_m, 0.0) / cfg.CELL_M, self._max_cells(node, heading))
        dx, dy = DELTA[heading]
        return (node[0] + dx * cells, node[1] + dy * cells)

    # -- événements ---------------------------------------------------------------
    def seen_intersection(self, observed, dist_m):
        """observed : branches vues par la caméra, ex. ['gauche', 'devant'].
        dist_m : distance encodeur depuis le dernier centre. Calcule un candidat sans
        modifier le dernier nœud confirmé."""
        if self._pending_node is not None:
            raise RuntimeError("l'intersection précédente n'a pas encore été centrée")

        observed = set(observed)
        if self._returning_to_node:
            expected = self._expected(0)
            matched = expected == observed
            self._pending_node = self.node
            self._pending_confidence = 1.0 if matched else 0.5
            self._pending_origin = (self.node, self.heading)
            self._returning_to_node = False
            self._return_heading = None
            self._return_distance = 0.0
            return Fix(self.node, 0, matched, expected, observed)

        k_est = (dist_m + cfg.APPROACH_OFFSET_M) / cfg.CELL_M
        reach = self._max_cells()

        if reach == 0:                  # on pointe vers l'extérieur de la grille : incohérent
            self._pending_node = self.node
            self._pending_confidence = 0.0
            self._pending_origin = (self.node, self.heading)
            return Fix(self.node, 0, False, set(), observed)

        cands = [k for k in range(1, reach + 1) if abs(k - k_est) <= cfg.MAX_K_ERROR]
        if not cands:                   # l'encodeur dérive trop : on prend la case la plus proche
            cands = [min(reach, max(1, round(k_est)))]
        ok = [k for k in cands if self._expected(k) == observed]
        k = min(ok or cands, key=lambda c: abs(c - k_est))

        matched = bool(ok)
        candidate = step(self.node, self.heading, k)
        self._pending_node = candidate
        self._pending_confidence = 1.0 if matched else 0.5
        self._pending_origin = (self.node, self.heading)
        return Fix(candidate, k - 1, matched, self._expected(0), observed)

    def centered_on_intersection(self):
        """Confirme le nœud candidat une fois le robot arrivé au centre."""
        if self._pending_node is None:
            raise RuntimeError("aucune intersection détectée à confirmer")
        self.node = self._pending_node
        self.confidence = self._pending_confidence
        self._pending_node = None
        self._pending_confidence = None
        self._pending_origin = None
        self._return_heading = None
        self._return_distance = 0.0
        return self.node

    def begin_return_to_confirmed_node(self, distance_m):
        """Re-anchor after a U-turn between intersections, on the return to the last node."""
        self._return_heading = self.heading
        self._return_distance = max(0.0, distance_m)
        self.heading = heading_after(heading_after(self.heading, 'gauche'), 'gauche')
        self._returning_to_node = True

    def on_turn(self, rel):
        """À appeler quand le robot a choisi 'gauche', 'devant' ou 'droite' à un nœud."""
        self.heading = heading_after(self.heading, rel)
