"""Interactive map editor for known obstacles on a grid mission."""

import math
import tkinter as tk


class ObstacleEditor:
    def __init__(self, root, start, goal, grid_size, heading):
        self.root = root
        self.start = start
        self.goal = goal
        self.width, self.height = grid_size
        self.heading = heading
        self.blocked_nodes = set()
        self.blocked_edges = set()
        self.result = None

        self.window = root
        self.window.title("Select known grid obstacles")
        self.window.protocol("WM_DELETE_WINDOW", self._cancel)
        frame = tk.Frame(self.window, padx=12, pady=12)
        frame.pack(fill=tk.BOTH, expand=True)
        tk.Label(
            frame,
            anchor="w",
            justify=tk.LEFT,
            text=(
                f"Grid {self.width} × {self.height} | Start: {start} | Goal: {goal} | Heading: {heading}\n"
                "Click a node to block all connected branches. Click a line to block only that segment.\n"
                "Click a selected obstacle again to remove it."
            ),
        ).pack(fill=tk.X, pady=(0, 8))
        self.canvas = tk.Canvas(
            frame,
            width=640,
            height=520,
            background="white",
            highlightthickness=0,
        )
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.canvas.bind("<Configure>", self._on_resize)
        self.canvas.bind("<Button-1>", self._on_click)

        buttons = tk.Frame(frame)
        buttons.pack(fill=tk.X, pady=(8, 0))
        tk.Button(buttons, text="Cancel", command=self._cancel).pack(side=tk.RIGHT)
        tk.Button(buttons, text="Use selected obstacles", command=self._submit).pack(
            side=tk.RIGHT, padx=(0, 8)
        )

        self._draw()

    def _submit(self):
        self.result = {
            "blocked_nodes": [list(node) for node in sorted(self.blocked_nodes)],
            "blocked_edges": [
                [list(node) for node in sorted(edge)]
                for edge in sorted(self.blocked_edges, key=lambda item: tuple(sorted(item)))
            ],
        }
        self.window.quit()

    def _cancel(self):
        self.result = None
        self.window.quit()

    def _on_resize(self, _event):
        self._draw()

    def _layout(self):
        canvas_width = max(self.canvas.winfo_width(), 240)
        canvas_height = max(self.canvas.winfo_height(), 200)
        margin = 44
        cell = min(
            (canvas_width - 2 * margin) / max(self.width - 1, 1),
            (canvas_height - 2 * margin) / max(self.height - 1, 1),
        )
        x_offset = (canvas_width - cell * (self.width - 1)) / 2
        y_offset = (canvas_height - cell * (self.height - 1)) / 2

        def pixel(node):
            x, y = node
            return x_offset + x * cell, canvas_height - y_offset - y * cell

        return canvas_width, canvas_height, cell, pixel

    def _draw(self):
        if not hasattr(self, "canvas") or not self.canvas.winfo_exists():
            return
        self.canvas.delete("all")
        _, _, _, pixel = self._layout()
        for x in range(self.width):
            for y in range(self.height):
                node = (x, y)
                px, py = pixel(node)
                for neighbor in ((x + 1, y), (x, y + 1)):
                    if not self._in_bounds(neighbor):
                        continue
                    nx, ny = pixel(neighbor)
                    edge = frozenset((node, neighbor))
                    blocked = edge in self.blocked_edges or node in self.blocked_nodes or neighbor in self.blocked_nodes
                    self.canvas.create_line(
                        px,
                        py,
                        nx,
                        ny,
                        fill="#dc2626" if blocked else "#cbd5e1",
                        width=5 if blocked else 2,
                    )

                self.canvas.create_text(
                    px - 14,
                    py + 16,
                    text=f"{x},{y}",
                    fill="#667085",
                    font=("TkDefaultFont", 8),
                )

        self._draw_marker(self.start, "S", "#2563eb", pixel)
        self._draw_marker(self.goal, "G", "#16a34a", pixel)
        for node in self.blocked_nodes:
            px, py = pixel(node)
            self.canvas.create_oval(
                px - 11, py - 11, px + 11, py + 11,
                fill="#fee2e2", outline="#dc2626", width=2,
            )
            self.canvas.create_text(px, py, text="X", fill="#b91c1c", font=("TkDefaultFont", 10, "bold"))

    def _draw_marker(self, node, label, color, pixel):
        px, py = pixel(node)
        offset = 15
        self.canvas.create_oval(
            px - offset, py - offset, px + offset, py + offset,
            fill=color, outline="white", width=2,
        )
        self.canvas.create_text(
            px, py, text=label, fill="white", font=("TkDefaultFont", 9, "bold")
        )

    def _on_click(self, event):
        _, _, cell, pixel = self._layout()
        closest_node = min(
            ((x, y) for x in range(self.width) for y in range(self.height)),
            key=lambda node: self._distance((event.x, event.y), pixel(node)),
        )
        if self._distance((event.x, event.y), pixel(closest_node)) <= min(18, cell * 0.3):
            if closest_node in self.blocked_nodes:
                self.blocked_nodes.remove(closest_node)
            else:
                self.blocked_nodes.add(closest_node)
            self._draw()
            return

        closest_edge = None
        closest_distance = math.inf
        for node, neighbor in self._edges():
            distance = self._distance_to_segment((event.x, event.y), pixel(node), pixel(neighbor))
            if distance < closest_distance:
                closest_edge, closest_distance = frozenset((node, neighbor)), distance
        if closest_edge is not None and closest_distance <= 12:
            if closest_edge in self.blocked_edges:
                self.blocked_edges.remove(closest_edge)
            else:
                self.blocked_edges.add(closest_edge)
            self._draw()

    def _edges(self):
        for x in range(self.width):
            for y in range(self.height):
                node = (x, y)
                for neighbor in ((x + 1, y), (x, y + 1)):
                    if self._in_bounds(neighbor):
                        yield node, neighbor

    def _in_bounds(self, node):
        return 0 <= node[0] < self.width and 0 <= node[1] < self.height

    @staticmethod
    def _distance(first, second):
        return math.hypot(first[0] - second[0], first[1] - second[1])

    @staticmethod
    def _distance_to_segment(point, start, end):
        dx, dy = end[0] - start[0], end[1] - start[1]
        length_squared = dx * dx + dy * dy
        if length_squared == 0:
            return math.hypot(point[0] - start[0], point[1] - start[1])
        projection = max(
            0.0,
            min(1.0, ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / length_squared),
        )
        closest = (start[0] + projection * dx, start[1] + projection * dy)
        return math.hypot(point[0] - closest[0], point[1] - closest[1])
