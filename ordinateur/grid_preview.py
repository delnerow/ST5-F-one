"""Tkinter window showing the live progress of a grid mission."""

import math
import tkinter as tk


class GridPreview:
    def __init__(self, root):
        self.root = root
        self.root.title("Grid mission preview")
        self.root.protocol("WM_DELETE_WINDOW", self._close)
        self.frame = tk.Frame(root, padx=12, pady=12)
        self.frame.pack(fill=tk.BOTH, expand=True)
        self.status = tk.Label(self.frame, anchor="w", justify=tk.LEFT)
        self.status.pack(fill=tk.X, pady=(0, 8))
        self.canvas = tk.Canvas(
            self.frame,
            width=640,
            height=520,
            background="white",
            highlightthickness=0,
        )
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.canvas.bind("<Configure>", self._on_resize)
        self.root.withdraw()
        self._reset()

    def show_mission(self, start, goal, grid_size, heading):
        width, height = grid_size
        if width < 1 or height < 1:
            raise ValueError(f"invalid grid size: {grid_size!r}")
        self._reset()
        self.start = self._valid_node(start, width, height)
        self.goal = self._valid_node(goal, width, height)
        self.width, self.height = width, height
        self.heading = heading
        self.status.config(
            text=(
                f"Grid mission | Start: {self.start}  Goal: {self.goal}  Heading: {heading}\n"
                "S: start  G: destination  R: robot  Blue line: travelled path"
            )
        )
        self.root.deiconify()
        self._draw()

    def update_position(self, robot_id, update, redraw=True):
        if self.width is None:
            return
        x, y = update.get("x"), update.get("y")
        if not isinstance(x, (int, float)) or not isinstance(y, (int, float)):
            return
        if not math.isfinite(x) or not math.isfinite(y):
            return
        if not (0 <= x <= self.width - 1 and 0 <= y <= self.height - 1):
            return

        point = (float(x), float(y))
        if not self.path or math.dist(self.path[-1], point) >= 0.02:
            self.path.append(point)
        self.position = point
        candidate = update.get("candidate_node", update.get("node"))
        self.status.config(
            text=(
                f"Grid mission | Robot: {robot_id}  Position: ({x:.2f}, {y:.2f})"
                f"  Node: {candidate}  Heading: {update.get('heading', '?')}"
                f"  Distance: {update.get('distance_m', 0.0):.2f} m"
                f"  Confidence: {update.get('confidence', '?')}\n"
                "S: start  G: destination  R: robot  Blue line: travelled path"
            )
        )
        if redraw:
            self._draw()

    def refresh(self):
        self._draw()

    def show_event(self, event):
        messages = {
            "mission_completed": "Mission completed",
            "mission_stopped": "Mission stopped",
            "mission_interrupted": "Mission interrupted",
        }
        if event.get("event") in messages:
            self.status.config(text=messages[event["event"]])

    def hide(self):
        self._reset()
        self.root.withdraw()

    def _reset(self):
        self.width = None
        self.height = None
        self.start = None
        self.goal = None
        self.heading = None
        self.position = None
        self.path = []
        if hasattr(self, "canvas"):
            self.canvas.delete("all")
            self.status.config(text="")

    @staticmethod
    def _valid_node(node, width, height):
        if not isinstance(node, (list, tuple)) or len(node) != 2:
            raise ValueError(f"invalid grid node: {node!r}")
        x, y = node
        if not isinstance(x, int) or not isinstance(y, int) or not (0 <= x < width and 0 <= y < height):
            raise ValueError(f"grid node {node!r} is outside {(width, height)}")
        return x, y

    def _on_resize(self, _event):
        self._draw()

    def _draw(self):
        if self.width is None:
            return
        self.canvas.delete("all")
        canvas_width = max(self.canvas.winfo_width(), 240)
        canvas_height = max(self.canvas.winfo_height(), 200)
        margin = 44
        cell = min(
            (canvas_width - 2 * margin) / max(self.width - 1, 1),
            (canvas_height - 2 * margin) / max(self.height - 1, 1),
        )
        x_offset = (canvas_width - cell * (self.width - 1)) / 2
        y_offset = (canvas_height - cell * (self.height - 1)) / 2

        def pixel(point):
            x, y = point
            return x_offset + x * cell, canvas_height - y_offset - y * cell

        for x in range(self.width):
            for y in range(self.height):
                px, py = pixel((x, y))
                if x + 1 < self.width:
                    nx, ny = pixel((x + 1, y))
                    self.canvas.create_line(px, py, nx, ny, fill="#d0d5dd", width=2)
                if y + 1 < self.height:
                    nx, ny = pixel((x, y + 1))
                    self.canvas.create_line(px, py, nx, ny, fill="#d0d5dd", width=2)
                self.canvas.create_text(
                    px - 14,
                    py + 16,
                    text=f"{x},{y}",
                    fill="#667085",
                    font=("TkDefaultFont", 8),
                )

        if len(self.path) > 1:
            coords = [pixel(point) for point in self.path]
            self.canvas.create_line(
                *[coordinate for point in coords for coordinate in point],
                fill="#2563eb",
                width=5,
                capstyle=tk.ROUND,
                joinstyle=tk.ROUND,
            )

        self._draw_marker(self.start, "S", "#2563eb", pixel)
        self._draw_marker(self.goal, "G", "#16a34a", pixel)
        if self.position is not None:
            self._draw_marker(self.position, "R", "#dc2626", pixel)

    def _draw_marker(self, point, label, color, pixel):
        px, py = pixel(point)
        radius = 13
        self.canvas.create_oval(
            px - radius,
            py - radius,
            px + radius,
            py + radius,
            fill=color,
            outline="white",
            width=2,
        )
        self.canvas.create_text(px, py, text=label, fill="white", font=("TkDefaultFont", 9, "bold"))

    def _close(self):
        self.root.withdraw()
