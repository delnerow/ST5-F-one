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
                "S: start  G: destination  R: robot  T: next target  "
                "Orange dashed: planned route  Blue: travelled path"
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
        target = update.get("target_node")
        try:
            self.target = (
                self._valid_node(target, self.width, self.height)
                if target is not None
                else None
            )
        except ValueError:
            self.target = None
        planned_path = update.get("planned_path", ())
        if isinstance(planned_path, (list, tuple)):
            self.planned_path = []
            for node in planned_path:
                try:
                    self.planned_path.append(
                        self._valid_node(node, self.width, self.height)
                    )
                except ValueError:
                    continue
        else:
            self.planned_path = []
        self.planned_path = self._stop_path_at_obstacle(self.planned_path)
        if self.target not in self.planned_path:
            self.target = None
        candidate = update.get("candidate_node", update.get("node"))
        self.status.config(
            text=(
                f"Grid mission | Robot: {robot_id}  Position: ({x:.2f}, {y:.2f})"
                f"  Node: {candidate}  Target: {self.target or '?'}"
                f"  Heading: {update.get('heading', '?')}"
                f"  Distance: {update.get('distance_m', 0.0):.2f} m"
                f"  Confidence: {update.get('confidence', '?')}\n"
                "S: start  G: destination  R: robot  T: next target  "
                "Orange dashed: planned route  Blue: travelled path"
            )
        )
        if redraw:
            self._draw()

    def refresh(self):
        self._draw()

    def show_event(self, event):
        if event.get("event") == "fixed_obstacle":
            blocked_edge = event.get("blocked_edge")
            if self.width is None or not isinstance(blocked_edge, (list, tuple)) or len(blocked_edge) != 2:
                return
            try:
                first = self._valid_node(blocked_edge[0], self.width, self.height)
                second = self._valid_node(blocked_edge[1], self.width, self.height)
            except ValueError:
                return
            if abs(first[0] - second[0]) + abs(first[1] - second[1]) != 1:
                return
            self.fixed_obstacles.add(tuple(sorted((first, second))))
            self.planned_path = self._stop_path_at_obstacle(self.planned_path)
            if self.target not in self.planned_path:
                self.target = None
            self.status.config(text=f"Fixed obstacle detected by Arduino: {first} -> {second}")
            self._draw()
            return

        messages = {
            "mission_completed": "Mission completed",
            "mission_stopped": "Mission stopped",
            "mission_interrupted": "Mission interrupted",
        }
        event_name = event.get("event")
        if event_name not in messages:
            return

        final_node = event.get("node")
        if final_node is None and event_name == "mission_completed":
            final_node = self.goal
        if final_node is not None and self.width is not None:
            try:
                final_node = self._valid_node(final_node, self.width, self.height)
            except ValueError:
                final_node = None
            if final_node is not None:
                point = (float(final_node[0]), float(final_node[1]))
                if not self.path or math.dist(self.path[-1], point) >= 0.02:
                    self.path.append(point)
                self.position = point
                self.target = None
                self.planned_path = []

        self.status.config(text=messages[event_name])
        self._draw()

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
        self.target = None
        self.planned_path = []
        self.path = []
        self.fixed_obstacles = set()
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

    def _stop_path_at_obstacle(self, path):
        for index, (first, second) in enumerate(zip(path, path[1:])):
            if tuple(sorted((first, second))) in self.fixed_obstacles:
                return path[: index + 1]
        return path

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

        if len(self.planned_path) > 1:
            coords = [pixel(point) for point in self.planned_path]
            self.canvas.create_line(
                *[coordinate for point in coords for coordinate in point],
                fill="#f59e0b",
                width=4,
                dash=(7, 5),
                capstyle=tk.ROUND,
                joinstyle=tk.ROUND,
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

        for first, second in self.fixed_obstacles:
            x1, y1 = pixel(first)
            x2, y2 = pixel(second)
            self.canvas.create_line(
                x1,
                y1,
                x2,
                y2,
                fill="#dc2626",
                width=9,
                capstyle=tk.ROUND,
            )
            midpoint_x, midpoint_y = (x1 + x2) / 2, (y1 + y2) / 2
            self.canvas.create_line(
                midpoint_x - 6,
                midpoint_y - 6,
                midpoint_x + 6,
                midpoint_y + 6,
                fill="white",
                width=2,
            )
            self.canvas.create_line(
                midpoint_x - 6,
                midpoint_y + 6,
                midpoint_x + 6,
                midpoint_y - 6,
                fill="white",
                width=2,
            )

        self._draw_marker(self.start, "S", "#2563eb", pixel)
        self._draw_marker(self.goal, "G", "#16a34a", pixel)
        if self.target is not None:
            self._draw_target_marker(self.target, pixel)
        if self.position is not None:
            self._draw_marker(self.position, "R", "#dc2626", pixel)

    def _draw_target_marker(self, point, pixel):
        px, py = pixel(point)
        radius = 20
        self.canvas.create_polygon(
            px,
            py - radius,
            px + radius,
            py,
            px,
            py + radius,
            px - radius,
            py,
            fill="",
            outline="#9333ea",
            width=3,
        )
        self.canvas.create_text(
            px + radius,
            py - radius,
            text="T",
            fill="#7e22ce",
            font=("TkDefaultFont", 9, "bold"),
        )

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
