#!/usr/bin/env python3
"""Interactive vehicle/pallet pose editor for navigation_layout_1to1.svg.

The GUI uses the pre-rendered PNG as its background while preserving the SVG's
centimetre coordinate system: origin at the lower-left of the main floor,
+X right, +Y up, and yaw positive counter-clockwise from +X.
"""

import argparse
import heapq
import json
import math
import os
import tkinter as tk
import time
from datetime import datetime, timezone
from fractions import Fraction
from pathlib import Path
from tkinter import messagebox, ttk
import xml.etree.ElementTree as ET


DEFAULT_VEHICLE_SIZE = (40.0, 17.0)
DEFAULT_PALLET_SIZE = (13.0, 13.0)
MAP_X_RANGE = (0.0, 180.0)
MAP_Y_RANGE = (0.0, 315.0)
ZOOM_OPTIONS = (33, 50, 67, 75, 100)
PLANNER_GRID_CM = 2.0
PLANNER_HEADING_COUNT = 16
PLANNER_HEADING_STEP_DEG = 360.0 / PLANNER_HEADING_COUNT
PLANNER_MOVE_STEP_CM = 4.0
FIXED_OBSTACLES = (
    (118.9, 20.0, 121.1, 180.0),     # wall between lower main floor and dock side
    (100.0, 178.9, 121.1, 181.1),    # FRESH-to-dock corner wall
    (11.0, 10.0, 101.0, 16.0),       # conveyor
    (35.0, 17.5, 61.0, 29.5),       # DOFBOT + rail
    (0.0, 105.0, 18.0, 175.0),      # PICO elevator
    (60.0, 256.0, 70.0, 314.0),     # charger
    (125.0, 169.5, 145.0, 181.5),   # dock guards
    (155.0, 169.5, 175.0, 181.5),
    (125.0, 313.5, 145.0, 325.5),
    (155.0, 313.5, 175.0, 325.5),
)


def utc_now():
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def normalize_yaw(value):
    return ((value + 180.0) % 360.0) - 180.0


class MapTransform:
    def __init__(self, svg_path, image_width, image_height):
        root = ET.parse(svg_path).getroot()
        view_box = [float(value) for value in root.attrib["viewBox"].split()]
        self.vx, self.vy, self.vw, self.vh = view_box
        self.scale = min(image_width / self.vw, image_height / self.vh)
        self.pad_x = (image_width - self.vw * self.scale) / 2.0
        self.pad_y = (image_height - self.vh * self.scale) / 2.0
        # The SVG places the world origin at (20, 335) and flips its Y axis.
        self.svg_origin_x = 20.0
        self.svg_origin_y = 335.0

    def world_to_canvas(self, x_cm, y_cm):
        x_svg = self.svg_origin_x + x_cm
        y_svg = self.svg_origin_y - y_cm
        return (
            self.pad_x + (x_svg - self.vx) * self.scale,
            self.pad_y + (y_svg - self.vy) * self.scale,
        )

    def canvas_to_world(self, canvas_x, canvas_y):
        x_svg = (canvas_x - self.pad_x) / self.scale + self.vx
        y_svg = (canvas_y - self.pad_y) / self.scale + self.vy
        return x_svg - self.svg_origin_x, self.svg_origin_y - y_svg


class PoseEditor:
    def __init__(self, root, args):
        self.root = root
        self.args = args
        self.state_path = args.state.resolve()
        self.events_path = args.events.resolve()
        self.sessions_path = args.sessions.resolve()
        self.items = {}
        self.selected_id = None
        self.drag_offset = (0.0, 0.0)
        self.rotate_start_offset = 0.0
        self.dragging_item_id = None
        self.rotating_item_id = None
        self.dragging_path_point = None
        self.rotating_path_point = None
        self.path_drag_offset = (0.0, 0.0)
        self.path_rotate_start_offset = 0.0
        self.write_after_id = None
        self.pending_event = "startup"
        self.recording = False
        self.record_session_id = None
        self.record_started_monotonic = None
        self.current_session = None
        self.history_window = None
        self.sessions, self.next_session_index = self.load_sessions()
        self.path_a = None
        self.path_b = None
        self.path_a_yaw = 0.0
        self.path_b_yaw = 0.0
        self.path_a_yaw_var = tk.DoubleVar(value=0.0)
        self.path_b_yaw_var = tk.DoubleVar(value=0.0)
        self.path_points = []
        self.path_modes = []
        self.ready_pose = None
        self.path_status_text = "Set A and B"
        self.last_planner_vehicle_id = None
        self.last_clearance_radius_cm = None
        self.last_map_click = None

        root.title("Navigation Pose Board")
        root.geometry("1200x950")
        root.minsize(850, 600)
        root.protocol("WM_DELETE_WINDOW", self.close)

        self.source_background = tk.PhotoImage(file=str(args.map_png))
        self.zoom_percent = tk.IntVar(value=args.zoom)
        self.background = self.scaled_background(args.zoom)
        self.transform = MapTransform(
            args.map_svg, self.background.width(), self.background.height()
        )
        self._build_ui()
        self._load_or_initialize()
        self.render_all()
        self.persist("startup", immediate=True)

    def _build_ui(self):
        outer = tk.PanedWindow(self.root, orient=tk.HORIZONTAL, sashwidth=5)
        outer.pack(fill=tk.BOTH, expand=True)

        map_frame = tk.Frame(outer)
        side = tk.Frame(outer, padx=10, pady=6, width=310)
        outer.add(map_frame, stretch="always")
        outer.add(side, minsize=290)

        self.canvas = tk.Canvas(map_frame, bg="#333", highlightthickness=0)
        xbar = tk.Scrollbar(map_frame, orient=tk.HORIZONTAL, command=self.canvas.xview)
        ybar = tk.Scrollbar(map_frame, orient=tk.VERTICAL, command=self.canvas.yview)
        self.canvas.configure(xscrollcommand=xbar.set, yscrollcommand=ybar.set)
        self.canvas.grid(row=0, column=0, sticky="nsew")
        ybar.grid(row=0, column=1, sticky="ns")
        xbar.grid(row=1, column=0, sticky="ew")
        map_frame.rowconfigure(0, weight=1)
        map_frame.columnconfigure(0, weight=1)
        self.map_image_id = self.canvas.create_image(
            0, 0, image=self.background, anchor="nw", tags=("map",)
        )
        self.canvas.configure(
            scrollregion=(0, 0, self.background.width(), self.background.height())
        )

        tk.Label(side, text="POSE STICKERS", font=("Sans", 13, "bold")).pack(anchor="w")
        tk.Label(
            side,
            text=(
                "Left-drag: move\n"
                "Right-drag / wheel: rotate\n"
                "Middle-click: rotate 90°\n"
                "Arrow keys: 1 cm · Shift+arrows: 5 cm"
            ),
            justify="left",
        ).pack(anchor="w", pady=(4, 10))

        zoom_row = tk.Frame(side)
        zoom_row.pack(fill="x", pady=(0, 10))
        tk.Label(zoom_row, text="Map zoom").pack(side="left")
        tk.OptionMenu(
            zoom_row,
            self.zoom_percent,
            *ZOOM_OPTIONS,
            command=self.change_zoom,
        ).pack(side="right", fill="x", expand=True, padx=(12, 0))

        self.listbox = tk.Listbox(side, height=4, exportselection=False)
        self.listbox.pack(fill="x")
        self.listbox.bind("<<ListboxSelect>>", self.on_list_select)

        button_row = tk.Frame(side)
        button_row.pack(fill="x", pady=6)
        tk.Button(button_row, text="+ Vehicle", command=lambda: self.add_item("vehicle")).pack(
            side="left", expand=True, fill="x"
        )
        tk.Button(button_row, text="+ Pallet", command=lambda: self.add_item("pallet")).pack(
            side="left", expand=True, fill="x", padx=(4, 0)
        )
        tk.Button(side, text="Delete selected", command=self.delete_selected).pack(fill="x")

        attach_row = tk.Frame(side)
        attach_row.pack(fill="x", pady=(8, 0))
        tk.Label(attach_row, text="Pallet attached to").pack(side="left")
        self.attachment_var = tk.StringVar(value="Detached")
        self.attachment_combo = ttk.Combobox(
            attach_row, textvariable=self.attachment_var, state="disabled", width=15
        )
        self.attachment_combo.pack(side="right", fill="x", expand=True, padx=(8, 0))
        self.attachment_combo.bind("<<ComboboxSelected>>", self.on_attachment_changed)

        self.record_button = tk.Button(
            side, text="● Start recording", command=self.toggle_recording,
            bg="#1f7a3f", fg="white", activebackground="#1f7a3f",
        )
        self.record_button.pack(fill="x", pady=(10, 0))
        self.record_status = tk.StringVar(value="Not recording")
        tk.Label(side, textvariable=self.record_status).pack(anchor="w", pady=(3, 0))
        tk.Label(side, text="Session comment").pack(anchor="w", pady=(6, 0))
        self.session_comment = tk.StringVar()
        tk.Entry(side, textvariable=self.session_comment).pack(fill="x")
        tk.Button(side, text="Recording history", command=self.open_history).pack(
            fill="x", pady=(5, 0)
        )

        self.pose_text = tk.StringVar(value="No selection")
        tk.Label(
            side, textvariable=self.pose_text, justify="left", anchor="w",
            font=("DejaVu Sans Mono", 11), relief="groove", padx=8, pady=8,
        ).pack(fill="x", pady=(12, 6))

        rotate_row = tk.Frame(side)
        rotate_row.pack(fill="x")
        tk.Button(rotate_row, text="-15°", command=lambda: self.rotate_selected(-15)).pack(
            side="left", expand=True, fill="x"
        )
        tk.Button(rotate_row, text="+15°", command=lambda: self.rotate_selected(15)).pack(
            side="left", expand=True, fill="x", padx=(4, 0)
        )

        tk.Label(side, text="PATH PLANNER", font=("Sans", 10, "bold")).pack(
            anchor="w", pady=(12, 2)
        )
        tk.Label(
            side,
            text="Drag A/B to move · right-drag or wheel to rotate",
            wraplength=280, justify="left",
        ).pack(anchor="w")
        plan_row = tk.Frame(side)
        plan_row.pack(fill="x", pady=(4, 0))
        tk.Button(plan_row, text="Calculate path", command=self.calculate_path).pack(
            side="left", expand=True, fill="x"
        )
        tk.Button(plan_row, text="Clear", command=self.clear_path).pack(side="right", padx=(4, 0))
        margin_row = tk.Frame(side)
        margin_row.pack(fill="x", pady=(3, 0))
        tk.Label(margin_row, text="Safety margin (cm)").pack(side="left")
        self.planner_margin = tk.DoubleVar(value=self.args.planner_margin)
        tk.Spinbox(
            margin_row, from_=0.0, to=20.0, increment=0.5,
            textvariable=self.planner_margin, width=6,
        ).pack(side="right")
        self.path_status = tk.StringVar(value=self.path_status_text)
        tk.Label(
            side, textvariable=self.path_status, justify="left", wraplength=280,
            fg="#1f4d7a",
        ).pack(anchor="w", pady=(3, 0))
        self.map_click_text = tk.StringVar(value="Map click: -")
        tk.Label(
            side, textvariable=self.map_click_text, justify="left",
            font=("DejaVu Sans Mono", 10), fg="#333",
        ).pack(anchor="w", pady=(3, 0))

        tk.Label(side, text="Live snapshot", font=("Sans", 10, "bold")).pack(
            anchor="w", pady=(16, 0)
        )
        tk.Label(side, text=str(self.state_path), wraplength=280, justify="left").pack(anchor="w")
        tk.Label(side, text="Event log", font=("Sans", 10, "bold")).pack(
            anchor="w", pady=(8, 0)
        )
        tk.Label(side, text=str(self.events_path), wraplength=280, justify="left").pack(anchor="w")
        self.save_status = tk.StringVar(value="")
        tk.Label(side, textvariable=self.save_status, fg="#28643b").pack(anchor="w", pady=(8, 0))

        self.canvas.tag_bind("sticker", "<ButtonPress-1>", self.start_move)
        self.canvas.tag_bind("sticker", "<ButtonPress-3>", self.start_rotate)
        self.canvas.tag_bind("sticker", "<ButtonPress-2>", self.middle_click_rotate)
        self.canvas.tag_bind("path_point", "<ButtonPress-1>", self.start_path_point_move)
        self.canvas.tag_bind("path_point", "<ButtonPress-3>", self.start_path_point_rotate)
        self.canvas.tag_bind("path_point", "<ButtonPress-2>", self.middle_click_path_point)
        self.canvas.bind("<B1-Motion>", self.move_drag)
        self.canvas.bind("<ButtonRelease-1>", self.end_interaction)
        self.canvas.bind("<B3-Motion>", self.rotate_drag)
        self.canvas.bind("<ButtonRelease-3>", self.end_interaction)
        self.canvas.bind("<Button-4>", lambda event: self.wheel_rotate(event, 5))
        self.canvas.bind("<Button-5>", lambda event: self.wheel_rotate(event, -5))
        self.canvas.bind("<MouseWheel>", self.mousewheel_rotate)
        self.canvas.bind("<ButtonPress-1>", self.on_empty_map_click, add="+")
        self.root.bind("<KeyPress>", self.on_key)

    def load_sessions(self):
        if not self.sessions_path.exists():
            return [], 1
        try:
            payload = json.loads(self.sessions_path.read_text(encoding="utf-8"))
            sessions = payload.get("sessions", [])
            highest = max((int(session["index"]) for session in sessions), default=0)
            next_index = max(int(payload.get("next_index", highest + 1)), highest + 1)
            return sessions, next_index
        except (OSError, ValueError, KeyError, TypeError) as error:
            messagebox.showwarning("Session history load failed", str(error))
            return [], 1

    def save_sessions(self):
        payload = {
            "schema": "navigation-pose-sessions/v1",
            "updated_at": utc_now(),
            "next_index": self.next_session_index,
            "sessions": self.sessions,
        }
        self.sessions_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.sessions_path.with_suffix(self.sessions_path.suffix + ".tmp")
        temporary.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        os.replace(temporary, self.sessions_path)

    def open_history(self):
        if self.history_window is not None and self.history_window.winfo_exists():
            self.history_window.lift()
            self.refresh_history_tree()
            return
        window = tk.Toplevel(self.root)
        self.history_window = window
        window.title("Recording History")
        window.geometry("850x470")
        window.protocol("WM_DELETE_WINDOW", self.close_history)

        columns = ("index", "started", "duration", "events", "comment")
        self.history_tree = ttk.Treeview(window, columns=columns, show="headings", height=11)
        headings = {
            "index": "#", "started": "Started (UTC)", "duration": "Duration",
            "events": "Events", "comment": "Comment",
        }
        widths = {"index": 45, "started": 180, "duration": 80, "events": 70, "comment": 390}
        for column in columns:
            self.history_tree.heading(column, text=headings[column])
            self.history_tree.column(column, width=widths[column], anchor="w")
        self.history_tree.pack(fill="both", expand=True, padx=10, pady=(10, 5))
        self.history_tree.bind("<<TreeviewSelect>>", self.on_history_select)

        tk.Label(window, text="Comment").pack(anchor="w", padx=10)
        self.history_comment = tk.Text(window, height=4, wrap="word")
        self.history_comment.pack(fill="x", padx=10, pady=(0, 6))
        buttons = tk.Frame(window)
        buttons.pack(fill="x", padx=10, pady=(0, 10))
        tk.Button(buttons, text="Save comment", command=self.save_history_comment).pack(
            side="left"
        )
        tk.Button(
            buttons, text="Delete session", command=self.delete_history_session,
            bg="#b42318", fg="white",
        ).pack(side="right")
        self.refresh_history_tree()

    def close_history(self):
        if self.history_window is not None:
            self.history_window.destroy()
        self.history_window = None

    def refresh_history_tree(self):
        if self.history_window is None or not self.history_window.winfo_exists():
            return
        selected = self.history_tree.selection()
        selected_id = selected[0] if selected else None
        for row in self.history_tree.get_children():
            self.history_tree.delete(row)
        for session in reversed(self.sessions):
            duration = session.get("duration_s")
            duration_text = "REC" if duration is None else f"{duration:.1f}s"
            self.history_tree.insert(
                "", "end", iid=session["session_id"],
                values=(
                    session["index"], session["started_at"], duration_text,
                    session.get("event_count", 0), session.get("comment", ""),
                ),
            )
        if selected_id and self.history_tree.exists(selected_id):
            self.history_tree.selection_set(selected_id)

    def selected_history_session(self):
        if self.history_window is None:
            return None
        selected = self.history_tree.selection()
        if not selected:
            return None
        session_id = selected[0]
        return next(
            (session for session in self.sessions if session["session_id"] == session_id),
            None,
        )

    def on_history_select(self, _event):
        session = self.selected_history_session()
        if session is None:
            return
        self.history_comment.delete("1.0", tk.END)
        self.history_comment.insert("1.0", session.get("comment", ""))

    def save_history_comment(self):
        session = self.selected_history_session()
        if session is None:
            return
        session["comment"] = self.history_comment.get("1.0", "end-1c").strip()
        self.save_sessions()
        self.refresh_history_tree()

    def delete_history_session(self):
        session = self.selected_history_session()
        if session is None:
            return
        if self.recording and session["session_id"] == self.record_session_id:
            messagebox.showwarning("Recording active", "Stop recording before deleting this session.")
            return
        if not messagebox.askyesno(
            "Delete recording", f"Delete session #{session['index']}?\nA recovery copy will be kept."
        ):
            return
        self.archive_and_remove_session(session)
        self.sessions.remove(session)
        self.save_sessions()
        self.refresh_history_tree()
        self.history_comment.delete("1.0", tk.END)

    def archive_and_remove_session(self, session):
        deleted_at = utc_now()
        metadata_trash = self.sessions_path.with_name(
            f"{self.sessions_path.stem}_trash.jsonl"
        )
        metadata_trash.parent.mkdir(parents=True, exist_ok=True)
        with metadata_trash.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(
                {"deleted_at": deleted_at, "session": session},
                ensure_ascii=False, separators=(",", ":"),
            ) + "\n")
        if not self.events_path.exists():
            return
        event_trash = self.events_path.with_name(f"{self.events_path.stem}_trash.jsonl")
        temporary = self.events_path.with_suffix(self.events_path.suffix + ".tmp")
        with self.events_path.open("r", encoding="utf-8") as source, \
                temporary.open("w", encoding="utf-8") as keep, \
                event_trash.open("a", encoding="utf-8") as trash:
            for line in source:
                try:
                    event = json.loads(line)
                except ValueError:
                    keep.write(line)
                    continue
                if event.get("session_id") == session["session_id"]:
                    trash.write(json.dumps(
                        {"deleted_at": deleted_at, "event": event},
                        ensure_ascii=False, separators=(",", ":"),
                    ) + "\n")
                else:
                    keep.write(line)
        os.replace(temporary, self.events_path)

    def scaled_background(self, percent):
        ratio = Fraction(percent, 100).limit_denominator(10)
        if ratio.numerator == ratio.denominator:
            return self.source_background
        return self.source_background.zoom(ratio.numerator).subsample(ratio.denominator)

    def change_zoom(self, value):
        percent = int(value)
        self.background = self.scaled_background(percent)
        self.transform = MapTransform(
            self.args.map_svg, self.background.width(), self.background.height()
        )
        self.canvas.itemconfigure(self.map_image_id, image=self.background)
        self.canvas.configure(
            scrollregion=(0, 0, self.background.width(), self.background.height())
        )
        self.canvas.xview_moveto(0)
        self.canvas.yview_moveto(0)
        self.render_all()

    def clear_path(self):
        self.path_points = []
        self.path_modes = []
        self.ready_pose = None
        self.last_planner_vehicle_id = None
        self.last_clearance_radius_cm = None
        self.path_status_text = "A/B ready"
        self.path_status.set(self.path_status_text)
        self.render_all()
        self.persist("clear_path", immediate=True)

    def planning_vehicle(self):
        selected = self.items.get(self.selected_id)
        if selected and selected["type"] == "vehicle":
            return selected
        return next((item for item in self.items.values() if item["type"] == "vehicle"), None)

    @staticmethod
    def inside_floor(x_cm, y_cm):
        return (
            0.0 <= x_cm <= 120.0 and 0.0 <= y_cm <= 315.0
        ) or (
            120.0 <= x_cm <= 180.0 and 180.0 <= y_cm <= 315.0
        )

    @staticmethod
    def rectangle_polygon(x_cm, y_cm, length_cm, width_cm, yaw_deg):
        angle = math.radians(yaw_deg)
        ux, uy = math.cos(angle), math.sin(angle)
        vx, vy = -uy, ux
        return [
            (
                x_cm + forward * length_cm / 2.0 * ux + side * width_cm / 2.0 * vx,
                y_cm + forward * length_cm / 2.0 * uy + side * width_cm / 2.0 * vy,
            )
            for forward, side in ((1, 1), (1, -1), (-1, -1), (-1, 1))
        ]

    @staticmethod
    def polygons_intersect(first, second):
        for polygon in (first, second):
            for index in range(len(polygon)):
                x1, y1 = polygon[index]
                x2, y2 = polygon[(index + 1) % len(polygon)]
                axis_x, axis_y = -(y2 - y1), x2 - x1
                first_projection = [x * axis_x + y * axis_y for x, y in first]
                second_projection = [x * axis_x + y * axis_y for x, y in second]
                if max(first_projection) < min(second_projection) \
                        or max(second_projection) < min(first_projection):
                    return False
        return True

    def polygon_inside_floor(self, polygon):
        for start, end in zip(polygon, polygon[1:] + polygon[:1]):
            distance = math.hypot(end[0] - start[0], end[1] - start[1])
            steps = max(1, math.ceil(distance / 2.0))
            for index in range(steps + 1):
                ratio = index / steps
                x_cm = start[0] + (end[0] - start[0]) * ratio
                y_cm = start[1] + (end[1] - start[1]) * ratio
                if not self.inside_floor(x_cm, y_cm):
                    return False
        return True

    def moving_polygons(self, front_x, front_y, yaw_deg, vehicle, margin):
        angle = math.radians(yaw_deg)
        center_x = front_x - vehicle["length_cm"] / 2.0 * math.cos(angle)
        center_y = front_y - vehicle["length_cm"] / 2.0 * math.sin(angle)
        polygons = [self.rectangle_polygon(
            center_x, center_y,
            vehicle["length_cm"] + 2.0 * margin,
            vehicle["width_cm"] + 2.0 * margin,
            yaw_deg,
        )]
        for pallet in self.items.values():
            if pallet.get("attached_to") != vehicle["id"]:
                continue
            ox, oy = pallet["offset_x_cm"], pallet["offset_y_cm"]
            pallet_x = center_x + math.cos(angle) * ox - math.sin(angle) * oy
            pallet_y = center_y + math.sin(angle) * ox + math.cos(angle) * oy
            polygons.append(self.rectangle_polygon(
                pallet_x, pallet_y,
                pallet["length_cm"] + 2.0 * margin,
                pallet["width_cm"] + 2.0 * margin,
                yaw_deg + pallet["offset_yaw_deg"],
            ))
        return polygons

    def obstacle_polygons(self, planning_vehicle_id):
        obstacles = [
            self.rectangle_polygon(
                (x_min + x_max) / 2.0, (y_min + y_max) / 2.0,
                x_max - x_min, y_max - y_min, 0.0,
            )
            for x_min, y_min, x_max, y_max in FIXED_OBSTACLES
        ]
        for item in self.items.values():
            if item["id"] == planning_vehicle_id or item.get("attached_to") == planning_vehicle_id:
                continue
            obstacles.append(self.rectangle_polygon(
                item["x_cm"], item["y_cm"], item["length_cm"],
                item["width_cm"], item["yaw_deg"],
            ))
        return obstacles

    def pose_is_free(self, front_x, front_y, yaw_deg, vehicle, margin, obstacles):
        for moving in self.moving_polygons(front_x, front_y, yaw_deg, vehicle, margin):
            if not self.polygon_inside_floor(moving):
                return False
            if any(self.polygons_intersect(moving, obstacle) for obstacle in obstacles):
                return False
        return True

    def transition_is_free(self, start, end, vehicle, margin, obstacles):
        distance = math.hypot(end[0] - start[0], end[1] - start[1])
        yaw_delta = normalize_yaw(end[2] - start[2])
        steps = max(2, math.ceil(distance / 1.0), math.ceil(abs(yaw_delta) / 5.0))
        for index in range(steps + 1):
            ratio = index / steps
            pose = (
                start[0] + (end[0] - start[0]) * ratio,
                start[1] + (end[1] - start[1]) * ratio,
                normalize_yaw(start[2] + yaw_delta * ratio),
            )
            if not self.pose_is_free(*pose, vehicle, margin, obstacles):
                return False
        return True

    def calculate_path(self):
        if self.path_a is None or self.path_b is None:
            self.path_status_text = "Set both A and B first"
            self.path_status.set(self.path_status_text)
            return
        vehicle = self.planning_vehicle()
        if vehicle is None:
            self.path_status_text = "Add a vehicle sticker first"
            self.path_status.set(self.path_status_text)
            return
        self.path_a_yaw = normalize_yaw(self.path_a_yaw_var.get())
        self.path_b_yaw = normalize_yaw(self.path_b_yaw_var.get())
        margin = max(0.0, self.planner_margin.get())
        obstacles = self.obstacle_polygons(vehicle["id"])
        self.last_planner_vehicle_id = vehicle["id"]
        self.last_clearance_radius_cm = None
        for label, point, yaw in (
            ("A", self.path_a, self.path_a_yaw),
            ("B", self.path_b, self.path_b_yaw),
        ):
            if not self.pose_is_free(point[0], point[1], yaw, vehicle, margin, obstacles):
                self.path_points = []
                self.path_modes = []
                self.ready_pose = None
                self.path_status_text = (
                    f"Front point {label} at yaw {yaw:.1f}° collides"
                )
                self.path_status.set(self.path_status_text)
                self.render_all()
                self.persist("path_failed", immediate=True)
                return
        path, modes, ready_pose = self.oriented_astar_path(
            (*self.path_a, self.path_a_yaw),
            (*self.path_b, self.path_b_yaw),
            vehicle, margin, obstacles,
        )
        if not path:
            self.path_points = []
            self.path_modes = []
            self.ready_pose = None
            self.path_status_text = "No reverse-escape + front-entry path"
            event = "path_failed"
        else:
            self.path_points = [
                [round(x, 2), round(y, 2), round(normalize_yaw(yaw), 1)]
                for x, y, yaw in path
            ]
            self.path_modes = modes
            self.ready_pose = (
                [round(ready_pose[0], 2), round(ready_pose[1], 2),
                 round(normalize_yaw(ready_pose[2]), 1)]
                if ready_pose else None
            )
            length = sum(
                math.hypot(b[0] - a[0], b[1] - a[1])
                for a, b in zip(path, path[1:])
            )
            reverse_length = sum(
                math.hypot(b[0] - a[0], b[1] - a[1])
                for a, b, mode in zip(path, path[1:], modes) if mode == "R"
            )
            self.path_status_text = (
                f"Front path: {length:.1f} cm · {len(path)} poses\n"
                f"Reverse escape: {reverse_length:.1f} cm · then front entry"
            )
            event = "path_calculated"
        self.path_status.set(self.path_status_text)
        self.render_all()
        self.persist(event, immediate=True)

    def oriented_astar_path(self, start, goal, vehicle, margin, obstacles):
        resolution = PLANNER_GRID_CM
        heading_step = PLANNER_HEADING_STEP_DEG
        start_node = (
            round(start[0] / resolution), round(start[1] / resolution),
            round(start[2] / heading_step) % PLANNER_HEADING_COUNT, 0,
        )
        goal_node = (
            round(goal[0] / resolution), round(goal[1] / resolution),
            round(goal[2] / heading_step) % PLANNER_HEADING_COUNT,
        )
        free_cache = {}

        def node_pose(node):
            return (
                node[0] * resolution, node[1] * resolution,
                normalize_yaw(node[2] * heading_step),
            )

        def node_free(node):
            pose_key = node[:3]
            if pose_key not in free_cache:
                free_cache[pose_key] = self.pose_is_free(
                    *node_pose(node), vehicle, margin, obstacles
                )
            return free_cache[pose_key]

        if not node_free(start_node):
            return []
        queue = [(0.0, 0.0, start_node)]
        cost = {start_node: 0.0}
        parent = {}
        edge_mode = {}
        reached = None
        while queue:
            _priority, current_cost, current = heapq.heappop(queue)
            current_pose = node_pose(current)
            goal_dx = goal[0] - current_pose[0]
            goal_dy = goal[1] - current_pose[1]
            goal_angle = math.radians(goal[2])
            forward_error = goal_dx * math.cos(goal_angle) + goal_dy * math.sin(goal_angle)
            lateral_error = abs(-goal_dx * math.sin(goal_angle) + goal_dy * math.cos(goal_angle))
            if current[3] == 1 and current[2] == goal_node[2] \
                    and -0.01 <= forward_error <= PLANNER_MOVE_STEP_CM + 0.01 \
                    and lateral_error <= resolution / 2.0 \
                    and edge_mode.get(current) == "F" \
                    and self.transition_is_free(
                        current_pose, goal, vehicle, margin, obstacles
                    ):
                reached = current
                break
            if current_cost > cost.get(current, math.inf):
                continue
            candidates = []
            if current[3] == 0:
                current_angle = math.radians(current_pose[2])
                reverse_x = current_pose[0] - PLANNER_MOVE_STEP_CM * math.cos(current_angle)
                reverse_y = current_pose[1] - PLANNER_MOVE_STEP_CM * math.sin(current_angle)
                candidates.append((
                    (round(reverse_x / resolution), round(reverse_y / resolution), current[2], 0),
                    "R", PLANNER_MOVE_STEP_CM * 1.15,
                ))
                candidates.append(((current[0], current[1], current[2], 1), "READY", 0.2))
            else:
                for heading_delta in (-1, 0, 1):
                    next_heading = (current[2] + heading_delta) % PLANNER_HEADING_COUNT
                    next_yaw = next_heading * heading_step
                    next_x = current_pose[0] + PLANNER_MOVE_STEP_CM * math.cos(
                        math.radians(next_yaw)
                    )
                    next_y = current_pose[1] + PLANNER_MOVE_STEP_CM * math.sin(
                        math.radians(next_yaw)
                    )
                    candidates.append((
                        (round(next_x / resolution), round(next_y / resolution), next_heading, 1),
                        "F", PLANNER_MOVE_STEP_CM + abs(heading_delta) * 0.6,
                    ))
                for heading_delta in (-1, 1):
                    next_heading = (current[2] + heading_delta) % PLANNER_HEADING_COUNT
                    candidates.append((
                        (current[0], current[1], next_heading, 1), "TURN", 2.5,
                    ))
            for neighbor, mode, action_cost in candidates:
                if neighbor == current or not node_free(neighbor):
                    continue
                neighbor_pose = node_pose(neighbor)
                if mode != "READY" and not self.transition_is_free(
                    current_pose, neighbor_pose, vehicle, margin, obstacles
                ):
                    continue
                new_cost = current_cost + action_cost
                if new_cost >= cost.get(neighbor, math.inf):
                    continue
                cost[neighbor] = new_cost
                parent[neighbor] = current
                edge_mode[neighbor] = mode
                distance = math.hypot(
                    neighbor_pose[0] - goal[0], neighbor_pose[1] - goal[1]
                )
                heading_error = min(
                    (neighbor[2] - goal_node[2]) % PLANNER_HEADING_COUNT,
                    (goal_node[2] - neighbor[2]) % PLANNER_HEADING_COUNT,
                )
                heuristic = distance + heading_error * 1.5 + (0.2 if neighbor[3] == 0 else 0.0)
                heapq.heappush(queue, (new_cost + heuristic, new_cost, neighbor))
        if reached is None:
            return [], [], None
        nodes = [reached]
        while nodes[-1] != start_node:
            nodes.append(parent[nodes[-1]])
        nodes.reverse()
        path = [tuple(start)] + [node_pose(node) for node in nodes[1:]] + [tuple(goal)]
        modes = [edge_mode[node] for node in nodes[1:]] + ["F"]
        ready_pose = next(
            (node_pose(node) for node in nodes[1:] if edge_mode[node] == "READY"),
            tuple(start),
        )
        if not all(
            self.transition_is_free(first, second, vehicle, margin, obstacles)
            for first, second, mode in zip(path, path[1:], modes) if mode != "READY"
        ):
            return [], [], None
        return path, modes, ready_pose

    @staticmethod
    def compress_oriented_path(path):
        if len(path) < 3:
            return path
        result = [path[0]]
        for index in range(1, len(path) - 1):
            previous, current, following = path[index - 1], path[index], path[index + 1]
            if current[2] != previous[2] or following[2] != current[2]:
                result.append(current)
        result.append(path[-1])
        return result

    def _load_or_initialize(self):
        if self.state_path.exists():
            try:
                payload = json.loads(self.state_path.read_text(encoding="utf-8"))
                self.last_map_click = payload.get("last_map_click_cm")
                if self.last_map_click is not None:
                    self.map_click_text.set(
                        f"Map click: x={self.last_map_click[0]:.2f}, "
                        f"y={self.last_map_click[1]:.2f} cm"
                    )
                for item in payload.get("items", []):
                    self._accept_item(item)
                planner = payload.get("path_planner", {})
                self.path_a = planner.get("point_a")
                self.path_b = planner.get("point_b")
                self.path_a_yaw = float(planner.get("point_a_yaw_deg", 0.0))
                self.path_b_yaw = float(planner.get("point_b_yaw_deg", 0.0))
                self.path_a_yaw_var.set(self.path_a_yaw)
                self.path_b_yaw_var.set(self.path_b_yaw)
                self.path_points = planner.get("path", [])
                self.path_modes = planner.get("path_modes", [])
                self.ready_pose = planner.get("ready_pose")
                self.path_status_text = planner.get("status", "Set A and B")
                self.last_planner_vehicle_id = planner.get("vehicle_id")
                self.last_clearance_radius_cm = planner.get("clearance_radius_cm")
                self.path_status.set(self.path_status_text)
            except (OSError, ValueError, KeyError, TypeError) as error:
                messagebox.showwarning("State load failed", str(error))
        if not self.items:
            self._accept_item(self.new_item("vehicle", 80.0, 266.0, 0.0))
            self._accept_item(self.new_item("pallet", 90.0, 150.0, 0.0))
        vehicle = next(
            (item for item in self.items.values() if item["type"] == "vehicle"), None
        )
        if vehicle is None:
            self._accept_item(self.new_item("vehicle", 80.0, 266.0, 0.0))
            vehicle = next(item for item in self.items.values() if item["type"] == "vehicle")
        if self.path_a is None or self.path_b is None:
            angle = math.radians(vehicle["yaw_deg"])
            front_x = vehicle["x_cm"] + vehicle["length_cm"] / 2.0 * math.cos(angle)
            front_y = vehicle["y_cm"] + vehicle["length_cm"] / 2.0 * math.sin(angle)
            self.path_a = [front_x, front_y]
            self.path_b = [
                front_x + 30.0 * math.cos(angle),
                front_y + 30.0 * math.sin(angle),
            ]
            self.path_a_yaw = vehicle["yaw_deg"]
            self.path_b_yaw = vehicle["yaw_deg"]
            self.path_a_yaw_var.set(self.path_a_yaw)
            self.path_b_yaw_var.set(self.path_b_yaw)
            self.path_status_text = "Drag A/B, then calculate"
            self.path_status.set(self.path_status_text)
        self.selected_id = next(iter(self.items))
        self.refresh_listbox()

    def _accept_item(self, item):
        kind = item["type"]
        if kind not in ("vehicle", "pallet"):
            raise ValueError(f"Unknown sticker type: {kind}")
        width, height = DEFAULT_VEHICLE_SIZE if kind == "vehicle" else DEFAULT_PALLET_SIZE
        clean = {
            "id": str(item["id"]),
            "type": kind,
            "x_cm": float(item["x_cm"]),
            "y_cm": float(item["y_cm"]),
            "yaw_deg": normalize_yaw(float(item.get("yaw_deg", 0.0))),
            "length_cm": float(item.get("length_cm", width)),
            "width_cm": float(item.get("width_cm", height)),
            "attached_to": item.get("attached_to") if kind == "pallet" else None,
            "offset_x_cm": float(item.get("offset_x_cm", 0.0)),
            "offset_y_cm": float(item.get("offset_y_cm", 0.0)),
            "offset_yaw_deg": normalize_yaw(float(item.get("offset_yaw_deg", 0.0))),
        }
        self.items[clean["id"]] = clean

    def new_item(self, kind, x_cm=60.0, y_cm=80.0, yaw_deg=0.0):
        prefix = "vehicle" if kind == "vehicle" else "pallet"
        number = 1
        while f"{prefix}_{number}" in self.items:
            number += 1
        length, width = DEFAULT_VEHICLE_SIZE if kind == "vehicle" else DEFAULT_PALLET_SIZE
        return {
            "id": f"{prefix}_{number}", "type": kind,
            "x_cm": x_cm, "y_cm": y_cm, "yaw_deg": yaw_deg,
            "length_cm": length, "width_cm": width,
            "attached_to": None, "offset_x_cm": 0.0,
            "offset_y_cm": 0.0, "offset_yaw_deg": 0.0,
        }

    def add_item(self, kind):
        item = self.new_item(kind)
        self.items[item["id"]] = item
        self.selected_id = item["id"]
        self.refresh_listbox()
        self.render_all()
        self.persist("add", immediate=True)

    def delete_selected(self):
        if self.selected_id is None:
            return
        deleted = self.selected_id
        if self.items[deleted]["type"] == "vehicle":
            for item in self.items.values():
                if item.get("attached_to") == deleted:
                    item["attached_to"] = None
        del self.items[deleted]
        self.selected_id = next(iter(self.items), None)
        self.refresh_listbox()
        self.render_all()
        self.persist("delete", immediate=True, extra={"deleted_id": deleted})

    def toggle_recording(self):
        if self.recording:
            self.stop_recording()
        else:
            self.start_recording()

    def start_recording(self):
        session_index = self.next_session_index
        self.next_session_index += 1
        started_at = utc_now()
        self.recording = True
        self.record_session_id = f"session-{session_index:04d}"
        self.record_started_monotonic = time.monotonic()
        self.current_session = {
            "index": session_index,
            "session_id": self.record_session_id,
            "started_at": started_at,
            "ended_at": None,
            "duration_s": None,
            "event_count": 0,
            "comment": self.session_comment.get().strip(),
        }
        self.sessions.append(self.current_session)
        self.save_sessions()
        self.record_button.configure(
            text="■ Stop recording", bg="#b42318", activebackground="#b42318"
        )
        self.record_status.set(f"REC  #{session_index}")
        self.persist("record_start", immediate=True)
        self.refresh_history_tree()

    def stop_recording(self):
        if not self.recording:
            return
        self.current_session["comment"] = self.session_comment.get().strip()
        self.persist("record_stop", immediate=True)
        self.current_session["ended_at"] = utc_now()
        self.current_session["duration_s"] = round(
            time.monotonic() - self.record_started_monotonic, 3
        )
        self.save_sessions()
        self.recording = False
        self.record_session_id = None
        self.record_started_monotonic = None
        self.current_session = None
        self.record_button.configure(
            text="● Start recording", bg="#1f7a3f", activebackground="#1f7a3f"
        )
        self.record_status.set("Not recording")
        self.session_comment.set("")
        self.persist("recording_off", immediate=True)
        self.refresh_history_tree()

    def refresh_listbox(self):
        self.listbox.delete(0, tk.END)
        ids = list(self.items)
        for item_id in ids:
            self.listbox.insert(tk.END, item_id)
        if self.selected_id in ids:
            index = ids.index(self.selected_id)
            self.listbox.selection_set(index)
            self.listbox.see(index)
        self.update_pose_text()
        self.refresh_attachment_control()

    def on_list_select(self, _event):
        selection = self.listbox.curselection()
        if not selection:
            return
        self.selected_id = list(self.items)[selection[0]]
        self.render_all()
        self.refresh_attachment_control()

    def refresh_attachment_control(self):
        if not hasattr(self, "attachment_combo"):
            return
        item = self.items.get(self.selected_id)
        vehicles = [entry["id"] for entry in self.items.values() if entry["type"] == "vehicle"]
        self.attachment_combo.configure(values=["Detached", *vehicles])
        if item is None or item["type"] != "pallet":
            self.attachment_var.set("Detached")
            self.attachment_combo.configure(state="disabled")
            return
        self.attachment_combo.configure(state="readonly")
        self.attachment_var.set(item.get("attached_to") or "Detached")

    def on_attachment_changed(self, _event):
        item = self.items.get(self.selected_id)
        if item is None or item["type"] != "pallet":
            return
        vehicle_id = self.attachment_var.get()
        if vehicle_id == "Detached":
            item["attached_to"] = None
            event = "detach"
        elif vehicle_id in self.items and self.items[vehicle_id]["type"] == "vehicle":
            item["attached_to"] = vehicle_id
            self.update_attachment_offset(item)
            event = "attach"
        else:
            return
        self.render_all()
        self.persist(event, immediate=True)

    def update_attachment_offset(self, pallet):
        vehicle = self.items.get(pallet.get("attached_to"))
        if vehicle is None:
            return
        dx = pallet["x_cm"] - vehicle["x_cm"]
        dy = pallet["y_cm"] - vehicle["y_cm"]
        angle = math.radians(vehicle["yaw_deg"])
        pallet["offset_x_cm"] = math.cos(angle) * dx + math.sin(angle) * dy
        pallet["offset_y_cm"] = -math.sin(angle) * dx + math.cos(angle) * dy
        pallet["offset_yaw_deg"] = normalize_yaw(
            pallet["yaw_deg"] - vehicle["yaw_deg"]
        )

    def sync_attached_pallets(self, vehicle_id):
        vehicle = self.items.get(vehicle_id)
        if vehicle is None:
            return
        angle = math.radians(vehicle["yaw_deg"])
        for pallet in self.items.values():
            if pallet.get("attached_to") != vehicle_id:
                continue
            ox = pallet["offset_x_cm"]
            oy = pallet["offset_y_cm"]
            pallet["x_cm"] = vehicle["x_cm"] + math.cos(angle) * ox - math.sin(angle) * oy
            pallet["y_cm"] = vehicle["y_cm"] + math.sin(angle) * ox + math.cos(angle) * oy
            pallet["yaw_deg"] = normalize_yaw(
                vehicle["yaw_deg"] + pallet["offset_yaw_deg"]
            )

    def after_pose_change(self, item):
        if item["type"] == "vehicle":
            self.sync_attached_pallets(item["id"])
        elif item.get("attached_to"):
            self.update_attachment_offset(item)
        if self.path_points:
            self.path_status_text = "Layout changed — recalculate path"
            self.path_status.set(self.path_status_text)

    def item_id_from_current(self):
        current = self.canvas.find_withtag("current")
        if not current:
            return None
        for tag in self.canvas.gettags(current[0]):
            if tag.startswith("id:"):
                return tag[3:]
        return None

    def select_canvas_item(self):
        item_id = self.item_id_from_current()
        if item_id is None:
            return None
        self.selected_id = item_id
        self.refresh_listbox()
        self.render_all()
        self.canvas.focus_set()
        return self.items[item_id]

    def event_world(self, event):
        canvas_x = self.canvas.canvasx(event.x)
        canvas_y = self.canvas.canvasy(event.y)
        return self.transform.canvas_to_world(canvas_x, canvas_y)

    def on_empty_map_click(self, event):
        if self.dragging_item_id is not None or self.dragging_path_point is not None:
            return
        current = self.canvas.find_withtag("current")
        if current:
            tags = self.canvas.gettags(current[0])
            if "sticker" in tags or "path_point" in tags:
                return
        x_cm, y_cm = self.event_world(event)
        self.last_map_click = [round(x_cm, 2), round(y_cm, 2)]
        self.map_click_text.set(
            f"Map click: x={self.last_map_click[0]:.2f}, "
            f"y={self.last_map_click[1]:.2f} cm"
        )
        self.persist("map_click", immediate=True)

    def start_move(self, event):
        item = self.select_canvas_item()
        if item is None:
            return
        self.dragging_item_id = item["id"]
        self.dragging_path_point = None
        x_cm, y_cm = self.event_world(event)
        self.drag_offset = (item["x_cm"] - x_cm, item["y_cm"] - y_cm)

    def move_drag(self, event):
        if self.dragging_path_point is not None:
            self.move_path_point(event)
            return
        if self.dragging_item_id is None:
            return
        x_cm, y_cm = self.event_world(event)
        item = self.items.get(self.dragging_item_id)
        if item is None:
            return
        item["x_cm"] = min(max(x_cm + self.drag_offset[0], MAP_X_RANGE[0]), MAP_X_RANGE[1])
        item["y_cm"] = min(max(y_cm + self.drag_offset[1], MAP_Y_RANGE[0]), MAP_Y_RANGE[1])
        self.after_pose_change(item)
        self.render_all()
        self.persist("move")

    def start_rotate(self, event):
        item = self.select_canvas_item()
        if item is None:
            return
        self.rotating_item_id = item["id"]
        self.rotating_path_point = None
        pointer_yaw = self.pointer_yaw(event, item)
        self.rotate_start_offset = item["yaw_deg"] - pointer_yaw

    def rotate_drag(self, event):
        if self.rotating_path_point is not None:
            self.rotate_path_point_drag(event)
            return
        if self.rotating_item_id is None:
            return
        item = self.items.get(self.rotating_item_id)
        if item is None:
            return
        item["yaw_deg"] = normalize_yaw(self.pointer_yaw(event, item) + self.rotate_start_offset)
        self.after_pose_change(item)
        self.render_all()
        self.persist("rotate")

    def pointer_yaw(self, event, item):
        x_cm, y_cm = self.event_world(event)
        return math.degrees(math.atan2(y_cm - item["y_cm"], x_cm - item["x_cm"]))

    def path_point_from_current(self):
        current = self.canvas.find_withtag("current")
        if not current:
            return None
        for tag in self.canvas.gettags(current[0]):
            if tag.startswith("point:"):
                return tag[6:]
        return None

    def get_path_point(self, name):
        return self.path_a if name == "A" else self.path_b

    def get_path_yaw(self, name):
        return self.path_a_yaw if name == "A" else self.path_b_yaw

    def set_path_yaw(self, name, yaw):
        yaw = normalize_yaw(yaw)
        if name == "A":
            self.path_a_yaw = yaw
            self.path_a_yaw_var.set(yaw)
        else:
            self.path_b_yaw = yaw
            self.path_b_yaw_var.set(yaw)

    def invalidate_calculated_path(self, message="A/B changed — recalculate"):
        self.path_points = []
        self.path_modes = []
        self.ready_pose = None
        self.last_planner_vehicle_id = None
        self.last_clearance_radius_cm = None
        self.path_status_text = message
        self.path_status.set(message)

    def start_path_point_move(self, event):
        name = self.path_point_from_current()
        if name is None:
            return
        self.dragging_path_point = name
        self.dragging_item_id = None
        x_cm, y_cm = self.event_world(event)
        point = self.get_path_point(name)
        self.path_drag_offset = (point[0] - x_cm, point[1] - y_cm)
        return "break"

    def move_path_point(self, event):
        name = self.dragging_path_point
        if name is None:
            return
        x_cm, y_cm = self.event_world(event)
        point = self.get_path_point(name)
        point[0] = min(max(x_cm + self.path_drag_offset[0], MAP_X_RANGE[0]), MAP_X_RANGE[1])
        point[1] = min(max(y_cm + self.path_drag_offset[1], MAP_Y_RANGE[0]), MAP_Y_RANGE[1])
        self.invalidate_calculated_path()
        self.render_all()
        self.persist(f"move_path_{name.lower()}")

    def start_path_point_rotate(self, event):
        name = self.path_point_from_current()
        if name is None:
            return
        self.rotating_path_point = name
        self.rotating_item_id = None
        x_cm, y_cm = self.event_world(event)
        point = self.get_path_point(name)
        pointer_yaw = math.degrees(math.atan2(y_cm - point[1], x_cm - point[0]))
        self.path_rotate_start_offset = self.get_path_yaw(name) - pointer_yaw
        return "break"

    def rotate_path_point_drag(self, event):
        name = self.rotating_path_point
        if name is None:
            return
        x_cm, y_cm = self.event_world(event)
        point = self.get_path_point(name)
        pointer_yaw = math.degrees(math.atan2(y_cm - point[1], x_cm - point[0]))
        self.set_path_yaw(name, pointer_yaw + self.path_rotate_start_offset)
        self.invalidate_calculated_path()
        self.render_all()
        self.persist(f"rotate_path_{name.lower()}")

    def rotate_path_point(self, name, delta):
        self.set_path_yaw(name, self.get_path_yaw(name) + delta)
        self.invalidate_calculated_path()
        self.render_all()
        self.persist(f"rotate_path_{name.lower()}", immediate=True)

    def middle_click_path_point(self, _event):
        name = self.path_point_from_current()
        if name is None:
            return
        self.rotate_path_point(name, 90.0)
        return "break"

    def end_interaction(self, _event):
        had_interaction = any((
            self.dragging_item_id, self.rotating_item_id,
            self.dragging_path_point, self.rotating_path_point,
        ))
        self.dragging_item_id = None
        self.rotating_item_id = None
        self.dragging_path_point = None
        self.rotating_path_point = None
        if not had_interaction:
            return
        self.persist("interaction_end", immediate=True)

    def wheel_rotate(self, event, delta):
        path_point = self.path_point_from_current()
        if path_point is not None:
            self.rotate_path_point(path_point, delta)
            return "break"
        item_id = self.item_id_from_current()
        if item_id is None:
            return
        self.selected_id = item_id
        self.rotate_selected(delta)
        return "break"

    def mousewheel_rotate(self, event):
        delta = 5 if event.delta > 0 else -5
        return self.wheel_rotate(event, delta)

    def middle_click_rotate(self, _event):
        item = self.select_canvas_item()
        if item is None:
            return
        item["yaw_deg"] = normalize_yaw(item["yaw_deg"] + 90.0)
        self.after_pose_change(item)
        self.render_all()
        self.persist("rotate_90", immediate=True)
        return "break"

    def rotate_selected(self, delta):
        if self.selected_id is None:
            return
        item = self.items[self.selected_id]
        item["yaw_deg"] = normalize_yaw(item["yaw_deg"] + delta)
        self.after_pose_change(item)
        self.refresh_listbox()
        self.render_all()
        self.persist("rotate", immediate=True)

    def on_key(self, event):
        if self.selected_id is None or event.keysym not in ("Left", "Right", "Up", "Down"):
            return
        step = 5.0 if (event.state & 0x0001) else 1.0
        dx = step if event.keysym == "Right" else -step if event.keysym == "Left" else 0.0
        dy = step if event.keysym == "Up" else -step if event.keysym == "Down" else 0.0
        item = self.items[self.selected_id]
        item["x_cm"] = min(max(item["x_cm"] + dx, MAP_X_RANGE[0]), MAP_X_RANGE[1])
        item["y_cm"] = min(max(item["y_cm"] + dy, MAP_Y_RANGE[0]), MAP_Y_RANGE[1])
        self.after_pose_change(item)
        self.render_all()
        self.persist("key_move", immediate=True)
        return "break"

    def render_all(self):
        self.canvas.delete("sticker")
        self.canvas.delete("path_overlay")
        self.render_path_overlay()
        for item in self.items.values():
            self.render_item(item)
        self.canvas.tag_raise("path_point")
        self.update_pose_text()

    def render_path_overlay(self):
        if len(self.path_points) >= 2:
            vehicle = self.items.get(self.last_planner_vehicle_id)
            if vehicle and all(len(pose) >= 3 for pose in self.path_points):
                sampled = []
                for start, end in zip(self.path_points, self.path_points[1:]):
                    distance = math.hypot(end[0] - start[0], end[1] - start[1])
                    yaw_delta = normalize_yaw(end[2] - start[2])
                    steps = max(1, math.ceil(distance / 3.0), math.ceil(abs(yaw_delta) / 8.0))
                    for index in range(steps):
                        ratio = index / steps
                        sampled.append((
                            start[0] + (end[0] - start[0]) * ratio,
                            start[1] + (end[1] - start[1]) * ratio,
                            normalize_yaw(start[2] + yaw_delta * ratio),
                        ))
                sampled.append(tuple(self.path_points[-1]))
                for pose in sampled:
                    for polygon in self.moving_polygons(
                        *pose, vehicle, max(0.0, self.planner_margin.get())
                    ):
                        canvas_polygon = []
                        for x_cm, y_cm in polygon:
                            canvas_polygon.extend(self.transform.world_to_canvas(x_cm, y_cm))
                        self.canvas.create_polygon(
                            canvas_polygon, fill="#8ec5ff", outline="#5599dd",
                            width=1, stipple="gray50", tags=("path_overlay",),
                        )
            modes = self.path_modes
            if len(modes) != len(self.path_points) - 1:
                modes = ["F"] * (len(self.path_points) - 1)
            colors = {"R": "#e07a16", "READY": "#7c3aed", "TURN": "#7c3aed", "F": "#0066ff"}
            for index, (start, end, mode) in enumerate(
                zip(self.path_points, self.path_points[1:], modes)
            ):
                start_canvas = self.transform.world_to_canvas(start[0], start[1])
                end_canvas = self.transform.world_to_canvas(end[0], end[1])
                self.canvas.create_line(
                    *start_canvas, *end_canvas, fill=colors.get(mode, "#0066ff"),
                    width=5, arrow=tk.LAST if index == len(modes) - 1 else tk.NONE,
                    tags=("path_overlay",),
                )
        if self.ready_pose is not None:
            ready_x, ready_y = self.transform.world_to_canvas(
                self.ready_pose[0], self.ready_pose[1]
            )
            self.canvas.create_text(
                ready_x, ready_y - 12, text="READY", fill="#7c3aed",
                font=("Sans", 9, "bold"), tags=("path_overlay",),
            )
        for label, point, yaw, color in (
            ("A", self.path_a, self.path_a_yaw, "#15803d"),
            ("B", self.path_b, self.path_b_yaw, "#b42318"),
        ):
            if point is None:
                continue
            x_canvas, y_canvas = self.transform.world_to_canvas(point[0], point[1])
            radius_px = max(7, self.transform.scale * 2.0)
            point_tags = ("path_overlay", "path_point", f"point:{label}")
            self.canvas.create_oval(
                x_canvas - radius_px, y_canvas - radius_px,
                x_canvas + radius_px, y_canvas + radius_px,
                fill=color, outline="white", width=2, tags=point_tags,
            )
            self.canvas.create_text(
                x_canvas, y_canvas, text=label, fill="white",
                font=("Sans", 10, "bold"), tags=point_tags,
            )
            arrow_x, arrow_y = self.transform.world_to_canvas(
                point[0] + 8.0 * math.cos(math.radians(yaw)),
                point[1] + 8.0 * math.sin(math.radians(yaw)),
            )
            self.canvas.create_line(
                x_canvas, y_canvas, arrow_x, arrow_y, fill=color, width=4,
                arrow=tk.LAST, tags=point_tags,
            )

    def render_item(self, item):
        cx, cy = self.transform.world_to_canvas(item["x_cm"], item["y_cm"])
        length = item["length_cm"] * self.transform.scale
        width = item["width_cm"] * self.transform.scale
        angle = math.radians(item["yaw_deg"])
        ux, uy = math.cos(angle), -math.sin(angle)
        vx, vy = -uy, ux
        points = []
        for forward, side in ((1, 1), (1, -1), (-1, -1), (-1, 1)):
            points.extend((
                cx + forward * length / 2 * ux + side * width / 2 * vx,
                cy + forward * length / 2 * uy + side * width / 2 * vy,
            ))
        selected = item["id"] == self.selected_id
        fill = "#ff7272" if item["type"] == "vehicle" else "#e9aa52"
        outline = "#0066ff" if selected else "#5d1c0b"
        common_tags = ("sticker", f"id:{item['id']}")
        self.canvas.create_polygon(
            points, fill=fill, outline=outline, width=4 if selected else 2,
            stipple="gray25", tags=common_tags,
        )
        front_x = cx + length / 2 * ux
        front_y = cy + length / 2 * uy
        self.canvas.create_line(
            cx, cy, front_x, front_y, fill="#111", width=4,
            arrow=tk.LAST, arrowshape=(12, 14, 5), tags=common_tags,
        )
        label = item["id"]
        if item.get("attached_to"):
            label += f"\n→ {item['attached_to']}"
        self.canvas.create_text(
            cx, cy, text=label, fill="#111", font=("Sans", 10, "bold"),
            tags=common_tags,
        )

    def update_pose_text(self):
        if self.selected_id is None:
            self.pose_text.set("No selection")
            return
        item = self.items[self.selected_id]
        attachment = item.get("attached_to") or "-"
        self.pose_text.set(
            f"id:  {item['id']}\n"
            f"x:   {item['x_cm']:.2f} cm\n"
            f"y:   {item['y_cm']:.2f} cm\n"
            f"yaw: {item['yaw_deg']:.1f}°\n"
            f"to:  {attachment}"
        )

    def snapshot(self):
        return {
            "schema": "navigation-pose-board/v1",
            "updated_at": utc_now(),
            "coordinate_frame": {
                "name": "main_floor_cm",
                "origin": "lower-left of main floor",
                "x_axis": "right",
                "y_axis": "up",
                "yaw": "degrees, counter-clockwise from +X",
            },
            "map": str(self.args.map_svg.resolve()),
            "selected_id": self.selected_id,
            "recording": self.recording,
            "record_session_id": self.record_session_id,
            "last_map_click_cm": self.last_map_click,
            "items": list(self.items.values()),
            "path_planner": {
                "point_a": self.path_a,
                "point_b": self.path_b,
                "point_a_yaw_deg": self.path_a_yaw,
                "point_b_yaw_deg": self.path_b_yaw,
                "path": self.path_points,
                "path_modes": self.path_modes,
                "ready_pose": self.ready_pose,
                "status": self.path_status_text,
                "vehicle_id": self.last_planner_vehicle_id,
                "clearance_radius_cm": self.last_clearance_radius_cm,
                "reference_point": "front-face center",
                "motion_constraint": "straight reverse escape, then turn/transit, front-entry at B",
                "collision_model": "oriented rectangles with swept transition checks",
                "grid_resolution_cm": PLANNER_GRID_CM,
                "safety_margin_cm": self.planner_margin.get(),
            },
        }

    def persist(self, event_name, immediate=False, extra=None):
        self.pending_event = event_name
        if extra:
            self.pending_extra = extra
        else:
            self.pending_extra = {}
        if immediate:
            if self.write_after_id is not None:
                self.root.after_cancel(self.write_after_id)
                self.write_after_id = None
            self._write_files()
        elif self.write_after_id is None:
            # Throttle rather than debounce: a long drag remains visible to CLI readers.
            self.write_after_id = self.root.after(60, self._write_files)

    def _write_files(self):
        self.write_after_id = None
        snapshot = self.snapshot()
        self.state_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.state_path.with_suffix(self.state_path.suffix + ".tmp")
        temporary.write_text(
            json.dumps(snapshot, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        os.replace(temporary, self.state_path)
        if self.recording:
            selected = self.items.get(self.selected_id)
            event = {
                "timestamp": snapshot["updated_at"],
                "elapsed_s": round(time.monotonic() - self.record_started_monotonic, 3),
                "session_id": self.record_session_id,
                "session_index": self.current_session["index"],
                "event": self.pending_event,
                "selected_id": self.selected_id,
                "pose": dict(selected) if selected else None,
                **getattr(self, "pending_extra", {}),
            }
            if self.pending_event in ("record_start", "record_stop"):
                event["items"] = snapshot["items"]
                event["comment"] = self.current_session.get("comment", "")
            self.events_path.parent.mkdir(parents=True, exist_ok=True)
            with self.events_path.open("a", encoding="utf-8") as stream:
                stream.write(
                    json.dumps(event, ensure_ascii=False, separators=(",", ":")) + "\n"
                )
            self.current_session["event_count"] += 1
        self.save_status.set(f"saved {snapshot['updated_at'][11:23]} UTC")

    def close(self):
        if self.write_after_id is not None:
            self.root.after_cancel(self.write_after_id)
            self.write_after_id = None
        if self.recording:
            self.stop_recording()
        self.pending_event = "close"
        self.pending_extra = {}
        self._write_files()
        self.root.destroy()


def parse_args():
    project_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--map-svg", type=Path,
        default=project_root / "docs" / "navigation_layout_1to1.svg",
    )
    parser.add_argument(
        "--map-png", type=Path,
        default=project_root / "docs" / "navigation_layout_1to1.png",
    )
    parser.add_argument(
        "--state", type=Path,
        default=project_root / "runtime" / "navigation_pose_live.json",
    )
    parser.add_argument(
        "--events", type=Path,
        default=project_root / "runtime" / "navigation_pose_events.jsonl",
    )
    parser.add_argument(
        "--sessions", type=Path,
        default=project_root / "runtime" / "navigation_pose_sessions.json",
    )
    parser.add_argument(
        "--zoom", type=int, choices=ZOOM_OPTIONS, default=50,
        help="initial map zoom percentage (default: 50)",
    )
    parser.add_argument(
        "--planner-margin", type=float, default=2.0,
        help="extra obstacle clearance in centimetres (default: 2.0)",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    for path in (args.map_svg, args.map_png):
        if not path.exists():
            raise SystemExit(f"Map file not found: {path}")
    root = tk.Tk()
    PoseEditor(root, args)
    root.mainloop()


if __name__ == "__main__":
    main()
