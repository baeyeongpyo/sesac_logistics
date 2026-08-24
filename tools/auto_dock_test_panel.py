#!/usr/bin/env python3
"""Manual ROS event panel for real-vehicle auto-dock integration tests.

This tool does not simulate vehicle motion and never publishes cmd_vel. It only
injects the Nav2/Fork events that auto_dock normally receives and passively
shows auto_dock outputs plus the latest YOLO perception message.
"""

import argparse
import json
import os
import tkinter as tk
from datetime import datetime
from tkinter import ttk

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty, String


SYMBOLS = ("spade", "heart", "clover", "diamond", "star")
LOCATIONS = ("DOCK_1", "NORMAL", "FRESH", "Y1", "Y2", "Y3", "Y4")


class TestPanelNode(Node):
    def __init__(self, vehicle, log_callback, yolo_callback):
        super().__init__("auto_dock_test_panel")
        robot = f"/robot_{vehicle}"
        self.log_callback = log_callback
        self.yolo_callback = yolo_callback

        self.arrival_pub = self.create_publisher(String, f"{robot}/nav2/arrival", 10)
        self.approach_result_pub = self.create_publisher(
            String, f"{robot}/nav2/approach_result", 10
        )
        self.fork_state_pub = self.create_publisher(String, f"{robot}/fork/state", 10)
        self.legacy_up_pub = self.create_publisher(Empty, f"{robot}/lift/up_complete", 10)
        self.stop_pub = self.create_publisher(Empty, f"{robot}/auto_dock/stop", 10)

        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            String, f"{robot}/auto_dock/status",
            lambda msg: self.received(f"{robot}/auto_dock/status", msg.data),
            status_qos,
        )
        self.create_subscription(
            String, "/fork/command",
            lambda msg: self.received("/fork/command", msg.data), 10,
        )
        self.create_subscription(
            String, f"{robot}/fork/state",
            lambda msg: self.received(f"{robot}/fork/state", msg.data), 10,
        )
        self.create_subscription(
            PoseStamped, f"{robot}/nav2/approach_goal",
            lambda msg: self.received(
                f"{robot}/nav2/approach_goal",
                (
                    f"frame={msg.header.frame_id} "
                    f"x={msg.pose.position.x:.3f} y={msg.pose.position.y:.3f} "
                    f"qz={msg.pose.orientation.z:.3f} qw={msg.pose.orientation.w:.3f}"
                ),
            ), 10,
        )
        self.create_subscription(
            Empty, f"{robot}/auto_dock/entry_complete",
            lambda _msg: self.received(f"{robot}/auto_dock/entry_complete", "<Empty>"), 10,
        )
        self.create_subscription(
            Empty, f"{robot}/auto_dock/drive_ready",
            lambda _msg: self.received(f"{robot}/auto_dock/drive_ready", "<Empty>"), 10,
        )
        self.create_subscription(
            String, f"{robot}/symbol_seg/detections", self.received_yolo, 10,
        )

    def received(self, topic, value):
        self.log_callback("SUB", topic, value)

    def received_yolo(self, message):
        self.yolo_callback(message.data)

    def publish_arrival(self, payload):
        data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self.arrival_pub.publish(String(data=data))
        self.log_callback("PUB", self.arrival_pub.topic_name, data)

    def publish_legacy_arrival(self, left, right):
        data = f"arrived {left} {right}"
        self.arrival_pub.publish(String(data=data))
        self.log_callback("PUB", self.arrival_pub.topic_name, data)

    def publish_approach_result(self, status, reason):
        data = json.dumps(
            {"status": status, "reason": reason, "source": "auto_dock_test_panel"},
            separators=(",", ":"),
        )
        self.approach_result_pub.publish(String(data=data))
        self.log_callback("PUB", self.approach_result_pub.topic_name, data)

    def publish_fork_state(self, state, error=""):
        data = json.dumps({"state": state, "error": error}, separators=(",", ":"))
        self.fork_state_pub.publish(String(data=data))
        self.log_callback("PUB", self.fork_state_pub.topic_name, data)

    def publish_legacy_up(self):
        self.legacy_up_pub.publish(Empty())
        self.log_callback("PUB", self.legacy_up_pub.topic_name, "<Empty>")

    def publish_stop(self):
        self.stop_pub.publish(Empty())
        self.log_callback("PUB", self.stop_pub.topic_name, "<Empty>")


class TestPanel:
    def __init__(self, root, args):
        self.root = root
        self.args = args
        self.enabled = tk.BooleanVar(value=False)
        self.arrival_status = tk.StringVar(value="SUCCEEDED")
        self.arrival_reason = tk.StringVar(value="manual_test_failure")
        self.location = tk.StringVar(value="DOCK_1")
        self.operation = tk.StringVar(value="PICK")
        self.product_type = tk.StringVar(value="NORMAL")
        self.target_type = tk.StringVar(value="SYMBOLS")
        self.left_symbol = tk.StringVar(value="spade")
        self.right_symbol = tk.StringVar(value="spade")
        self.slot_id = tk.StringVar(value="AUTO")
        self.error_text = tk.StringVar(value="test failure")
        self.last_yolo_signature = None

        root.title(f"AUTO-DOCK REAL TEST PANEL · robot_{args.vehicle}")
        root.geometry("1440x820")
        root.minsize(1080, 620)
        root.protocol("WM_DELETE_WINDOW", self.close)

        self.node = TestPanelNode(args.vehicle, self.append_log, self.update_yolo)
        self.action_buttons = []
        self.build_ui()
        self.update_enabled()
        self.root.after(20, self.poll_ros)

    def build_ui(self):
        warning = tk.Label(
            self.root,
            text=(
                "REAL VEHICLE TEST · 이 GUI는 실제 ROS 토픽을 발행합니다.\n"
                "cmd_vel은 발행하지 않지만 auto_dock이 수신하면 실차가 움직일 수 있습니다."
            ),
            bg="#8b1e1e", fg="white", font=("DejaVu Sans", 11, "bold"), pady=8,
        )
        warning.pack(fill="x")

        enable = tk.Checkbutton(
            self.root, text="실차 이벤트 발행 활성화", variable=self.enabled,
            command=self.update_enabled, fg="#8b1e1e", font=("DejaVu Sans", 10, "bold"),
        )
        enable.pack(anchor="w", padx=12, pady=(8, 2))

        panes = ttk.Panedwindow(self.root, orient="horizontal")
        panes.pack(fill="both", expand=True, padx=8, pady=(2, 8))
        controls = ttk.Frame(panes)
        monitor = ttk.Frame(panes)
        panes.add(controls, weight=3)
        panes.add(monitor, weight=2)

        nav = ttk.LabelFrame(controls, text="Nav2 담당자 대신 arrival 발행", padding=10)
        nav.pack(fill="x", padx=12, pady=6)
        fields = (
            ("status", self.arrival_status, ("SUCCEEDED", "FAILED"), "readonly"),
            ("location", self.location, LOCATIONS, "normal"),
            ("operation", self.operation, ("PICK", "PLACE"), "readonly"),
            ("product_type", self.product_type, ("NORMAL", "FRESH"), "readonly"),
            ("target type", self.target_type, ("SYMBOLS", "SLOT", "AUTO_SLOT", "NONE"), "readonly"),
            ("left", self.left_symbol, SYMBOLS, "readonly"),
            ("right", self.right_symbol, SYMBOLS, "readonly"),
        )
        for column, (label, variable, values, state) in enumerate(fields):
            ttk.Label(nav, text=label).grid(row=0, column=column, sticky="w", padx=3)
            ttk.Combobox(
                nav, textvariable=variable, values=values, state=state, width=11,
            ).grid(row=1, column=column, sticky="ew", padx=3)
            nav.columnconfigure(column, weight=1)
        ttk.Label(nav, text="slot id").grid(row=2, column=0, sticky="w", padx=3, pady=(8, 0))
        ttk.Label(nav, text="FAILED reason").grid(
            row=2, column=3, sticky="w", padx=3, pady=(8, 0)
        )
        ttk.Entry(nav, textvariable=self.slot_id).grid(
            row=3, column=0, columnspan=3, sticky="ew", padx=3
        )
        ttk.Entry(nav, textvariable=self.arrival_reason).grid(
            row=3, column=3, columnspan=2, sticky="ew", padx=3
        )
        self.add_action_button(
            nav, "Publish structured arrival", self.publish_arrival,
            row=3, column=5, columnspan=2,
        )
        self.add_action_button(
            nav, "Legacy: arrived <left> <right>", self.publish_legacy_arrival,
            row=4, column=0, columnspan=7, pady=(8, 0),
        )

        approach = ttk.LabelFrame(
            controls, text="Nav2 담당자 대신 approach_result 발행", padding=10
        )
        approach.pack(fill="x", padx=12, pady=6)
        self.add_action_button(
            approach, "SUCCEEDED", lambda: self.publish_approach_result("succeeded"),
            row=0, column=0, padx=3,
        )
        self.add_action_button(
            approach, "FAILED", lambda: self.publish_approach_result("failed"),
            row=0, column=1, padx=3,
        )
        approach.columnconfigure(0, weight=1)
        approach.columnconfigure(1, weight=1)

        quick = ttk.LabelFrame(controls, text="빠른 Arrival", padding=10)
        quick.pack(fill="x", padx=12, pady=6)
        scenarios = (
            ("DOCK PICK", "DOCK_1", "PICK", "NORMAL", "SYMBOLS", "AUTO"),
            ("NORMAL PLACE", "NORMAL", "PLACE", "NORMAL", "AUTO_SLOT", "AUTO"),
            ("FRESH PLACE", "FRESH", "PLACE", "FRESH", "AUTO_SLOT", "AUTO"),
            ("FRESH PICK", "FRESH", "PICK", "FRESH", "AUTO_SLOT", "AUTO"),
            ("Y1 PLACE", "Y1", "PLACE", "FRESH", "SLOT", "Y1"),
            ("Y2 PLACE", "Y2", "PLACE", "FRESH", "SLOT", "Y2"),
            ("Y3 PLACE", "Y3", "PLACE", "FRESH", "SLOT", "Y3"),
            ("Y4 PLACE", "Y4", "PLACE", "FRESH", "SLOT", "Y4"),
        )
        for index, scenario in enumerate(scenarios):
            self.add_action_button(
                quick, scenario[0], lambda values=scenario: self.publish_scenario(values),
                row=index // 4, column=index % 4, padx=3, pady=3,
            )
            quick.columnconfigure(index % 4, weight=1)

        fork = ttk.LabelFrame(controls, text="Fork 담당자 대신 완료 발행", padding=10)
        fork.pack(fill="x", padx=12, pady=6)
        for column, (text, command) in enumerate((
            ("UP_COMPLETE", lambda: self.publish_fork("UP_COMPLETE")),
            ("DOWN_COMPLETE", lambda: self.publish_fork("DOWN_COMPLETE")),
            ("FAILED", lambda: self.publish_fork("FAILED")),
            ("Legacy UP <Empty>", self.publish_legacy_up),
        )):
            self.add_action_button(fork, text, command, row=0, column=column, padx=3)
            fork.columnconfigure(column, weight=1)
        ttk.Entry(fork, textvariable=self.error_text).grid(
            row=1, column=0, columnspan=4, sticky="ew", padx=3, pady=(7, 0)
        )

        stop = tk.Button(
            controls, text="■ AUTO-DOCK STOP", command=self.publish_stop,
            bg="#b42318", fg="white", activebackground="#8b1e1e",
            font=("DejaVu Sans", 12, "bold"), pady=7,
        )
        stop.pack(fill="x", padx=12, pady=6)

        yolo_frame = ttk.LabelFrame(
            monitor,
            text=f"YOLO 최신 인식 · /robot_{self.args.vehicle}/symbol_seg/detections",
            padding=6,
        )
        yolo_frame.pack(fill="both", expand=True, padx=4, pady=(6, 3))
        self.yolo = tk.Text(
            yolo_frame, height=17, wrap="word", state="disabled",
            font=("DejaVu Sans Mono", 9), bg="#071a12", fg="#d1fae5",
        )
        yolo_scrollbar = ttk.Scrollbar(
            yolo_frame, orient="vertical", command=self.yolo.yview
        )
        self.yolo.configure(yscrollcommand=yolo_scrollbar.set)
        self.yolo.pack(side="left", fill="both", expand=True)
        yolo_scrollbar.pack(side="right", fill="y")

        log_frame = ttk.LabelFrame(
            monitor, text="토픽 송수신 로그 · Auto Dock status 포함", padding=6
        )
        log_frame.pack(fill="both", expand=True, padx=4, pady=(3, 6))
        self.log = tk.Text(
            log_frame, height=14, wrap="word", state="disabled",
            font=("DejaVu Sans Mono", 9), bg="#111827", fg="#e5e7eb",
        )
        scrollbar = ttk.Scrollbar(log_frame, orient="vertical", command=self.log.yview)
        self.log.configure(yscrollcommand=scrollbar.set)
        self.log.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

    def add_action_button(self, parent, text, command, **grid):
        button = ttk.Button(parent, text=text, command=command)
        button.grid(sticky="ew", **grid)
        self.action_buttons.append(button)
        return button

    def update_enabled(self):
        state = "normal" if self.enabled.get() else "disabled"
        for button in self.action_buttons:
            button.configure(state=state)

    def require_enabled(self):
        return self.enabled.get()

    def target_payload(self):
        kind = self.target_type.get()
        if kind == "SYMBOLS":
            return {"type": kind, "left": self.left_symbol.get(), "right": self.right_symbol.get()}
        if kind == "SLOT":
            return {"type": kind, "slot_id": self.slot_id.get().strip()}
        if kind == "AUTO_SLOT":
            return {"type": kind}
        return None

    def arrival_payload(self):
        payload = {
            "status": self.arrival_status.get(),
            "location": self.location.get(),
            "operation": self.operation.get(),
            "product_type": self.product_type.get(),
            "target": self.target_payload(),
        }
        if self.arrival_status.get() == "FAILED":
            payload["reason"] = self.arrival_reason.get().strip()
        return payload

    def publish_arrival(self):
        if self.require_enabled():
            self.node.publish_arrival(self.arrival_payload())

    def publish_legacy_arrival(self):
        if self.require_enabled():
            self.node.publish_legacy_arrival(self.left_symbol.get(), self.right_symbol.get())

    def publish_approach_result(self, status):
        if self.require_enabled():
            reason = "manual_test_success" if status == "succeeded" else "manual_test_failure"
            self.node.publish_approach_result(status, reason)

    def publish_scenario(self, values):
        if not self.require_enabled():
            return
        _, location, operation, product, target_type, slot = values
        self.arrival_status.set("SUCCEEDED")
        self.location.set(location)
        self.operation.set(operation)
        self.product_type.set(product)
        self.target_type.set(target_type)
        self.slot_id.set(slot)
        self.node.publish_arrival(self.arrival_payload())

    def publish_fork(self, state):
        if self.require_enabled():
            error = self.error_text.get().strip() if state == "FAILED" else ""
            self.node.publish_fork_state(state, error)

    def publish_legacy_up(self):
        if self.require_enabled():
            self.node.publish_legacy_up()

    def publish_stop(self):
        self.node.publish_stop()

    def append_log(self, direction, topic, value):
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        line = f"{timestamp} {direction:<3} {topic}\n    {value}\n"
        self.log.configure(state="normal")
        self.log.insert("end", line)
        self.log.see("end")
        self.log.configure(state="disabled")

    def update_yolo(self, raw):
        try:
            payload = json.loads(raw)
            target = payload.get("target_top")
            candidate = payload.get("candidate")
            partial = payload.get("tracked_partial")
            detections = payload.get("detections") or []
            seen = [
                f"{item.get('class', '?')} {float(item.get('confidence', 0.0)):.2f}"
                for item in detections
                if isinstance(item, dict)
            ]
            rendered = "\n".join((
                f"수신 시각: {datetime.now().strftime('%H:%M:%S.%f')[:-3]}",
                f"요청 target_top: {target}",
                f"확정 candidate: {'없음' if candidate is None else candidate}",
                f"부분 추적: {'없음' if partial is None else partial}",
                f"화면 검출({len(seen)}): {', '.join(seen) if seen else '없음'}",
                "",
                "원본 JSON",
                json.dumps(payload, ensure_ascii=False, indent=2),
            ))
            signature = json.dumps(
                {"target_top": target, "candidate": candidate},
                ensure_ascii=False, sort_keys=True, default=str,
            )
            if signature != self.last_yolo_signature:
                self.last_yolo_signature = signature
                state = "NONE" if candidate is None else "FOUND"
                self.append_log(
                    "SUB", f"/robot_{self.args.vehicle}/symbol_seg/detections",
                    f"target_top={target} candidate={state}",
                )
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            rendered = f"YOLO JSON 해석 실패: {exc}\n\n{raw}"
        self.yolo.configure(state="normal")
        self.yolo.delete("1.0", "end")
        self.yolo.insert("1.0", rendered)
        self.yolo.configure(state="disabled")

    def poll_ros(self):
        if not rclpy.ok():
            return
        rclpy.spin_once(self.node, timeout_sec=0.0)
        self.root.after(20, self.poll_ros)

    def close(self):
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self.root.destroy()


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--vehicle", type=int, choices=(1, 2), default=1)
    parser.add_argument("--ros-domain-id", type=int)
    return parser.parse_args()


def main():
    args = parse_args()
    os.environ["ROS_DOMAIN_ID"] = str(args.ros_domain_id or 214 + args.vehicle)
    rclpy.init()
    root = tk.Tk()
    TestPanel(root, args)
    root.mainloop()


if __name__ == "__main__":
    main()
