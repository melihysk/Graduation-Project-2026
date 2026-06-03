"""
DKR/iDKR modları için bağımsız görev gönderici.

RMF task API'si KULLANMAZ. Görevleri /dkr_task_requests topic'ine
JSON String olarak yayınlar. DKR standalone traffic manager bu
topic'i dinleyerek görevleri robotlara atar.

Görev tamamlanma takibi /dkr_events topic'inden yapılır.
"""

from __future__ import annotations

import json
import time
import uuid

import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSHistoryPolicy as History,
    QoSReliabilityPolicy as Reliability,
    QoSDurabilityPolicy as Durability,
)
from std_msgs.msg import String


class TaskDispatcherDkr(Node):

    def __init__(self):
        super().__init__("task_dispatcher_dkr")

        self.declare_parameter("scenario_file", "")
        self.declare_parameter("delay_between_tasks_sec", 2.0)
        self.declare_parameter("max_pending", 6)

        scenario_file = self.get_parameter("scenario_file").value
        self._delay = self.get_parameter("delay_between_tasks_sec").value
        self._max_pending = self.get_parameter("max_pending").value

        if not scenario_file:
            self.get_logger().error("scenario_file parameter is required!")
            raise SystemExit(1)

        self._tasks = self._load_scenario(scenario_file)
        self._current_idx = 0
        self._completed_count = 0
        self._dispatched_ids: set[str] = set()

        reliable_qos = QoSProfile(
            history=History.KEEP_LAST,
            depth=10,
            reliability=Reliability.RELIABLE,
            durability=Durability.VOLATILE,
        )

        self._task_pub = self.create_publisher(
            String, "/dkr_task_requests", reliable_qos,
        )
        self.create_subscription(
            String, "/dkr_events", self._on_dkr_event, reliable_qos,
        )

        self._start_time = time.time()
        self.create_timer(self._delay, self._dispatch_loop)

        self.get_logger().info(
            f"DKR TaskDispatcher loaded {len(self._tasks)} tasks "
            f"from {scenario_file} (max_pending={self._max_pending})"
        )

    def _load_scenario(self, filepath: str) -> list[dict]:
        with open(filepath, "r") as f:
            data = yaml.safe_load(f)
        return data.get("tasks", [])

    def _on_dkr_event(self, msg: String):
        try:
            event = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        if event.get("type") == "task_completed":
            task_id = event.get("task_id", "")
            if task_id in self._dispatched_ids:
                self._completed_count += 1
                remaining = len(self._tasks) - self._current_idx
                self.get_logger().info(
                    f"Task completed ({self._completed_count} total)"
                    + (f", {remaining} still to dispatch" if remaining > 0 else "")
                )

    def _dispatch_loop(self):
        if time.time() - self._start_time < 5.0:
            return

        if self._current_idx >= len(self._tasks):
            return

        if self._max_pending > 0:
            pending = self._current_idx - self._completed_count
            if pending >= self._max_pending:
                return

        task_def = self._tasks[self._current_idx]
        self._dispatch_task(task_def)
        self._current_idx += 1

    def _dispatch_task(self, task_def: dict):
        task_id = f"dkr-{uuid.uuid4().hex[:10]}"
        task_type = task_def.get("type", "delivery")

        if task_type == "delivery":
            payload = {
                "task_id": task_id,
                "type": "delivery",
                "pickup_place": task_def["pickup"]["place"],
                "dropoff_place": task_def["dropoff"]["place"],
                "priority": task_def.get("priority", 0),
            }
        elif task_type == "patrol":
            payload = {
                "task_id": task_id,
                "type": "patrol",
                "places": task_def["places"],
                "rounds": task_def.get("rounds", 1),
                "priority": task_def.get("priority", 0),
            }
        else:
            self.get_logger().warn(f"Unknown task type: {task_type}")
            return

        msg = String()
        msg.data = json.dumps(payload)
        self._task_pub.publish(msg)
        self._dispatched_ids.add(task_id)

        self.get_logger().info(
            f"Dispatched task {self._current_idx + 1}/{len(self._tasks)}: "
            f"{task_type} — {task_def.get('description', '')}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = TaskDispatcherDkr()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
