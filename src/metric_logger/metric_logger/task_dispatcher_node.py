"""
Senaryo bazlı görev gönderici node.

YAML dosyasından sabit görev listesini okur ve RMF task API'sine gönderir.
Her senaryo için farklı YAML dosyası kullanılır.

max_pending parametresi ile RMF planner'ın toplu planlama limitine takılması
önlenir: aynı anda en fazla max_pending kadar görev kuyrukta bekler, geri
kalanı tamamlanma sinyali gelince gönderilir.

Kullanım:
  ros2 run metric_logger task_dispatcher_node \
    --ros-args -p scenario_file:=/path/to/scenario.yaml -p max_pending:=6
"""

import json
import time
import uuid
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy as Reliability

from rmf_fleet_msgs.msg import FleetState
from rmf_task_msgs.msg import ApiRequest, ApiResponse


class TaskDispatcherNode(Node):

    def __init__(self):
        super().__init__('task_dispatcher_scenario')

        self.declare_parameter('scenario_file', '')
        self.declare_parameter('delay_between_tasks_sec', 2.0)
        self.declare_parameter('batch_mode', False)
        self.declare_parameter('max_pending', 0)

        scenario_file = self.get_parameter('scenario_file').value
        self._delay = self.get_parameter('delay_between_tasks_sec').value
        self._batch_mode = self.get_parameter('batch_mode').value
        self._max_pending = self.get_parameter('max_pending').value

        if not scenario_file:
            self.get_logger().error('scenario_file parameter is required!')
            raise SystemExit(1)

        self._tasks = self._load_scenario(scenario_file)
        self._current_task_idx = 0
        self._dispatched_ids: list[str] = []
        self._waiting_for_ack = False

        # Track completions via fleet_states so we can throttle dispatch.
        self._robot_prev_task_ids: dict[str, str] = {}
        self._completed_count = 0

        # RMF task API aboneleri TRANSIENT_LOCAL bekler (rmf_demos dispatch_* ile aynı).
        task_api_pub_qos = QoSProfile(
            history=History.KEEP_LAST,
            depth=1,
            reliability=Reliability.RELIABLE,
            durability=Durability.TRANSIENT_LOCAL,
        )
        task_api_sub_qos = QoSProfile(
            history=History.KEEP_LAST,
            depth=10,
            reliability=Reliability.RELIABLE,
            durability=Durability.VOLATILE,
        )
        fleet_qos = QoSProfile(
            history=History.KEEP_LAST,
            depth=10,
            reliability=Reliability.RELIABLE,
            durability=Durability.VOLATILE,
        )
        self._pub = self.create_publisher(
            ApiRequest, '/task_api_requests', task_api_pub_qos
        )
        self.create_subscription(
            ApiResponse, '/task_api_responses', self._on_response, task_api_sub_qos
        )
        self.create_subscription(
            FleetState, '/fleet_states', self._on_fleet_state, fleet_qos
        )

        # Wait for RMF to be ready before dispatching
        self._startup_delay = 5.0
        self._start_time = time.time()
        self.create_timer(self._delay, self._dispatch_loop)

        if self._max_pending > 0:
            self.get_logger().info(
                f'TaskDispatcher loaded {len(self._tasks)} tasks from {scenario_file} '
                f'(max_pending={self._max_pending})'
            )
        else:
            self.get_logger().info(
                f'TaskDispatcher loaded {len(self._tasks)} tasks from {scenario_file}'
            )

    def _load_scenario(self, filepath: str) -> list[dict]:
        with open(filepath, 'r') as f:
            data = yaml.safe_load(f)
        return data.get('tasks', [])

    def _on_fleet_state(self, msg):
        """Track delivery task completions to drive max_pending throttle."""
        for robot in msg.robots:
            name = robot.name
            task_id = (robot.task_id or "").strip()
            prev = self._robot_prev_task_ids.get(name, "")
            if (
                prev
                and prev.startswith("delivery.dispatch-")
                and task_id != prev
            ):
                self._completed_count += 1
                remaining = len(self._tasks) - self._current_task_idx
                if remaining > 0:
                    self.get_logger().info(
                        f'Completion detected ({self._completed_count} total), '
                        f'{remaining} task(s) still to dispatch'
                    )
            self._robot_prev_task_ids[name] = task_id

    def _dispatch_loop(self):
        if time.time() - self._start_time < self._startup_delay:
            return

        if self._current_task_idx >= len(self._tasks):
            return

        if self._waiting_for_ack and not self._batch_mode:
            return

        if self._max_pending > 0:
            pending = self._current_task_idx - self._completed_count
            if pending >= self._max_pending:
                return

        task_def = self._tasks[self._current_task_idx]
        self._dispatch_task(task_def)
        self._current_task_idx += 1

        if not self._batch_mode:
            self._waiting_for_ack = True

    def _dispatch_task(self, task_def: dict):
        task_type = task_def.get('type', 'delivery')
        request_id = str(uuid.uuid4())

        if task_type == 'delivery':
            payload = self._build_delivery_request(task_def, request_id)
        elif task_type == 'patrol':
            payload = self._build_patrol_request(task_def, request_id)
        else:
            self.get_logger().warn(f'Unknown task type: {task_type}')
            return

        msg = ApiRequest()
        msg.request_id = request_id
        msg.json_msg = json.dumps(payload)

        self._pub.publish(msg)
        self._dispatched_ids.append(request_id)

        self.get_logger().info(
            f'Dispatched task {self._current_task_idx + 1}/{len(self._tasks)}: '
            f'{task_type} — {task_def.get("description", "")}'
        )

    def _build_delivery_request(self, task_def: dict, request_id: str) -> dict:
        """Build RMF delivery task JSON."""
        pickup = task_def['pickup']
        dropoff = task_def['dropoff']

        return {
            "type": "dispatch_task_request",
            "request": {
                "unix_millis_earliest_start_time": 0,
                "priority": {"type": "binary", "value": task_def.get('priority', 0)},
                "category": "delivery",
                "description": {
                    "pickup": {
                        "place": pickup['place'],
                        "handler": pickup['handler'],
                        "payload": [{"sku": pickup.get('sku', 'item'), "quantity": 1}],
                    },
                    "dropoff": {
                        "place": dropoff['place'],
                        "handler": dropoff['handler'],
                        "payload": [{"sku": pickup.get('sku', 'item'), "quantity": 1}],
                    },
                },
            },
        }

    def _build_patrol_request(self, task_def: dict, request_id: str) -> dict:
        """Build RMF patrol task JSON."""
        places = task_def['places']
        rounds = task_def.get('rounds', 1)

        return {
            "type": "dispatch_task_request",
            "request": {
                "unix_millis_earliest_start_time": 0,
                "priority": {"type": "binary", "value": task_def.get('priority', 0)},
                "category": "patrol",
                "description": {
                    "places": places,
                    "rounds": rounds,
                },
            },
        }

    def _on_response(self, msg: ApiResponse):
        if msg.request_id in self._dispatched_ids:
            if msg.type == 2:  # TYPE_RESPONDING
                self._waiting_for_ack = False


def main(args=None):
    rclpy.init(args=args)
    node = TaskDispatcherNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
