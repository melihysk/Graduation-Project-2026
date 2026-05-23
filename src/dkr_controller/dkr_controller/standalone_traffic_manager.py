"""
DKR Standalone Traffic Manager — RMF'den bağımsız tam trafik yönetimi.

Fleet adapter, fleet manager ve rmf_traffic_schedule OLMADAN çalışır.
Görevleri doğrudan alır, robotlara atar, rota planlar, kaynak reserve eder
ve PathRequest'leri slotcar plugin'e gönderir.

Delivery FSM:
  IDLE → GO_TO_PICKUP → AT_PICKUP → GO_TO_DROPOFF → AT_DROPOFF → IDLE → GO_TO_CHARGER

Akış:
  1. /dkr_task_requests'ten görev gelir
  2. En yakın boş robota atanır
  3. BFS ile rota planlanır, kaynaklar reserve edilir
  4. PathRequest slotcar'a gönderilir
  5. robot_state ile ilerleme takip edilir, geçilen kaynaklar release edilir
  6. Hedefe varınca FSM bir sonraki faza geçer
"""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSHistoryPolicy as History,
    QoSReliabilityPolicy as Reliability,
    QoSDurabilityPolicy as Durability,
)

from rmf_fleet_msgs.msg import PathRequest, RobotState, Location, RobotMode
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

from reservation_viz import RoutePath, build_route_markers, route_markers_topic

from .resource_graph import ResourceGraph
from .reservation_server import ReservationServer
from .deadlock_detector import DeadlockDetector


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------

class DeliveryPhase(Enum):
    IDLE = auto()
    GO_TO_PICKUP = auto()
    AT_PICKUP = auto()
    GO_TO_DROPOFF = auto()
    AT_DROPOFF = auto()


@dataclass
class DeliveryTask:
    task_id: str
    pickup_node: int
    dropoff_node: int
    priority: int = 0
    assigned_robot: str = ""
    phase: DeliveryPhase = DeliveryPhase.IDLE
    phase_start_time: float = 0.0


@dataclass
class RobotInfo:
    name: str
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    charger_node: int = -1
    idle: bool = True
    last_state_time: float = 0.0


@dataclass
class ActivePath:
    """Robot'un aktif navigasyon yolu (DKR reservation ile)."""
    robot_name: str
    task_id: str
    waypoint_node_indices: list[int] = field(default_factory=list)
    all_resources: list[str] = field(default_factory=list)
    current_segment_idx: int = 0
    held_resources: set[str] = field(default_factory=set)
    waiting: bool = False
    waiting_since: float = 0.0
    assign_time: float = 0.0


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

_ARRIVAL_TOLERANCE = 1.5
_DWELL_TIME_SEC = 3.0  # pickup/dropoff bekleme süresi
_STALE_WAIT_SEC = 15.0
_MAP_LEVEL = "L1"
_CHARGER_RETURN_TASK_ID = "__charger_return__"


class StandaloneTrafficManager(Node):

    def __init__(self):
        super().__init__("dkr_traffic_manager")

        self.declare_parameter("nav_graph_file", "")
        self.declare_parameter("retry_interval_sec", 0.5)
        self.declare_parameter("deadlock_check_interval_sec", 2.0)
        self.declare_parameter("arrival_tolerance", _ARRIVAL_TOLERANCE)
        self.declare_parameter("fleet_name", "warehouseRobot")
        self.declare_parameter("marker_topic_prefix", "dkr")

        nav_graph_file = self.get_parameter("nav_graph_file").value
        retry_interval = self.get_parameter("retry_interval_sec").value
        deadlock_interval = self.get_parameter("deadlock_check_interval_sec").value
        self._arrival_tol = self.get_parameter("arrival_tolerance").value
        self._fleet_name = self.get_parameter("fleet_name").value

        if not nav_graph_file:
            self.get_logger().error("nav_graph_file parameter is required")
            raise SystemExit(1)

        # DKR core
        self._resource_graph = ResourceGraph.from_graph_file(nav_graph_file)
        self._reservation_server = ReservationServer(
            self._resource_graph.all_resource_ids()
        )
        self._deadlock_detector = DeadlockDetector()
        self._reservation_server.set_deadlock_detector(self._deadlock_detector)

        self.get_logger().info(
            f"Resource graph loaded: {self._resource_graph}"
        )

        # Robot fleet tracking
        self._robots: dict[str, RobotInfo] = {}
        self._init_robots_from_graph()

        # Task management
        self._task_queue: list[DeliveryTask] = []
        self._active_tasks: dict[str, DeliveryTask] = {}  # robot → task

        # Active navigation paths
        self._active_paths: dict[str, ActivePath] = {}

        # Pickup/dropoff node held while robot dwells at station
        self._dwell_nodes: dict[str, int] = {}

        # Cmd ID counter for PathRequests
        self._cmd_id = 100

        # QoS
        reliable_qos = QoSProfile(
            history=History.KEEP_LAST,
            depth=10,
            reliability=Reliability.RELIABLE,
            durability=Durability.VOLATILE,
        )

        # Subscriptions
        self.create_subscription(
            RobotState, "robot_state", self._on_robot_state, reliable_qos,
        )
        self.create_subscription(
            String, "/dkr_task_requests", self._on_task_request, reliable_qos,
        )

        # Publishers
        self._path_pub = self.create_publisher(
            PathRequest, "robot_path_requests", reliable_qos,
        )
        self._event_pub = self.create_publisher(
            String, "/dkr_events", reliable_qos,
        )
        marker_prefix = self.get_parameter("marker_topic_prefix").value
        self._route_marker_pub = self.create_publisher(
            MarkerArray, route_markers_topic(marker_prefix), reliable_qos,
        )

        # Timers
        self.create_timer(retry_interval, self._retry_waiting_robots)
        self.create_timer(deadlock_interval, self._check_deadlocks)
        self.create_timer(1.0, self._assign_pending_tasks)
        self.create_timer(5.0, self._watchdog_stale_waits)
        self.create_timer(1.0, self._publish_route_visualization)

        self.get_logger().info(
            "DKR Standalone Traffic Manager started "
            f"({len(self._robots)} robots detected from graph)"
        )

    # ------------------------------------------------------------------
    # Initialization
    # ------------------------------------------------------------------

    def _init_robots_from_graph(self):
        """Nav graph'taki charger node'larından robot listesini oluştur."""
        for idx, node in self._resource_graph.nodes.items():
            if node.name.endswith("_charger"):
                robot_name = node.name.replace("_charger", "")
                self._robots[robot_name] = RobotInfo(
                    name=robot_name,
                    x=node.x,
                    y=node.y,
                    charger_node=idx,
                )
                self.get_logger().info(
                    f"  Robot '{robot_name}' at charger node {idx} "
                    f"({node.x:.1f}, {node.y:.1f})"
                )

    # ------------------------------------------------------------------
    # Task management
    # ------------------------------------------------------------------

    def _on_task_request(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        if data.get("type") != "delivery":
            self.get_logger().warn(f"Unsupported task type: {data.get('type')}")
            return

        pickup_name = data["pickup_place"]
        dropoff_name = data["dropoff_place"]

        pickup_idx = self._resource_graph.node_idx_by_name(pickup_name)
        dropoff_idx = self._resource_graph.node_idx_by_name(dropoff_name)

        if pickup_idx is None or dropoff_idx is None:
            self.get_logger().error(
                f"Unknown place: pickup={pickup_name} dropoff={dropoff_name}"
            )
            return

        task = DeliveryTask(
            task_id=data["task_id"],
            pickup_node=pickup_idx,
            dropoff_node=dropoff_idx,
            priority=data.get("priority", 0),
        )
        self._task_queue.append(task)

        self.get_logger().info(
            f"Task queued: {task.task_id} — "
            f"{pickup_name} → {dropoff_name}"
        )

        self._publish_event("task_queued", {
            "task_id": task.task_id,
            "pickup": pickup_name,
            "dropoff": dropoff_name,
        })

        self._assign_pending_tasks()

    def _assign_pending_tasks(self):
        """Bekleyen görevleri boş robotlara ata."""
        if not self._task_queue:
            self._send_idle_robots_to_charger()
            return

        idle_robots = [
            r for r in self._robots.values()
            if r.idle
            and r.last_state_time > 0
            and r.name not in self._active_tasks
            and not self._is_charger_return(r.name)
        ]
        if not idle_robots:
            return

        tasks_to_remove = []
        for task in self._task_queue:
            if not idle_robots:
                break

            if not self._is_pickup_available(task.pickup_node):
                continue

            pickup_node = self._resource_graph.nodes.get(task.pickup_node)
            if pickup_node is None:
                continue

            best_robot = min(
                idle_robots,
                key=lambda r: math.hypot(r.x - pickup_node.x, r.y - pickup_node.y),
            )

            task.assigned_robot = best_robot.name
            task.phase = DeliveryPhase.GO_TO_PICKUP
            task.phase_start_time = time.time()
            best_robot.idle = False

            self._active_tasks[best_robot.name] = task
            tasks_to_remove.append(task)
            idle_robots.remove(best_robot)

            self.get_logger().info(
                f"[{best_robot.name}] Assigned task {task.task_id}: "
                f"GO_TO_PICKUP ({self._node_name(task.pickup_node)})"
            )

            self._publish_event("task_assigned", {
                "task_id": task.task_id,
                "robot": best_robot.name,
                "pickup": self._node_name(task.pickup_node),
                "dropoff": self._node_name(task.dropoff_node),
            })

            self._start_navigation(best_robot.name, task.pickup_node, task.task_id)

        for t in tasks_to_remove:
            self._task_queue.remove(t)

        if not self._task_queue:
            self._send_idle_robots_to_charger()

    def _advance_delivery(self, robot_name: str):
        """Delivery FSM: mevcut faz tamamlandığında bir sonraki faza geç."""
        task = self._active_tasks.get(robot_name)
        if task is None:
            return

        if task.phase == DeliveryPhase.GO_TO_PICKUP:
            task.phase = DeliveryPhase.AT_PICKUP
            task.phase_start_time = time.time()
            self.get_logger().info(
                f"[{robot_name}] AT_PICKUP — waiting {_DWELL_TIME_SEC}s"
            )

        elif task.phase == DeliveryPhase.AT_PICKUP:
            if time.time() - task.phase_start_time >= _DWELL_TIME_SEC:
                task.phase = DeliveryPhase.GO_TO_DROPOFF
                task.phase_start_time = time.time()
                self.get_logger().info(
                    f"[{robot_name}] GO_TO_DROPOFF "
                    f"({self._node_name(task.dropoff_node)})"
                )
                self._start_navigation(
                    robot_name, task.dropoff_node, task.task_id,
                )

        elif task.phase == DeliveryPhase.GO_TO_DROPOFF:
            task.phase = DeliveryPhase.AT_DROPOFF
            task.phase_start_time = time.time()
            self.get_logger().info(
                f"[{robot_name}] AT_DROPOFF — waiting {_DWELL_TIME_SEC}s"
            )

        elif task.phase == DeliveryPhase.AT_DROPOFF:
            if time.time() - task.phase_start_time >= _DWELL_TIME_SEC:
                self._complete_task(robot_name)

    def _complete_task(self, robot_name: str):
        task = self._active_tasks.pop(robot_name, None)
        if task is None:
            return

        # Dropoff node stays reserved until the robot starts its next route.
        task.phase = DeliveryPhase.IDLE
        robot = self._robots.get(robot_name)
        if robot:
            robot.idle = True

        self.get_logger().info(
            f"[{robot_name}] Task {task.task_id} COMPLETED"
        )

        self._publish_event("task_completed", {
            "task_id": task.task_id,
            "robot": robot_name,
            "pickup": self._node_name(task.pickup_node),
            "dropoff": self._node_name(task.dropoff_node),
        })

        self._assign_pending_tasks()

    def _send_idle_robots_to_charger(self):
        """Boşta kalan robotları kendi şarj istasyonuna gönder."""
        for robot in self._robots.values():
            self._try_send_robot_to_charger(robot.name)

    def _try_send_robot_to_charger(self, robot_name: str):
        if self._task_queue:
            return

        robot = self._robots.get(robot_name)
        if robot is None or robot.charger_node < 0:
            return
        if robot_name in self._active_tasks:
            return
        if robot_name in self._active_paths:
            return
        if not robot.idle:
            return
        if self._is_at_charger(robot_name):
            return

        self.get_logger().info(
            f"[{robot_name}] Returning to charger "
            f"({self._node_name(robot.charger_node)})"
        )
        robot.idle = False
        self._start_navigation(
            robot_name, robot.charger_node, _CHARGER_RETURN_TASK_ID,
        )
        if robot_name not in self._active_paths:
            robot.idle = True

    # ------------------------------------------------------------------
    # Navigation (DKR resource reservation)
    # ------------------------------------------------------------------

    def _start_navigation(self, robot_name: str, target_node: int, task_id: str):
        """Hedefe BFS rota planla, kaynakları reserve et, PathRequest gönder."""
        # Mevcut aktif path varsa temizle
        if robot_name in self._active_paths:
            self._release_robot_resources(robot_name)
            del self._active_paths[robot_name]

        robot = self._robots.get(robot_name)
        if robot is None:
            return

        start_node = self._closest_node(robot.x, robot.y)
        if start_node is None or start_node == target_node:
            self._on_navigation_complete(robot_name)
            return

        route = self._resource_graph.find_path_bfs(start_node, target_node)
        if not route:
            self.get_logger().error(
                f"[{robot_name}] No route from "
                f"{self._node_name(start_node)} to {self._node_name(target_node)}"
            )
            return

        all_resources = self._resource_graph.get_path_resources(route)

        self._cmd_id += 1
        ap = ActivePath(
            robot_name=robot_name,
            task_id=task_id,
            waypoint_node_indices=route,
            all_resources=all_resources,
            assign_time=time.time(),
        )
        self._active_paths[robot_name] = ap

        self.get_logger().info(
            f"[{robot_name}] Route: {len(route)} nodes, "
            f"{len(all_resources)} resources → {self._node_name(target_node)}"
        )

        self._publish_event("path_received", {
            "robot": robot_name,
            "task_id": task_id,
            "waypoint_count": len(route),
            "total_resources": len(all_resources),
        })

        self._try_reserve_and_go(robot_name)

    def _try_reserve_and_go(self, robot_name: str) -> bool:
        """Tüm kalan yol kaynaklarını reserve et. Başarılıysa PathRequest gönder."""
        ap = self._active_paths.get(robot_name)
        if ap is None:
            return False

        total_seg = len(ap.waypoint_node_indices) - 1
        if ap.current_segment_idx >= total_seg:
            self._on_navigation_complete(robot_name)
            return True

        needed = self._collect_needed_resources(ap)
        if not needed:
            self._send_path_request(robot_name)
            ap.waiting = False
            return True

        result = self._reservation_server.reserve(robot_name, needed)

        if result.granted:
            ap.held_resources.update(needed)
            ap.waiting = False
            ap.waiting_since = 0.0
            self._deadlock_detector.remove_wait(robot_name)
            self._promote_dwell_to_path_hold(robot_name, ap)

            self._publish_event("grant", {
                "robot": robot_name,
                "resources": needed,
                "segment_idx": ap.current_segment_idx,
            })

            self._send_path_request(robot_name)
            return True
        else:
            ap.waiting = True
            if not ap.waiting_since:
                ap.waiting_since = time.time()
            if result.blocking_robot:
                self._deadlock_detector.add_wait(
                    robot_name, result.blocking_robot,
                    result.blocking_resources[0] if result.blocking_resources else "",
                )

            self._publish_event("deny", {
                "robot": robot_name,
                "blocking_robot": result.blocking_robot,
                "reason": result.reason,
            })

            self.get_logger().debug(f"[{robot_name}] Waiting: {result.reason}")
            return False

    def _send_path_request(self, robot_name: str):
        """Kalan yolu PathRequest olarak slotcar'a gönder."""
        ap = self._active_paths.get(robot_name)
        if ap is None:
            return

        total_seg = len(ap.waypoint_node_indices) - 1

        msg = PathRequest()
        msg.fleet_name = self._fleet_name
        msg.robot_name = robot_name
        # Slotcar yeni path'i kabul etsin diye komut kimliği; delivery task_id ActivePath'te kalır.
        msg.task_id = str(self._cmd_id)

        robot = self._robots.get(robot_name)
        if robot and robot.last_state_time > 0:
            cur = Location()
            cur.x, cur.y, cur.yaw = robot.x, robot.y, robot.yaw
            cur.level_name = _MAP_LEVEL
            msg.path.append(cur)
        else:
            msg.path.append(
                self._loc_from_node(ap.waypoint_node_indices[ap.current_segment_idx])
            )

        for i in range(ap.current_segment_idx + 1, total_seg + 1):
            msg.path.append(
                self._loc_from_node(ap.waypoint_node_indices[i])
            )

        self._path_pub.publish(msg)

    def _send_hold_at_current_position(self, robot_name: str) -> None:
        """Slotcar'ı mevcut konumda durdur (eski path iptali için)."""
        robot = self._robots.get(robot_name)
        if robot is None or robot.last_state_time <= 0:
            return

        self._cmd_id += 1
        msg = PathRequest()
        msg.fleet_name = self._fleet_name
        msg.robot_name = robot_name
        msg.task_id = str(self._cmd_id)

        loc = Location()
        loc.x, loc.y, loc.yaw = robot.x, robot.y, robot.yaw
        loc.level_name = _MAP_LEVEL
        msg.path.append(loc)
        self._path_pub.publish(msg)

    def _on_navigation_complete(self, robot_name: str):
        """Navigasyon tamamlandı — kaynakları bırak, FSM'i ilerlet."""
        ap = self._active_paths.get(robot_name)
        if ap and ap.task_id == _CHARGER_RETURN_TASK_ID:
            self._release_robot_resources(robot_name)
            self._deadlock_detector.clear_robot(robot_name)
            if robot_name in self._active_paths:
                del self._active_paths[robot_name]

            robot = self._robots.get(robot_name)
            if robot:
                robot.idle = True
                self._hold_dwell_node(robot_name, robot.charger_node)

            self.get_logger().info(f"[{robot_name}] Arrived at charger")
            self._publish_event("charger_arrival", {"robot": robot_name})
            return

        task = self._active_tasks.get(robot_name)
        dwell_node_idx: int | None = None
        if task:
            if task.phase == DeliveryPhase.GO_TO_PICKUP:
                dwell_node_idx = task.pickup_node
            elif task.phase == DeliveryPhase.GO_TO_DROPOFF:
                dwell_node_idx = task.dropoff_node

        self._release_robot_resources(robot_name)
        self._deadlock_detector.clear_robot(robot_name)

        if robot_name in self._active_paths:
            del self._active_paths[robot_name]

        if dwell_node_idx is not None:
            if not self._hold_dwell_node(robot_name, dwell_node_idx):
                owner = self._reservation_server.get_owner(f"node_{dwell_node_idx}")
                self.get_logger().warn(
                    f"[{robot_name}] Could not reserve dwell node "
                    f"{self._node_name(dwell_node_idx)}"
                    + (f" (held by {owner})" if owner else "")
                )

        self.get_logger().info(
            f"[{robot_name}] Navigation complete, resources released."
        )

        self._advance_delivery(robot_name)

    # ------------------------------------------------------------------
    # Robot state monitoring
    # ------------------------------------------------------------------

    def _on_robot_state(self, msg: RobotState):
        robot_name = msg.name
        if robot_name not in self._robots:
            return

        robot = self._robots[robot_name]
        robot.x = msg.location.x
        robot.y = msg.location.y
        robot.yaw = msg.location.yaw
        robot.last_state_time = time.time()

        # Check dwell phases (AT_PICKUP / AT_DROPOFF)
        task = self._active_tasks.get(robot_name)
        if task and task.phase in (DeliveryPhase.AT_PICKUP, DeliveryPhase.AT_DROPOFF):
            self._advance_delivery(robot_name)
            return

        # Check navigation progress
        ap = self._active_paths.get(robot_name)
        if ap is None or ap.waiting:
            return

        total_seg = len(ap.waypoint_node_indices) - 1
        if ap.current_segment_idx >= total_seg:
            return

        if time.time() - ap.assign_time < 1.5:
            return

        # Multi-segment advancement
        best_seg = ap.current_segment_idx
        for seg_i in range(ap.current_segment_idx + 1, total_seg + 1):
            node_idx = ap.waypoint_node_indices[seg_i]
            node = self._resource_graph.nodes.get(node_idx)
            if node is None:
                continue
            dist = math.hypot(msg.location.x - node.x, msg.location.y - node.y)
            if dist <= self._arrival_tol:
                best_seg = seg_i

        if best_seg > ap.current_segment_idx:
            released = self._get_passed_resources(ap, best_seg)
            if released:
                self._reservation_server.release(robot_name, released)
                ap.held_resources -= set(released)
                self._publish_event("release", {
                    "robot": robot_name,
                    "resources": released,
                })

            ap.current_segment_idx = best_seg

            if ap.current_segment_idx >= total_seg:
                self._on_navigation_complete(robot_name)

    # ------------------------------------------------------------------
    # Retry, deadlock, stale timers
    # ------------------------------------------------------------------

    def _retry_waiting_robots(self):
        for name in list(self._active_paths.keys()):
            ap = self._active_paths.get(name)
            if ap and ap.waiting:
                self._try_reserve_and_go(name)

    def _check_deadlocks(self):
        cycle = self._deadlock_detector.detect_cycle()
        if cycle is None:
            return

        victim = self._deadlock_detector.resolve_deadlock(cycle)
        if victim is None:
            return

        self.get_logger().warn(f"DEADLOCK: {cycle}. Victim: {victim}")

        self._publish_event("deadlock", {
            "cycle": cycle,
            "victim": victim,
        })

        self._release_robot_resources(victim, keep_dwell=False)
        self._deadlock_detector.clear_robot(victim)

        ap = self._active_paths.get(victim)
        if ap:
            ap.held_resources.clear()
            ap.waiting = False
            ap.waiting_since = 0.0
            actual_seg = self._find_current_segment(victim, ap)
            ap.current_segment_idx = actual_seg
            self._try_reserve_and_go(victim)

    def _watchdog_stale_waits(self):
        """Uzun beklemeleri logla; kaynak çalmadan yeniden dene (DKR semantiği)."""
        now = time.time()
        for name in list(self._active_paths.keys()):
            ap = self._active_paths.get(name)
            if ap is None or not ap.waiting or not ap.waiting_since:
                continue
            wait_sec = now - ap.waiting_since
            if wait_sec < _STALE_WAIT_SEC:
                continue

            blocker_name = ""
            for edge in self._deadlock_detector.get_wait_graph_snapshot():
                if edge.waiting_robot == name:
                    blocker_name = edge.blocking_robot
                    break

            self.get_logger().warn(
                f"[{name}] Long wait ({wait_sec:.0f}s)"
                + (f", blocked by {blocker_name}" if blocker_name else "")
                + " — retrying reservation"
            )

            self._publish_event("stale_wait", {
                "robot": name,
                "blocker": blocker_name,
                "wait_sec": round(wait_sec, 1),
            })

            self._try_reserve_and_go(name)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _collect_needed_resources(self, ap: ActivePath) -> list[str]:
        total_seg = len(ap.waypoint_node_indices) - 1
        if ap.current_segment_idx >= total_seg:
            return []

        needed: list[str] = []
        for seg_i in range(ap.current_segment_idx, total_seg):
            from_idx = ap.waypoint_node_indices[seg_i]
            to_idx = ap.waypoint_node_indices[seg_i + 1]
            for r in self._resource_graph.get_segment_resources(from_idx, to_idx):
                if r not in needed and r not in ap.held_resources:
                    needed.append(r)

        cur_node = f"node_{ap.waypoint_node_indices[ap.current_segment_idx]}"
        if cur_node not in ap.held_resources and cur_node not in needed:
            needed.insert(0, cur_node)
        return needed

    def _get_passed_resources(self, ap: ActivePath, current_end: int) -> list[str]:
        to_release: list[str] = []
        current_node_id = f"node_{ap.waypoint_node_indices[current_end]}"

        for rid in list(ap.held_resources):
            if rid == current_node_id:
                continue
            if rid.startswith("node_"):
                node_idx = int(rid.split("_")[1])
                try:
                    pos = ap.waypoint_node_indices.index(node_idx)
                    if pos < current_end:
                        to_release.append(rid)
                except ValueError:
                    to_release.append(rid)
            elif rid.startswith("edge_"):
                parts = rid.split("_")
                from_n = int(parts[1])
                try:
                    pos = ap.waypoint_node_indices.index(from_n)
                    if pos < current_end:
                        to_release.append(rid)
                except ValueError:
                    to_release.append(rid)

        return to_release

    def _find_current_segment(self, robot_name: str, ap: ActivePath) -> int:
        robot = self._robots.get(robot_name)
        if not robot or robot.last_state_time == 0:
            return ap.current_segment_idx

        best_seg = 0
        best_dist = float("inf")
        for seg_i, node_idx in enumerate(ap.waypoint_node_indices):
            node = self._resource_graph.nodes.get(node_idx)
            if node is None:
                continue
            dist = math.hypot(robot.x - node.x, robot.y - node.y)
            if dist < best_dist:
                best_dist = dist
                best_seg = seg_i

        return max(best_seg, ap.current_segment_idx)

    def _closest_node(self, x: float, y: float) -> int | None:
        best_idx = None
        best_dist = float("inf")
        for idx, node in self._resource_graph.nodes.items():
            dist = math.hypot(node.x - x, node.y - y)
            if dist < best_dist:
                best_dist = dist
                best_idx = idx
        return best_idx

    def _node_name(self, idx: int) -> str:
        node = self._resource_graph.nodes.get(idx)
        return node.name if node else f"node_{idx}"

    def _loc_from_node(self, node_idx: int) -> Location:
        node = self._resource_graph.nodes[node_idx]
        loc = Location()
        loc.x = node.x
        loc.y = node.y
        loc.yaw = 0.0
        loc.level_name = _MAP_LEVEL
        return loc

    def _release_robot_resources(self, robot_name: str, keep_dwell: bool = True):
        dwell_node_id = None
        dwell_idx = self._dwell_nodes.get(robot_name)
        if dwell_idx is not None:
            dwell_node_id = f"node_{dwell_idx}"

        released = self._reservation_server.release_all(robot_name)
        ap = self._active_paths.get(robot_name)
        if ap:
            ap.held_resources.clear()
        if released:
            self._publish_event("release", {
                "robot": robot_name,
                "resources": released,
            })

        if not keep_dwell:
            self._dwell_nodes.pop(robot_name, None)
        elif dwell_node_id is not None:
            self._reservation_server.reserve(robot_name, [dwell_node_id])

    def _is_at_charger(self, robot_name: str) -> bool:
        robot = self._robots.get(robot_name)
        if robot is None or robot.charger_node < 0:
            return False
        node = self._resource_graph.nodes.get(robot.charger_node)
        if node is None:
            return False
        return math.hypot(robot.x - node.x, robot.y - node.y) <= self._arrival_tol

    def _is_charger_return(self, robot_name: str) -> bool:
        ap = self._active_paths.get(robot_name)
        return ap is not None and ap.task_id == _CHARGER_RETURN_TASK_ID

    def _cancel_active_navigation(self, robot_name: str) -> None:
        if robot_name not in self._active_paths:
            return
        was_charger_return = self._is_charger_return(robot_name)
        self._release_robot_resources(robot_name)
        del self._active_paths[robot_name]
        self._deadlock_detector.clear_robot(robot_name)
        self._send_hold_at_current_position(robot_name)
        if was_charger_return:
            robot = self._robots.get(robot_name)
            if robot:
                robot.idle = True

    def _is_pickup_available(self, pickup_node_idx: int) -> bool:
        node_id = f"node_{pickup_node_idx}"
        if self._reservation_server.get_owner(node_id) is not None:
            return False

        for task in self._active_tasks.values():
            if task.pickup_node != pickup_node_idx:
                continue
            if task.phase in (
                DeliveryPhase.GO_TO_PICKUP,
                DeliveryPhase.AT_PICKUP,
            ):
                return False
        return True

    def _hold_dwell_node(self, robot_name: str, node_idx: int) -> bool:
        node_id = f"node_{node_idx}"
        result = self._reservation_server.reserve(robot_name, [node_id])
        if result.granted:
            self._dwell_nodes[robot_name] = node_idx
        return result.granted

    def _release_dwell_node(self, robot_name: str) -> None:
        node_idx = self._dwell_nodes.pop(robot_name, None)
        if node_idx is None:
            return
        self._reservation_server.release(robot_name, [f"node_{node_idx}"])

    def _promote_dwell_to_path_hold(
        self, robot_name: str, ap: ActivePath,
    ) -> None:
        """Dwell rezervasyonunu path rezervasyonuna devret; başarısız beklemede düğümü koru."""
        dwell_idx = self._dwell_nodes.get(robot_name)
        if dwell_idx is None:
            return
        dwell_id = f"node_{dwell_idx}"
        if dwell_id in ap.held_resources:
            del self._dwell_nodes[robot_name]

    # ------------------------------------------------------------------
    # RViz route visualization (/{prefix}_route_markers)
    # ------------------------------------------------------------------

    def _publish_route_visualization(self):
        paths = [
            RoutePath(
                robot_name=robot_name,
                waypoint_node_indices=ap.waypoint_node_indices,
                current_segment_idx=ap.current_segment_idx,
            )
            for robot_name, ap in self._active_paths.items()
        ]
        markers = build_route_markers(paths, self._resource_graph.nodes)
        self._route_marker_pub.publish(markers)

    def _publish_event(self, event_type: str, data: dict):
        event = {"type": event_type, "timestamp": time.time(), **data}
        msg = String()
        msg.data = json.dumps(event)
        self._event_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StandaloneTrafficManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
