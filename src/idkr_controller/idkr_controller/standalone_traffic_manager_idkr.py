"""
IDRR Standalone Traffic Manager — Verma, Olm, Suárez (IEEE Access, 2024).

DKR'den temel algoritmik farklar:
  1. Kavşak düğümleri CP alt-kaynaklarına bölünür (eş zamanlı erişim)
  2. Kavşakta max 2 robot sınırı
  3. Deny → Res1 (engelleyen robot boş CP'ye kaydırılır)
  4. Res1 başarısız → alternatif rota (engelden kaçınma)

Görev atama, Delivery FSM, şarj'a dönüş ve rota planlama mantığı
DKR ile birebir aynıdır (saf algoritma karşılaştırması).
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

from rmf_fleet_msgs.msg import PathRequest, RobotState, Location
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

from reservation_viz import RoutePath, build_route_markers, route_markers_topic

from .resource_graph_idkr import ResourceGraphIDKR
from .reservation_server_idkr import ReservationServerIDKR
from .deadlock_detector import DeadlockDetector
from .conflict_classifier import ConflictClassifier


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
    robot_name: str
    task_id: str
    waypoint_node_indices: list[int] = field(default_factory=list)
    all_resources: list[str] = field(default_factory=list)
    current_segment_idx: int = 0
    held_resources: set[str] = field(default_factory=set)
    waiting: bool = False
    waiting_since: float = 0.0
    assign_time: float = 0.0
    res2_count: int = 0


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

_ARRIVAL_TOLERANCE = 1.5
_DWELL_TIME_SEC = 3.0
_STALE_WAIT_SEC = 15.0
_MAP_LEVEL = "L1"
_CHARGER_RETURN_TASK_ID = "__charger_return__"
_MAX_RES2_PER_NAV = 2
_JUNCTION_GUARD_DIST = 2.0


class StandaloneTrafficManagerIDKR(Node):

    def __init__(self):
        super().__init__("idkr_traffic_manager")

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

        # IDKR core
        self._resource_graph = ResourceGraphIDKR.from_graph_file(nav_graph_file)
        self._cp_manager = self._resource_graph.cp_manager
        self._reservation_server = ReservationServerIDKR(
            self._resource_graph.all_resource_ids(),
            self._cp_manager,
        )
        self._deadlock_detector = DeadlockDetector()
        self._reservation_server.set_deadlock_detector(self._deadlock_detector)
        self._conflict_classifier = ConflictClassifier(self._cp_manager)

        self.get_logger().info(
            f"İDKR resource graph loaded: {self._resource_graph}"
        )

        # Robot fleet tracking
        self._robots: dict[str, RobotInfo] = {}
        self._init_robots_from_graph()

        # Task management
        self._task_queue: list[DeliveryTask] = []
        self._active_tasks: dict[str, DeliveryTask] = {}

        # Active navigation paths
        self._active_paths: dict[str, ActivePath] = {}

        # Dwell node reservations
        self._dwell_nodes: dict[str, int] = {}

        # Cmd ID counter
        self._cmd_id = 100

        # Res1/Res2 statistics
        self._res1_attempts = 0
        self._res1_success = 0
        self._res2_attempts = 0
        self._res2_success = 0

        # Collision proximity tracking
        self._near_miss_count = 0
        self._collision_pairs_logged: set[tuple[str, str]] = set()

        # Deduplicate repeated FAIL logs
        self._last_fail: dict[str, str] = {}

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
        self.create_timer(0.5, self._check_collision_proximity)

        self.get_logger().info(
            "İDKR Standalone Traffic Manager started "
            f"({len(self._robots)} robots, "
            f"{len(self._cp_manager.junctions)} junctions detected)"
        )

    # ------------------------------------------------------------------
    # Initialization
    # ------------------------------------------------------------------

    def _init_robots_from_graph(self):
        for idx, node in self._resource_graph.nodes.items():
            if node.name.endswith("_charger"):
                robot_name = node.name.replace("_charger", "")
                self._robots[robot_name] = RobotInfo(
                    name=robot_name,
                    x=node.x,
                    y=node.y,
                    charger_node=idx,
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

        self._publish_event("task_queued", {
            "task_id": task.task_id,
            "pickup": pickup_name,
            "dropoff": dropoff_name,
        })

        self._assign_pending_tasks()

    def _assign_pending_tasks(self):
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

            self._publish_event("task_assigned", {
                "task_id": task.task_id,
                "robot": best_robot.name,
                "pickup": self._node_name(task.pickup_node),
                "dropoff": self._node_name(task.dropoff_node),
            })

            self._start_navigation(best_robot.name, task.pickup_node, task.task_id)

        for t in tasks_to_remove:
            self._task_queue.remove(t)

    def _advance_delivery(self, robot_name: str):
        task = self._active_tasks.get(robot_name)
        if task is None:
            return

        if task.phase == DeliveryPhase.GO_TO_PICKUP:
            task.phase = DeliveryPhase.AT_PICKUP
            task.phase_start_time = time.time()

        elif task.phase == DeliveryPhase.AT_PICKUP:
            if time.time() - task.phase_start_time >= _DWELL_TIME_SEC:
                task.phase = DeliveryPhase.GO_TO_DROPOFF
                task.phase_start_time = time.time()
                self._start_navigation(
                    robot_name, task.dropoff_node, task.task_id,
                )

        elif task.phase == DeliveryPhase.GO_TO_DROPOFF:
            task.phase = DeliveryPhase.AT_DROPOFF
            task.phase_start_time = time.time()

        elif task.phase == DeliveryPhase.AT_DROPOFF:
            if time.time() - task.phase_start_time >= _DWELL_TIME_SEC:
                self._complete_task(robot_name)

    def _complete_task(self, robot_name: str):
        task = self._active_tasks.pop(robot_name, None)
        if task is None:
            return

        task.phase = DeliveryPhase.IDLE
        robot = self._robots.get(robot_name)
        if robot:
            robot.idle = True

        self._publish_event("task_completed", {
            "task_id": task.task_id,
            "robot": robot_name,
            "pickup": self._node_name(task.pickup_node),
            "dropoff": self._node_name(task.dropoff_node),
        })

        self._assign_pending_tasks()

    def _send_idle_robots_to_charger(self):
        """Tüm görevler tamamlandıktan sonra robotları şarj'a gönder."""
        if not self._active_tasks and not hasattr(self, "_summary_logged"):
            self._summary_logged = True
            self.get_logger().info(
                f"[SUMMARY] Res1={self._res1_success}/{self._res1_attempts} "
                f"Res2={self._res2_success}/{self._res2_attempts} "
                f"deadlocks={self._deadlock_detector.deadlock_count} "
                f"near_miss={self._near_miss_count}"
            )
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

        robot.idle = False
        self._start_navigation(
            robot_name, robot.charger_node, _CHARGER_RETURN_TASK_ID,
        )
        if robot_name not in self._active_paths:
            robot.idle = True

    # ------------------------------------------------------------------
    # Navigation (IDKR CP-aware resource reservation)
    # ------------------------------------------------------------------

    def _start_navigation(self, robot_name: str, target_node: int, task_id: str):
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

        all_resources = self._resource_graph.get_path_resources_idkr(route)

        self._cmd_id += 1
        ap = ActivePath(
            robot_name=robot_name,
            task_id=task_id,
            waypoint_node_indices=route,
            all_resources=all_resources,
            assign_time=time.time(),
        )
        self._active_paths[robot_name] = ap

        self._publish_event("path_received", {
            "robot": robot_name,
            "task_id": task_id,
            "waypoint_count": len(route),
            "total_resources": len(all_resources),
        })

        self._try_reserve_and_go(robot_name)

    def _try_reserve_and_go(self, robot_name: str) -> bool:
        ap = self._active_paths.get(robot_name)
        if ap is None:
            return False

        total_seg = len(ap.waypoint_node_indices) - 1
        if ap.current_segment_idx >= total_seg:
            self._on_navigation_complete(robot_name)
            return True

        needed = self._collect_needed_resources(ap)
        if not needed:
            full = self._send_path_request(robot_name)
            ap.waiting = not full
            if not full:
                if not ap.waiting_since:
                    ap.waiting_since = time.time()
            else:
                ap.waiting_since = 0.0
            return True

        for rid in needed:
            if rid.startswith("cp_"):
                parts = rid.split("_")
                jnode = int(parts[1])
                if not self._is_junction_physically_clear(jnode, robot_name):
                    ap.waiting = True
                    if not ap.waiting_since:
                        ap.waiting_since = time.time()
                    return False
                break

        result = self._reservation_server.reserve(robot_name, needed)

        if result.granted:
            ap.held_resources.update(needed)
            self._deadlock_detector.remove_wait(robot_name)
            self._promote_dwell_to_path_hold(robot_name, ap)

            self._publish_event("grant", {
                "robot": robot_name,
                "resources": needed,
                "segment_idx": ap.current_segment_idx,
            })

            full = self._send_path_request(robot_name)
            if not full:
                ap.waiting = True
                if not ap.waiting_since:
                    ap.waiting_since = time.time()
            else:
                ap.waiting = False
                ap.waiting_since = 0.0
            return True
        else:
            res1_success = False

            if result.blocking_resources and result.blocking_robot:
                blocked_rid = result.blocking_resources[0]

                # --- Res1: kavşak CP relocation ---
                if blocked_rid.startswith("cp_"):
                    conflict = self._conflict_classifier.classify(
                        requesting_robot=robot_name,
                        blocking_robot=result.blocking_robot,
                        blocked_resource=blocked_rid,
                        requesting_path=ap.waypoint_node_indices,
                        blocking_path=self._get_robot_path(result.blocking_robot),
                        requesting_segment_idx=ap.current_segment_idx,
                    )

                    if self._conflict_classifier.should_attempt_res1(conflict):
                        self._res1_attempts += 1
                        res1_result = self._reservation_server.try_res1(
                            robot_name, blocked_rid,
                        )

                        self._publish_event("res1_triggered", {
                            "robot": robot_name,
                            "blocking_robot": result.blocking_robot,
                            "cp": blocked_rid,
                            "conflict_type": conflict.conflict_type.name,
                            "granted": res1_result.granted,
                            "reason": res1_result.reason,
                            "res1_attempts": self._res1_attempts,
                            "res1_success": self._res1_success,
                        })

                        if not res1_result.granted:
                            fail_key = f"res1:{robot_name}:{blocked_rid}"
                            if self._last_fail.get(robot_name) != fail_key:
                                self._last_fail[robot_name] = fail_key
                                self.get_logger().info(
                                    f"[RES1] {robot_name} FAIL cp={blocked_rid} "
                                    f"blocker={result.blocking_robot}"
                                )

                        if res1_result.granted:
                            self._res1_success += 1
                            if res1_result.res1_applied:
                                blocker_ap = self._active_paths.get(
                                    res1_result.res1_blocking_robot,
                                )
                                if blocker_ap:
                                    blocker_ap.held_resources.discard(
                                        res1_result.res1_original_cp,
                                    )
                                    blocker_ap.held_resources.add(
                                        res1_result.res1_new_cp,
                                    )
                                self._deadlock_detector.update_waits_after_res1(
                                    res1_result.res1_blocking_robot,
                                    res1_result.res1_original_cp,
                                )
                            remaining = [r for r in needed if r != blocked_rid]
                            if remaining:
                                result2 = self._reservation_server.reserve(
                                    robot_name, remaining,
                                )
                                if result2.granted:
                                    res1_success = True
                                else:
                                    self._reservation_server.release(
                                        robot_name, [blocked_rid],
                                    )
                                    self._reservation_server.undo_res1(res1_result)
                                    if res1_result.res1_applied and blocker_ap:
                                        blocker_ap.held_resources.discard(
                                            res1_result.res1_new_cp,
                                        )
                                        blocker_ap.held_resources.add(
                                            res1_result.res1_original_cp,
                                        )
                            else:
                                res1_success = True

                            if res1_success:
                                ap.held_resources.update(needed)
                                self._deadlock_detector.remove_wait(robot_name)
                                self._promote_dwell_to_path_hold(robot_name, ap)
                                self._last_fail.pop(robot_name, None)
                                self.get_logger().info(
                                    f"[RES1] {robot_name} OK "
                                    f"{res1_result.res1_blocking_robot}: "
                                    f"{res1_result.res1_original_cp}"
                                    f"->{res1_result.res1_new_cp}"
                                )
                                self._publish_event("grant", {
                                    "robot": robot_name,
                                    "resources": needed,
                                    "segment_idx": ap.current_segment_idx,
                                    "via_res1": True,
                                })
                                full = self._send_path_request(robot_name)
                                if not full:
                                    ap.waiting = True
                                    if not ap.waiting_since:
                                        ap.waiting_since = time.time()
                                else:
                                    ap.waiting = False
                                    ap.waiting_since = 0.0
                                return True

            # --- Res2: alternatif rota ile yeniden planlama ---
            if not res1_success:
                if self._try_res2_replan(robot_name, result):
                    return True

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
            return False

    def _try_res2_replan(
        self,
        robot_name: str,
        deny_result,
    ) -> bool:
        """Res2: engelli kaynaktan kaçınarak alternatif rota bul ve değiştir.

        Returns True ise aktif yol değiştirildi (grant veya retry beklenecek).
        """
        ap = self._active_paths.get(robot_name)
        if ap is None or ap.res2_count >= _MAX_RES2_PER_NAV:
            return False
        if not deny_result.blocking_resources:
            return False

        blocked_rid = deny_result.blocking_resources[0]

        avoid_nodes: set[int] = set()
        avoid_edges: set[tuple[int, int]] = set()

        if blocked_rid.startswith("node_"):
            avoid_nodes.add(int(blocked_rid.split("_")[1]))
        elif blocked_rid.startswith("edge_"):
            parts = blocked_rid.split("_")
            avoid_edges.add((int(parts[1]), int(parts[2])))
        elif blocked_rid.startswith("cp_"):
            parts = blocked_rid.split("_")
            junction_node = int(parts[1])
            cp_idx = int(parts[2])
            junction_info = self._cp_manager.get_junction_info(junction_node)
            if junction_info:
                for cp in junction_info.control_points:
                    if cp.cp_index == cp_idx:
                        avoid_edges.add((cp.direction_neighbor, junction_node))
                        break
            else:
                avoid_nodes.add(junction_node)
        else:
            return False

        current_node = ap.waypoint_node_indices[ap.current_segment_idx]
        target_node = ap.waypoint_node_indices[-1]

        avoid_nodes.discard(current_node)
        avoid_nodes.discard(target_node)

        alt_route = self._resource_graph.find_path_bfs_avoiding(
            current_node, target_node, avoid_nodes, avoid_edges,
        )

        self._res2_attempts += 1

        if alt_route is None:
            fail_key = f"res2:{robot_name}:{blocked_rid}"
            if self._last_fail.get(robot_name) != fail_key:
                self._last_fail[robot_name] = fail_key
                self.get_logger().info(
                    f"[RES2] {robot_name} FAIL blocked={blocked_rid} "
                    f"by={deny_result.blocking_robot}"
                )
            self._publish_event("res2_triggered", {
                "robot": robot_name,
                "blocked_resource": blocked_rid,
                "blocking_robot": deny_result.blocking_robot,
                "granted": False,
                "reason": "no_alternative_path",
                "res2_attempts": self._res2_attempts,
                "res2_success": self._res2_success,
            })
            return False

        remaining_old = ap.waypoint_node_indices[ap.current_segment_idx:]
        if alt_route == remaining_old:
            return False

        cur_node_rid = self._current_node_resource_id(ap, ap.current_segment_idx)

        # Mevcut düğüm kaynağını koruyarak yalnızca geri kalanını release et
        resources_to_keep = {cur_node_rid}
        resources_to_release = [
            r for r in ap.held_resources if r not in resources_to_keep
        ]
        if resources_to_release:
            self._reservation_server.release(robot_name, resources_to_release)
        self._deadlock_detector.clear_robot(robot_name)

        all_resources = self._resource_graph.get_path_resources_idkr(alt_route)
        self._cmd_id += 1

        held_init: set[str] = set()
        if cur_node_rid in ap.held_resources:
            held_init.add(cur_node_rid)
        else:
            cur_hold = self._reservation_server.reserve(robot_name, [cur_node_rid])
            if cur_hold.granted:
                held_init.add(cur_node_rid)

        new_ap = ActivePath(
            robot_name=robot_name,
            task_id=ap.task_id,
            waypoint_node_indices=alt_route,
            all_resources=all_resources,
            assign_time=time.time(),
            res2_count=ap.res2_count + 1,
            held_resources=held_init,
        )
        self._active_paths[robot_name] = new_ap

        needed = self._collect_needed_resources(new_ap)
        if needed:
            res = self._reservation_server.reserve(robot_name, needed)
            if res.granted:
                self._res2_success += 1
                new_ap.held_resources.update(needed)
                self._promote_dwell_to_path_hold(robot_name, new_ap)
                self._last_fail.pop(robot_name, None)
                self.get_logger().info(
                    f"[RES2] {robot_name} OK avoiding={blocked_rid} "
                    f"route={len(alt_route)}nodes"
                )
                self._publish_event("res2_triggered", {
                    "robot": robot_name,
                    "blocked_resource": blocked_rid,
                    "blocking_robot": deny_result.blocking_robot,
                    "granted": True,
                    "original_waypoints": len(remaining_old),
                    "new_waypoints": len(alt_route),
                    "res2_attempts": self._res2_attempts,
                    "res2_success": self._res2_success,
                })
                self._publish_event("grant", {
                    "robot": robot_name,
                    "resources": needed,
                    "segment_idx": new_ap.current_segment_idx,
                    "via_res2": True,
                })
                full = self._send_path_request(robot_name)
                if not full:
                    new_ap.waiting = True
                    if not new_ap.waiting_since:
                        new_ap.waiting_since = time.time()
                else:
                    new_ap.waiting = False
                    new_ap.waiting_since = 0.0
                return True

        fail_key = f"res2r:{robot_name}:{blocked_rid}"
        if self._last_fail.get(robot_name) != fail_key:
            self._last_fail[robot_name] = fail_key
            self.get_logger().info(
                f"[RES2] {robot_name} FAIL reserve_failed "
                f"blocked={blocked_rid}"
            )
        self._publish_event("res2_triggered", {
            "robot": robot_name,
            "blocked_resource": blocked_rid,
            "blocking_robot": deny_result.blocking_robot,
            "granted": False,
            "reason": "alternative_reservation_failed",
            "original_waypoints": len(remaining_old),
            "new_waypoints": len(alt_route),
            "res2_attempts": self._res2_attempts,
            "res2_success": self._res2_success,
        })
        new_ap.waiting = True
        new_ap.waiting_since = time.time()
        return True

    def _send_path_request(self, robot_name: str) -> bool:
        """Yol gönder. True=tam yol, False=kavşak guard nedeniyle kesilmiş yol."""
        ap = self._active_paths.get(robot_name)
        if ap is None:
            return True

        total_seg = len(ap.waypoint_node_indices) - 1

        send_up_to = total_seg + 1
        for i in range(ap.current_segment_idx + 1, total_seg + 1):
            node_idx = ap.waypoint_node_indices[i]
            if self._cp_manager.is_junction(node_idx):
                if not self._is_junction_physically_clear(node_idx, robot_name):
                    send_up_to = i
                    break

        truncated = send_up_to <= total_seg

        if send_up_to <= ap.current_segment_idx:
            self._send_hold_at_current_position(robot_name)
            return False

        self._cmd_id += 1
        msg = PathRequest()
        msg.fleet_name = self._fleet_name
        msg.robot_name = robot_name
        msg.task_id = str(self._cmd_id)

        robot = self._robots.get(robot_name)
        if robot and robot.last_state_time > 0:
            cur = Location()
            cur.x, cur.y, cur.yaw = robot.x, robot.y, robot.yaw
            cur.level_name = _MAP_LEVEL
            msg.path.append(cur)
        else:
            msg.path.append(
                self._loc_from_node(
                    ap.waypoint_node_indices[ap.current_segment_idx],
                )
            )

        for i in range(ap.current_segment_idx + 1, send_up_to):
            cur_idx = ap.waypoint_node_indices[i]
            msg.path.append(self._loc_from_node(cur_idx))

        self._path_pub.publish(msg)
        return not truncated

    def _send_hold_at_current_position(self, robot_name: str) -> None:
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
                owner = self._reservation_server.get_owner(
                    self._dwell_resource_id(dwell_node_idx)
                )
                self.get_logger().warn(
                    f"[{robot_name}] Could not reserve dwell node "
                    f"{self._node_name(dwell_node_idx)}"
                    + (f" (held by {owner})" if owner else "")
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

        task = self._active_tasks.get(robot_name)
        if task and task.phase in (DeliveryPhase.AT_PICKUP, DeliveryPhase.AT_DROPOFF):
            self._advance_delivery(robot_name)
            return

        ap = self._active_paths.get(robot_name)
        if ap is None or ap.waiting:
            return

        total_seg = len(ap.waypoint_node_indices) - 1
        if ap.current_segment_idx >= total_seg:
            return

        if time.time() - ap.assign_time < 1.5:
            return

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
            for r in self._resource_graph.get_segment_resources_idkr(from_idx, to_idx):
                if r not in needed and r not in ap.held_resources:
                    needed.append(r)

        cur_node_idx = ap.waypoint_node_indices[ap.current_segment_idx]
        cur_rid = self._current_node_resource_id(ap, ap.current_segment_idx)
        if cur_rid not in ap.held_resources and cur_rid not in needed:
            needed.insert(0, cur_rid)
        return needed

    def _current_node_resource_id(self, ap: ActivePath, seg_idx: int) -> str:
        """Mevcut segment'teki düğümün kaynak ID'sini hesapla."""
        node_idx = ap.waypoint_node_indices[seg_idx]
        prev_node = ap.waypoint_node_indices[seg_idx - 1] if seg_idx > 0 else None
        return self._resource_graph._node_resource_id(node_idx, prev_node)

    def _get_passed_resources(self, ap: ActivePath, current_end: int) -> list[str]:
        to_release: list[str] = []
        current_node_idx = ap.waypoint_node_indices[current_end]
        prev_node = ap.waypoint_node_indices[current_end - 1] if current_end > 0 else None
        current_rid = self._resource_graph._node_resource_id(current_node_idx, prev_node)

        for rid in list(ap.held_resources):
            if rid == current_rid:
                continue

            if rid.startswith("node_"):
                node_idx = int(rid.split("_")[1])
                try:
                    pos = ap.waypoint_node_indices.index(node_idx)
                    if pos < current_end:
                        to_release.append(rid)
                except ValueError:
                    to_release.append(rid)

            elif rid.startswith("cp_"):
                parts = rid.split("_")
                node_idx = int(parts[1])
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

    def _dwell_resource_id(self, node_idx: int) -> str:
        """Dwell node için uygun kaynak ID'si.

        Kavşak düğümlerinde ilk CP kullanılır (node_X artık yok).
        Normal düğümlerde node_X kullanılır.
        """
        return self._resource_graph._node_resource_id(node_idx, prev_node=None)

    def _release_robot_resources(self, robot_name: str, keep_dwell: bool = True):
        dwell_idx = self._dwell_nodes.get(robot_name)

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
        elif dwell_idx is not None:
            dwell_rid = self._dwell_resource_id(dwell_idx)
            self._reservation_server.reserve(robot_name, [dwell_rid])

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

    def _is_pickup_available(self, pickup_node_idx: int) -> bool:
        rid = self._dwell_resource_id(pickup_node_idx)
        if self._reservation_server.get_owner(rid) is not None:
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
        rid = self._dwell_resource_id(node_idx)
        result = self._reservation_server.reserve(robot_name, [rid])
        if result.granted:
            self._dwell_nodes[robot_name] = node_idx
        return result.granted

    def _promote_dwell_to_path_hold(
        self, robot_name: str, ap: ActivePath,
    ) -> None:
        dwell_idx = self._dwell_nodes.get(robot_name)
        if dwell_idx is None:
            return

        dwell_rid = self._dwell_resource_id(dwell_idx)
        if dwell_rid in ap.held_resources:
            del self._dwell_nodes[robot_name]
            return

        if self._resource_graph.cp_manager.is_junction(dwell_idx):
            info = self._resource_graph.cp_manager.get_junction_info(dwell_idx)
            if info:
                for cp in info.control_points:
                    if cp.resource_id in ap.held_resources:
                        self._reservation_server.release(robot_name, [dwell_rid])
                        del self._dwell_nodes[robot_name]
                        return

    def _is_junction_physically_clear(
        self, junction_node: int, robot_name: str,
    ) -> bool:
        """Kavşak merkezine yakın başka aktif robot var mı kontrol et."""
        jn = self._resource_graph.nodes.get(junction_node)
        if jn is None:
            return True
        for name, robot in self._robots.items():
            if name == robot_name or robot.last_state_time <= 0:
                continue
            if name not in self._active_paths:
                continue
            dist = math.hypot(robot.x - jn.x, robot.y - jn.y)
            if dist < _JUNCTION_GUARD_DIST:
                return False
        return True

    def _check_collision_proximity(self) -> None:
        """Hareket halindeki robotlar arası mesafeyi kontrol et, yakın geçişleri logla."""
        _NEAR_MISS_DIST = 0.8
        active = [
            (name, r) for name, r in self._robots.items()
            if r.last_state_time > 0 and name in self._active_paths
        ]
        now = time.time()
        still_close: set[tuple[str, str]] = set()

        for i in range(len(active)):
            for j in range(i + 1, len(active)):
                n1, r1 = active[i]
                n2, r2 = active[j]
                dist = math.hypot(r1.x - r2.x, r1.y - r2.y)
                if dist >= _NEAR_MISS_DIST:
                    continue
                pair = (min(n1, n2), max(n1, n2))
                still_close.add(pair)
                if pair in self._collision_pairs_logged:
                    continue
                self._collision_pairs_logged.add(pair)
                self._near_miss_count += 1
                nearest = self._closest_node(
                    (r1.x + r2.x) / 2, (r1.y + r2.y) / 2,
                )
                self.get_logger().warn(
                    f"[COLLISION] {n1} <-> {n2} dist={dist:.2f}m "
                    f"near={self._node_name(nearest) if nearest else '?'} "
                    f"(total={self._near_miss_count})"
                )

        self._collision_pairs_logged &= still_close

    def _get_robot_path(self, robot_name: str) -> list[int] | None:
        ap = self._active_paths.get(robot_name)
        if ap:
            return ap.waypoint_node_indices
        return None

    # ------------------------------------------------------------------
    # RViz route visualization
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
    node = StandaloneTrafficManagerIDKR()
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
