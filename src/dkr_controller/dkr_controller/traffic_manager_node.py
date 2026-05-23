"""
DKR Traffic Manager — ana koordinasyon node'u.

Fleet manager'dan gelen PathRequest'leri yakalar, kaynak grafiğe dönüştürür
ve tam yol rezervasyonu ile robota iletir.

Akış:
  1. PathRequest (raw) gelir → BFS ile tam yol bulunur → kaynak listesine çevrilir
  2. Tüm yol kaynakları tek seferde reserve istenir (all-or-nothing)
  3. Grant → robot'a TÜM yol PathRequest olarak gönderilir
  4. Robot ilerledikçe geçtiği kaynaklar release edilir
  5. Deny → robot bekler, retry timer çalışır
  6. Deadlock → victim geri çekilir
"""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field
from pathlib import Path

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

from .resource_graph import ResourceGraph
from .reservation_server import ReservationServer
from .deadlock_detector import DeadlockDetector


@dataclass
class RobotPath:
    """Bir robotun DKR tarafından yönetilen aktif yolu."""
    robot_name: str
    task_id: str
    # Full path waypoints (Location list from original PathRequest)
    waypoints: list = field(default_factory=list)
    # Nav graph node indices corresponding to waypoints
    waypoint_node_indices: list[int] = field(default_factory=list)
    # Full resource sequence for the path
    all_resources: list[str] = field(default_factory=list)
    # Current segment index (which waypoint pair we're on)
    current_segment_idx: int = 0
    # Resources currently held by this robot
    held_resources: set[str] = field(default_factory=set)
    # Whether the robot is waiting for a grant
    waiting: bool = False
    # Timestamp path was assigned
    assign_time: float = 0.0
    # When waiting=True, time we started waiting (for stale watchdog)
    waiting_since: float = 0.0


# Position tolerance for segment completion detection (metres)
_ARRIVAL_TOLERANCE = 1.5
# Snap PathRequest coordinates to nav graph vertices (fleet waypoints may be offset)
_SNAP_TOLERANCE_M = 4.0
# Fleet manager cmd_id changes every cycle — dedup by goal node, not task_id
_SAME_GOAL_EPS_M = 0.5
# If robot waits longer than this for a full-path grant, trigger resolution (seconds)
_STALE_WAIT_SEC = 12.0
# After stale resolution, bypass DKR for this duration so robot can move
_DKR_BYPASS_COOLDOWN_SEC = 30.0
_LONG_BYPASS_SEC = 60.0
_MAX_STALE_RETRIES = 2
# Paths shorter than this many graph nodes skip DKR (micro-legs, no corridor control needed)
_MIN_GRAPH_NODES_FOR_DKR = 4
# Ignore repeat PathRequest to same goal shortly after completion
_GOAL_DEBOUNCE_SEC = 3.0
_MAP_LEVEL = "L1"


class DkrTrafficManagerNode(Node):

    def __init__(self):
        super().__init__("dkr_traffic_manager")

        # Parameters
        self.declare_parameter("nav_graph_file", "")
        self.declare_parameter("building_yaml", "")  # deprecated — use nav_graph_file
        self.declare_parameter("lookahead", 1)
        self.declare_parameter("retry_interval_sec", 0.5)
        self.declare_parameter("deadlock_check_interval_sec", 2.0)
        self.declare_parameter("arrival_tolerance", _ARRIVAL_TOLERANCE)

        nav_graph_file = self.get_parameter("nav_graph_file").value
        building_yaml = self.get_parameter("building_yaml").value
        graph_file = nav_graph_file or building_yaml

        self._lookahead = self.get_parameter("lookahead").value
        retry_interval = self.get_parameter("retry_interval_sec").value
        deadlock_interval = self.get_parameter("deadlock_check_interval_sec").value
        self._arrival_tol = self.get_parameter("arrival_tolerance").value

        if not graph_file:
            self.get_logger().error(
                "nav_graph_file parameter is required "
                "(rmf_demos_maps/.../nav_graphs/0.yaml)"
            )
            raise SystemExit(1)

        # Core DKR components — nav graph metre koordinatları (PathRequest ile uyumlu)
        self._resource_graph = ResourceGraph.from_graph_file(graph_file)
        self._reservation_server = ReservationServer(
            self._resource_graph.all_resource_ids()
        )
        self._deadlock_detector = DeadlockDetector()
        self._reservation_server.set_deadlock_detector(self._deadlock_detector)

        self.get_logger().info(
            f"Resource graph loaded from {graph_file}: {self._resource_graph}"
        )
        self.get_logger().info(
            "Listening on robot_path_requests_raw → publishing robot_path_requests"
        )

        # Active robot paths managed by DKR
        self._robot_paths: dict[str, RobotPath] = {}
        # Latest robot positions
        self._robot_positions: dict[str, tuple[float, float, float]] = {}
        # Recently completed goal per robot — debounce fleet_manager re-sends
        self._recent_goal_done: dict[str, tuple[int, float]] = {}
        # Last raw PathRequest per robot (for bypass after stale wait)
        self._last_raw_request: dict[str, PathRequest] = {}
        # Until this time, robot bypasses DKR and uses raw paths only
        self._dkr_bypass_until: dict[str, float] = {}
        # Stale-wait count per (robot, goal_node) for escalating bypass
        self._stale_counts: dict[tuple[str, int], int] = {}

        # QoS profiles
        reliable_volatile = QoSProfile(
            history=History.KEEP_LAST,
            depth=10,
            reliability=Reliability.RELIABLE,
            durability=Durability.VOLATILE,
        )
        system_default = QoSProfile(
            history=History.KEEP_LAST,
            depth=10,
            reliability=Reliability.RELIABLE,
            durability=Durability.VOLATILE,
        )

        # Subscriptions
        self.create_subscription(
            PathRequest,
            "robot_path_requests_raw",
            self._on_path_request,
            system_default,
        )
        self.create_subscription(
            RobotState,
            "robot_state",
            self._on_robot_state,
            reliable_volatile,
        )

        # Publishers
        self._path_pub = self.create_publisher(
            PathRequest, "robot_path_requests", system_default
        )
        self._event_pub = self.create_publisher(
            String, "/dkr_events", reliable_volatile
        )

        # Timers
        self.create_timer(retry_interval, self._retry_waiting_robots)
        self.create_timer(deadlock_interval, self._check_deadlocks)
        self.create_timer(5.0, self._watchdog_stale_waits)

        self.get_logger().info("DKR Traffic Manager started.")

    # ------------------------------------------------------------------
    # PathRequest handling
    # ------------------------------------------------------------------

    def _on_path_request(self, msg: PathRequest):
        """Fleet manager'dan gelen ham PathRequest'i yakala ve DKR yönetimine al."""
        robot_name = msg.robot_name
        task_id = msg.task_id

        self._last_raw_request[robot_name] = msg

        # Bypass DKR after repeated stale waits — forward fleet path directly
        bypass_until = self._dkr_bypass_until.get(robot_name, 0.0)
        if time.time() < bypass_until:
            self._path_pub.publish(msg)
            return

        if len(msg.path) < 2:
            # Stop command or empty path — forward directly
            self._path_pub.publish(msg)
            return

        # Robot already at destination (micro-adjustment at dispenser/dropoff)
        if self._path_is_at_destination(msg.path):
            self._path_pub.publish(msg)
            return

        waypoint_indices = self._resolve_path_to_graph_indices(robot_name, msg.path)

        if not waypoint_indices or len(waypoint_indices) < 2:
            self.get_logger().debug(
                f"[{robot_name}] Short/unresolved path "
                f"({msg.path[0].x:.2f},{msg.path[0].y:.2f})→"
                f"({msg.path[-1].x:.2f},{msg.path[-1].y:.2f}), forward raw."
            )
            self._release_robot_resources(robot_name)
            self._path_pub.publish(msg)
            return

        goal_idx = waypoint_indices[-1]

        # Fleet re-sends navigate to same waypoint right after DKR released it
        recent = self._recent_goal_done.get(robot_name)
        if recent and recent[0] == goal_idx:
            if time.time() - recent[1] < _GOAL_DEBOUNCE_SEC:
                self._path_pub.publish(msg)
                return

        # Micro-legs (< 4 graph nodes): no corridor to reserve, forward directly
        if len(waypoint_indices) < _MIN_GRAPH_NODES_FOR_DKR:
            self._path_pub.publish(msg)
            return

        # Fleet manager sends new PathRequest every cycle with a NEW task_id (cmd_id).
        # Dedup by goal node only while still travelling toward that goal.
        existing = self._robot_paths.get(robot_name)
        if existing is not None and existing.waypoint_node_indices[-1] == goal_idx:
            total_seg = len(existing.waypoint_node_indices) - 1
            still_en_route = existing.current_segment_idx < total_seg
            if still_en_route or existing.waiting:
                if existing.waiting:
                    self._try_advance_robot(robot_name)
                return

        all_resources = self._resource_graph.get_path_resources(waypoint_indices)

        if robot_name in self._robot_paths:
            old_goal = self._robot_paths[robot_name].waypoint_node_indices[-1]
            if old_goal != goal_idx:
                self._release_robot_resources(robot_name)
                self._deadlock_detector.clear_robot(robot_name)

        rp = RobotPath(
            robot_name=robot_name,
            task_id=task_id,
            waypoints=list(msg.path),
            waypoint_node_indices=waypoint_indices,
            all_resources=all_resources,
            current_segment_idx=0,
            assign_time=time.time(),
        )
        self._robot_paths[robot_name] = rp

        self._publish_event("path_received", {
            "robot": robot_name,
            "task_id": task_id,
            "waypoint_count": len(waypoint_indices),
            "total_resources": len(all_resources),
        })

        self.get_logger().info(
            f"[{robot_name}] New path: {len(waypoint_indices)} graph nodes, "
            f"{len(all_resources)} resources, goal={self._node_name(goal_idx)}, "
            f"task={task_id[:40]}"
        )

        self._try_advance_robot(robot_name)

    def _node_name(self, idx: int) -> str:
        node = self._resource_graph.nodes.get(idx)
        return node.name if node else f"node_{idx}"

    def _resolve_path_to_graph_indices(
        self, robot_name: str, path: list
    ) -> list[int]:
        """
        PathRequest waypoints → nav graph node list.

        Fleet manager typically sends only [current, target]. Expand via BFS on
        the graph so DKR reserves every corridor segment along the route.
        """
        if len(path) < 2:
            return []

        start_idx = self._snap_to_node(path[0].x, path[0].y)
        goal_idx = self._snap_to_node(path[-1].x, path[-1].y)

        if start_idx is None:
            pos = self._robot_positions.get(robot_name)
            if pos:
                start_idx = self._snap_to_node(pos[0], pos[1])
        if start_idx is None:
            start_idx = goal_idx
        if goal_idx is None:
            goal_idx = self._closest_node_any(path[-1].x, path[-1].y)
        if start_idx is None:
            start_idx = goal_idx
        if goal_idx is None:
            return []

        if start_idx == goal_idx:
            return []

        full_path = self._resource_graph.find_path_bfs(start_idx, goal_idx)
        if full_path:
            return full_path

        # BFS failed — no route exists in graph. Only use direct edge if it
        # actually exists, otherwise return empty (will be forwarded as raw).
        edge_id = f"edge_{start_idx}_{goal_idx}"
        if edge_id in self._resource_graph.edges:
            return [start_idx, goal_idx]

        self.get_logger().warn(
            f"[{robot_name}] No BFS route from "
            f"{self._node_name(start_idx)} to {self._node_name(goal_idx)}"
        )
        return []

    def _snap_to_node(self, x: float, y: float) -> int | None:
        """Snap (x,y) to nearest graph vertex within tolerance."""
        best_idx = None
        best_dist = float("inf")
        for idx, node in self._resource_graph.nodes.items():
            dist = math.hypot(node.x - x, node.y - y)
            if dist < best_dist:
                best_dist = dist
                best_idx = idx
        if best_idx is not None and best_dist <= _SNAP_TOLERANCE_M:
            return best_idx
        return None

    def _closest_node_any(self, x: float, y: float) -> int | None:
        """Nearest graph vertex, no distance limit (fallback when snap fails)."""
        best_idx = None
        best_dist = float("inf")
        for idx, node in self._resource_graph.nodes.items():
            dist = math.hypot(node.x - x, node.y - y)
            if dist < best_dist:
                best_dist = dist
                best_idx = idx
        return best_idx

    def _path_is_at_destination(self, path: list) -> bool:
        """True when start and goal are the same graph vertex (within epsilon)."""
        if len(path) < 2:
            return False
        dx = path[-1].x - path[0].x
        dy = path[-1].y - path[0].y
        if math.hypot(dx, dy) <= _SAME_GOAL_EPS_M:
            return True
        s = self._snap_to_node(path[0].x, path[0].y)
        g = self._snap_to_node(path[-1].x, path[-1].y)
        if s is not None and g is not None and s == g:
            return True
        return False

    def _location_from_node(self, node_idx: int) -> Location:
        """Build RMF Location from nav graph vertex coordinates."""
        node = self._resource_graph.nodes[node_idx]
        loc = Location()
        loc.x = node.x
        loc.y = node.y
        loc.yaw = 0.0
        loc.level_name = _MAP_LEVEL
        return loc

    def _collect_needed_resources(self, robot_name: str, rp: RobotPath) -> list[str]:
        """All resources still needed for the FULL remaining path (not just lookahead)."""
        total_segments = len(rp.waypoint_node_indices) - 1
        if rp.current_segment_idx >= total_segments:
            return []

        needed: list[str] = []
        for seg_i in range(rp.current_segment_idx, total_segments):
            from_idx = rp.waypoint_node_indices[seg_i]
            to_idx = rp.waypoint_node_indices[seg_i + 1]
            for r in self._resource_graph.get_segment_resources(from_idx, to_idx):
                if r not in needed and r not in rp.held_resources:
                    needed.append(r)

        current_node = f"node_{rp.waypoint_node_indices[rp.current_segment_idx]}"
        if current_node not in rp.held_resources and current_node not in needed:
            needed.insert(0, current_node)
        return needed

    # ------------------------------------------------------------------
    # Segment advancement
    # ------------------------------------------------------------------

    def _try_advance_robot(self, robot_name: str) -> bool:
        """
        Robotun TÜM kalan yolunu tek seferde reserve etmeyi dene.

        Başarılı olursa robota tam kalan PathRequest gönderir ve True döner.
        Başarısızsa robot bekler, retry timer tekrar dener.
        """
        rp = self._robot_paths.get(robot_name)
        if rp is None:
            return False

        total_segments = len(rp.waypoint_node_indices) - 1
        if rp.current_segment_idx >= total_segments:
            self._complete_path(robot_name)
            return True

        needed_resources = self._collect_needed_resources(robot_name, rp)

        if not needed_resources:
            self._send_full_remaining_path(robot_name)
            rp.waiting = False
            return True

        # All-or-nothing: reserve entire remaining path at once
        result = self._reservation_server.reserve(robot_name, needed_resources)

        if result.granted:
            rp.held_resources.update(needed_resources)
            rp.waiting = False
            rp.waiting_since = 0.0
            self._deadlock_detector.remove_wait(robot_name)

            self._publish_event("grant", {
                "robot": robot_name,
                "resources": needed_resources,
                "segment_idx": rp.current_segment_idx,
                "total_reserved": len(rp.held_resources),
            })

            self._send_full_remaining_path(robot_name)
            return True
        else:
            rp.waiting = True
            if not rp.waiting_since:
                rp.waiting_since = time.time()
            if result.blocking_robot:
                self._deadlock_detector.add_wait(
                    robot_name,
                    result.blocking_robot,
                    result.blocking_resources[0] if result.blocking_resources else "",
                )

            self._publish_event("deny", {
                "robot": robot_name,
                "resources_requested": needed_resources,
                "blocking_robot": result.blocking_robot,
                "reason": result.reason,
            })

            self.get_logger().debug(
                f"[{robot_name}] Waiting: {result.reason}"
            )
            return False

    def _send_full_remaining_path(self, robot_name: str) -> None:
        """Robot'a kalan TÜM yolu tek PathRequest olarak gönder."""
        rp = self._robot_paths.get(robot_name)
        if rp is None:
            return

        total_segments = len(rp.waypoint_node_indices) - 1

        msg = PathRequest()
        msg.fleet_name = "warehouseRobot"
        msg.robot_name = robot_name
        msg.task_id = rp.task_id

        pos = self._robot_positions.get(robot_name)
        if pos:
            cur = Location()
            cur.x, cur.y, cur.yaw = pos[0], pos[1], pos[2]
            cur.level_name = _MAP_LEVEL
            msg.path.append(cur)
        else:
            msg.path.append(
                self._location_from_node(rp.waypoint_node_indices[rp.current_segment_idx])
            )

        for node_i in range(rp.current_segment_idx + 1, total_segments + 1):
            msg.path.append(
                self._location_from_node(rp.waypoint_node_indices[node_i])
            )

        self._path_pub.publish(msg)

    def _complete_path(self, robot_name: str) -> None:
        """Robot yolunu tamamladı — kaynakları serbest bırak."""
        goal_idx = None
        if robot_name in self._robot_paths:
            goal_idx = self._robot_paths[robot_name].waypoint_node_indices[-1]

        self._release_robot_resources(robot_name)
        self._deadlock_detector.clear_robot(robot_name)

        if robot_name in self._robot_paths:
            del self._robot_paths[robot_name]

        if goal_idx is not None:
            self._recent_goal_done[robot_name] = (goal_idx, time.time())
            # Clear stale counter for this goal on successful completion
            self._stale_counts.pop((robot_name, goal_idx), None)

        self.get_logger().info(
            f"[{robot_name}] Path completed, resources released."
            + (f" goal={self._node_name(goal_idx)}" if goal_idx is not None else "")
        )

    # ------------------------------------------------------------------
    # Robot state monitoring — segment completion detection
    # ------------------------------------------------------------------

    def _on_robot_state(self, msg: RobotState):
        """Robot konumunu izle, geçilen kaynakları release et, yol tamamlanmasını tespit et."""
        robot_name = msg.name
        x = msg.location.x
        y = msg.location.y
        yaw = msg.location.yaw
        self._robot_positions[robot_name] = (x, y, yaw)

        rp = self._robot_paths.get(robot_name)
        if rp is None or rp.waiting:
            return

        total_segments = len(rp.waypoint_node_indices) - 1
        if rp.current_segment_idx >= total_segments:
            return

        if time.time() - rp.assign_time < 1.5:
            return

        # Find the furthest segment the robot has reached (not just the next one).
        # This handles cases where the robot passes multiple nodes between callbacks.
        best_seg = rp.current_segment_idx
        for seg_i in range(rp.current_segment_idx + 1, total_segments + 1):
            node_idx = rp.waypoint_node_indices[seg_i]
            node = self._resource_graph.nodes.get(node_idx)
            if node is None:
                continue
            dist = math.hypot(x - node.x, y - node.y)
            if dist <= self._arrival_tol:
                best_seg = seg_i

        if best_seg > rp.current_segment_idx:
            resources_to_release = self._get_passed_resources(rp, best_seg)
            if resources_to_release:
                self._reservation_server.release(robot_name, resources_to_release)
                rp.held_resources -= set(resources_to_release)
                self._publish_event("release", {
                    "robot": robot_name,
                    "resources": resources_to_release,
                })

            rp.current_segment_idx = best_seg

            if rp.current_segment_idx >= total_segments:
                self._complete_path(robot_name)

    def _get_passed_resources(
        self, rp: RobotPath, current_end_seg: int
    ) -> list[str]:
        """
        Robot'un geçtiği (artık ihtiyaç duymadığı) kaynakları belirle.

        Kuralı: current node'u TUT, arkadaki node ve edge'leri release et.
        """
        to_release: list[str] = []
        # Keep current node, release everything before it
        current_node_id = f"node_{rp.waypoint_node_indices[current_end_seg]}"

        for rid in list(rp.held_resources):
            if rid == current_node_id:
                continue
            # Keep resources ahead of current position
            # Release resources behind (node indices before current_end_seg)
            if rid.startswith("node_"):
                node_idx = int(rid.split("_")[1])
                # Keep if it's at or ahead of current position in the path
                try:
                    pos_in_path = rp.waypoint_node_indices.index(node_idx)
                    if pos_in_path < current_end_seg:
                        to_release.append(rid)
                except ValueError:
                    to_release.append(rid)
            elif rid.startswith("edge_"):
                parts = rid.split("_")
                from_n = int(parts[1])
                # Release edges whose from_node is behind current position
                try:
                    pos_in_path = rp.waypoint_node_indices.index(from_n)
                    if pos_in_path < current_end_seg:
                        to_release.append(rid)
                except ValueError:
                    to_release.append(rid)

        return to_release

    # ------------------------------------------------------------------
    # Retry and deadlock timers
    # ------------------------------------------------------------------

    def _retry_waiting_robots(self):
        """Bekleyen robotlar için periyodik olarak reserve'ı yeniden dene."""
        for robot_name in list(self._robot_paths.keys()):
            rp = self._robot_paths.get(robot_name)
            if rp and rp.waiting:
                self._try_advance_robot(robot_name)

    def _watchdog_stale_waits(self):
        """Long waits → release only contested blocker resources, then retry."""
        now = time.time()
        for robot_name in list(self._robot_paths.keys()):
            rp = self._robot_paths.get(robot_name)
            if rp is None or not rp.waiting or not rp.waiting_since:
                continue
            if now - rp.waiting_since < _STALE_WAIT_SEC:
                continue

            goal_idx = rp.waypoint_node_indices[-1]
            blocker_name = ""
            contested_resource = ""
            for edge in self._deadlock_detector.get_wait_graph_snapshot():
                if edge.waiting_robot == robot_name:
                    blocker_name = edge.blocking_robot
                    contested_resource = edge.contested_resource
                    break

            stale_key = (robot_name, goal_idx)
            stale_n = self._stale_counts.get(stale_key, 0) + 1
            self._stale_counts[stale_key] = stale_n

            preempted = 0
            if blocker_name:
                if stale_n < _MAX_STALE_RETRIES:
                    # First attempts: only release the contested resources from
                    # the blocker (resources the waiting robot actually needs).
                    needed = set(self._collect_needed_resources(robot_name, rp))
                    blocker_held = self._reservation_server.get_robot_resources(
                        blocker_name
                    )
                    contested = list(needed & blocker_held)
                    if contested:
                        self._reservation_server.release(blocker_name, contested)
                        blocker_rp = self._robot_paths.get(blocker_name)
                        if blocker_rp:
                            blocker_rp.held_resources -= set(contested)
                        preempted = len(contested)
                else:
                    # Escalation: release all of blocker's resources
                    blocker_resources = self._reservation_server.release_all(
                        blocker_name
                    )
                    preempted = len(blocker_resources)
                    blocker_rp = self._robot_paths.get(blocker_name)
                    if blocker_rp:
                        blocker_rp.held_resources.clear()

                # Let blocker re-reserve from its current position
                blocker_rp = self._robot_paths.get(blocker_name)
                if blocker_rp:
                    blocker_rp.waiting = False
                    blocker_rp.waiting_since = 0.0
                    actual_seg = self._find_current_segment(blocker_name, blocker_rp)
                    blocker_rp.current_segment_idx = actual_seg
                self._deadlock_detector.clear_robot(blocker_name)

            # Reset the waiting robot and retry reservation immediately
            rp.waiting = False
            rp.waiting_since = 0.0
            self._deadlock_detector.clear_robot(robot_name)

            bypass_sec = (
                _LONG_BYPASS_SEC
                if stale_n >= _MAX_STALE_RETRIES
                else _DKR_BYPASS_COOLDOWN_SEC
            )

            self.get_logger().warn(
                f"[{robot_name}] Stale wait {now - rp.waiting_since:.0f}s "
                f"for {self._node_name(goal_idx)}"
                + (f" (blocked by {blocker_name})" if blocker_name else "")
                + f" — preempted {preempted} resources (attempt {stale_n})"
            )

            self._publish_event("stale_bypass", {
                "robot": robot_name,
                "goal": self._node_name(goal_idx),
                "blocker": blocker_name,
                "preempted": preempted,
                "attempt": stale_n,
            })

            # Try to grant the waiting robot immediately after preemption
            granted = self._try_advance_robot(robot_name)

            # Only bypass DKR entirely as last resort
            if not granted and stale_n >= _MAX_STALE_RETRIES:
                self._dkr_bypass_until[robot_name] = now + bypass_sec
                self._release_robot_resources(robot_name)
                self._deadlock_detector.clear_robot(robot_name)
                if robot_name in self._robot_paths:
                    del self._robot_paths[robot_name]
                raw = self._last_raw_request.get(robot_name)
                if raw:
                    self._path_pub.publish(raw)
                self.get_logger().warn(
                    f"[{robot_name}] DKR bypass {bypass_sec:.0f}s (escalation)"
                )

    def _check_deadlocks(self):
        """Periyodik deadlock kontrolü ve çözümü."""
        cycle = self._deadlock_detector.detect_cycle()
        if cycle is None:
            return

        victim = self._deadlock_detector.resolve_deadlock(cycle)
        if victim is None:
            return

        self.get_logger().warn(
            f"DEADLOCK detected: {cycle}. Victim: {victim}"
        )

        self._publish_event("deadlock", {
            "cycle": cycle,
            "victim": victim,
            "resolution": "yield",
        })

        # Resolution: victim releases all resources and retries from current position
        self._release_robot_resources(victim)
        self._deadlock_detector.clear_robot(victim)

        rp = self._robot_paths.get(victim)
        if rp:
            rp.held_resources.clear()
            rp.waiting = False
            rp.waiting_since = 0.0
            # Find the victim's actual position on the path instead of resetting to 0
            actual_seg = self._find_current_segment(victim, rp)
            rp.current_segment_idx = actual_seg
            self._try_advance_robot(victim)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _find_current_segment(self, robot_name: str, rp: RobotPath) -> int:
        """Determine which segment the robot is physically closest to on its path."""
        pos = self._robot_positions.get(robot_name)
        if not pos:
            return rp.current_segment_idx

        best_seg = 0
        best_dist = float("inf")
        for seg_i, node_idx in enumerate(rp.waypoint_node_indices):
            node = self._resource_graph.nodes.get(node_idx)
            if node is None:
                continue
            dist = math.hypot(pos[0] - node.x, pos[1] - node.y)
            if dist < best_dist:
                best_dist = dist
                best_seg = seg_i

        # Don't go backwards from where we already were
        return max(best_seg, rp.current_segment_idx)

    def _release_robot_resources(self, robot_name: str) -> None:
        """Robot'un tüm kaynaklarını serbest bırak."""
        released = self._reservation_server.release_all(robot_name)
        rp = self._robot_paths.get(robot_name)
        if rp:
            rp.held_resources.clear()
        if released:
            self._publish_event("release", {
                "robot": robot_name,
                "resources": released,
            })

    def _publish_event(self, event_type: str, data: dict) -> None:
        """Publish a DKR event for the metric logger."""
        event = {
            "type": event_type,
            "timestamp": time.time(),
            **data,
        }
        msg = String()
        msg.data = json.dumps(event)
        self._event_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DkrTrafficManagerNode()
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
