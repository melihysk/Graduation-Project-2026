"""
Robot durumu takibi.

/fleet_states topic'inden her robotun MODE_MOVING, MODE_IDLE, MODE_WAITING
sürelerini hesaplar. Bekleme süresi varyansı (fairness) metriğini üretir.
"""

import time
import math
from dataclasses import dataclass, field


@dataclass
class RobotRecord:
    name: str
    total_moving_time: float = 0.0
    total_idle_time: float = 0.0
    total_waiting_time: float = 0.0
    total_charging_time: float = 0.0
    last_mode: int = 0  # MODE_IDLE
    last_update_time: float = 0.0
    last_x: float = 0.0
    last_y: float = 0.0
    last_yaw: float = 0.0
    total_distance: float = 0.0


# RobotMode constants
MODE_IDLE = 0
MODE_CHARGING = 1
MODE_MOVING = 2
MODE_PAUSED = 3
MODE_WAITING = 4


class RobotTracker:

    def __init__(self, logger):
        self._logger = logger
        self._robots: dict[str, RobotRecord] = {}
        self._start_time: float = 0.0

    @property
    def robots(self) -> dict[str, RobotRecord]:
        return self._robots

    # task_id patterns that are NOT real delivery work (park / charge tasks).
    _IDLE_TASK_PREFIXES = ("park", "charge", "idle", "loop")

    def on_fleet_state(self, msg):
        """rmf_fleet_msgs/msg/FleetState callback."""
        now = time.time()
        if not self._start_time:
            self._start_time = now

        for robot_state in msg.robots:
            task_id = getattr(robot_state, 'task_id', '').strip()
            self._update_robot(robot_state, now, task_id)

    def _is_active_task(self, task_id: str) -> bool:
        """Return True if task_id represents real delivery/dispatch work."""
        if not task_id or task_id == "0":
            return False
        tid_lower = task_id.lower()
        return not any(tid_lower.startswith(p) for p in self._IDLE_TASK_PREFIXES)

    def _update_robot(self, robot_state, now: float, task_id: str = ""):
        name = robot_state.name
        mode = robot_state.mode.mode
        x = robot_state.location.x
        y = robot_state.location.y
        yaw = robot_state.location.yaw

        if name not in self._robots:
            self._robots[name] = RobotRecord(
                name=name,
                last_mode=mode,
                last_update_time=now,
                last_x=x,
                last_y=y,
                last_yaw=yaw,
            )
            return

        record = self._robots[name]
        dt = now - record.last_update_time

        if dt <= 0 or dt > 5.0:
            record.last_update_time = now
            record.last_mode = mode
            record.last_x = x
            record.last_y = y
            record.last_yaw = yaw
            return

        dx = x - record.last_x
        dy = y - record.last_y
        dist = math.hypot(dx, dy)
        if dist < 10.0:  # sanity check
            record.total_distance += dist

        # Yaw (heading) change — normalize to [0, pi]
        dyaw = abs(yaw - record.last_yaw)
        if dyaw > math.pi:
            dyaw = 2.0 * math.pi - dyaw

        # Determine effective state:
        #   1. Position or yaw changed >= threshold -> MOVING (includes turning in place)
        #   2. Stationary + active task_id          -> WAITING (traffic/dispenser wait)
        #   3. Stationary + no active task          -> fall back to reported mode
        MIN_MOVING_DIST = 0.02   # metres
        MIN_TURNING_RAD = 0.05   # radians (~3 degrees)
        if dist >= MIN_MOVING_DIST or dyaw >= MIN_TURNING_RAD:
            effective_mode = MODE_MOVING
        elif self._is_active_task(task_id):
            effective_mode = MODE_WAITING
        else:
            effective_mode = mode

        if effective_mode == MODE_MOVING:
            record.total_moving_time += dt
        elif effective_mode == MODE_WAITING or effective_mode == MODE_PAUSED:
            record.total_waiting_time += dt
        elif effective_mode == MODE_CHARGING:
            record.total_charging_time += dt
        else:
            record.total_idle_time += dt

        record.last_update_time = now
        record.last_mode = mode
        record.last_x = x
        record.last_y = y
        record.last_yaw = yaw

    def get_metrics(self) -> dict:
        if not self._robots:
            return {
                "wait_time_variance": 0.0,
                "avg_wait_time_sec": 0.0,
                "per_robot": {},
            }

        wait_times = [r.total_waiting_time for r in self._robots.values()]
        avg_wait = sum(wait_times) / len(wait_times) if wait_times else 0.0
        variance = (
            sum((w - avg_wait) ** 2 for w in wait_times) / len(wait_times)
            if wait_times else 0.0
        )

        per_robot = {}
        for r in self._robots.values():
            per_robot[r.name] = {
                "moving_time_sec": round(r.total_moving_time, 2),
                "waiting_time_sec": round(r.total_waiting_time, 2),
                "idle_time_sec": round(r.total_idle_time, 2),
                "charging_time_sec": round(r.total_charging_time, 2),
                "total_distance_m": round(r.total_distance, 2),
            }

        return {
            "wait_time_variance": round(variance, 4),
            "avg_wait_time_sec": round(avg_wait, 2),
            "max_wait_time_sec": round(max(wait_times), 2) if wait_times else 0.0,
            "per_robot": per_robot,
        }
