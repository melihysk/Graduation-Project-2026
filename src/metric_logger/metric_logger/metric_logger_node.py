"""
Ana metrik toplama node'u.

Tüm tracker'ları koordine eder, deney bitince JSON dosyasına yazar.
Bağımsız çalışır — warehouse_starter.launch.xml'e dokunmaz.

Kullanım:
  ros2 run metric_logger metric_logger_node \
    --ros-args -p traffic_mode:=rmf -p scenario_id:=normal -p run_id:=1
"""

import json
import os
import time
from pathlib import Path


# #region agent log
def _agent_debug_ndjson(payload: dict) -> None:
    """Append one NDJSON line for debug-mode analysis (session e6fb85)."""
    log_path = "/home/melih/Desktop/graduation_project/.cursor/debug-e6fb85.log"
    base = {
        "sessionId": "e6fb85",
        "timestamp": int(time.time() * 1000),
        "runId": payload.pop("runId", "repro"),
    }
    try:
        with open(log_path, "a") as dbg:
            dbg.write(json.dumps({**base, **payload}) + "\n")
    except Exception:
        pass


# #endregion

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from rmf_fleet_msgs.msg import FleetState
from rmf_task_msgs.msg import ApiResponse, DispatchStates, TaskSummary
from rmf_traffic_msgs.msg import (
    NegotiationNotice,
    NegotiationConclusion,
    NegotiationStatuses,
    BlockadeSet,
    BlockadeRelease,
)
from std_msgs.msg import String

from .task_tracker import TaskTracker
from .robot_tracker import RobotTracker
from .conflict_tracker import ConflictTracker
from .energy_estimator import EnergyEstimator


class MetricLoggerNode(Node):

    def __init__(self):
        super().__init__('metric_logger')

        # Parameters
        self.declare_parameter('traffic_mode', 'rmf')
        self.declare_parameter('scenario_id', 'normal')
        self.declare_parameter('run_id', 1)
        self.declare_parameter('expected_tasks', 0)
        self.declare_parameter('expected_robots', 4)
        self.declare_parameter('output_dir', '')
        self.declare_parameter('auto_finish_timeout_sec', 60.0)

        self._traffic_mode = self.get_parameter('traffic_mode').value
        self._scenario_id = self.get_parameter('scenario_id').value
        self._run_id = self.get_parameter('run_id').value
        self._expected_tasks = self.get_parameter('expected_tasks').value
        self._expected_robots = int(self.get_parameter('expected_robots').value)
        self._auto_finish_timeout = self.get_parameter('auto_finish_timeout_sec').value

        output_dir = self.get_parameter('output_dir').value
        if not output_dir:
            output_dir = os.path.expanduser(
                '~/Desktop/graduation_project/results'
            )
        self._output_dir = Path(output_dir) / self._traffic_mode / self._scenario_id
        self._output_dir.mkdir(parents=True, exist_ok=True)

        # Trackers
        self._task_tracker = TaskTracker(
            self.get_logger(),
            agent_debug=lambda p: _agent_debug_ndjson(dict(p)),
        )
        self._robot_tracker = RobotTracker(self.get_logger())
        self._conflict_tracker = ConflictTracker(self.get_logger())
        self._energy_estimator = EnergyEstimator(self.get_logger())

        if self._expected_tasks > 0:
            self._task_tracker.set_expected_task_count(self._expected_tasks)

        # RELIABLE + VOLATILE: for fleet_states bursts (many sequential task transitions).
        reliable_volatile = QoSProfile(
            history=History.KEEP_LAST,
            depth=200,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        # RELIABLE + TRANSIENT_LOCAL: publisher keeps history; we get messages
        # published before we subscribed. Required for task_api_responses and
        # dispatch_states (both published with TRANSIENT_LOCAL by RMF).
        reliable_transient = QoSProfile(
            history=History.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        # BEST_EFFORT + VOLATILE: rmf_traffic_blockade uses best-effort on blockade topics.
        best_effort_volatile = QoSProfile(
            history=History.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Previous task_id per robot for transition detection.
        self._robot_prev_task_ids: dict[str, str] = {}
        # Track when a robot first moved with a given task_id (for start_time).
        self._robot_prev_modes: dict[str, int] = {}

        # Fleet state — VOLATILE publisher.
        # Combined handler: robot metrics + task completion detection.
        self.create_subscription(
            FleetState, '/fleet_states', self._on_fleet_state, reliable_volatile
        )

        # Task API responses — TRANSIENT_LOCAL publisher (rmf_dispatcher_node + fleet_adapter)
        # Must match to receive both live updates and history (tasks completed before we subscribed).
        self.create_subscription(
            ApiResponse, '/task_api_responses',
            self._task_tracker.on_task_api_response, reliable_transient
        )

        # Dispatch states — keep VOLATILE+RELIABLE; depth avoids bursty backlog drops.
        self.create_subscription(
            DispatchStates, '/dispatch_states',
            self._on_dispatch_states_cb, reliable_volatile
        )

        # Task summaries — same profile (matches common VOLATILE adapters).
        self.create_subscription(
            TaskSummary, '/task_summaries',
            self._task_tracker.on_task_summary, reliable_volatile
        )

        # Negotiation subscribers — VOLATILE publisher
        self.create_subscription(
            NegotiationNotice, '/rmf_traffic/negotiation_notice',
            self._conflict_tracker.on_negotiation_notice, reliable_volatile
        )
        self.create_subscription(
            NegotiationConclusion, '/rmf_traffic/negotiation_conclusion',
            self._conflict_tracker.on_negotiation_conclusion, reliable_volatile
        )
        self.create_subscription(
            NegotiationStatuses, '/rmf_traffic/negotiation_statuses',
            self._conflict_tracker.on_negotiation_statuses, reliable_volatile
        )

        # Blockade subscribers — BEST_EFFORT publisher (rmf_traffic_blockade)
        self.create_subscription(
            BlockadeSet, '/rmf_traffic/blockade_set',
            self._conflict_tracker.on_blockade_set, best_effort_volatile
        )
        self.create_subscription(
            BlockadeRelease, '/rmf_traffic/blockade_release',
            self._conflict_tracker.on_blockade_release, best_effort_volatile
        )

        # DKR/İDKR event subscriber (Mode 2-3)
        self.create_subscription(
            String, '/dkr_events',
            self._on_dkr_event, reliable_volatile
        )

        # Status timer — check if experiment should end
        self._experiment_start = time.time()
        self._last_task_completion_time = 0.0
        self._last_known_completed_count = 0
        self._finished = False
        self._last_fleet_robot_tasks: list[tuple[str, str]] = []
        self._last_agent_status_log_wall = 0.0
        self._stale_timeout_sec = 90.0
        self._robots_at_charger: set[str] = set()
        self._last_charger_wait_log_wall = 0.0
        self._last_charger_count_logged = -1
        self.create_timer(2.0, self._check_status)

        self.get_logger().info(
            f'MetricLogger started: mode={self._traffic_mode}, '
            f'scenario={self._scenario_id}, run={self._run_id}, '
            f'expected_tasks={self._expected_tasks}'
        )

    def _on_dispatch_states_cb(self, msg):
        """Delegate to TaskTracker — kept as a seam for QoS/trace hooks."""
        self._task_tracker.on_dispatch_states(msg)

    def _on_dkr_event(self, msg):
        """DKR deny/grant/deadlock + delivery lifecycle (dkr/idkr modes)."""
        self._conflict_tracker.on_dkr_event(msg)
        if self._traffic_mode not in ('dkr', 'idkr'):
            return

        try:
            event = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return

        event_type = event.get('type', '')
        now = time.time()

        if event_type == 'task_assigned':
            task_id = event.get('task_id', '')
            robot = event.get('robot', '')
            if task_id and robot:
                self._robots_at_charger.discard(robot)
                self._task_tracker.mark_started_via_fleet(task_id, robot, now)

        elif event_type == 'task_completed':
            task_id = event.get('task_id', '')
            robot = event.get('robot', '')
            if task_id:
                self._task_tracker.mark_completed_via_fleet(task_id, now, robot)

        elif event_type == 'charger_arrival':
            robot = event.get('robot', '')
            if robot:
                self._robots_at_charger.add(robot)
                self.get_logger().info(
                    f'Robot at charger: {robot} '
                    f'({len(self._robots_at_charger)}/{self._expected_robots})'
                )

    def _on_fleet_state(self, msg):
        """Combined fleet_states handler: robot metrics + task completion detection."""
        self._robot_tracker.on_fleet_state(msg)
        self._last_fleet_robot_tasks = [
            (robot.name, (robot.task_id or "").strip()) for robot in msg.robots
        ]
        self._detect_task_transitions(msg)

    def _detect_task_transitions(self, msg):
        """
        Detect task start / completion by monitoring task_id changes in fleet_states.

        Completion: robot.task_id changes to ANY different value (including empty,
                    a charge/park task, or even a new delivery task directly).
        Start:      robot has a non-empty task_id that we haven't seen started yet.
                    We don't rely on MODE_MOVING because the fleet adapter always
                    reports mode=0 (IDLE) in this rmf_demos setup.
        """
        now = time.time()

        if self._traffic_mode in ('dkr', 'idkr'):
            return

        for robot in msg.robots:
            robot_name = robot.name
            task_id = robot.task_id.strip()
            mode = robot.mode.mode

            prev_task_id = self._robot_prev_task_ids.get(robot_name, "")

            # Detect task START: robot has a task we haven't marked started yet.
            if task_id and task_id not in ("0", ""):
                self._task_tracker.mark_started_via_fleet(task_id, robot_name, now)

            # Detect task COMPLETION: ANY change in task_id means the old task ended.
            # This covers: task_id → "", task_id → charger-task, task_id → new-delivery.
            if prev_task_id and prev_task_id not in ("0", "") and task_id != prev_task_id:
                # #region agent log
                _agent_debug_ndjson({
                    "hypothesisId": "H5",
                    "location": "metric_logger_node.py:_detect_task_transitions",
                    "message": "fleet_task_id_transition",
                    "data": {
                        "robot": robot_name,
                        "prev_task_id": prev_task_id[:80],
                        "new_task_id": task_id[:80] if task_id else "",
                    },
                    "runId": "repro",
                })
                # #endregion
                self._task_tracker.mark_completed_via_fleet(prev_task_id, now, robot_name)

            self._robot_prev_task_ids[robot_name] = task_id
            self._robot_prev_modes[robot_name] = mode

    def _check_status(self):
        if self._finished:
            return

        completed = self._task_tracker.completed_count

        # #region agent log
        now_wall = time.time()
        if now_wall - self._last_agent_status_log_wall >= 12.0:
            self._last_agent_status_log_wall = now_wall
            status_buckets: dict[str, int] = {}
            for t in self._task_tracker.tasks.values():
                status_buckets[t.status] = status_buckets.get(t.status, 0) + 1
            _agent_debug_ndjson({
                "hypothesisId": "H1",
                "location": "metric_logger_node.py:_check_status",
                "message": "progress_snapshot",
                "data": {
                    "completed": completed,
                    "expected_tasks_param": self._expected_tasks,
                    "status_buckets": status_buckets,
                    "tasks_tracked_total": len(self._task_tracker.tasks),
                    "fleet_robot_task_ids": [
                        {"robot": n, "task_id": tid[:72] if tid else ""}
                        for n, tid in self._last_fleet_robot_tasks
                    ],
                },
                "runId": "repro",
            })
        # #endregion

        # Only update the completion timestamp when a NEW task finishes.
        if completed > self._last_known_completed_count:
            self._last_known_completed_count = completed
            self._last_task_completion_time = time.time()
            pending_dispatch = sum(
                1 for t in self._task_tracker.tasks.values()
                if t.task_id.startswith("delivery.dispatch-") and t.status in (
                    "queued", "dispatched", "unknown"
                )
            )
            self.get_logger().info(
                f'Progress: {completed} task(s) completed'
                + (f' / {self._expected_tasks} expected' if self._expected_tasks > 0 else '')
                + (f' ({pending_dispatch} delivery dispatch backlog in tracker)'
                   if pending_dispatch > 0 and self._expected_tasks > 0 else '')
            )

        # Finish condition 1: all expected tasks completed.
        all_done = (
            self._expected_tasks > 0 and self._task_tracker.all_tasks_completed
        )

        if all_done and self._traffic_mode in ('dkr', 'idkr'):
            at_charger = len(self._robots_at_charger)
            if at_charger >= self._expected_robots:
                self.get_logger().info(
                    f'All {self._expected_tasks} tasks completed and '
                    f'all {self._expected_robots} robots at charger. '
                    f'Saving results...'
                )
                self._finish_experiment()
                return

            now_wall = time.time()
            if (
                at_charger != self._last_charger_count_logged
                or now_wall - self._last_charger_wait_log_wall >= 12.0
            ):
                self._last_charger_count_logged = at_charger
                self._last_charger_wait_log_wall = now_wall
                self.get_logger().info(
                    f'All tasks completed — waiting for robots at charger '
                    f'({at_charger}/{self._expected_robots})...'
                )
            return

        if all_done:
            self.get_logger().info('All expected tasks completed. Saving results...')
            self._finish_experiment()
            return

        # Finish condition 2: stale-completion timeout.
        # When expected_tasks > 0 and no new completions arrive for
        # _stale_timeout_sec, all robots are likely idle and RMF won't
        # assign the remaining tasks — save partial results instead of
        # hanging forever.
        if self._last_task_completion_time > 0:
            idle_duration = time.time() - self._last_task_completion_time
            timeout = (
                self._stale_timeout_sec
                if self._expected_tasks > 0
                else self._auto_finish_timeout
            )
            if idle_duration > timeout:
                remaining = self._expected_tasks - completed
                if self._expected_tasks > 0 and remaining > 0:
                    self.get_logger().warn(
                        f'No new completions for {idle_duration:.0f}s — '
                        f'{remaining} task(s) were never assigned by RMF. '
                        f'Saving partial results ({completed}/{self._expected_tasks})...'
                    )
                else:
                    self.get_logger().info(
                        f'No new task completions for {idle_duration:.0f}s. Finishing...'
                    )
                self._finish_experiment()
                return

    def _finish_experiment(self):
        self._finished = True

        task_metrics = self._task_tracker.get_metrics()
        robot_metrics = self._robot_tracker.get_metrics()
        conflict_metrics = self._conflict_tracker.get_metrics()
        energy_metrics = self._energy_estimator.estimate(robot_metrics.get("per_robot", {}))

        results = {
            "metadata": {
                "traffic_mode": self._traffic_mode,
                "scenario_id": self._scenario_id,
                "run_id": self._run_id,
                "experiment_timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
                "total_elapsed_sec": round(time.time() - self._experiment_start, 2),
            },
            "task_metrics": task_metrics,
            "robot_metrics": robot_metrics,
            "conflict_metrics": conflict_metrics,
            "energy_metrics": energy_metrics,
            "summary": {
                "throughput_per_min": task_metrics["throughput_per_min"],
                "avg_completion_time_sec": task_metrics["avg_completion_time_sec"],
                "wait_time_variance": robot_metrics["wait_time_variance"],
                "deadlock_count": conflict_metrics["deadlock_count"],
                "conflict_count": conflict_metrics["total_conflicts"],
                "total_energy_wh": energy_metrics["total_energy_wh"],
            },
        }

        output_file = self._output_dir / f"run_{self._run_id:03d}.json"
        # open() does not create parent dirs; recreate if results/ was removed mid-run.
        output_file.parent.mkdir(parents=True, exist_ok=True)
        with open(output_file, 'w') as f:
            json.dump(results, f, indent=2, ensure_ascii=False)

        self.get_logger().info(f'Results saved to: {output_file}')
        self._print_summary(results["summary"])

        # Stop rclpy spin so main() can destroy_node(); launch may hook process exit
        # to shut down sibling nodes (see metric_logger.launch.py).
        rclpy.shutdown()

    def _print_summary(self, summary: dict):
        self.get_logger().info('=' * 50)
        self.get_logger().info('EXPERIMENT RESULTS SUMMARY')
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'  Throughput:        {summary["throughput_per_min"]:.2f} tasks/min')
        self.get_logger().info(f'  Avg completion:    {summary["avg_completion_time_sec"]:.2f} sec')
        self.get_logger().info(f'  Wait variance:     {summary["wait_time_variance"]:.4f}')
        self.get_logger().info(f'  Deadlocks:         {summary["deadlock_count"]}')
        self.get_logger().info(f'  Conflicts:         {summary["conflict_count"]}')
        self.get_logger().info(f'  Energy:            {summary["total_energy_wh"]:.4f} Wh')
        self.get_logger().info('=' * 50)


def main(args=None):
    rclpy.init(args=args)
    node = MetricLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
