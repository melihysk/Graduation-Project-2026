"""
Task lifecycle tracking.

Completion detection merges several sources into ``_complete_delivery_tracking``:

1. fleet_states  (PRIMARY — task_id clears / switches on a robot row)
   Tracked in metric_logger_node._detect_task_transitions().

2. task_summaries / task_api_response  (QoS-available backups)
   State COMPLETED or JSON status ``completed``/``finished`` when published.

DispatchStates only updates assignment / bookkeeping; FAILED_TO_ASSIGN /
CANCELED_IN_FLIGHT are recorded so ``expected_tasks`` can finish without
fabricating fleet completions.
"""

import json
import time
from dataclasses import dataclass


@dataclass
class TaskRecord:
    task_id: str
    dispatch_time: float = 0.0
    start_time: float = 0.0
    end_time: float = 0.0
    status: str = "unknown"
    assigned_robot: str = ""


class TaskTracker:

    def __init__(self, logger, agent_debug=None):
        self._logger = logger
        self._agent_debug = agent_debug  # optional callable(payload: dict) -> None
        self._tasks: dict[str, TaskRecord] = {}
        self._completed_task_ids: set[str] = set()   # append-only — never cleared
        self._expected_task_count: int = 0

    @property
    def tasks(self) -> dict[str, TaskRecord]:
        return self._tasks

    @property
    def all_tasks_completed(self) -> bool:
        if self._expected_task_count == 0:
            return False
        done = len(self._completed_task_ids) + sum(
            1 for t in self._tasks.values()
            if t.status in ("failed", "canceled")
            and t.task_id not in self._completed_task_ids
        )
        return done >= self._expected_task_count

    @property
    def completed_count(self) -> int:
        return len(self._completed_task_ids)

    def set_expected_task_count(self, count: int):
        self._expected_task_count = count

    def on_task_api_response(self, msg):
        """rmf_task_msgs/msg/ApiResponse callback."""
        if msg.type not in (1, 2):  # TYPE_ACKNOWLEDGE | TYPE_RESPONDING
            return

        raw = msg.json_msg
        if isinstance(raw, str):
            stripped = raw.strip()
        elif raw:
            stripped = str(raw).strip()
        else:
            stripped = ""
        if not stripped:
            return

        try:
            data = json.loads(stripped)
        except (json.JSONDecodeError, TypeError):
            return

        self._process_task_state(data)

    # TaskSummary state constants
    _STATE_QUEUED = 0
    _STATE_ACTIVE = 1
    _STATE_COMPLETED = 2
    _STATE_FAILED = 3
    _STATE_CANCELED = 4
    _STATE_PENDING = 5

    # DispatchState.status (rmf_task_msgs/msg/DispatchState.msg)
    _DISPATCH_STATUS_FAILED_TO_ASSIGN = 4
    _DISPATCH_STATUS_CANCELED_IN_FLIGHT = 5

    # ------------------------------------------------------------------
    # fleet_states-based completion detection  (PRIMARY method)
    # ------------------------------------------------------------------

    def mark_started_via_fleet(self, task_id: str, robot_name: str, timestamp: float):
        """Called when a robot first begins moving with a given task_id."""
        if task_id not in self._tasks:
            self._tasks[task_id] = TaskRecord(task_id=task_id, dispatch_time=timestamp)
        record = self._tasks[task_id]
        if not record.assigned_robot:
            record.assigned_robot = robot_name
        if not record.start_time:
            record.start_time = timestamp
            self._logger.info(
                f"[task_tracker] {task_id[:32]} STARTED on {robot_name}"
            )
            # #region agent log
            if self._agent_debug:
                self._agent_debug({
                    "hypothesisId": "H3",
                    "location": "task_tracker.py:mark_started_via_fleet",
                    "message": "fleet_task_started",
                    "data": {
                        "task_id": task_id[:64],
                        "robot": robot_name,
                        "tracked_tasks": len(self._tasks),
                    },
                })
            # #endregion
        if record.status not in ("executing", "completed"):
            record.status = "executing"

    def _complete_delivery_tracking(
        self,
        task_id: str,
        timestamp: float,
        *,
        source: str,
        robot_hint: str = "",
        start_override: float | None = None,
        end_override: float | None = None,
    ) -> None:
        """
        Single path for marking a tracked delivery dispatch task as completed.

        fleet_states and task_api/state updates both funnel here when they
        observe a confirmed completion.
        """
        if task_id not in self._tasks:
            self._tasks[task_id] = TaskRecord(task_id=task_id)
        record = self._tasks[task_id]
        if record.status == "completed":
            return
        # Fleet-side truth overrides stale API assignment guesses.
        if robot_hint:
            record.assigned_robot = robot_hint
        if start_override is not None and start_override > 0 and not record.start_time:
            record.start_time = start_override
        if end_override is not None and end_override > 0 and not record.end_time:
            record.end_time = end_override
        if not record.end_time:
            record.end_time = timestamp
        if not record.start_time:
            record.start_time = record.dispatch_time or timestamp
        duration = record.end_time - record.start_time
        self._logger.info(
            f"[task_tracker] {task_id[:32]} COMPLETED {source}"
            f" ({duration:.1f}s, robot={record.assigned_robot})"
        )
        record.status = "completed"
        self._completed_task_ids.add(task_id)
        # #region agent log
        if self._agent_debug:
            self._agent_debug({
                "hypothesisId": "H4",
                "location": "task_tracker.py:_complete_delivery_tracking",
                "message": "task_completed",
                "data": {
                    "task_id": task_id[:64],
                    "completed_total": len(self._completed_task_ids),
                    "robot": record.assigned_robot,
                    "via": source,
                },
            })
        # #endregion

    def mark_completed_via_fleet(
        self, task_id: str, timestamp: float, clearing_robot: str = ""
    ):
        """Called when a robot's fleet task_id clears or changes."""
        self._complete_delivery_tracking(
            task_id, timestamp,
            source="via fleet_states",
            robot_hint=clearing_robot,
        )

    # ------------------------------------------------------------------
    # task_summaries backup
    # ------------------------------------------------------------------

    def on_task_summary(self, msg):
        """
        rmf_task_msgs/msg/TaskSummary callback — PRIMARY completion source.

        The fleet adapter publishes here for every state transition:
        QUEUED → ACTIVE → COMPLETED / FAILED / CANCELED.
        start_time and end_time fields are wall-clock seconds from epoch.
        """
        now = time.time()

        # task_id lives in the outer field AND inside task_profile.task_id
        task_id = msg.task_id or msg.task_profile.task_id
        if not task_id:
            return

        if task_id not in self._tasks:
            self._tasks[task_id] = TaskRecord(task_id=task_id, dispatch_time=now)

        record = self._tasks[task_id]

        if msg.robot_name and not record.assigned_robot:
            record.assigned_robot = msg.robot_name

        state = msg.state  # uint32

        if state == self._STATE_QUEUED or state == self._STATE_PENDING:
            if not record.dispatch_time:
                record.dispatch_time = now
            record.status = "queued"

        elif state == self._STATE_ACTIVE:
            if not record.start_time:
                # Prefer the message's start_time if non-zero
                t = msg.start_time
                record.start_time = t.sec + t.nanosec * 1e-9 if t.sec > 0 else now
                self._logger.info(
                    f"[task_tracker] {task_id[:30]} ACTIVE on {record.assigned_robot}"
                )
            record.status = "executing"

        elif state == self._STATE_COMPLETED:
            ts = msg.start_time
            ss = ts.sec + ts.nanosec * 1e-9 if ts.sec > 0 else 0.0
            te_msg = msg.end_time
            es = te_msg.sec + te_msg.nanosec * 1e-9 if te_msg.sec > 0 else 0.0
            self._complete_delivery_tracking(
                task_id,
                now,
                source="via task_summary",
                robot_hint=msg.robot_name or record.assigned_robot or "",
                start_override=ss if ss > 0 else None,
                end_override=es if es > 0 else None,
            )

        elif state == self._STATE_FAILED:
            if not record.end_time:
                record.end_time = now
            record.status = "failed"
            self._logger.warn(f"[task_tracker] {task_id[:30]} FAILED")

        elif state == self._STATE_CANCELED:
            if not record.end_time:
                record.end_time = now
            record.status = "canceled"

    def on_dispatch_states(self, msg):
        """rmf_task_msgs/msg/DispatchStates callback."""
        now = time.time()

        for ds in msg.active:
            task_id = ds.task_id
            if task_id not in self._tasks:
                self._tasks[task_id] = TaskRecord(task_id=task_id, dispatch_time=now)
            record = self._tasks[task_id]
            # Only set assigned_robot + initial status; never override
            # "executing" or "completed" that fleet_states detection already set.
            if ds.assignment.is_assigned and not record.assigned_robot:
                record.assigned_robot = ds.assignment.expected_robot_name
            if record.status not in ("executing", "completed", "failed", "canceled"):
                record.status = "dispatched"

        for ds in msg.finished:
            task_id = ds.task_id
            if task_id not in self._tasks:
                self._tasks[task_id] = TaskRecord(task_id=task_id)
            record = self._tasks[task_id]
            # DispatchState.status enums from rmf_task_msgs/msg/DispatchState.msg
            if ds.status == self._DISPATCH_STATUS_FAILED_TO_ASSIGN:
                if task_id not in self._completed_task_ids:
                    record.status = "failed"
                    if not record.end_time:
                        record.end_time = now
                continue
            if ds.status == self._DISPATCH_STATUS_CANCELED_IN_FLIGHT:
                if task_id not in self._completed_task_ids:
                    record.status = "canceled"
                    if not record.end_time:
                        record.end_time = now
                continue
            # Do NOT set "dispatched" from finished list — it overwrites "completed".

    def _process_task_state(self, data: dict):
        """Parse task state JSON from API response."""
        if isinstance(data, list):
            for item in data:
                self._process_single_task(item)
        elif isinstance(data, dict):
            self._process_single_task(data)

    def _process_single_task(self, task_data: dict):
        now = time.time()

        # RMF wraps the task state under a "state" key.
        # {"state": {"booking": {"id": "..."}, "status": "...", "assigned_to": {...}}}
        state = task_data.get("state", task_data)

        task_id = state.get("booking", {}).get("id", "")
        if not task_id:
            task_id = state.get("task_id", "")
        if not task_id:
            return

        is_new = task_id not in self._tasks
        if is_new:
            self._tasks[task_id] = TaskRecord(task_id=task_id, dispatch_time=now)

        record = self._tasks[task_id]

        status = state.get("status", "")
        if isinstance(status, dict):
            status = status.get("status", "")

        # Log every unique (task_id, status) transition for debugging.
        if is_new or status != record.status:
            self._logger.info(f"[task_tracker] {task_id[:30]} → status='{status}'")
            # #region agent log
            if self._agent_debug and is_new:
                self._agent_debug({
                    "hypothesisId": "H2",
                    "location": "task_tracker.py:_process_single_task",
                    "message": "new_task_from_api_response",
                    "data": {
                        "task_id": task_id[:64],
                        "status": str(status),
                        "tasks_dict_size_after": len(self._tasks),
                    },
                })
            # #endregion

        # assigned_to: {"group": "fleet", "name": "robotX"} in RMF API v2
        assigned = state.get("assigned_to", {})
        if isinstance(assigned, dict):
            robot_name = assigned.get("name", "")
            if robot_name:
                record.assigned_robot = robot_name

        if status in ("queued", "standby", "pending", "selected"):
            if not record.dispatch_time:
                record.dispatch_time = now
            record.status = "queued"
        elif status in ("executing", "underway", "active", "delayed", "blocked"):
            if not record.start_time:
                record.start_time = now
                self._logger.info(f"Task {task_id[:30]} started on {record.assigned_robot}")
            record.status = "executing"
        elif status in ("completed", "finished"):
            self._complete_delivery_tracking(
                task_id,
                now,
                source="via task_api_response",
                robot_hint=record.assigned_robot,
            )
        elif status in ("failed", "error"):
            if not record.end_time:
                record.end_time = now
            record.status = "failed"
        elif status in ("canceled", "killed", "skipped"):
            if not record.end_time:
                record.end_time = now
            record.status = "canceled"

    def get_metrics(self) -> dict:
        completed_tasks = [
            self._tasks[tid] for tid in self._completed_task_ids
            if tid in self._tasks
        ]
        if not completed_tasks:
            return {
                "tasks_completed": 0,
                "tasks_failed": sum(1 for t in self._tasks.values() if t.status == "failed"),
                "throughput_per_min": 0.0,
                "avg_completion_time_sec": 0.0,
                "min_completion_time_sec": 0.0,
                "max_completion_time_sec": 0.0,
            }

        completion_times = []
        for t in completed_tasks:
            if t.start_time and t.end_time:
                completion_times.append(t.end_time - t.start_time)
            elif t.dispatch_time and t.end_time:
                completion_times.append(t.end_time - t.dispatch_time)

        all_tasks_with_time = [t for t in self._tasks.values() if t.dispatch_time > 0]
        if all_tasks_with_time:
            first_dispatch = min(t.dispatch_time for t in all_tasks_with_time)
            last_end = max(
                (t.end_time for t in completed_tasks if t.end_time > 0),
                default=time.time()
            )
            total_duration_min = (last_end - first_dispatch) / 60.0
            throughput = len(completed_tasks) / total_duration_min if total_duration_min > 0 else 0.0
        else:
            throughput = 0.0
            total_duration_min = 0.0

        return {
            "tasks_completed": len(completed_tasks),
            "tasks_failed": sum(1 for t in self._tasks.values() if t.status == "failed"),
            "throughput_per_min": round(throughput, 3),
            "avg_completion_time_sec": round(
                sum(completion_times) / len(completion_times), 2
            ) if completion_times else 0.0,
            "min_completion_time_sec": round(min(completion_times), 2) if completion_times else 0.0,
            "max_completion_time_sec": round(max(completion_times), 2) if completion_times else 0.0,
            "total_experiment_duration_sec": round(total_duration_min * 60, 2),
        }
