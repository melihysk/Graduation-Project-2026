"""
Çatışma ve deadlock takibi — 3 modda ortak metrikler.

Mode 1 (RMF baseline):
  - /rmf_traffic/negotiation_notice → çatışma tespit edildi
  - /rmf_traffic/negotiation_conclusion → çözüldü/çözülemedi

Mode 2-3 (DKR/İDKR):
  - /dkr_events topic'inden JSON event'ler okunur
  - deny → benzersiz bekleme episode'ları (total_conflicts)
  - grant → episode çözümü (resolved_conflicts, resolution_time)
  - deadlock → deadlock_count

Ortak çıktı: total_conflicts, resolved/unresolved, deadlock_count,
avg/max_resolution_time_sec.
"""

import json
import time
from dataclasses import dataclass


@dataclass
class ConflictEvent:
    conflict_version: int
    participants: list
    start_time: float
    end_time: float = 0.0
    resolved: bool = False


class ConflictTracker:

    def __init__(self, logger):
        self._logger = logger
        self._conflicts: dict[int, ConflictEvent] = {}
        self._deadlock_count: int = 0
        self._deadlock_threshold_sec: float = 30.0

        # DKR/İDKR event tracking (feeds total_conflicts, resolved, deadlock)
        self._dkr_active_waits: dict[str, tuple[str, float]] = {}
        self._dkr_unique_conflict_count: int = 0
        self._dkr_resolved_conflict_count: int = 0
        self._dkr_resolution_times: list[float] = []

    @property
    def conflict_count(self) -> int:
        return len(self._conflicts)

    @property
    def deadlock_count(self) -> int:
        return self._deadlock_count

    def on_negotiation_notice(self, msg):
        """rmf_traffic_msgs/msg/NegotiationNotice callback."""
        cv = msg.conflict_version
        if cv not in self._conflicts:
            self._conflicts[cv] = ConflictEvent(
                conflict_version=cv,
                participants=list(msg.participants),
                start_time=time.time(),
            )
            self._logger.info(
                f"Conflict #{cv} detected — participants: {list(msg.participants)}"
            )

    def on_negotiation_conclusion(self, msg):
        """rmf_traffic_msgs/msg/NegotiationConclusion callback."""
        cv = msg.conflict_version
        now = time.time()

        if cv in self._conflicts:
            event = self._conflicts[cv]
            event.end_time = now
            event.resolved = msg.resolved

            duration = now - event.start_time
            if not msg.resolved:
                self._logger.warn(
                    f"Conflict #{cv} UNRESOLVED after {duration:.1f}s"
                )
            else:
                self._logger.info(
                    f"Conflict #{cv} resolved in {duration:.1f}s"
                )
        else:
            self._conflicts[cv] = ConflictEvent(
                conflict_version=cv,
                participants=[],
                start_time=now,
                end_time=now,
                resolved=msg.resolved,
            )

    def on_negotiation_statuses(self, msg):
        """rmf_traffic_msgs/msg/NegotiationStatuses — check for long-running negotiations (deadlock)."""
        now = time.time()
        for status in msg.negotiations:
            cv = status.conflict_version
            if cv in self._conflicts:
                event = self._conflicts[cv]
                if not event.end_time:
                    duration = now - event.start_time
                    if duration > self._deadlock_threshold_sec:
                        if not hasattr(event, '_deadlock_flagged'):
                            event._deadlock_flagged = True
                            self._deadlock_count += 1
                            self._logger.warn(
                                f"DEADLOCK detected: conflict #{cv} "
                                f"unresolved for {duration:.1f}s"
                            )

    # ------------------------------------------------------------------
    # DKR/İDKR event handling
    # ------------------------------------------------------------------

    def _dkr_blocker_key(self, event: dict) -> str:
        blocker = event.get("blocking_robot")
        if blocker:
            return str(blocker)
        reason = event.get("reason")
        if reason:
            return str(reason)
        return "unknown"

    def _begin_dkr_wait_episode(self, robot: str, blocker: str, now: float) -> None:
        active = self._dkr_active_waits.get(robot)
        if active is not None and active[0] == blocker:
            return

        self._dkr_unique_conflict_count += 1
        self._dkr_active_waits[robot] = (blocker, now)

    def _resolve_dkr_wait_episode(self, robot: str, now: float) -> None:
        active = self._dkr_active_waits.pop(robot, None)
        if active is None:
            return

        _, start_time = active
        duration = now - start_time
        if duration > 0:
            self._dkr_resolution_times.append(duration)
        self._dkr_resolved_conflict_count += 1

    def on_dkr_event(self, msg):
        """std_msgs/String callback for /dkr_events topic (JSON payload)."""
        try:
            event = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return

        event_type = event.get("type", "")
        now = time.time()

        if event_type == "deny":
            robot = event.get("robot", "")
            if robot:
                self._begin_dkr_wait_episode(robot, self._dkr_blocker_key(event), now)
            self._logger.debug(
                f"DKR deny: {event.get('robot')} blocked by {event.get('blocking_robot')}"
            )

        elif event_type == "grant":
            robot = event.get("robot", "")
            if robot:
                self._resolve_dkr_wait_episode(robot, now)

        elif event_type == "deadlock":
            self._deadlock_count += 1
            cycle = event.get("cycle", [])
            victim = event.get("victim", "")
            if victim:
                self._resolve_dkr_wait_episode(victim, now)
            self._logger.warn(
                f"DKR DEADLOCK: cycle={cycle}, victim={victim}"
            )

    def get_metrics(self) -> dict:
        resolved_conflicts = [c for c in self._conflicts.values() if c.resolved]
        unresolved_conflicts = [
            c for c in self._conflicts.values()
            if not c.resolved and c.end_time > 0
        ]

        resolution_times = []
        for c in resolved_conflicts:
            if c.end_time > c.start_time:
                resolution_times.append(c.end_time - c.start_time)
        resolution_times.extend(self._dkr_resolution_times)

        dkr_unresolved = len(self._dkr_active_waits)
        total_conflicts = len(self._conflicts) + self._dkr_unique_conflict_count
        resolved_total = len(resolved_conflicts) + self._dkr_resolved_conflict_count
        unresolved_total = len(unresolved_conflicts) + dkr_unresolved

        return {
            "total_conflicts": total_conflicts,
            "resolved_conflicts": resolved_total,
            "unresolved_conflicts": unresolved_total,
            "deadlock_count": self._deadlock_count,
            "avg_resolution_time_sec": round(
                sum(resolution_times) / len(resolution_times), 3
            ) if resolution_times else 0.0,
            "max_resolution_time_sec": round(
                max(resolution_times), 3
            ) if resolution_times else 0.0,
        }
