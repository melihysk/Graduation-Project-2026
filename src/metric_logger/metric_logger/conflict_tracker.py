"""
Çatışma ve deadlock takibi.

Mode 1 (RMF baseline):
  - /rmf_traffic/negotiation_notice → çatışma tespit edildi
  - /rmf_traffic/negotiation_conclusion → çözüldü/çözülemedi
  - /rmf_traffic/blockade_set → dar geçitte blokaj

Mode 2-3 (DKR/İDKR):
  - Custom /traffic_events topic'inden okunacak (Faz 3-4'te implemente edilecek)
"""

import time
from dataclasses import dataclass, field


@dataclass
class ConflictEvent:
    conflict_version: int
    participants: list
    start_time: float
    end_time: float = 0.0
    resolved: bool = False


@dataclass
class BlockadeEvent:
    participant: int
    reservation: int
    start_time: float
    end_time: float = 0.0
    checkpoint_count: int = 0


class ConflictTracker:

    def __init__(self, logger):
        self._logger = logger
        self._conflicts: dict[int, ConflictEvent] = {}
        self._blockades: dict[tuple, BlockadeEvent] = {}
        self._deadlock_count: int = 0
        self._deadlock_threshold_sec: float = 30.0

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

    def on_blockade_set(self, msg):
        """rmf_traffic_msgs/msg/BlockadeSet callback."""
        key = (msg.participant, msg.reservation)
        if key not in self._blockades:
            self._blockades[key] = BlockadeEvent(
                participant=msg.participant,
                reservation=msg.reservation,
                start_time=time.time(),
                checkpoint_count=len(msg.path),
            )

    def on_blockade_release(self, msg):
        """rmf_traffic_msgs/msg/BlockadeRelease callback."""
        key = (msg.participant, msg.reservation)
        if key in self._blockades:
            self._blockades[key].end_time = time.time()

    def get_metrics(self) -> dict:
        resolved_conflicts = [c for c in self._conflicts.values() if c.resolved]
        unresolved_conflicts = [c for c in self._conflicts.values() if not c.resolved and c.end_time > 0]

        resolution_times = []
        for c in resolved_conflicts:
            if c.end_time > c.start_time:
                resolution_times.append(c.end_time - c.start_time)

        return {
            "total_conflicts": len(self._conflicts),
            "resolved_conflicts": len(resolved_conflicts),
            "unresolved_conflicts": len(unresolved_conflicts),
            "deadlock_count": self._deadlock_count,
            "avg_resolution_time_sec": round(
                sum(resolution_times) / len(resolution_times), 3
            ) if resolution_times else 0.0,
            "max_resolution_time_sec": round(
                max(resolution_times), 3
            ) if resolution_times else 0.0,
            "blockade_count": len(self._blockades),
        }
