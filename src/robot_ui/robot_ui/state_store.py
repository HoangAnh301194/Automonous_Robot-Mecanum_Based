from __future__ import annotations

import time
from copy import deepcopy
from threading import RLock
from typing import Any

from robot_ui.contracts import initial_state, websocket_envelope


class StateStore:
    def __init__(self, config_source: str):
        self._lock = RLock()
        self._sequence = 0
        self._state = initial_state(config_source)

    def patch_section(self, section: str, values: dict[str, Any]) -> None:
        with self._lock:
            current = self._state.setdefault(section, {})
            current.update(values)
            current["updated_at"] = time.time()
            self._sequence += 1

    def ensure_topic(
        self,
        topic: str,
        expected_rate_hz: float | None = None,
        health_monitored: bool = False,
    ) -> None:
        with self._lock:
            current = self._state["topics"].setdefault(topic, {})
            current.setdefault("message_count", 0)
            current.setdefault("state", "WAITING")
            current["health_monitored"] = health_monitored
            if expected_rate_hz is not None:
                current["expected_rate_hz"] = expected_rate_hz
            self._sequence += 1

    def update_topic(
        self,
        topic: str,
        values: dict[str, Any],
        expected_rate_hz: float | None = None,
    ) -> None:
        with self._lock:
            now = time.time()
            current = self._state["topics"].setdefault(topic, {})
            previous_state = current.get("state")
            previous_message_at = current.get("last_message_at")
            previous_rate = current.get("rate_hz")
            if previous_message_at is not None and now > previous_message_at:
                instant_rate = 1.0 / (now - previous_message_at)
                current["rate_hz"] = (
                    instant_rate
                    if previous_rate is None
                    else previous_rate * 0.8 + instant_rate * 0.2
                )
            current.update(values)
            current["message_count"] = current.get("message_count", 0) + 1
            current["last_message_at"] = now
            current["age_seconds"] = 0.0
            current["state"] = "OK"
            if previous_state != "OK":
                current["health_state_changed_at"] = now
            if expected_rate_hz is not None:
                current["expected_rate_hz"] = expected_rate_hz
            if previous_state in ("WARN", "STALE"):
                self._append_event_unlocked(
                    "INFO",
                    "topic_health",
                    f"{topic} recovered from {previous_state}",
                )
            self._sequence += 1

    def refresh_topic_health(
        self,
        stale_multiplier: float,
        minimum_stale_seconds: float,
        warn_rate_ratio: float,
    ) -> list[dict[str, str]]:
        transitions: list[dict[str, str]] = []
        with self._lock:
            now = time.time()
            for topic, current in self._state["topics"].items():
                if not current.get("health_monitored", False):
                    continue
                previous_state = str(current.get("state", "WAITING"))
                last_message_at = current.get("last_message_at")
                expected_rate_hz = current.get("expected_rate_hz")
                if last_message_at is None:
                    current["age_seconds"] = None
                    current["stale_after_seconds"] = max(
                        minimum_stale_seconds,
                        stale_multiplier / expected_rate_hz,
                    ) if expected_rate_hz else minimum_stale_seconds
                    new_state = "WAITING"
                else:
                    age_seconds = max(0.0, now - float(last_message_at))
                    stale_after_seconds = max(
                        minimum_stale_seconds,
                        stale_multiplier / expected_rate_hz,
                    ) if expected_rate_hz else minimum_stale_seconds
                    current["age_seconds"] = age_seconds
                    current["stale_after_seconds"] = stale_after_seconds
                    rate_hz = current.get("rate_hz")
                    if age_seconds > stale_after_seconds:
                        new_state = "STALE"
                    elif (
                        expected_rate_hz
                        and rate_hz is not None
                        and current.get("message_count", 0) > 2
                        and rate_hz < expected_rate_hz * warn_rate_ratio
                    ):
                        new_state = "WARN"
                    else:
                        new_state = "OK"
                current["state"] = new_state
                if new_state != previous_state:
                    current["health_state_changed_at"] = now
                    transitions.append(
                        {
                            "topic": topic,
                            "previous": previous_state,
                            "state": new_state,
                        }
                    )
            self._sequence += 1
        return transitions

    def append_ros_log(self, entry: dict[str, Any], max_entries: int = 500) -> None:
        with self._lock:
            self._state["ros_logs"].append(entry)
            self._state["ros_logs"] = self._state["ros_logs"][-max_entries:]
            self._sequence += 1

    def _append_event_unlocked(self, level: str, source: str, message: str) -> None:
        self._state["events"].append(
            {
                "timestamp": time.time(),
                "level": level,
                "source": source,
                "message": message,
            }
        )
        self._state["events"] = self._state["events"][-200:]

    def append_event(self, level: str, source: str, message: str) -> None:
        with self._lock:
            self._append_event_unlocked(level, source, message)
            self._sequence += 1

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return deepcopy(self._state)

    def envelope(self, message_type: str = "state.snapshot") -> dict[str, Any]:
        with self._lock:
            return websocket_envelope(
                message_type=message_type,
                sequence=self._sequence,
                timestamp=time.time(),
                data=deepcopy(self._state),
            )
