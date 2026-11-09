from __future__ import annotations

from typing import Any


SCHEMA_VERSION = "0.1.0"


def initial_state(config_source: str) -> dict[str, Any]:
    return {
        "schema_version": SCHEMA_VERSION,
        "robot": {
            "state": "STARTING",
            "ros_connected": False,
            "config_source": config_source,
        },
        "system": {},
        "navigation": {},
        "hardware": {},
        "vision": {},
        "mission": {},
        "diagnostics": {
            "summary": {},
            "statuses": [],
        },
        "ros_logs": [],
        "ros_graph": {
            "nodes": [],
            "edges": [],
            "summary": {},
        },
        "topics": {},
        "events": [],
    }


def websocket_envelope(
    message_type: str,
    sequence: int,
    timestamp: float,
    data: dict[str, Any],
) -> dict[str, Any]:
    return {
        "schema_version": SCHEMA_VERSION,
        "type": message_type,
        "sequence": sequence,
        "timestamp": timestamp,
        "data": data,
    }
