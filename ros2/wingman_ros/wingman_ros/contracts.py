"""ROS-independent state normalization used by the ROS bridge and tests."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Any, Optional


def _finite(value: Any) -> Optional[float]:
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


@dataclass
class StateStore:
    stale_after_s: float = 2.0
    map_data: Optional[dict] = None
    telemetry: dict = field(default_factory=dict)
    rgbd: dict = field(default_factory=dict)
    updated_at: dict[str, float] = field(default_factory=dict)
    version: int = 0

    def update(self, channel: str, value: dict, now: Optional[float] = None):
        timestamp = time.time() if now is None else float(now)
        if not math.isfinite(timestamp):
            raise ValueError("timestamp must be finite")
        if channel == "map":
            self.map_data = value
        elif channel == "telemetry":
            self.telemetry = value
        elif channel == "rgbd":
            self.rgbd = value
        else:
            raise ValueError(f"unknown state channel: {channel}")
        self.updated_at[channel] = timestamp
        self.version += 1

    def health(self, now: Optional[float] = None) -> dict:
        timestamp = time.time() if now is None else float(now)
        ages = {
            key: round(max(0.0, timestamp - value), 3)
            for key, value in self.updated_at.items()
        }
        stale = [
            key for key in ("map", "telemetry", "rgbd")
            if key not in self.updated_at
            or ages[key] > self.stale_after_s
        ]
        return {
            "schema": "wingman.health.v1",
            "timestamp": timestamp,
            "stale_channels": stale,
            "ages_s": ages,
            "version": self.version,
        }

    def dashboard_state(self, now: Optional[float] = None) -> dict:
        timestamp = time.time() if now is None else float(now)
        return {
            "schema": "wingman.dashboard.v1",
            "timestamp": timestamp,
            "map": self.map_data,
            "telemetry": self.telemetry,
            "rgbd": self.rgbd,
            "health": self.health(timestamp),
        }


def vector3(value: Any) -> dict:
    return {
        "x": _finite(getattr(value, "x", None)),
        "y": _finite(getattr(value, "y", None)),
        "z": _finite(getattr(value, "z", None)),
    }
