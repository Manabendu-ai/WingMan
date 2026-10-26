"""Shared live state: log lines, latest telemetry, latest VLM observation."""
import collections
import threading
import time


class Hub:
    def __init__(self, log_size: int = 300):
        self._lock = threading.RLock()
        self._log = collections.deque(maxlen=log_size)
        self.telemetry = {}
        self.last_obs = None       # {scene, target_visible, x, y, confidence}
        self.frame_path = None

    def say(self, msg: str) -> None:
        with self._lock:
            self._log.append({"t": time.strftime("%H:%M:%S"), "msg": str(msg).strip()})

    def reset_run(self) -> None:
        with self._lock:
            self.telemetry, self.last_obs, self.frame_path = {}, None, None

    def set_telemetry(self, t: dict) -> None:
        with self._lock:
            self.telemetry = t

    def set_frame(self, path, obs) -> None:
        with self._lock:
            self.frame_path, self.last_obs = path, obs

    def view(self, log_n: int = 100) -> dict:
        with self._lock:
            return {"telemetry": dict(self.telemetry), "last_obs": self.last_obs,
                    "log": list(self._log)[-log_n:]}
