"""Turns an executor status report into a safe summary for the LLM.

The scene label inside a status comes from the VLM, which is untrusted text.
Only whitelisted fields are kept, and free text is cut down to a safe
character set and length before the LLM ever sees it.
"""
import re
from typing import Any, Dict, Optional

_UNSAFE = re.compile(r"[^A-Za-z0-9 _.,:()/-]")


def _clean(value: Any, n: int = 60) -> Optional[str]:
    if value is None:
        return None
    return _UNSAFE.sub("", str(value))[:n]


def _num(value: Any) -> Optional[float]:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    return value


def summarize_status(status: Dict[str, Any]) -> Dict[str, Any]:
    obs = status.get("last_observation")
    if not isinstance(obs, dict):
        obs = {}
    events = []
    for e in (status.get("events") or [])[-6:]:
        if isinstance(e, (list, tuple)) and len(e) == 2:
            events.append([_num(e[0]), _clean(e[1], 80)])
    visible = obs.get("target_visible")
    return {
        "state": _clean(status.get("state"), 20),
        "step_action": _clean(status.get("step_action"), 30),
        "step_index": _num(status.get("step_index")),
        "steps_total": _num(status.get("steps_total")),
        "elapsed_s": _num(status.get("elapsed_s")),
        "target_found": bool(status.get("target_found")),
        "failure_reason": _clean(status.get("failure_reason"), 80),
        "last_scene": _clean(obs.get("scene"), 40),
        "last_target_visible": visible if isinstance(visible, bool) else None,
        "events": events,
    }
