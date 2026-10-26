"""Ground-side orchestration: request -> plan -> approve -> send -> monitor
-> replan on failure. The operator approves every mission, including replans."""
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Protocol, Tuple

from .mission import Mission
from .planner import MAX_REQUEST_CHARS, plan_mission
from .replan import replan_mission


class Link(Protocol):
    """Ground <-> drone connection. A real one will be asynchronous."""
    def send_mission(self, mission_json: str) -> bool: ...   # True = ACK
    def run_until_done(self) -> Dict[str, Any]: ...          # final status


@dataclass
class SessionResult:
    outcome: str    # success | failed | aborted | rejected | no_mission | link_error
    reason: str = ""
    mission_ids: List[str] = field(default_factory=list)
    final_status: Optional[Dict[str, Any]] = None

    def as_dict(self) -> Dict[str, Any]:
        st = self.final_status or {}
        return {"outcome": self.outcome, "reason": self.reason,
                "missions_sent": self.mission_ids,
                "final_state": st.get("state"),
                "failure_reason": st.get("failure_reason")}


def mission_summary(m: Mission) -> str:
    steps = " -> ".join(s.action for s in m.steps)
    lim = m.limits
    return (f"  goal:   {m.goal}\n  target: {m.target or '-'}\n"
            f"  steps:  {steps}\n"
            f"  limits: speed<={lim.max_speed_mps} m/s, "
            f"altitude<={lim.max_altitude_m} m, time<={lim.max_time_s} s")


class GroundSession:
    def __init__(self, llm, link: Link, ask: Callable[[str], str] = input,
                 say: Callable[[str], None] = print,
                 auto_approve: bool = False, max_replans: int = 2,
                 max_clarifications: int = 2):
        self.llm = llm
        self.link = link
        self.ask = ask
        self.say = say
        self.auto_approve = auto_approve
        self.max_replans = max_replans
        self.max_clarifications = max_clarifications

    def _plan(self, request: str, mission_id: str
              ) -> Tuple[Optional[Mission], str, str]:
        for _ in range(self.max_clarifications + 1):
            res = plan_mission(request, self.llm, mission_id=mission_id)
            if res.ok:
                return res.mission, request, ""
            if res.clarification:
                answer = self.ask(f"LLM asks: {res.clarification}\n> ").strip()
                if not answer:
                    return None, request, "no answer to the clarification question"
                request = f"{request}. {answer}"[:MAX_REQUEST_CHARS]
                continue
            return None, request, "; ".join(res.errors) or "planning failed"
        return None, request, "too many clarification rounds"

    def _approve(self, m: Mission, label: str) -> bool:
        self.say(f"\n{label}\n{mission_summary(m)}")
        if self.auto_approve:
            self.say("  (auto-approved)")
            return True
        return self.ask("Approve and send to the drone? [y/N] "
                        ).strip().lower() in ("y", "yes")

    def run(self, request: str) -> SessionResult:
        base = time.strftime("m_%Y%m%d_%H%M%S")
        ids: List[str] = []
        current, request, why = self._plan(request, base)
        if current is None:
            return SessionResult("no_mission", why)

        status: Optional[Dict[str, Any]] = None
        for attempt in range(self.max_replans + 1):
            label = ("PROPOSED MISSION" if attempt == 0 else
                     f"REPLANNED MISSION ({attempt}/{self.max_replans})")
            if not self._approve(current, label):
                return SessionResult("rejected", "operator rejected the mission",
                                     ids, status)
            if not self.link.send_mission(current.model_dump_json()):
                return SessionResult("link_error",
                                     "drone did not acknowledge the mission",
                                     ids, status)
            ids.append(current.mission_id)
            self.say(f"sent {current.mission_id}; waiting for the drone...")

            status = self.link.run_until_done()
            state = status.get("state")
            if state == "complete":
                return SessionResult("success", "", ids, status)
            if state == "aborted":
                return SessionResult("aborted",
                                     str(status.get("failure_reason")), ids, status)

            reason = status.get("failure_reason") or f"ended in state {state}"
            if attempt == self.max_replans:
                return SessionResult(
                    "failed", f"gave up after {attempt} replan(s): {reason}",
                    ids, status)
            self.say(f"mission failed: {reason}. "
                     f"Asking the LLM to replan ({attempt + 1}/{self.max_replans})...")

            nxt: Optional[Mission] = None
            for _ in range(self.max_clarifications + 1):
                rp = replan_mission(request, current, status, self.llm,
                                    mission_id=f"{base}_r{attempt + 1}")
                if rp.ok:
                    nxt = rp.mission
                    break
                if rp.clarification:
                    answer = self.ask(f"LLM asks: {rp.clarification}\n> ").strip()
                    if not answer:
                        break
                    request = f"{request}. {answer}"[:MAX_REQUEST_CHARS]
                    continue
                break
            if nxt is None:
                return SessionResult("failed",
                                     f"replanning failed after: {reason}",
                                     ids, status)
            current = nxt
        return SessionResult("failed", "unexpected end of loop", ids, status)
