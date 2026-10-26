"""Runs GroundSession in a background thread; approval and clarification
questions become 'pending' items answered over REST instead of input()."""
import threading
from typing import Optional

from wingman_ground.session import GroundSession

MAX_REQUEST_CHARS = 500


class Busy(Exception):
    pass


class BadAnswer(Exception):
    pass


class MissionManager:
    def __init__(self, llm, link, hub, approval_timeout_s: float = 600.0):
        self.link, self.hub, self.timeout = link, hub, approval_timeout_s
        self._lock = threading.RLock()
        self._evt = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._pending = None
        self._answer = ""
        self._cancel = False
        self._last_say = ""
        self._n = 0
        self.request = ""
        self.result = None
        self.session = GroundSession(llm, link, ask=self._ask, say=self._say, auto_approve=False)

    # ---- hooks called from the session thread ----
    def _say(self, text: str) -> None:
        self._last_say = text.strip()
        self.hub.say(text)

    def _ask(self, prompt: str) -> str:
        kind = "clarification" if prompt.startswith("LLM asks") else "approval"
        with self._lock:
            if self._cancel:
                return "n" if kind == "approval" else ""
            self._n += 1
            self._answer = ""
            self._evt.clear()
            self._pending = {"id": self._n, "kind": kind, "prompt": prompt.strip(),
                             "detail": self._last_say if kind == "approval" else ""}
        self._evt.wait(self.timeout)
        with self._lock:
            ans, cancelled = self._answer, self._cancel
            self._pending = None
        if cancelled:
            return "n" if kind == "approval" else ""
        return ans

    def _run(self, request: str) -> None:
        try:
            self.result = self.session.run(request).as_dict()
        except Exception as e:     # keep the dashboard alive whatever the planner does
            self.hub.say(f"ERROR: {type(e).__name__}: {e}")
            self.result = {"outcome": "error", "reason": f"{type(e).__name__}: {e}"[:200]}
        self.hub.say(f"=== RESULT: {self.result['outcome']} {self.result.get('reason') or ''}")

    # ---- called from the API ----
    def start(self, request: str) -> None:
        request = (request or "").strip()
        if not 1 <= len(request) <= MAX_REQUEST_CHARS:
            raise ValueError(f"request must be 1-{MAX_REQUEST_CHARS} characters")
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                raise Busy("a mission is already in progress")
            self._cancel, self._pending, self.result, self.request = False, None, None, request
            self._thread = threading.Thread(target=self._run, args=(request,), daemon=True)
            self._thread.start()
        self.hub.say(f"operator request: {request}")

    def answer(self, kind: str, value) -> None:
        with self._lock:
            p = self._pending
            if p is None or p["kind"] != kind:
                raise BadAnswer(f"no pending {kind}")
            self._answer = ("y" if value else "n") if kind == "approval" else str(value)[:300]
            self._evt.set()

    def abort(self) -> None:
        with self._lock:
            self._cancel = True
            self._evt.set()
        self.link.request_abort()
        self.hub.say("ABORT requested by operator")

    def phase(self) -> str:
        with self._lock:
            if self._pending:
                return f"awaiting_{self._pending['kind']}"
        if self._thread is not None and self._thread.is_alive():
            return "executing" if self.link.running else "planning"
        return "done" if self.result else "idle"

    def snapshot(self) -> dict:
        with self._lock:
            pending, result, req = (dict(self._pending) if self._pending else None,
                                    self.result, self.request)
        return {"phase": self.phase(), "request": req, "pending": pending,
                "result": result, **self.hub.view()}
