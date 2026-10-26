import copy
import json

from wingman_ground.mission import Mission
from wingman_ground.prompts import EXAMPLE_MISSION
from wingman_ground.session import GroundSession, mission_summary


def quiet(*a, **k):
    pass


COMPLETE = {"state": "complete", "failure_reason": None, "events": []}
FAILED = {"state": "failed", "failure_reason": "takeoff timeout",
          "step_action": "takeoff", "step_index": 0, "steps_total": 4,
          "elapsed_s": 31.0, "target_found": False,
          "last_observation": None,
          "events": [[31.0, "FAILED: takeoff timeout"]]}
ABORTED = {"state": "aborted", "failure_reason": "operator abort", "events": []}
CLAR = '{"clarification_needed": "What should I look for?"}'


def good():
    return json.dumps(copy.deepcopy(EXAMPLE_MISSION))


class FakeLLM:
    def __init__(self, replies):
        self.replies = list(replies)
        self.calls = []
        self.last_model = "fake"

    def complete(self, messages):
        self.calls.append([dict(m) for m in messages])
        return self.replies.pop(0)


class FakeLink:
    def __init__(self, statuses, ack=True):
        self.statuses = list(statuses)
        self.ack = ack
        self.sent = []

    def send_mission(self, mission_json):
        self.sent.append(json.loads(mission_json))
        return self.ack

    def run_until_done(self):
        return self.statuses.pop(0) if len(self.statuses) > 1 else self.statuses[0]


class Ask:
    def __init__(self, answers):
        self.answers = list(answers)
        self.prompts = []

    def __call__(self, prompt):
        self.prompts.append(prompt)
        return self.answers.pop(0)


def session(llm, link, answers=(), auto=True, max_replans=2):
    return GroundSession(llm, link, ask=Ask(answers), say=quiet,
                         auto_approve=auto, max_replans=max_replans)


def test_success_on_first_try():
    link = FakeLink([COMPLETE])
    res = session(FakeLLM([good()]), link).run("solve the maze")
    assert res.outcome == "success" and len(link.sent) == 1
    assert len(res.mission_ids) == 1


def test_failure_then_replan_then_success():
    llm = FakeLLM([good(), good()])
    link = FakeLink([FAILED, COMPLETE])
    res = session(llm, link).run("solve the maze")
    assert res.outcome == "success" and len(link.sent) == 2
    assert res.mission_ids[1].endswith("_r1")
    assert "takeoff timeout" in llm.calls[1][1]["content"]


def test_gives_up_after_max_replans():
    llm = FakeLLM([good()] * 3)
    link = FakeLink([FAILED])
    res = session(llm, link, max_replans=2).run("solve the maze")
    assert res.outcome == "failed" and len(link.sent) == 3
    assert "gave up" in res.reason


def test_operator_can_reject():
    link = FakeLink([COMPLETE])
    res = session(FakeLLM([good()]), link, answers=["n"], auto=False
                  ).run("solve the maze")
    assert res.outcome == "rejected" and link.sent == []


def test_operator_approval_sends_mission():
    link = FakeLink([COMPLETE])
    res = session(FakeLLM([good()]), link, answers=["y"], auto=False
                  ).run("solve the maze")
    assert res.outcome == "success" and len(link.sent) == 1


def test_clarification_is_answered_and_used():
    llm = FakeLLM([CLAR, good()])
    link = FakeLink([COMPLETE])
    res = session(llm, link, answers=["the red box"]).run("find it")
    assert res.outcome == "success"
    assert "the red box" in llm.calls[1][1]["content"]


def test_unanswered_clarification_ends_without_a_mission():
    link = FakeLink([COMPLETE])
    res = session(FakeLLM([CLAR]), link, answers=[""]).run("find it")
    assert res.outcome == "no_mission" and link.sent == []


def test_link_without_ack_is_reported():
    link = FakeLink([COMPLETE], ack=False)
    res = session(FakeLLM([good()]), link).run("solve the maze")
    assert res.outcome == "link_error"


def test_abort_is_not_replanned():
    llm = FakeLLM([good()])
    res = session(llm, FakeLink([ABORTED])).run("solve the maze")
    assert res.outcome == "aborted" and len(llm.calls) == 1


def test_replan_that_cannot_proceed_reports_failure():
    llm = FakeLLM([good(), CLAR])
    link = FakeLink([FAILED])
    res = session(llm, link, answers=[""]).run("solve the maze")
    assert res.outcome == "failed" and res.reason.startswith("replanning failed")
    assert len(link.sent) == 1


def test_mission_summary_is_readable():
    s = mission_summary(Mission.model_validate(EXAMPLE_MISSION))
    assert "takeoff -> explore -> approach_target -> hover" in s
    assert "maze exit" in s
