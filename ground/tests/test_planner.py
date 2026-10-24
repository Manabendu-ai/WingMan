import copy
import json

from wingman_ground.llm_client import LLMError
from wingman_ground.mission import MAX_ALTITUDE_M, MAX_STEPS
from wingman_ground.planner import plan_mission
from wingman_ground.prompts import EXAMPLE_MISSION, build_system_prompt
from wingman_ground.validator import validate_mission


class FakeLLM:
    def __init__(self, replies):
        self.replies = list(replies)
        self.calls = []
        self.last_model = "fake-model"

    def complete(self, messages):
        self.calls.append([dict(m) for m in messages])
        r = self.replies.pop(0)
        if isinstance(r, Exception):
            raise r
        return r


def good(**over):
    m = copy.deepcopy(EXAMPLE_MISSION)
    m.update(over)
    return json.dumps(m)


def bad_speed():
    m = copy.deepcopy(EXAMPLE_MISSION)
    m["limits"]["max_speed_mps"] = 9.0
    return json.dumps(m)


def test_valid_first_try_and_mission_id_is_ours():
    llm = FakeLLM([good(mission_id="weird id!!")])
    res = plan_mission("solve the maze", llm, mission_id="m_test")
    assert res.ok and res.attempts == 1
    assert res.mission.mission_id == "m_test"
    assert res.model_used == "fake-model"


def test_repair_loop_feeds_errors_back():
    llm = FakeLLM([bad_speed(), good()])
    res = plan_mission("solve the maze", llm)
    assert res.ok and res.attempts == 2
    assert "max_speed" in llm.calls[1][-1]["content"]


def test_gives_up_after_max_attempts():
    llm = FakeLLM([bad_speed()] * 3)
    res = plan_mission("solve the maze", llm)
    assert not res.ok and res.attempts == 3 and res.errors


def test_clarification_is_returned_without_repair():
    llm = FakeLLM(['{"clarification_needed": "What should I look for?"}'])
    res = plan_mission("find it", llm)
    assert not res.ok and res.clarification == "What should I look for?"
    assert res.attempts == 1 and len(llm.calls) == 1


def test_llm_unavailable_is_reported():
    llm = FakeLLM([LLMError("all models failed: a: HTTP 404")])
    res = plan_mission("solve the maze", llm)
    assert not res.ok and "all models failed" in res.errors[0]


def test_empty_request_never_calls_llm():
    llm = FakeLLM([])
    res = plan_mission("   ", llm)
    assert not res.ok and llm.calls == []


def test_too_long_request_is_rejected():
    llm = FakeLLM([])
    assert not plan_mission("x" * 501, llm).ok and llm.calls == []


def test_example_in_prompt_is_a_valid_mission():
    assert validate_mission(EXAMPLE_MISSION).ok


def test_prompt_is_built_from_the_real_limits():
    p = build_system_prompt()
    assert str(MAX_STEPS) in p and str(MAX_ALTITUDE_M) in p
    assert "__" not in p          # no unreplaced placeholders


def test_operator_request_is_delimited():
    llm = FakeLLM([good()])
    plan_mission("solve the maze", llm)
    user_msg = llm.calls[0][1]["content"]
    assert "<<<" in user_msg and ">>>" in user_msg and "solve the maze" in user_msg


def test_fenced_output_is_accepted():
    llm = FakeLLM(["```json\n" + good() + "\n```"])
    assert plan_mission("solve the maze", llm).ok


def test_missing_mission_id_is_filled_in():
    m = copy.deepcopy(EXAMPLE_MISSION)
    del m["mission_id"]
    res = plan_mission("solve the maze", FakeLLM([json.dumps(m)]))
    assert res.ok and res.mission.mission_id.startswith("m_")
