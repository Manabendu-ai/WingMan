import pytest
import requests

from wingman_ground.llm_client import LLMClient, LLMConfig, LLMError

MSG = [{"role": "user", "content": "hi"}]


class FakeResp:
    def __init__(self, status=200, content="ok"):
        self.status_code = status
        self._content = content

    def json(self):
        return {"choices": [{"message": {"content": self._content}}]}


def make_client(models, script, retries=1):
    calls = []

    def post(url, headers=None, json=None, timeout=None):
        model = json["model"]
        calls.append(model)
        item = script[model].pop(0)
        if isinstance(item, Exception):
            raise item
        return item

    cfg = LLMConfig("http://x", "k", models, retries_per_model=retries)
    return LLMClient(cfg, post=post, sleep=lambda s: None), calls


def test_falls_back_on_404():
    c, calls = make_client(["a", "b"], {"a": [FakeResp(404)],
                                        "b": [FakeResp(200, "hello")]})
    assert c.complete(MSG) == "hello"
    assert c.last_model == "b" and calls == ["a", "b"]


def test_retries_429_then_succeeds():
    c, calls = make_client(["a", "b"], {"a": [FakeResp(429), FakeResp(200, "hi")],
                                        "b": []})
    assert c.complete(MSG) == "hi" and calls == ["a", "a"]


def test_empty_content_falls_to_next_model():
    c, _ = make_client(["a", "b"], {"a": [FakeResp(200, "")],
                                    "b": [FakeResp(200, "real")]})
    assert c.complete(MSG) == "real"


def test_auth_error_raises_immediately():
    c, calls = make_client(["a", "b"], {"a": [FakeResp(401)],
                                        "b": [FakeResp(200, "x")]})
    with pytest.raises(LLMError, match="authentication"):
        c.complete(MSG)
    assert calls == ["a"]


def test_network_error_falls_back():
    err = requests.ConnectionError("boom")
    c, calls = make_client(["a", "b"], {"a": [err, err],
                                        "b": [FakeResp(200, "ok")]})
    assert c.complete(MSG) == "ok" and calls == ["a", "a", "b"]


def test_all_models_failing_raises():
    c, _ = make_client(["a", "b"], {"a": [FakeResp(404)],
                                    "b": [FakeResp(500), FakeResp(500)]})
    with pytest.raises(LLMError, match="all models failed"):
        c.complete(MSG)


def test_config_from_env_builds_chain(monkeypatch):
    monkeypatch.setenv("LLM_MODEL", "m1")
    monkeypatch.setenv("LLM_FALLBACK_MODELS", "m2, m1 ,m3")
    assert LLMConfig.from_env().models == ["m1", "m2", "m3"]
