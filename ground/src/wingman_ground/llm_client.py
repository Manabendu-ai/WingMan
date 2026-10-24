"""Small OpenAI-compatible chat client with a model fallback chain."""
import os
import time
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional

import requests


class LLMError(Exception):
    """Raised when no model in the chain could produce an answer."""


def _csv(s: str) -> List[str]:
    return [x.strip() for x in s.split(",") if x.strip()]


@dataclass
class LLMConfig:
    base_url: str
    api_key: str
    models: List[str]
    max_tokens: int = 3000
    temperature: float = 0.2
    timeout_s: float = 90.0
    retries_per_model: int = 1

    @classmethod
    def from_env(cls) -> "LLMConfig":
        primary = os.getenv("LLM_MODEL", "nvidia/nemotron-3-ultra-550b-a55b:free")
        extra = [m for m in _csv(os.getenv("LLM_FALLBACK_MODELS", ""))
                 if m != primary]
        return cls(
            base_url=os.getenv("LLM_BASE_URL",
                               "https://openrouter.ai/api/v1").rstrip("/"),
            api_key=os.getenv("LLM_API_KEY", ""),
            models=[primary] + extra,
            max_tokens=int(os.getenv("LLM_MAX_TOKENS", "3000")),
        )


class LLMClient:
    def __init__(self, cfg: LLMConfig, post: Optional[Callable] = None,
                 sleep: Callable[[float], None] = time.sleep):
        self.cfg = cfg
        self._post = post or requests.post
        self._sleep = sleep
        self.last_model: Optional[str] = None

    def complete(self, messages: List[Dict[str, str]]) -> str:
        c = self.cfg
        url = f"{c.base_url}/chat/completions"
        headers = {"Authorization": f"Bearer {c.api_key}",
                   "Content-Type": "application/json"}
        errors: List[str] = []
        for model in c.models:
            payload = {"model": model, "messages": messages,
                       "temperature": c.temperature,
                       "max_tokens": c.max_tokens}
            for attempt in range(c.retries_per_model + 1):
                last_try = attempt == c.retries_per_model
                try:
                    r = self._post(url, headers=headers, json=payload,
                                   timeout=c.timeout_s)
                except requests.RequestException as e:
                    errors.append(f"{model}: {type(e).__name__}")
                    if not last_try:
                        self._sleep(1.0)
                    continue
                code = r.status_code
                if code in (401, 403):
                    raise LLMError(
                        f"authentication failed (HTTP {code}); check LLM_API_KEY")
                if code == 429 or code >= 500:
                    errors.append(f"{model}: HTTP {code}")
                    if not last_try:
                        self._sleep(1.5 * (attempt + 1))
                    continue
                if code != 200:
                    errors.append(f"{model}: HTTP {code}")
                    break                      # try the next model
                try:
                    msg = r.json()["choices"][0]["message"]
                    content = (msg.get("content") or "").strip()
                except (ValueError, KeyError, IndexError, TypeError,
                        AttributeError):
                    errors.append(f"{model}: unreadable response")
                    break
                if not content:
                    errors.append(f"{model}: empty content")
                    break
                self.last_model = model
                return content
        raise LLMError("all models failed: " + "; ".join(errors))
