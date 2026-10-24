import base64
import io
import json
import os
import re
import time
from dataclasses import dataclass

import requests
from PIL import Image

from .prompts import build_prompt
from .schema import Observation, fallback_observation

URL = os.getenv("VLM_URL", "http://127.0.0.1:8080/v1/chat/completions")
MODEL = os.getenv("VLM_MODEL", "qwen3-vl")
KEY = os.getenv("VLM_KEY", "")
HEADERS = {"Authorization": f"Bearer {KEY}"} if KEY else {}


@dataclass
class Result:
    obs: Observation
    latency: float
    attempts: int
    ok: bool      # False means the fallback answer was used
    raw: str


def encode_image(path: str, max_side: int = 640) -> str:
    img = Image.open(path).convert("RGB")
    img.thumbnail((max_side, max_side))
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=85)
    return base64.b64encode(buf.getvalue()).decode()


def _extract_json(text: str) -> dict:
    match = re.search(r"\{.*\}", text, re.S)
    return json.loads(match.group(0) if match else text)


def observe(image_path: str, task: str = "find the exit", retries: int = 1) -> Result:
    b64 = encode_image(image_path)
    payload = {
        "model": MODEL,
        "temperature": 0.1,
        "max_tokens": 300,
        "messages": [{
            "role": "user",
            "content": [
                {"type": "text", "text": build_prompt(task)},
                {"type": "image_url",
                 "image_url": {"url": "data:image/jpeg;base64," + b64}},
            ],
        }],
    }
    t0 = time.time()
    raw, error = "", "unknown"
    for attempt in range(retries + 1):
        try:
            r = requests.post(URL, json=payload, headers=HEADERS, timeout=120)
            r.raise_for_status()
            raw = r.json()["choices"][0]["message"]["content"]
            obs = Observation(**_extract_json(raw))
            return Result(obs, time.time() - t0, attempt, True, raw)
        except (requests.RequestException, KeyError, ValueError) as e:
            error = f"{type(e).__name__}: {e}"
    return Result(fallback_observation(error[:120]), time.time() - t0,
                  retries + 1, False, raw)
