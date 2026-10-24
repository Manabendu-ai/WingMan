"""Try each free Nemotron model with one tiny request and report which work.
Usage: python ground/scripts/probe_free.py [extra_model_id ...]
"""
import os
import sys
import time

import requests

BASE = os.getenv("LLM_BASE_URL", "https://openrouter.ai/api/v1").rstrip("/")
KEY = os.getenv("LLM_API_KEY")
if not KEY:
    sys.exit("LLM_API_KEY not set. Run: set -a; source ~/WingMan/.env; set +a")
H = {"Authorization": f"Bearer {KEY}", "Content-Type": "application/json"}

CANDIDATES = [
    "nvidia/nemotron-3-ultra-550b-a55b:free",
    "nvidia/nemotron-3-super-120b-a12b:free",
    "nvidia/nemotron-3.5-lightning:free",
    "nvidia/nemotron-3-nano-omni-30b-a3b-reasoning:free",
] + sys.argv[1:]

PROMPT = 'Reply with exactly this JSON and nothing else: {"ok": true}'

for model in CANDIDATES:
    payload = {"model": model, "temperature": 0.1, "max_tokens": 1000,
               "messages": [{"role": "user", "content": PROMPT}]}
    t0 = time.time()
    try:
        r = requests.post(f"{BASE}/chat/completions", headers=H,
                          json=payload, timeout=90)
    except requests.RequestException as e:
        print(f"FAIL  {model:55} {type(e).__name__}")
        continue
    dt = time.time() - t0
    if r.ok:
        msg = r.json()["choices"][0]["message"]
        content = (msg.get("content") or "").strip()
        status = "OK   " if content else "EMPTY"
        print(f"{status} {model:55} {dt:5.1f}s  {content[:60]!r}")
    else:
        print(f"FAIL  {model:55} HTTP {r.status_code} ({dt:.1f}s)")
