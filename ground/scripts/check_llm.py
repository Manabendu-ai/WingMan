"""Check an OpenAI-compatible LLM endpoint (OpenRouter, NVIDIA, Nebius...).
Reads LLM_BASE_URL, LLM_API_KEY, LLM_MODEL from the environment.
Usage: python ground/scripts/check_llm.py [model_id]
"""
import os
import sys
import time

import requests

BASE = os.getenv("LLM_BASE_URL", "https://openrouter.ai/api/v1").rstrip("/")
KEY = os.getenv("LLM_API_KEY")
MODEL = sys.argv[1] if len(sys.argv) > 1 else os.getenv(
    "LLM_MODEL", "nvidia/nemotron-3-super-120b-a12b:free")
if not KEY:
    sys.exit("LLM_API_KEY not set. Run: set -a; source ~/WingMan/.env; set +a")
H = {"Authorization": f"Bearer {KEY}", "Content-Type": "application/json"}


def list_models():
    r = requests.get(f"{BASE}/models", headers=H, timeout=30)
    print(f"GET /models -> {r.status_code}")
    if not r.ok:
        print(r.text[:300])
        return
    ids = sorted(m.get("id", "") for m in r.json().get("data", []))
    nv = [i for i in ids if "nemotron" in i.lower() or i.lower().startswith("nvidia/")]
    print(f"{len(ids)} models visible. NVIDIA / Nemotron ones:")
    for i in nv:
        print("  ", i)


def chat():
    payload = {
        "model": MODEL,
        "temperature": 0.1,
        "max_tokens": 1500,
        "messages": [{"role": "user",
                      "content": 'Reply with exactly this JSON and nothing else: {"ok": true}'}],
    }
    t0 = time.time()
    r = requests.post(f"{BASE}/chat/completions", headers=H, json=payload, timeout=120)
    print(f"\nPOST /chat/completions ({MODEL}) -> {r.status_code} in {time.time()-t0:.2f}s")
    if not r.ok:
        print(r.text[:500])
        return
    msg = r.json()["choices"][0]["message"]
    content = (msg.get("content") or "").strip()
    reasoning = msg.get("reasoning_content") or msg.get("reasoning") or ""
    print("content:", repr(content[:200]))
    print("has separate reasoning field:", bool(reasoning))
    if not content:
        print("WARNING: empty content (reasoning model?). Tell me, the client must handle it.")


if __name__ == "__main__":
    list_models()
    chat()
