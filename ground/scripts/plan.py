"""Turn a typed request into a validated mission using the LLM chain.
Usage: python ground/scripts/plan.py "solve the following maze"
"""
import sys
from pathlib import Path

from wingman_ground.llm_client import LLMClient, LLMConfig
from wingman_ground.planner import plan_mission

if len(sys.argv) < 2:
    sys.exit('Usage: python ground/scripts/plan.py "your request"')

cfg = LLMConfig.from_env()
if not cfg.api_key:
    sys.exit("LLM_API_KEY not set. Run: set -a; source ~/WingMan/.env; set +a")

res = plan_mission(" ".join(sys.argv[1:]), LLMClient(cfg))
print(f"model chain: {cfg.models}")
print(f"used: {res.model_used} | attempts: {res.attempts}")

if res.ok:
    out = Path(__file__).resolve().parents[1] / "out"
    out.mkdir(exist_ok=True)
    path = out / "last_mission.json"
    path.write_text(res.mission.model_dump_json(indent=2))
    print(res.mission.model_dump_json(indent=2))
    print(f"\nVLM task string: {res.mission.vlm_task()!r}")
    print(f"saved to {path}")
elif res.clarification:
    print(f"\nThe LLM needs clarification: {res.clarification}")
else:
    print("\nFAILED:")
    for e in res.errors:
        print(" -", e)
