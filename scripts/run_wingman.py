"""WINGMAN end-to-end offline demo.

  request -> LLM mission -> operator approval -> onboard executor + real VLM
  (on saved images, simulated drone) -> failure report -> LLM replan.

Needs the local VLM server running (vlm/scripts/run_server.sh) and the LLM
variables loaded:  set -a; source ~/WingMan/.env; set +a
"""
import argparse
import json
import sys
from pathlib import Path

from wingman_ground.llm_client import LLMClient, LLMConfig
from wingman_ground.session import GroundSession
from wingman_onboard.sim import InProcessLink, default_frames
from wingman_vlm.client import observe

ROOT = Path(__file__).resolve().parents[1]

ap = argparse.ArgumentParser()
ap.add_argument("request", help='e.g. "take off, find the red box, then hover over it"')
ap.add_argument("--yes", action="store_true", help="auto-approve missions")
ap.add_argument("--break-takeoff-once", action="store_true",
                help="make the first takeoff fail, to demo replanning")
ap.add_argument("--dt", type=float, default=1.0, help="simulated seconds per tick")
ap.add_argument("--max-replans", type=int, default=2)
args = ap.parse_args()

cfg = LLMConfig.from_env()
if not cfg.api_key:
    sys.exit("LLM_API_KEY not set. Run: set -a; source ~/WingMan/.env; set +a")
frames = default_frames(ROOT / "vlm/eval/data")
if not frames:
    sys.exit("no images in vlm/eval/data")

link = InProcessLink(observe, frames, dt=args.dt,
                     break_first_takeoff=args.break_takeoff_once)
session = GroundSession(LLMClient(cfg), link, auto_approve=args.yes,
                        max_replans=args.max_replans)
result = session.run(args.request)

print("\n=== RESULT ===")
print(json.dumps(result.as_dict(), indent=2))
