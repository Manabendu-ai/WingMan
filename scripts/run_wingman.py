"""Type a request -> LLM plans -> you approve -> (simulated) flight -> replan on failure.

Usage:
  python scripts/run_wingman.py "take off, find the red box, then hover over it"
  python scripts/run_wingman.py --inject-failure "find the red box"
Needs the VLM server running and the LLM env loaded (set -a; source .env; set +a).
"""
import argparse
import json
import sys
import time
from pathlib import Path

from wingman_ground.llm_client import LLMClient, LLMConfig
from wingman_ground.planner import plan_mission
from wingman_ground.replanner import plan_replan
from wingman_onboard.sim import ImageFrames, SimDrone, fly

ROOT = Path(__file__).resolve().parents[1]


def ask(prompt):
    try:
        return input(prompt).strip()
    except EOFError:
        return ""


def show(m):
    print(f"\n--- MISSION {m.mission_id} ---")
    print(f"goal:   {m.goal}\ntarget: {m.target}")
    for i, s in enumerate(m.steps, 1):
        print(f"  {i}. {json.dumps(s.model_dump(exclude_none=True))}")
    l = m.limits
    print(f"limits: speed<={l.max_speed_mps} m/s, alt<={l.max_altitude_m} m, "
          f"time<={l.max_time_s} s\n---")


def approve(m, auto):
    show(m)
    if auto:
        print("(auto-approved with --yes)")
        return True
    return ask("Approve and fly? [y/N] ").lower() in ("y", "yes")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("request", nargs="*")
    ap.add_argument("--yes", action="store_true", help="auto-approve missions")
    ap.add_argument("--inject-failure", action="store_true",
                    help="stall the first takeoff to demonstrate replanning")
    ap.add_argument("--max-replans", type=int, default=2)
    ap.add_argument("--images", default=str(ROOT / "vlm/eval/data"))
    args = ap.parse_args()

    cfg = LLMConfig.from_env()
    if not cfg.api_key:
        sys.exit("LLM_API_KEY not set. Run: set -a; source .env; set +a")
    llm = LLMClient(cfg)
    frames = ImageFrames(args.images)

    request = " ".join(args.request).strip() or ask("Request: ")
    if not request:
        return 1

    mission = None
    for _ in range(3):
        res = plan_mission(request, llm)
        if res.ok:
            mission = res.mission
            break
        if res.clarification:
            print(f"LLM asks: {res.clarification}")
            ans = ask("Your answer (blank to quit): ")
            if not ans:
                return 1
            request = f"{request}. {ans}"[:500]
            continue
        print("Planning failed:", *res.errors, sep="\n - ")
        return 1
    if mission is None:
        print("Could not get a valid mission.")
        return 1

    log = []
    for attempt in range(args.max_replans + 1):
        if not approve(mission, args.yes):
            print("Not approved. Nothing flown.")
            break
        frames.reset()
        drone = SimDrone(stuck_takeoff=args.inject_failure and attempt == 0)
        status = fly(mission, drone, frames)
        log.append({"mission": json.loads(mission.model_dump_json()),
                    "status": status})
        print(f"\nRESULT: {status['state']}"
              + (f" ({status['failure_reason']})" if status["failure_reason"] else ""))
        if status["state"] == "complete":
            print("Mission accomplished.")
            break
        if status["state"] == "aborted" or attempt == args.max_replans:
            print("Stopping. No more replans.")
            break

        print(f"\nAsking the LLM to replan (replan {attempt + 1}/{args.max_replans})...")
        new = None
        for _ in range(2):
            rp = plan_replan(request, mission, status, llm)
            if rp.ok:
                new = rp.mission
                break
            if rp.clarification:
                print(f"LLM asks: {rp.clarification}")
                ans = ask("Your answer (blank to stop): ")
                if not ans:
                    break
                request = f"{request}. {ans}"[:500]
                continue
            print("Replan failed:", *rp.errors, sep="\n - ")
            break
        if new is None:
            break
        mission = new

    out = ROOT / "ground/out"
    out.mkdir(exist_ok=True)
    path = out / f"run_{time.strftime('%Y%m%d_%H%M%S')}.json"
    path.write_text(json.dumps({"request": request, "runs": log}, indent=2))
    print(f"\nrun log saved to {path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
