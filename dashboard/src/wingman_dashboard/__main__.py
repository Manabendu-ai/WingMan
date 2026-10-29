"""python -m wingman_dashboard --mock      (no LLM key, no VLM server needed)"""
import argparse
import os
import secrets
import sys
from pathlib import Path

import uvicorn

from .app import create_app
from .camera import ReplayCamera
from .hub import Hub
from .live import LiveLink, PolicyLink
from .manager import MissionManager
from .mapsource import MockMapSource
from .mocks import MockLLM, MockVLM


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mock", action="store_true", help="mock LLM and mock VLM")
    ap.add_argument("--mock-llm", action="store_true")
    ap.add_argument("--mock-vlm", action="store_true")
    ap.add_argument("--frames", help="folder of images (e.g. vlm/eval/data)")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--token", help="access token (random if omitted)")
    ap.add_argument("--docs", action="store_true", help="enable /docs (Swagger UI, needs internet for its JS)")
    ap.add_argument("--tick", type=float, default=0.4, help="real seconds per sim tick")
    ap.add_argument("--break-takeoff-once", action="store_true",
                    help="demo: first takeoff fails; the policy stops instead of replanning")
    ap.add_argument("--remote-video", default=os.getenv("WINGMAN_REMOTE_VIDEO_URL", ""),
                    help="remote web_video_server stream URL")
    ap.add_argument("--remote-ros-ws", default=os.getenv("WINGMAN_REMOTE_ROS_WS", ""),
                    help="remote rosbridge WebSocket URL")
    a = ap.parse_args()

    if a.mock_llm or a.mock:
        llm = MockLLM()
    else:
        from wingman_ground.llm_client import LLMClient, LLMConfig
        cfg = LLMConfig.from_env()
        if not cfg.api_key:
            sys.exit("LLM_API_KEY not set. Run: set -a; source ~/WingMan/.env; set +a  (or use --mock)")
        llm = LLMClient(cfg)
    if a.mock_vlm or a.mock:
        observe = MockVLM()
    else:
        from wingman_vlm.client import observe
    if a.frames:
        from wingman_onboard.sim import default_frames
        frames = default_frames(Path(a.frames))
        if not frames:
            sys.exit(f"no images in {a.frames}")
    else:
        frames = [Path(f"mock_frame_{i}.jpg") for i in range(1, 13)]

    hub, mapsrc = Hub(), MockMapSource()
    link = PolicyLink(LiveLink(observe, frames, hub, mapsrc, tick_real_s=a.tick,
                               break_first_takeoff=a.break_takeoff_once))
    token = a.token or secrets.token_urlsafe(16)
    app = create_app(
        MissionManager(llm, link, hub), ReplayCamera(hub), mapsrc, token,
        docs=a.docs, remote_video_url=a.remote_video,
        remote_ros_url=a.remote_ros_ws)
    print(f"\n  WINGMAN dashboard on http://{a.host}:{a.port}/?token={token}")
    if a.docs:
        print(f"  API docs:  http://{a.host}:{a.port}/docs   (click Authorize, paste the token)")
    print()
    uvicorn.run(app, host=a.host, port=a.port, log_level="warning",
                timeout_graceful_shutdown=2)


if __name__ == "__main__":
    main()
