# WINGMAN dashboard backend (stage 1)

FastAPI backend for the operator dashboard. Runs your real `GroundSession`
(plan -> approve -> send -> monitor -> replan) in a background thread and
exposes it over REST + WebSocket + MJPEG. The drone is simulated
(`LiveSimDrone` + the real `MissionExecutor`); ROS comes later behind the same interfaces.

## Install (inside your WingMan repo)
    unzip wingman_dashboard_backend.zip -d ~/WingMan/      # creates ~/WingMan/dashboard
    cd ~/WingMan && source vlm/.venv/bin/activate
    pip install -e dashboard                                # fastapi, uvicorn, pillow, numpy

## Run
    # no LLM key, no VLM server (fully mocked):
    python -m wingman_dashboard --mock
    # real LLM + real VLM on your saved images (VLM server running, .env loaded):
    set -a; source ~/WingMan/.env; set +a
    python -m wingman_dashboard --frames ~/WingMan/vlm/eval/data
It prints a URL containing the access token and binds to 127.0.0.1 only.
Flags: `--token`, `--port`, `--tick` (real seconds per sim tick), `--break-takeoff-once`.

## API (every route needs header `X-Token` or `?token=`)
| Route | Purpose |
|---|---|
| `POST /api/mission {"request": "..."}` | start planning (409 if busy, 422 if bad length) |
| `GET /api/state` | phase, pending, result, telemetry, last_obs, log |
| `POST /api/approve {"approve": true/false}` | answer the pending approval |
| `POST /api/answer {"text": "..."}` | answer a pending LLM clarification |
| `POST /api/abort` | abort: rejects a pending approval, or stops the executor mid-flight |
| `GET /api/map` | occupancy grid (ROS OccupancyGrid layout) + pose + path |
| `GET /camera.mjpg` | MJPEG stream, VLM target circled |
| `WS /ws?token=` | state every 0.2 s; `map` included only when it changed |
| `GET /api/health` | open, no data |

`phase`: idle, planning, awaiting_approval, awaiting_clarification, executing, done.
Voice input must just POST its (operator-edited) transcript to `/api/mission`: same approval path.

## Layout / swap points
- `live.py` LiveLink: Link protocol with per-tick telemetry and an abort flag. Replace with the ROS link later.
- `mapsource.py` MapSource protocol: `MockMapSource` now; RTAB-Map `/map` subscriber later (same snapshot shape).
- `camera.py` ReplayCamera: shows the frame the VLM is analysing. Webcam/RTSP/ROS sources in stage 2.
- `PolicyLink` (live.py): takeoff/return/land timeouts are NOT auto-replanned; they come back as `aborted` with `policy_halt`.

## Assumptions to check
- VLM `target_x/target_y` are assumed to be on a 0..1000 grid for the overlay circle (`ReplayCamera(overlay_scale=...)`).
- `GroundSession` is used unmodified: approvals are detected by the prompt text ("LLM asks:" = clarification, else approval).
- Tests (`pytest dashboard`) were run against a stubbed ground/VLM layer plus your real executor and mission code; run them once in your repo.
