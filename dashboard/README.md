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

### Remote ROS monitor (no ROS on this laptop)

The browser can connect directly to a ROS 2 machine running `rosbridge_server`
and `web_video_server`. ROS does not need to be installed on the laptop
running the browser. Start the WingMan ROS bridge on the ROS machine first,
then open the dashboard with:

```text
http://DASHBOARD_HOST:8000/?ros_ws=ws%3A%2F%2FROS_HOST%3A9090&video=http%3A%2F%2FROS_HOST%3A8080%2Fstream%3Ftopic%3D%252Fcamera%252Fcolor%252Fimage_raw
```

In this mode the site displays the remote RGB stream, map, odometry/PX4
telemetry, mission status, command acknowledgements, VLM observations, and
stale-data health. Commands remain disabled unless
`--enable-remote-commands` is explicitly used.

For the currently available camera stream, run the dashboard on the laptop
with:

```bash
python -m wingman_dashboard --mock \
  --host 0.0.0.0 \
  --remote-video 'http://192.168.128.220:8080/stream?topic=/world/maze_exploration/model/x500_gps_denied_0/link/camera_link/sensor/IMX214/image'
```

Then open `http://127.0.0.1:8000/`. The remote video URL can also be supplied
with `WINGMAN_REMOTE_VIDEO_URL`.

To connect the mission box to the command gateway:

```bash
python -m wingman_dashboard --mock \
  --host 0.0.0.0 \
  --remote-video 'http://192.168.128.220:8080/stream?topic=/world/maze_exploration/model/x500_gps_denied_0/link/camera_link/sensor/IMX214/image' \
  --remote-ros-ws 'ws://192.168.128.220:9090' \
  --enable-remote-commands
```

Submitting the mission box publishes `{"id":"<generated>","prompt":"<text>"}`
to `/wingman/llm_command`. The ABORT button publishes a `hover` command.
Acknowledgements from `/wingman/llm_command_ack` are shown in the mission log.

On the ROS machine, start:

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
ros2 run web_video_server web_video_server
ros2 run wingman_ros bridge
```

The browser must be able to reach ports `9090` and `8080` on `ROS_HOST`.
For a public deployment, place these services behind a VPN or authenticated
reverse proxy; do not expose rosbridge directly to the internet.

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
