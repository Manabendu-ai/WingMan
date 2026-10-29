# WingMan <-> website interface

```
friend's website (LLM)  --ws:9090-->  rosbridge  -->  /wingman/llm_command
                                                          |  web_gateway: validate, clamp, ack
                                                          v
                                               /wingman/mission_command --> mission_autopilot --> /cmd_vel --> cmdvel_to_px4 --> PX4
                                                          |                        \--> /wingman/target_alt --/   (height hold on the rangefinder)
website  <--ws:9090--  /wingman/telemetry, /wingman/llm_command_ack, /map, /wingman/planner_debug/path
website  <--http:8080-- MJPEG camera stream
```
The VLM is not in the flight loop. Height is held by `cmdvel_to_px4` using the downward rangefinder
(`/wingman/range`); the mission's `altitude` is only the target it holds.

## Start-up (drone laptop)
Terminals 1-3 and 5 as before (PX4, XRCE agent, lidar bridge, `maze_bringup ... with_nav:=false`). Then:

```bash
# T6  flight stack (adds the rangefinder bridge; takeoff_alt is only the default height)
ros2 launch wingman_exploration mission.launch.py vlm:=false takeoff_alt:=0.75

# T7  everything the website uses: camera bridge, rosbridge :9090, video :8080, web_gateway
ros2 launch wingman_exploration website.launch.py
#   already running your own rosbridge / web_video_server / camera bridge? switch those parts off:
#   ros2 launch wingman_exploration website.launch.py rosbridge:=false video:=false camera_bridge:=false
```
Do not start the camera bridge (old Step 4) and `camera_bridge:=true` at the same time.
Drone laptop IP: `hostname -I` (e.g. 192.168.128.220). Allow ports 9090 and 8080 in the firewall if needed.

## Command JSON (website -> drone), publish to `/wingman/llm_command` (std_msgs/String, the JSON text in `data`)
```json
{"id": "42", "type": "explore", "then": "return", "altitude": 0.75}
{"id": "43", "type": "find", "target": "person", "then": "return", "altitude": 0.75}
{"id": "44", "type": "goto", "xy": [4, -10], "then": "hover", "altitude": 0.75}
{"id": "45", "type": "return"}      {"id": "46", "type": "land"}      {"id": "47", "type": "hover"}
{"id": "48", "prompt": "explore the maze and return to home while maintaining height of 0.75 m"}
```
- `type`: explore | find | goto | return | takeoff | land | hover
- `then`: return (fly back to the start and land) | land (land here) | hover | continue
- `altitude`: metres above the floor, clamped to 0.4-2.0. Omit it to keep the current/default height.
- `id`: any string; the same id inside 5 s is ignored (safe to retry). Unknown keys are dropped.
- "explore the maze and return home at 0.75 m" == `{"type":"explore","then":"return","altitude":0.75}`.

Ack on `/wingman/llm_command_ack`: `{"id":"42","accepted":true,"error":"","mission":{...}}` or `accepted:false` with the reason.

### Prompt for the friend's LLM
> You control a drone. Reply with ONE JSON object and nothing else, no markdown. Keys: id (string), type
> (explore|find|goto|return|takeoff|land|hover), target (string, for find), xy ([x,y], for goto), then
> (return|land|hover|continue), altitude (metres, 0.4-2.0). "return home" means then:"return".

## Telemetry (drone -> website), `/wingman/telemetry`, 2 Hz, one small JSON
`flight_state, height_m (rangefinder), target_height_m, range_ok, baro_height_m, ready, mission_state, mission{type,target,then,altitude}, pose[x,y,yaw_deg], goal, found, elapsed_s, last, stale{flight,mission}`
`height_m` is the real height above the floor; if `range_ok` is false the controller has fallen back to the barometer.

## roslibjs (website)
```js
const ros = new ROSLIB.Ros({ url: 'ws://192.168.128.220:9090' });
const cmd  = new ROSLIB.Topic({ ros, name: '/wingman/llm_command', messageType: 'std_msgs/String' });
const ack  = new ROSLIB.Topic({ ros, name: '/wingman/llm_command_ack', messageType: 'std_msgs/String' });
const tel  = new ROSLIB.Topic({ ros, name: '/wingman/telemetry', messageType: 'std_msgs/String', throttle_rate: 500 });
const map  = new ROSLIB.Topic({ ros, name: '/map', messageType: 'nav_msgs/OccupancyGrid',
                                throttle_rate: 2000, queue_length: 1, compression: 'png' });
const path = new ROSLIB.Topic({ ros, name: '/wingman/planner_debug/path', messageType: 'nav_msgs/Path', throttle_rate: 1000 });

const send = (obj) => cmd.publish(new ROSLIB.Message({ data: JSON.stringify(obj) }));
ack.subscribe(m => console.log('ack', JSON.parse(m.data)));
tel.subscribe(m => { const t = JSON.parse(m.data); /* t.height_m, t.target_height_m, t.mission_state ... */ });
send({ id: String(Date.now()), type: 'explore', then: 'return', altitude: 0.75 });
```
Camera: `<img src="http://192.168.128.220:8080/stream?topic=/world/maze_exploration/model/x500_gps_denied_0/link/camera_link/sensor/IMX214/image&type=mjpeg&quality=60">`

## Test without the website
```bash
ros2 topic echo /wingman/llm_command_ack &
ros2 topic pub --once /wingman/llm_command std_msgs/msg/String \
  "{data: '{\"id\":\"t1\",\"type\":\"explore\",\"then\":\"return\",\"altitude\":0.75}'}"
ros2 topic echo /wingman/telemetry --field height_m      # should settle near 0.75
ros2 topic echo /wingman/range --once                    # rangefinder alive?
```
Stop at any time: `{"type":"hover"}` or `{"type":"land"}`.
