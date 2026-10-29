# WingMan code review and staged VSLAM migration

## Scope of this review

The newly uploaded `/mnt/data/wingman.zip` contained only an empty `ws_wingman/` directory. This review therefore uses the available `install_maze_stack.sh`, `wingman_garudanex_update.zip`, and the additional `files.zip` snippets. The current live `~/ws_wingman` tree was not available, so this is a review of the supplied source artifacts, not a claim that the local machine has been patched.

## Architecture decision

The staged order should be:

1. **Camera input validation** — verify image/depth/camera-info types, stamps, camera intrinsics, depth registration, and TF from `base_link` to the camera frame.
2. **RGB-D visual odometry only** — run RTAB-Map `rgbd_odometry` in isolation, output `/odom_vslam` and `/vslam/odom_info`, and verify changing position during controlled Gazebo motion and stable position while stationary.
3. **PX4 estimator integration** — only after step 2 is reliable, enable the guarded external-odometry bridge and verify PX4 accepts it; inspect PX4 local position, yaw/heading validity, estimator warnings, and reset behavior.
4. **RTAB-Map mapping** — add the mapping node as a consumer of the proven odometry plus synchronized RGB-D images. Check map consistency, loop closures, and `map -> vslam_odom` TF.
5. **Navigation and local obstacle avoidance** — bring up Nav2 with a suitable 2D occupancy map/costmaps and a flight-specific controller that respects vertical clearance, limits speed/acceleration, and handles lost perception.
6. **Maze exploration manager** — frontier/path-cost/information-gain selection, coverage memory, target detection, and return to the entry point.
7. **LLM/VLM** — high-level mission intent only; it must not bypass deterministic safety/controller gates.

RTAB-Map's ROS packages explicitly separate `rgbd_odometry` (continuous local pose estimation) and `rtabmap` (mapping/loop closure). This is the reason to prove odometry first, then enable mapping as a separate step.

## Issues found in supplied files

### Critical: the active launch does not implement the intended visual-odometry pipeline

`install_maze_stack.sh` generates `lidar_slam.launch.py`; `maze_bringup.launch.py` launches it, starts `icp_odometry`, then launches the guarded bridge, Nav2, controller and explorer. That is the current LiDAR-ICP flow, not a VSLAM-first flow.

### Critical: two different visual/ICP odometry arrangements are floating around

The extra `vslam.launch.py` starts `rgbd_odometry` and `rtabmap`, but hard-codes `/rgbd/image`, `/rgbd/depth_image`, `/rgbd/camera_info` and a static transform to `camera_optical`. The available run logs instead mention Gazebo-native camera topics and `/depth_camera/points`. These are not automatically interchangeable; PointCloud2 is not a depth Image. The exact live topic types and frame IDs must be confirmed before using the RGB-D launch.

Do not start `vslam.launch.py` alongside `maze_bringup.launch.py`: both odometry designs use the same `/odom` name in the supplied snippets, which risks multiple publishers and conflicting TF.

### Critical: PX4 bridge health gates are ICP-specific

The supplied guarded `vio_to_px4.py` and `cmdvel_to_px4.py` rely on `icp_correspondences` and `icp_inliers_ratio`. Visual odometry should be gated using visual tracking metrics (`features`, `matches`, `inliers`) and covariance/staleness checks. This review bundle includes a candidate configurable bridge, but it has only been syntax-checked and must not be enabled until ROS message/interface compatibility is checked on the target workspace.

The existing code also sets `VehicleOdometry.quality = 100` for every accepted message, overstating confidence. The candidate bridge derives a bounded quality value from inlier/match ratio instead.

### Critical: local-position topic mismatch

Earlier runtime diagnostics showed that `/fmu/out/vehicle_local_position_v1` publishes while `/fmu/out/vehicle_local_position` had no publisher. The supplied controller subscribes to the latter. Confirm the message type with `ros2 topic type /fmu/out/vehicle_local_position_v1` and update the controller's topic and imported PX4 message type to match the installed PX4/px4_msgs interface. Do not assume the `_v1` topic has an identical type without checking.

### Serious: ICP status from past logs does not establish working tracking

Past `/odom_info` showed `lost: false` but `inliers: 0`, `matches: 0`, `local_map_size: 0` and zero distance travelled. `lost: false` alone is not a success criterion. Any future health gate should use nonzero/adequate inliers plus an actual pose change test.

### Serious: controller and exploration are started before localization is validated

The full bringup uses launch timers to start all the control/navigation layers. Timer delays do not prove odometry is ready. Stage 1 must launch odometry only. Do not start the takeoff service or arm the vehicle while debugging the localization chain.

### Serious: command ownership

The earlier diagnostics reported five publishers on `/cmd_vel`. Before Nav2 integration, identify each publisher and retain one intentional command source at a time. Avoid running an old `drone_agent`, duplicate controller, and Nav2 controller simultaneously.

### TF is not yet validated

The separate RGB-D launch hard-codes a camera extrinsic. That transform must be taken from the actual SDF/model mount pose and must use the actual camera frame ID. Never add a second static transform if the simulator already publishes the transform; never guess the optical-frame rotation.

## New files in this review bundle

- `launch/vslam_odometry.launch.py`: odometry-only stage, with configurable RGB, depth, CameraInfo, base and odom names. It publishes to `/odom_vslam` and `/vslam/odom_info`; it does not start mapping, Nav2, or PX4 control.
- `launch/rtabmap_mapping.launch.py`: separate mapping stage; consumes `/odom_vslam` and RGB-D streams; it does not start another odometry producer.
- `scripts/check_vslam_inputs.sh`: read-only topic-type/frequency/topic-endpoint audit.
- `wingman_control/vio_to_px4.py`: candidate configurable bridge with separate visual/ICP health modes. It is not installed into the active workspace by this bundle.

## Verification before using the new launch

1. Run `scripts/check_vslam_inputs.sh` against the actual camera topics.
2. Verify RGB and depth are both `sensor_msgs/msg/Image`; CameraInfo is `sensor_msgs/msg/CameraInfo`; depth is registered to RGB.
3. Verify message frame IDs and camera intrinsics. Inspect the camera SDF/model and confirm a correct TF to `base_link`.
4. Launch only `vslam_odometry.launch.py`. Do not launch the old `maze_bringup.launch.py` simultaneously.
5. Inspect `/odom_vslam` and `/vslam/odom_info`; the odometry must be fresh and reflect controlled motion, and stop drifting unacceptably while stationary.
6. Only after this, check and enable the guarded PX4 odometry bridge. Confirm estimator fusion using PX4 status/logs.
7. Run `rtabmap_mapping.launch.py`; inspect the occupancy map and `map -> vslam_odom` TF.
8. Integrate Nav2 and the flight controller last.

## Installation caution

This bundle is an isolated staged replacement, not a drop-in replacement for the complete live workspace. The topic names have defaults matching the supplied snippet but must be overridden with the actual verified camera topics. It cannot fix a missing depth image or incorrect SDF sensor configuration automatically.
