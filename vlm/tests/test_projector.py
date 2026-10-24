import numpy as np
from wingman_vlm.projector import Intrinsics, Pose, project_goal

K = Intrinsics(fx=300.0, fy=300.0, cx=160.0, cy=120.0)
depth = np.full((240, 320), 4.0, dtype=np.float32)    # wall 4 m away


def test_center_pixel_is_straight_ahead():
    g = project_goal(500, 500, depth, K, Pose(), standoff_m=0.0)
    assert g.ok
    assert abs(g.world[0] - 4.0) < 0.05     # 4 m forward
    assert abs(g.world[1]) < 0.1            # no sideways offset


def test_right_side_pixel_goes_right():
    g = project_goal(900, 500, depth, K, Pose(), standoff_m=0.0)
    assert g.world[1] < -0.5                # ROS: right = negative y


def test_left_side_pixel_goes_left():
    g = project_goal(100, 500, depth, K, Pose(), standoff_m=0.0)
    assert g.world[1] > 0.5


def test_standoff_stops_short():
    g = project_goal(500, 500, depth, K, Pose(), standoff_m=1.0)
    assert abs(g.world[0] - 3.0) < 0.1


def test_drone_pose_is_applied():
    g = project_goal(500, 500, depth, K, Pose(x=10, y=5, z=1), standoff_m=0.0)
    assert abs(g.world[0] - 14.0) < 0.1 and abs(g.world[1] - 5.0) < 0.1


def test_invalid_depth_is_rejected():
    bad = np.zeros((240, 320), dtype=np.float32)       # no valid readings
    assert not project_goal(500, 500, bad, K, Pose()).ok


def test_drone_yawed_90_degrees_left():
    # yaw +90 deg: quaternion (0, 0, sin45, cos45); forward becomes world +y
    q = np.sin(np.pi / 4)
    g = project_goal(500, 500, depth, K,
                     Pose(qz=q, qw=q), standoff_m=0.0)
    assert abs(g.world[1] - 4.0) < 0.1 and abs(g.world[0]) < 0.1
