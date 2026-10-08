from types import SimpleNamespace

import numpy as np

from examples_high_level.lidar.accumulate_3d_point_cloud import scan_to_map


def _kinematics(position=(0.0, 0.0, 0.0)):
    return {
        "location": list(position),
        "orientation": [0.0, 0.0, 0.0, 1.0],
    }


def test_scan_to_map_keeps_ros_positive_z_up_for_aligned_lidar():
    cloud = SimpleNamespace(
        valid_points=np.array([[0.0, 0.0, 1.0]]),
        valid=np.array([[True]]),
        time_offset_ns=np.array([[0]], dtype=np.uint32),
        scan_duration_ns=1,
        sensor_position_body=np.zeros(3),
        sensor_orientation_body=np.array([0.0, 0.0, 0.0, 1.0]),
    )

    result = scan_to_map(cloud, _kinematics((1.0, 2.0, 3.0)), _kinematics((1.0, 2.0, 3.0)))

    np.testing.assert_allclose(result, [[1.0, 2.0, 4.0]], atol=1e-6)


def test_scan_to_map_applies_lidar_socket_rotation_before_drone_pose():
    cloud = SimpleNamespace(
        valid_points=np.array([[0.0, 0.0, 1.0]]),
        valid=np.array([[True]]),
        time_offset_ns=np.array([[0]], dtype=np.uint32),
        scan_duration_ns=1,
        sensor_position_body=np.zeros(3),
        # 180 degrees around body X, stored as RPC quaternion (x, y, z, w).
        sensor_orientation_body=np.array([1.0, 0.0, 0.0, 0.0]),
    )

    result = scan_to_map(cloud, _kinematics(), _kinematics())

    np.testing.assert_allclose(result, [[0.0, 0.0, -1.0]], atol=1e-6)
