import struct
from unittest.mock import Mock, patch

import numpy as np

from agrotechsimapi.client import SimClient


def make_client_with_lidar_response(response):
    client = object.__new__(SimClient)
    client.rpc_client = Mock()
    client.rpc_client.call.return_value = response
    return client


def test_get_laser_scan_replaces_no_hit_minimum_readings_with_maximum_range():
    client = make_client_with_lidar_response([0.0, 0.1, 1.25, 3.5])

    result = client.get_laser_scan(range_max=10.0, num_ranges=4, is_clear=True)

    np.testing.assert_array_equal(result, np.array([10.0, 10.0, 1.25, 3.5]))
    client.rpc_client.call.assert_called_once_with(
        'getLaserScan', -np.pi / 2, np.pi / 2, 0.1, 10.0, 4
    )


def test_get_laser_scan_adds_noise_after_no_hit_normalization():
    client = make_client_with_lidar_response([0.1, 1.0])

    with patch('agrotechsimapi.client.np.random.normal', return_value=np.array([0.2, -0.1])):
        result = client.get_laser_scan(range_max=5.0, num_ranges=2, is_clear=False)

    np.testing.assert_allclose(result, np.array([5.2, 0.9]))


def test_get_lidar_point_cloud_decodes_pointcloud2_compatible_payload():
    fields = [
        ["x", 0, 7, 1],
        ["y", 4, 7, 1],
        ["z", 8, 7, 1],
        ["intensity", 12, 7, 1],
        ["ring", 16, 4, 1],
        ["return_type", 18, 2, 1],
        ["valid", 19, 2, 1],
        ["time_offset_ns", 20, 6, 1],
    ]
    data = b"".join([
        struct.pack("<ffffHBBI", 1.0, 2.0, 3.0, 0.0, 0, 0, 1, 10),
        struct.pack("<ffffHBBI", np.nan, np.nan, np.nan, 0.0, 0, 0, 0, 20),
    ])
    client = make_client_with_lidar_response([
        True, "", 2, 0, 7, "lidar_0_link", 12, 34, 50_000_000,
        1, 2, fields, False, 24, 48, data, False, 1, False,
        [0.1, -0.2, 0.3], [0.0, 0.0, 0.0, -1.0],
    ])

    cloud = client.get_lidar_point_cloud(
        angle_below_zero=0.1,
        angle_above_zero=0.2,
        range_min=0.3,
        range_max=40.0,
        channel_count=1,
        points_per_channel=2,
    )

    assert cloud.points.shape == (1, 2, 3)
    np.testing.assert_array_equal(cloud.valid, [[True, False]])
    np.testing.assert_allclose(cloud.valid_points, [[1.0, 2.0, 3.0]])
    np.testing.assert_array_equal(cloud.time_offset_ns, [[10, 20]])
    assert cloud.timestamp_ns == 12_000_000_034
    np.testing.assert_allclose(cloud.sensor_position_body, [0.1, -0.2, 0.3])
    np.testing.assert_allclose(cloud.sensor_orientation_body, [0.0, 0.0, 0.0, -1.0])
    assert cloud.data == data
    client.rpc_client.call.assert_called_once_with(
        "getLidarPointCloud", 0.1, 0.2, 0.3, 40.0, 1, 2
    )


def test_get_lidar_point_cloud_raises_simulator_error():
    client = make_client_with_lidar_response({
        "success": False,
        "error": "No 3D lidar is attached to socket Lidar3D0",
    })

    with np.testing.assert_raises_regex(RuntimeError, "Lidar3D0"):
        client.get_lidar_point_cloud()
