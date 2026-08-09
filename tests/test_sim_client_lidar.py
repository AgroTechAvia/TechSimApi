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
