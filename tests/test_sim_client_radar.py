from unittest.mock import Mock, patch

import pytest

from agrotechsimapi.client import SimClient


def make_client_with_radar_response(response):
    client = object.__new__(SimClient)
    client.rpc_client = Mock()
    client.rpc_client.call.return_value = response
    return client


def test_get_radar_point_returns_scalar_distance_response():
    client = make_client_with_radar_response(1.25)

    result = client.get_radar_point(radar_id=2, base_angle=45, range_min=0.15, range_max=2.0)

    assert result == 1.25
    client.rpc_client.call.assert_called_once_with('getRadarData', 2, 45, 0.15, 2.0)


def test_get_radar_point_preserves_negative_no_target_distance():
    client = make_client_with_radar_response(-1.0)

    result = client.get_radar_point()

    assert result == -1.0


def test_get_radar_point_applies_noise_to_scalar_response():
    client = make_client_with_radar_response(2.0)

    with patch('agrotechsimapi.client.np.random.normal', return_value=0.1):
        result = client.get_radar_point(is_clear=False)

    assert result == pytest.approx(2.1)


def test_get_radar_point_rejects_vector_response():
    client = make_client_with_radar_response([1.0, 2.0])

    with pytest.raises(RuntimeError, match='single distance value'):
        client.get_radar_point()
