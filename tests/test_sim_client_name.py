from unittest.mock import Mock

import pytest

from agrotechsimapi.client import SimClient


def make_client_with_name_response(response):
    client = object.__new__(SimClient)
    client.rpc_client = Mock()
    client.rpc_client.call.return_value = response
    client.drone_name = None
    return client


def test_get_drone_name_returns_and_caches_rpc_name():
    client = make_client_with_name_response("QUADCOPTER_X")

    result = client.get_drone_name()

    assert result == "QUADCOPTER_X"
    assert client.drone_name == "QUADCOPTER_X"
    client.rpc_client.call.assert_called_once_with("getName")


def test_get_drone_name_decodes_utf8_bytes_response():
    client = make_client_with_name_response("Дрон".encode("utf-8"))

    assert client.get_drone_name() == "Дрон"


def test_get_drone_name_rejects_non_string_response():
    client = make_client_with_name_response(42)

    with pytest.raises(RuntimeError, match="getName must return a string"):
        client.get_drone_name()
