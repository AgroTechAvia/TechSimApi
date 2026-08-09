import threading
from unittest.mock import Mock, patch

from agrotechsimapi.high_level_client import HighLevelSimClient


def test_arm_drone_uses_verified_raw_rc_sequence():
    client = object.__new__(HighLevelSimClient)
    client._armed_flag = False
    client._height_timer = Mock()
    client._arming_rc_frame = None
    client._send_rc_frame = Mock()
    client._hold_rc_frame = Mock()

    with patch("agrotechsimapi.high_level_client.time.sleep"):
        assert client.armDrone() is True

    assert [call.args[0] for call in client._send_rc_frame.call_args_list] == [
        client._ARM_RESET_FRAME,
        client._ARM_SWITCH_FRAME,
        client._ARMED_NEUTRAL_FRAME,
    ]
    assert client._rpy_vel_data == (1500, 1500, 1500)
    assert client._throttle_data == 1000
    assert client._arm_data == 2000
    assert client._arming_rc_frame is None
    client._hold_rc_frame.assert_called_once_with(
        client._ARMED_NEUTRAL_FRAME,
        duration=client._ARM_SETTLE_SECONDS,
        frequency=client._ARM_FRAME_FREQUENCY,
    )
    client._height_timer.start.assert_called_once_with()


def test_rc_timer_sends_arming_override_frame():
    client = object.__new__(HighLevelSimClient)
    client._simulator_alive = True
    client._msp_io_lock = threading.Lock()
    client._arming_rc_frame = client._ARMED_NEUTRAL_FRAME
    client._control = Mock()

    client.transmit_rc_to_sim()

    client._control.send_RAW_RC.assert_called_once_with(
        list(client._ARMED_NEUTRAL_FRAME)
    )


def test_hold_rc_frame_sends_direct_frames_at_requested_interval():
    client = object.__new__(HighLevelSimClient)
    client._send_rc_frame = Mock()

    with patch(
        "agrotechsimapi.high_level_client.time.monotonic",
        side_effect=[10.0, 10.0, 11.0],
    ), patch("agrotechsimapi.high_level_client.time.sleep") as sleep:
        client._hold_rc_frame((1, 2, 3), duration=1.0, frequency=20.0)

    client._send_rc_frame.assert_called_once_with((1, 2, 3))
    sleep.assert_called_once_with(0.05)


def test_althold_on_uses_aux3_value_and_sends_it_immediately():
    client = object.__new__(HighLevelSimClient)
    client._althold_range_configured = True
    client._poshold_flag = True
    client._althold_flag = False
    client._nav_mode = 1000
    client._base_throttle_hover = 1000
    client._rpy_vel_data = (1500, 1500, 1500)
    client._throttle_data = 1000
    client._arm_data = 2000
    client._fliyng_mode = 1000
    client._send_rc_frame = Mock()
    client.unlock_motors = Mock()

    client.altholdOn()

    assert client._poshold_flag is False
    assert client._althold_flag is True
    assert client._nav_mode == 1300
    client.unlock_motors.assert_called_once_with()
    client._send_rc_frame.assert_called_once_with(
        [1500, 1500, 1000, 1500, 2000, 1000, 1300]
    )


def test_add_range_for_althold_consumes_each_msp_response_and_verifies_result():
    client = object.__new__(HighLevelSimClient)
    client._msp_io_lock = threading.Lock()
    client._control = Mock()
    client._control.MODE_RANGES = []
    client._control.send_RAW_msg.return_value = True
    client._control.receive_msg.side_effect = [
        {"kind": "initial", "crcError": False, "packet_error": 0},
        {"kind": "set", "crcError": False, "packet_error": 0},
        {"kind": "eeprom", "crcError": False, "packet_error": 0},
        {"kind": "verified", "crcError": False, "packet_error": 0},
    ]

    empty_range = {"id": 0, "auxChannelIndex": 0, "range": {"start": 900, "end": 900}}
    configured_range = {
        "id": 3,
        "auxChannelIndex": 2,
        "range": {"start": 1250, "end": 1350},
    }

    def process_response(response):
        if response["kind"] == "initial":
            client._control.MODE_RANGES = [empty_range]
        elif response["kind"] == "verified":
            client._control.MODE_RANGES = [configured_range]
        return 0

    client._control.process_recv_data.side_effect = process_response

    assert client.add_range_for_althold() is True
    assert client._control.receive_msg.call_count == 4
    assert [call.args for call in client._control.send_RAW_msg.call_args_list] == [
        (34, []),
        (35, [0, 3, 2, 14, 18]),
        (250, []),
        (34, []),
    ]
