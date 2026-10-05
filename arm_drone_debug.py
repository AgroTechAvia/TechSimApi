"""Minimal MSP arming and flight-mode diagnostic for the TechSim drone.

Run from the repository root:
    python arm_drone_debug.py

The script keeps sending the armed RC frame until Ctrl+C, so the simulator has
time to show whether the flight controller accepts the ARM switch.
"""

import argparse
import sys
import time

from inavmspapi import MultirotorControl, TCPTransmitter
from inavmspapi.msp_codes import MSPCodes


DISARM_FRAME = [1000, 1000, 1000, 1000, 1000, 1000, 1000]
ARM_SWITCH_FRAME = [1000, 1000, 1000, 1000, 2000, 1000, 1000]
ARMED_NEUTRAL_FRAME = [1500, 1500, 1000, 1500, 2000, 1000, 1000]
POSHOLD_FRAME = [1500, 1500, 1000, 1500, 2000, 1000, 1500]
ALTHOLD_FRAME = [1500, 1500, 1000, 1500, 2000, 1000, 1300]

ALTHOLD_MODE_ID = 3
ALTHOLD_AUX_CHANNEL_INDEX = 2
ALTHOLD_RANGE_START = 1250
ALTHOLD_RANGE_END = 1350

INAV_ARMING_FLAG_NAMES = {
    2: "ARMED",
    3: "WAS_EVER_ARMED",
    4: "SIMULATOR_MODE_HITL",
    5: "SIMULATOR_MODE_SITL",
    6: "GEOZONE",
    7: "FAILSAFE_SYSTEM",
    8: "NOT_LEVEL",
    9: "SENSORS_CALIBRATING",
    10: "SYSTEM_OVERLOADED",
    11: "NAVIGATION_UNSAFE",
    12: "COMPASS_NOT_CALIBRATED",
    13: "ACCELEROMETER_NOT_CALIBRATED",
    14: "ARM_SWITCH",
    15: "HARDWARE_FAILURE",
    16: "BOXFAILSAFE",
    18: "RC_LINK",
    19: "THROTTLE",
    20: "CLI",
    21: "CMS_MENU",
    22: "OSD_MENU",
    23: "ROLLPITCH_NOT_CENTERED",
    26: "INVALID_SETTING",
}


def send_frame(
    control: MultirotorControl, frame: list[int], label: str, *, verbose: bool = True
) -> None:
    """Send an MSP RC frame and print its decoded response metadata."""
    if verbose:
        print(f"[rc] {label}: {frame}")
    result = control.send_RAW_RC(frame)
    if not result:
        raise ConnectionError("MSP did not accept the RC frame")

    response = control.receive_msg()
    if response is None:
        raise ConnectionError("MSP did not return a response to the RC frame")

    if verbose:
        print(
            "[msp] response: "
            f"code={response.get('code')}, "
            f"crc_error={response.get('crcError')}, "
            f"packet_error={response.get('packet_error')}"
        )


def request_msp(
    control: MultirotorControl, message_name: str, *, attempts: int = 3
) -> bool:
    """Request, decode, and retain one MSP telemetry message."""
    last_problem = "no response"
    for _ in range(attempts):
        result = control.send_RAW_msg(MSPCodes[message_name], data=[])
        if not result:
            last_problem = "controller did not accept request"
            continue

        response = control.receive_msg()
        if response is None:
            last_problem = "controller did not return a response"
            continue

        processed = control.process_recv_data(response)
        if processed is not None and processed >= 0:
            return True
        last_problem = f"response was not decoded ({processed})"

    print(f"[warning] {message_name}: {last_problem}")
    return False


def hold_frame(control: MultirotorControl, frame: list[int], duration: float, frequency: float) -> None:
    """Continuously send an RC frame for the requested interval."""
    deadline = time.monotonic() + duration
    period = 1.0 / frequency
    while time.monotonic() < deadline:
        send_frame(control, frame, "hold", verbose=False)
        time.sleep(period)


def print_arm_diagnostics(control: MultirotorControl) -> None:
    """Print the INAV state relevant to arming and NAV ALTHOLD."""
    print("[info] Reading INAV arming diagnostics...")
    request_msp(control, "MSP_BOXNAMES")
    request_msp(control, "MSP_BOXIDS")
    request_msp(control, "MSP_RC")
    request_msp(control, "MSP_STATUS")
    request_msp(control, "MSP_STATUS_EX")
    request_msp(control, "MSP_MODE_RANGES")
    sensor_config_received = request_msp(control, "MSP_SENSOR_CONFIG")
    altitude_received = request_msp(control, "MSP_ALTITUDE")

    channels = control.RC.get("channels", [])
    print(f"[diag] received RC channels: {channels[:8]}")

    status = control.CONFIG
    mode_mask = status.get("mode", 0)
    active_modes = control.process_mode(mode_mask)
    print(f"[diag] active modes: {active_modes}")
    print(f"[diag] configured modes (id, name): {list(zip(control.AUX_CONFIG_IDS, control.AUX_CONFIG))}")

    flags = status.get("armingDisableFlags")
    if flags is None:
        print("[diag] arming flags: unavailable")
    else:
        names = [
            name for bit, name in INAV_ARMING_FLAG_NAMES.items() if flags & (1 << bit)
        ]
        print(f"[diag] arming flags: {flags} ({', '.join(names) or 'none'})")
        print(f"[diag] controller armed: {bool(flags & (1 << 2))}")

    if sensor_config_received:
        print(f"[diag] configured altitude sensors: {control.SENSOR_CONFIG}")
    if altitude_received:
        print(f"[diag] MSP altitude: {control.SENSOR_DATA['altitude']} m")

    names_by_id = dict(zip(control.AUX_CONFIG_IDS, control.AUX_CONFIG))
    ranges = [
        {
            "mode": names_by_id.get(mode_range["id"], f"id={mode_range['id']}"),
            "aux_channel": mode_range["auxChannelIndex"] + 1,
            "range": mode_range["range"],
        }
        for mode_range in control.MODE_RANGES
    ]
    print(f"[diag] configured mode ranges: {ranges}")


def ensure_althold_range(control: MultirotorControl, *, persist: bool = False) -> bool:
    """Ensure NAV ALTHOLD is mapped to AUX3 at 1250–1350 µs.

    The range is used immediately after ``MSP_SET_MODE_RANGE``; persisting it
    in EEPROM is deliberately optional.  The simulator is initialized for
    every program run, so writing EEPROM is unnecessary for flight and makes
    it possible to isolate EEPROM-related MSP failures.
    """
    if not request_msp(control, "MSP_MODE_RANGES"):
        return False

    def is_althold(mode_range: dict) -> bool:
        return (
            mode_range["id"] == ALTHOLD_MODE_ID
            and mode_range["auxChannelIndex"] == ALTHOLD_AUX_CHANNEL_INDEX
            and mode_range["range"]["start"] == ALTHOLD_RANGE_START
            and mode_range["range"]["end"] == ALTHOLD_RANGE_END
        )

    if any(is_althold(mode_range) for mode_range in control.MODE_RANGES):
        print("[mode] NAV ALTHOLD range is already configured on AUX3")
        return True

    empty_index = next(
        (
            index
            for index, mode_range in enumerate(control.MODE_RANGES)
            if mode_range["id"] == 0
            and mode_range["range"] == {"start": 900, "end": 900}
        ),
        None,
    )
    if empty_index is None:
        print("[warning] No free MSP mode-range entry is available for NAV ALTHOLD")
        return False

    payload = [
        empty_index,
        ALTHOLD_MODE_ID,
        ALTHOLD_AUX_CHANNEL_INDEX,
        (ALTHOLD_RANGE_START - 900) // 25,
        (ALTHOLD_RANGE_END - 900) // 25,
    ]
    def send_config_write(message_name: str, data: list[int]) -> bool:
        if not control.send_RAW_msg(MSPCodes[message_name], data=data):
            print(f"[warning] {message_name} was not sent")
            return False
        response = control.receive_msg()
        if response is None:
            print(f"[warning] {message_name} did not return an ACK")
            return False
        processed = control.process_recv_data(response)
        if processed is None or processed < 0:
            print(f"[warning] {message_name} ACK was not decoded: {processed}")
            return False
        return True

    print("[mode] Adding NAV ALTHOLD range on AUX3: 1250-1350")
    if not send_config_write("MSP_SET_MODE_RANGE", payload):
        return False
    time.sleep(0.3)

    if not request_msp(control, "MSP_MODE_RANGES"):
        return False
    if not any(is_althold(mode_range) for mode_range in control.MODE_RANGES):
        print("[warning] NAV ALTHOLD range was not present after MSP_SET_MODE_RANGE")
        return False

    print("[mode] NAV ALTHOLD range configured in the active controller session")
    if not persist:
        return True

    print("[mode] Saving NAV ALTHOLD range to EEPROM...")
    if not send_config_write("MSP_EEPROM_WRITE", []):
        return False
    time.sleep(0.5)

    if not request_msp(control, "MSP_MODE_RANGES"):
        return False
    if any(is_althold(mode_range) for mode_range in control.MODE_RANGES):
        print("[mode] NAV ALTHOLD range persisted in EEPROM")
        return True

    print("[warning] NAV ALTHOLD range was not present after EEPROM write")
    return False


def select_flight_mode_frame(mode: str) -> list[int]:
    """Return the armed RC frame for the requested post-arm flight mode."""
    if mode == "none":
        return ARMED_NEUTRAL_FRAME
    if mode == "poshold":
        return POSHOLD_FRAME
    if mode == "althold":
        return ALTHOLD_FRAME
    raise ValueError(f"Unsupported flight mode: {mode}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Test whether adding NAV ALTHOLD affects ARM and NAV POSHOLD"
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5762)
    parser.add_argument("--frequency", type=float, default=20.0)
    parser.add_argument("--settle-seconds", type=float, default=1.0)
    parser.add_argument(
        "--mode",
        choices=("none", "poshold", "althold"),
        default="poshold",
        help="Flight mode to activate after arming (default: poshold)",
    )
    parser.add_argument(
        "--skip-althold-config",
        action="store_true",
        help="Do not add or verify the NAV ALTHOLD mode range (control experiment)",
    )
    parser.add_argument(
        "--persist-althold-config",
        action="store_true",
        help="Also save the newly added NAV ALTHOLD range to EEPROM",
    )
    args = parser.parse_args()

    if args.frequency <= 0:
        parser.error("--frequency must be positive")
    if args.settle_seconds < 0:
        parser.error("--settle-seconds must not be negative")

    transmitter = TCPTransmitter((args.host, args.port))
    transmitter.connect()
    if not transmitter.is_connect:
        raise ConnectionError(f"Could not connect to MSP at {args.host}:{args.port}")

    control = MultirotorControl(transmitter)
    print(f"[info] Connected to MSP at {args.host}:{args.port}")

    if request_msp(control, "MSP_FC_VARIANT"):
        identifier = control.CONFIG.get("flightControllerIdentifier", "")
        control.INAV = "INAV" in identifier
        print(f"[info] Flight controller: {identifier or 'unknown'} (INAV={control.INAV})")
    else:
        print("[warning] Could not identify flight controller; using default decoder")

    try:
        # This experiment deliberately writes the NAV ALTHOLD range *before*
        # arming, but keeps AUX3 in NAV POSHOLD afterwards.  It isolates the
        # configuration write from activating NAV ALTHOLD itself.
        if not args.skip_althold_config and not ensure_althold_range(
            control, persist=args.persist_althold_config
        ):
            raise RuntimeError("Could not configure NAV ALTHOLD range")

        send_frame(control, DISARM_FRAME, "reset/disarm")
        time.sleep(1)
        send_frame(control, ARM_SWITCH_FRAME, "arm switch on")
        # input_driver.py immediately follows the switch transition with this
        # neutral frame.  Keeping yaw/roll/pitch at 1000 can itself prevent
        # INAV from accepting the arm switch.
        send_frame(control, ARMED_NEUTRAL_FRAME, "armed neutral")
        hold_frame(control, ARMED_NEUTRAL_FRAME, args.settle_seconds, args.frequency)

        active_frame = select_flight_mode_frame(args.mode)
        if args.mode == "poshold":
            print("[mode] Enabling NAV POSHOLD (AUX3 = 1500)...")
            send_frame(control, active_frame, "NAV POSHOLD on")
            hold_frame(control, active_frame, args.settle_seconds, args.frequency)
        elif args.mode == "althold":
            print("[mode] Enabling NAV ALTHOLD (AUX3 = 1300)...")
            send_frame(control, active_frame, "NAV ALTHOLD on")
            hold_frame(control, active_frame, args.settle_seconds, args.frequency)

        print_arm_diagnostics(control)

        print(
            f"[info] Holding {args.mode.upper()} frame. "
            "Inspect the drone; press Ctrl+C to disarm."
        )
        period = 1.0 / args.frequency
        while True:
            send_frame(control, active_frame, "active mode", verbose=False)
            time.sleep(period)
    except KeyboardInterrupt:
        print("\n[info] Ctrl+C received")
    finally:
        try:
            send_frame(control, DISARM_FRAME, "disarm")
        except Exception as exc:
            print(f"[warning] Could not send disarm frame: {exc}")

        try:
            transmitter.disconnect()
        except Exception as exc:
            print(f"[warning] Could not close MSP connection: {exc}")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"[error] {exc}", file=sys.stderr)
        raise SystemExit(1) from exc
