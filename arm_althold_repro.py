"""Reproduce the legacy high-level ARM -> NAV ALTHOLD RC sequence.

This script deliberately performs no MSP configuration reads or writes.  It
only sends the same 50 Hz RC frames that the previously working
``HighLevelSimClient`` used:

1. neutral DISARM for two seconds;
2. neutral ARM for 0.2 seconds;
3. neutral ARM with NAV ALTHOLD selected on AUX3.

The NAV ALTHOLD AUX range must already exist in the flight-controller EEPROM.
Run from the repository root with ``python arm_althold_repro.py``.
"""

from __future__ import annotations

import argparse
import sys
import time

from inavmspapi import MultirotorControl, TCPTransmitter


# The frame order is roll, pitch, throttle, yaw, AUX1, AUX2, AUX3.
DISARM_NEUTRAL_FRAME = (1500, 1500, 1000, 1500, 1000, 1000, 1000)
ARMED_NEUTRAL_FRAME = (1500, 1500, 1000, 1500, 2000, 1000, 1000)
ARMED_ALTHOLD_FRAME = (1500, 1500, 1000, 1500, 2000, 1000, 1300)


def send_frame(control: MultirotorControl, frame: tuple[int, ...]) -> None:
    """Send one RC frame and consume its response, as the RC timer did."""
    if not control.send_RAW_RC(list(frame)):
        raise ConnectionError("MSP did not accept the RC frame")
    if control.receive_msg() is None:
        raise ConnectionError("MSP did not return an RC-frame response")


def hold_frame(
    control: MultirotorControl,
    frame: tuple[int, ...],
    duration: float,
    frequency: float,
) -> None:
    """Transmit an unchanged RC frame at a fixed frequency."""
    deadline = time.monotonic() + duration
    period = 1.0 / frequency
    while time.monotonic() < deadline:
        send_frame(control, frame)
        time.sleep(period)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Reproduce legacy high-level ARM -> NAV ALTHOLD RC sequence"
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5762)
    parser.add_argument("--frequency", type=float, default=50.0)
    parser.add_argument("--prearm-seconds", type=float, default=2.0)
    parser.add_argument("--arm-seconds", type=float, default=0.2)
    args = parser.parse_args()

    if args.frequency <= 0:
        parser.error("--frequency must be positive")
    if args.prearm_seconds < 0 or args.arm_seconds < 0:
        parser.error("frame durations must not be negative")

    transmitter = TCPTransmitter((args.host, args.port))
    transmitter.connect()
    if not transmitter.is_connect:
        raise ConnectionError(f"Could not connect to MSP at {args.host}:{args.port}")

    control = MultirotorControl(transmitter)
    print(f"[info] Connected to MSP at {args.host}:{args.port}")

    try:
        print(
            "[rc] Pre-arm neutral/disarm "
            f"for {args.prearm_seconds:g} s at {args.frequency:g} Hz: "
            f"{list(DISARM_NEUTRAL_FRAME)}"
        )
        hold_frame(control, DISARM_NEUTRAL_FRAME, args.prearm_seconds, args.frequency)

        print(
            "[rc] Neutral ARM "
            f"for {args.arm_seconds:g} s at {args.frequency:g} Hz: "
            f"{list(ARMED_NEUTRAL_FRAME)}"
        )
        hold_frame(control, ARMED_NEUTRAL_FRAME, args.arm_seconds, args.frequency)

        print(f"[rc] NAV ALTHOLD on AUX3=1300: {list(ARMED_ALTHOLD_FRAME)}")
        print("[info] Holding NAV ALTHOLD frame. Press Ctrl+C to disarm.")
        hold_frame(control, ARMED_ALTHOLD_FRAME, float("inf"), args.frequency)
    except KeyboardInterrupt:
        print("\n[info] Ctrl+C received")
    finally:
        try:
            print(f"[rc] Disarm: {list(DISARM_NEUTRAL_FRAME)}")
            hold_frame(control, DISARM_NEUTRAL_FRAME, 0.2, args.frequency)
        except Exception as exc:
            print(f"[warning] Could not send disarm frame: {exc}")
        finally:
            transmitter.disconnect()


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"[error] {exc}", file=sys.stderr)
        raise SystemExit(1) from exc
