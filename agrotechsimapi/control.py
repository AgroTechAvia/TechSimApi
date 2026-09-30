"""Shared flight control law used by calibration and the public client.

Legacy height/yaw/position/velocity gains use errors per control tick.
Acceleration uses a time integral with a bounded PWM contribution.
"""
from __future__ import annotations
import copy
import math
from dataclasses import dataclass
from typing import Any
from .pid import PID

PID_NAMES = (
    "pid_height", "pid_yaw", "pid_vel_pitch", "pid_vel_roll",
    "pid_accel_pitch", "pid_accel_roll", "pid_pos_x", "pid_pos_y",
)
STAGES = ("height", "yaw", "acceleration", "velocity", "position")
# Rates are configured per run. Defaults match the measured TechSim
# kinematics cadence: acceleration/kinematics 55 Hz, velocity 25 Hz,
# position 15 Hz.
def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(value, high))


def wrap_pi(value: float) -> float:
    return (value + math.pi) % (2 * math.pi) - math.pi


def acceleration_pid_defaults(config: dict[str, Any]) -> dict[str, dict[str, Any]]:
    """Standalone defaults; library presets deliberately do not contain these PIDs."""
    limit = config["max_accel_rc_offset"] / 100
    return {
        "pid_accel_pitch": {"kp": 1.0, "ki": 0.0, "kd": 0.02,
                            "max_control": limit, "i_limit": None},
        "pid_accel_roll": {"kp": 1.0, "ki": 0.0, "kd": 0.02,
                           "max_control": limit, "i_limit": None},
    }


class TimedAccelerationPID(PID):
    """Acceleration PID with a time integral and bounded I contribution.

    P and D retain the library PID's per-tick definitions so existing gains
    remain comparable. Only the standalone acceleration integral changes.
    """

    def __init__(self, *args: Any, integral_output_limit: float, **kwargs: Any):
        kwargs["i_limit"] = None  # Legacy tick-based integral limits do not apply.
        super().__init__(*args, **kwargs)
        self.integral_output_limit = integral_output_limit

    def update_control(self, current_error: float, *, dt: float,
                       reset_prev: bool = False) -> None:
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError("Acceleration PID dt must be finite and positive")
        if reset_prev:
            self.reset()
        self.previous_error = self.current_error
        self.current_error = current_error
        error = self._apply_nonlinearity(current_error)
        previous = self._apply_nonlinearity(self.previous_error)
        self.derivative = error - previous
        if self.ki:
            integral_limit = self.integral_output_limit / abs(self.ki)
            proposed = clamp(self.integral + error * dt,
                             -integral_limit, integral_limit)
            tentative = self.kp * error + self.ki * proposed + self.kd * self.derivative
            integral_change = self.ki * (proposed - self.integral)
            # Do not accumulate an I term that pushes an already saturated
            # actuator farther into the limit.
            if not ((tentative > self.max_control and integral_change > 0) or
                    (tentative < -self.max_control and integral_change < 0)):
                self.integral = proposed
        else:
            self.integral = 0.0
        raw = self.kp * error + self.ki * self.integral + self.kd * self.derivative
        self.control = self._processing_func(
            clamp(raw, -self.max_control, self.max_control))


@dataclass(frozen=True)
class Sample:
    t: float
    x: float
    y: float
    z: float
    yaw: float
    roll: float = 0.0
    pitch: float = 0.0
    # Simulator linear_velocity is expressed in the world frame, in m/s.
    # None is retained only for offline fixtures and the coordinate fallback.
    vx_world: float | None = None
    vy_world: float | None = None
    vz_world: float | None = None


class CascadedController:
    """Cascaded controller with explicit per-stage loop masks."""

    def __init__(self, config: dict[str, Any], pid_configs: dict[str, dict[str, Any]]):
        self.config = config
        self.base_throttle_rc = config["height_base_throttle_rc"]
        self.set_configs(pid_configs)

    def set_configs(self, pid_configs: dict[str, dict[str, Any]]) -> None:
        self.pid_configs = copy.deepcopy(pid_configs)
        for name, values in acceleration_pid_defaults(self.config).items():
            self.pid_configs.setdefault(name, values)
            self.pid_configs[name]["i_limit"] = None
            self.pid_configs[name]["max_control"] = min(
                self.pid_configs[name].get("max_control", math.inf), values["max_control"])
        self.pids = {
            name: (TimedAccelerationPID(
                **self.pid_configs[name],
                integral_output_limit=self.config["acceleration_i_max_rc_offset"] / 100)
                if name in ("pid_accel_pitch", "pid_accel_roll") else
                PID(**self.pid_configs[name]))
            for name in PID_NAMES
        }
        self.reset(None)

    def reset(self, sample: Sample | None) -> None:
        for pid in self.pids.values():
            pid.reset()
        self.previous_xy = None if sample is None else (sample.x, sample.y)
        self.previous_xy_t = None if sample is None else sample.t
        self.filtered_vx = 0.0
        self.filtered_vy = 0.0
        self.filter_initialized = False
        self.previous_target_vx = 0.0
        self.previous_target_vy = 0.0
        self.target_body_velocity = (0.0, 0.0)
        self.target_body_acceleration = (0.0, 0.0)
        self.filtered_ax = 0.0
        self.filtered_ay = 0.0
        self.previous_body_velocity: tuple[float, float] | None = None
        self.previous_body_velocity_t: float | None = None
        self.last_position = -math.inf
        self.last_yaw = -math.inf
        self.last_height = -math.inf
        self.last_velocity = -math.inf
        self.last_acceleration = -math.inf
        self.rc = [1500, 1500, 1500, 1500]
        self.last_actual_body_velocity = (0.0, 0.0)
        if not hasattr(self, "z_bias"):
            self.z_bias = 0.0

    def step(self, sample: Sample, stage: str, target: dict[str, Any]) -> dict[str, Any]:
        if stage not in (*STAGES, "validation", "braking"):
            raise ValueError(f"Unknown stage {stage}")
        c = self.config
        acceleration_period = 1 / c["acceleration_control_hz"]
        velocity_period = 1 / c["velocity_control_hz"]
        position_period = 1 / c["position_control_hz"]
        x, y, z, yaw = sample.x, sample.y, sample.z, sample.yaw
        cs, sn = math.cos(yaw), math.sin(yaw)
        height_on = target.get("height_enabled", True)
        yaw_on = stage in ("yaw", "acceleration", "velocity", "position",
                           "validation", "braking")
        yaw_on = yaw_on and target.get("yaw_enabled", True)
        velocity_on = stage in ("velocity", "position", "validation")
        acceleration_on = stage in ("acceleration", "velocity", "position", "validation")
        position_on = stage in ("position", "validation")
        active_axis = target.get("active_axis")
        if active_axis not in (None, "x", "y"):
            raise ValueError(f"Unknown active axis {active_axis}")
        saturated = False

        if sample.t - self.last_height >= acceleration_period - 1e-6 and height_on:
            error = target["height"] - z
            if target.get("height_relative") or target["height"] < 0.3:
                error += self.z_bias
            self.pids["pid_height"].update_control(error)
            offset = int(self.pids["pid_height"].get_control() * 100)
            bounded = int(clamp(offset, -c["max_height_rc_offset"], c["max_height_rc_offset"]))
            saturated |= bounded != offset
            self.rc[2] = int(clamp(self.base_throttle_rc + bounded, 1200, 1800))
            self.last_height = sample.t

        if yaw_on and sample.t - self.last_yaw >= 0.04 - 1e-6:
            error = wrap_pi(target["yaw"] - yaw)
            self.pids["pid_yaw"].update_control(error)
            offset = self.pids["pid_yaw"].get_control() * 100 * c["direction"]["yaw"]
            bounded = clamp(offset, -c["max_yaw_rc_offset"], c["max_yaw_rc_offset"])
            saturated |= bounded != offset
            self.rc[3] = int(1500 + bounded)
            self.last_yaw = sample.t
        elif not yaw_on:
            self.rc[3] = 1500

        if position_on and sample.t - self.last_position >= position_period - 1e-6:
            tx, ty = target["position"]
            if active_axis != "y":
                self.pids["pid_pos_x"].update_control(tx - x)
            if active_axis != "x":
                self.pids["pid_pos_y"].update_control(ty - y)
            vx_world = (clamp(self.pids["pid_pos_x"].get_control(),
                              -c["max_xy_speed"], c["max_xy_speed"])
                        if active_axis != "y" else 0.0)
            vy_world = (clamp(self.pids["pid_pos_y"].get_control(),
                              -c["max_xy_speed"], c["max_xy_speed"])
                        if active_axis != "x" else 0.0)
            # Position PIDs operate in world X/Y, exactly as in the client.
            # Even an isolated world axis generally needs both body velocity
            # controllers when the vehicle starts at a non-zero heading.
            self.target_body_velocity = (vx_world * cs - vy_world * sn,
                                         vx_world * sn + vy_world * cs)
            # Keep the requested 15 Hz position cadence even though samples
            # arrive faster from the 55 Hz kinematics loop.
            if not math.isfinite(self.last_position) or sample.t - self.last_position > 0.1:
                self.last_position = sample.t
            else:
                self.last_position += position_period
        elif velocity_on and not position_on:
            self.target_body_velocity = target["velocity"]
        elif stage == "acceleration":
            self.target_body_acceleration = target["acceleration"]

        # Physical velocity and acceleration are updated at the inner-loop
        # rate. Real flights use the simulator's world-frame linear velocity.
        # Position differentiation remains only for offline fixtures or a
        # legacy simulator reply which lacks linear_velocity.
        if acceleration_on:
            if sample.vx_world is not None and sample.vy_world is not None:
                raw_vx, raw_vy = sample.vx_world, sample.vy_world
                if not self.filter_initialized:
                    self.filtered_vx, self.filtered_vy = raw_vx, raw_vy
                    self.filter_initialized = True
                else:
                    alpha = c["velocity_filter_alpha"]
                    self.filtered_vx = alpha * raw_vx + (1 - alpha) * self.filtered_vx
                    self.filtered_vy = alpha * raw_vy + (1 - alpha) * self.filtered_vy
            else:
                measurement_dt = (sample.t - self.previous_xy_t
                                  if self.previous_xy_t is not None else 0.0)
                if self.previous_xy is not None and measurement_dt > 1e-6:
                    raw_vx = (x - self.previous_xy[0]) / measurement_dt
                    raw_vy = (y - self.previous_xy[1]) / measurement_dt
                    if not self.filter_initialized:
                        self.filtered_vx, self.filtered_vy = raw_vx, raw_vy
                        self.filter_initialized = True
                    else:
                        alpha = c["velocity_filter_alpha"]
                        self.filtered_vx = alpha * raw_vx + (1 - alpha) * self.filtered_vx
                        self.filtered_vy = alpha * raw_vy + (1 - alpha) * self.filtered_vy
            self.previous_xy = (x, y)
            self.previous_xy_t = sample.t
            vx_body = self.filtered_vx * cs - self.filtered_vy * sn
            vy_body = self.filtered_vx * sn + self.filtered_vy * cs
            self.last_actual_body_velocity = (vx_body, vy_body)

            if (self.previous_body_velocity is not None and
                    self.previous_body_velocity_t is not None):
                dt = sample.t - self.previous_body_velocity_t
                if dt > 1e-6:
                    raw_ax = (vx_body - self.previous_body_velocity[0]) / dt
                    raw_ay = (vy_body - self.previous_body_velocity[1]) / dt
                    alpha = c["acceleration_filter_alpha"]
                    self.filtered_ax = alpha * raw_ax + (1 - alpha) * self.filtered_ax
                    self.filtered_ay = alpha * raw_ay + (1 - alpha) * self.filtered_ay
            self.previous_body_velocity = (vx_body, vy_body)
            self.previous_body_velocity_t = sample.t
            actual_acceleration = (self.filtered_ax, self.filtered_ay)

        if velocity_on and sample.t - self.last_velocity >= velocity_period - 1e-6:
            tvx, tvy = self.target_body_velocity
            if stage == "velocity" and active_axis == "x":
                tvy = 0.0
            elif stage == "velocity" and active_axis == "y":
                tvx = 0.0
            dvx, dvy = tvx - self.previous_target_vx, tvy - self.previous_target_vy
            control_dt = (sample.t - self.last_velocity
                          if math.isfinite(self.last_velocity) else velocity_period)
            acceleration = math.hypot(dvx, dvy) / max(control_dt, 1e-6)
            if stage != "velocity" and acceleration > c["max_xy_acceleration"]:
                scale = c["max_xy_acceleration"] / acceleration
                tvx = self.previous_target_vx + dvx * scale
                tvy = self.previous_target_vy + dvy * scale
            self.previous_target_vx, self.previous_target_vy = tvx, tvy

            velocity_axis = active_axis if stage == "velocity" else None
            if velocity_axis != "y":
                self.pids["pid_vel_pitch"].update_control(tvx - vx_body)
            if velocity_axis != "x":
                self.pids["pid_vel_roll"].update_control(tvy - vy_body)
            target_ax = (self.pids["pid_vel_pitch"].get_control()
                         if velocity_axis != "y" else 0.0)
            target_ay = (self.pids["pid_vel_roll"].get_control()
                         if velocity_axis != "x" else 0.0)
            self.target_body_acceleration = (
                clamp(target_ax, -c["max_xy_acceleration"], c["max_xy_acceleration"]),
                clamp(target_ay, -c["max_xy_acceleration"], c["max_xy_acceleration"]),
            )
            saturated |= (abs(target_ax) > c["max_xy_acceleration"] or
                          abs(target_ay) > c["max_xy_acceleration"])
            self.last_velocity = sample.t

        if acceleration_on and sample.t - self.last_acceleration >= acceleration_period - 1e-6:
            tax, tay = self.target_body_acceleration
            acceleration_axis = active_axis if stage in ("acceleration", "velocity") else None
            control_dt = (sample.t - self.last_acceleration
                          if math.isfinite(self.last_acceleration) else acceleration_period)
            if acceleration_axis != "y":
                self.pids["pid_accel_pitch"].update_control(
                    tax - self.filtered_ax, dt=control_dt)
            if acceleration_axis != "x":
                self.pids["pid_accel_roll"].update_control(
                    tay - self.filtered_ay, dt=control_dt)
            pitch = (self.pids["pid_accel_pitch"].get_control()
                     if acceleration_axis != "y" else 0.0)
            roll = (self.pids["pid_accel_roll"].get_control()
                    if acceleration_axis != "x" else 0.0)
            saturated |= (abs(pitch) >= self.pids["pid_accel_pitch"].max_control or
                          abs(roll) >= self.pids["pid_accel_roll"].max_control)
            self.rc[1] = int(clamp(1500 + pitch * c["direction"]["pitch"] * 100,
                                   1000, 2000))
            self.rc[0] = int(clamp(1500 + roll * c["direction"]["roll"] * 100,
                                   1000, 2000))
            self.last_acceleration = sample.t
        elif not acceleration_on:
            self.rc[0] = self.rc[1] = 1500
            self.last_actual_body_velocity = (0.0, 0.0)

        for pid in self.pids.values():
            raw = (pid.kp * pid.current_error + pid.ki * pid.integral
                   + pid.kd * pid.derivative)
            saturated |= abs(raw) > pid.max_control
        details = {}
        for name, pid in self.pids.items():
            details[f"{name}_error"] = pid.current_error
            details[f"{name}_output"] = pid.get_control()
            if isinstance(pid, TimedAccelerationPID):
                details[f"{name}_integral_error_seconds"] = pid.integral
                details[f"{name}_integral_output"] = pid.ki * pid.integral
                details[f"{name}_integral_limited"] = int(
                    abs(pid.ki * pid.integral) >= pid.integral_output_limit - 1e-9)
        return {
            "rc_roll": self.rc[0], "rc_pitch": self.rc[1],
            "rc_throttle": self.rc[2], "rc_yaw": self.rc[3],
            "vx_body": self.last_actual_body_velocity[0],
            "vy_body": self.last_actual_body_velocity[1],
            "target_vx_body": self.previous_target_vx,
            "target_vy_body": self.previous_target_vy,
            "ax_body": self.filtered_ax,
            "ay_body": self.filtered_ay,
            "target_ax_body": self.target_body_acceleration[0],
            "target_ay_body": self.target_body_acceleration[1],
            "saturated": int(saturated),
            **details,
        }


def sample_from_kinematics(kin: dict, timestamp: float) -> Sample:
    """Convert simulator world coordinates and CCW quaternion to control data."""
    from transforms3d.euler import quat2euler
    x, y, z = (float(v) for v in kin["location"][:3])
    velocity = kin.get("linear_velocity")
    if not isinstance(velocity, (list, tuple)) or len(velocity) < 3:
        raise ConnectionError("Missing simulator linear_velocity")
    vx, vy, vz = (float(v) for v in velocity[:3])
    qx, qy, qz, qw = kin["orientation"]
    roll, pitch, yaw = quat2euler((qw, qx, qy, qz), axes="sxyz")
    if not all(math.isfinite(v) for v in (x,y,z,vx,vy,vz,roll,pitch,yaw)):
        raise ConnectionError("Non-finite simulator kinematics")
    return Sample(timestamp, x, y, z, -yaw, roll, pitch, vx, vy, vz)
