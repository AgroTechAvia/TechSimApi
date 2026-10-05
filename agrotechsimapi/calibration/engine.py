"""Simulator PID calibration engine. Also exposed by python -m agrotechsimapi.
"""

from __future__ import annotations

import argparse
import copy
import csv
import hashlib
import importlib.util
import json
import math
import os
import random
import sys
import threading
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from statistics import mean, median
from typing import Any, Callable

from .acceleration_trace import align_repeats, settled_band_response, smoothed_mean
from ..control import (PID_NAMES, STAGES, clamp, wrap_pi, acceleration_pid_defaults,
                       TimedAccelerationPID, Sample, CascadedController,
                       sample_from_kinematics)
from ..pid import PID
from ..utils.drone_setups import get_drone_pid_setup, get_drone_takeoff_height
from .paths import data_dir

CalibrationController = CascadedController
PACKAGE_ROOT = Path(__file__).resolve().parents[1]
ROOT = Path.cwd()  # Compatibility for callers supplying relative run paths.

RESET_FRAME = (1000, 1000, 1000, 1000, 1000, 1000, 1000)
ARM_SWITCH_FRAME = (1000, 1000, 1000, 1000, 2000, 1000, 1000)
ARMED_NEUTRAL_FRAME = (1500, 1500, 1000, 1500, 2000, 1000, 1000)
HOLD_FRAME = (1500, 1500, 1500, 1500, 2000, 1000, 1300)


def vertical_rate(rows: list[dict[str, Any]], window: float) -> float | None:
    """Measured Z velocity over a short window; None until enough data exists."""
    if len(rows) < 2:
        return None
    latest = rows[-1]
    older = next((row for row in reversed(rows[:-1])
                  if latest["t"] - row["t"] >= window), None)
    if older is None:
        return None
    return (latest["z"] - older["z"]) / (latest["t"] - older["t"])


def default_config() -> dict[str, Any]:
    """Conservative starting envelope; inspect it before an actual flight."""
    return {
        "drone_name": "DEFAULT",
        "host": "127.0.0.1",
        "sim_port": 8080,
        "msp_port": 5762,
        "output_dir": str(data_dir() / "runs"),
        "max_xy_speed": 1.0,
        "max_xy_acceleration": 1.5,
        "max_accel_rc_offset": 80,
        "acceleration_i_max_rc_offset": 75.0,
        "max_yaw_rc_offset": 80,
        "max_height_rc_offset": 170,
        "kinematics_hz": 55.0,
        "acceleration_control_hz": 55.0,
        "velocity_control_hz": 25.0,
        "position_control_hz": 15.0,
        "velocity_filter_alpha": 0.82,
        "acceleration_filter_alpha": 0.75,
        "acceleration_step": 1.0,
        "acceleration_trial_seconds": 1.5,
        "acceleration_p_target_arrival_seconds": 0.10,
        "acceleration_settling_band_fraction": 0.20,
        "acceleration_plateau_band_fraction": 0.20,
        "acceleration_plateau_score_weight": 3.0,
        "acceleration_d_min_plateau_fraction": 0.30,
        "acceleration_min_tolerance": 0.01,
        "acceleration_settling_seconds": 0.15,
        "acceleration_max_jerk": 3.0,
        "acceleration_p_coarse_step": 1.0,
        "acceleration_p_max_trials": 20,
        "acceleration_p_time_tolerance_seconds": 0.03,
        "acceleration_p_min_bracket_width": 0.02,
        "acceleration_d_coarse_step": 0.01,
        "acceleration_i_coarse_step": 0.04,
        "acceleration_search_coarse_trials": 15,
        "acceleration_i_search_coarse_trials": 30,
        "acceleration_i_success_hold_fraction": 0.90,
        "acceleration_search_refine_trials": 12,
        "acceleration_search_worse_streak": 2,
        "acceleration_search_reverse_fraction": 0.25,
        "acceleration_search_final_fraction": 0.07,
        "acceleration_repeats": 3,
        "acceleration_p_after_d_factor": 1.15,
        "acceleration_d_arrival_seconds": 3.0,
        "acceleration_i_earliest_score_seconds": 2.0,
        "acceleration_d_hold_seconds": 6.0,
        "acceleration_d_hold_fraction": 0.75,
        "acceleration_validation_steps": [0.05, 0.15, 0.5],
        "acceleration_validation_trial_seconds": 2.0,
        "acceleration_broad_gain_factors": [0.65, 1.5],
        "acceleration_gain_ceiling_factor": 4.0,
        "kinematics_duplicate_window": 3,
        "kinematics_duplicate_limit": 2,
        "kinematics_duplicate_max_fraction": 0.20,
        "direction": {"roll": -1, "pitch": 1, "yaw": 1},
        "hover_height": 1.0,
        "height_step": 0.2,
        "height_stage2_targets": [0.5, 1.0, 1.25, 1.5, 3.0],
        "height_stage2_tolerance": 0.05,
        "height_stage2_max_overshoot_m": 0.15,
        "height_stage2_settle_seconds": 1.0,
        "height_stage2_broad_gain_factors": [0.65, 1.5],
        "height_stage2_gain_ceiling_factor": 4.0,
        "height_stage2_base_rc_offsets": [-30, -15, 15],
        # Repeated vertical P -> D -> I calibration.  Each height candidate is
        # flown from the actual ground level three times; yaw uses three
        # signed out/back pairs from the heading measured after service braking.
        "vertical_repeated_mode": True,
        "vertical_repeats": 3,
        "vertical_trials_per_coefficient": 15,
        "vertical_search_worse_streak": 2,
        "vertical_search_reverse_fraction": 0.25,
        "vertical_search_final_fraction": 0.07,
        "height_pid_target_m": 1.0,
        "height_pid_trial_seconds": 10.0,
        "height_pid_hold_seconds": 5.0,
        "height_p_arrival_seconds": 2.5,
        "height_p_time_tolerance_seconds": 0.5,
        "height_p_arrival_tolerance_m": 0.07,
        "height_liftoff_threshold_m": 0.03,
        "height_p_step": 0.05,
        "height_p_trials": 100,
        "height_d_step": 1.0,
        "height_i_step": 0.01,
        "height_d_band_m": 0.03,
        "height_d_required_fraction": 0.85,
        "height_i_band_m": 0.07,
        "height_i_required_fraction": 0.70,
        "height_validation_targets_m": [1.0, 4.0, 3.75, 1.5],
        "yaw_stage_targets_deg": [35, 0, -35, 0, 90, 0, -90, 0],
        "yaw_stage_tolerance_deg": 3.0,
        "yaw_stage_max_overshoot_deg": 10.0,
        "yaw_stage_settle_seconds": 1.0,
        "yaw_stage_height_tolerance_m": 0.12,
        "yaw_stage_broad_gain_factors": [0.65, 1.5],
        "yaw_stage_gain_ceiling_factor": 4.0,
        "yaw_pid_target_deg": 75.0,
        "yaw_pid_trial_seconds": 10.0,
        "yaw_pid_hold_seconds": 5.0,
        "yaw_p_arrival_seconds": 4.0,
        "yaw_p_time_tolerance_seconds": 0.25,
        "yaw_p_arrival_tolerance_deg": 3.0,
        "yaw_p_step": 0.25,
        "yaw_d_step": 0.05,
        "yaw_i_step": 0.01,
        "yaw_d_band_deg": 3.0,
        "yaw_d_required_fraction": 0.85,
        "yaw_i_band_deg": 3.0,
        "yaw_i_required_fraction": 0.70,
        "yaw_brake_rate_deg_s": 2.0,
        "yaw_brake_observe_seconds": 0.35,
        "yaw_brake_neutral_seconds": 0.25,
        "yaw_brake_pwm_per_second": 12.0,
        "yaw_brake_max_offset": 80.0,
        "yaw_brake_timeout_seconds": 8.0,
        "manual_continue_seconds": 15.0,
        "height_ascent_min_m": 0.6,
        "height_ascent_max_m": 2.0,
        "height_ascent_targets": 2,
        "height_ascent_seed": 42,
        "height_ascent_speed_min": 0.20,
        "height_ascent_speed_max": 0.30,
        "height_ascent_search_rounds": 2,
        "height_ascent_extra_rounds": 2,
        "height_ascent_liftoff_height": 0.10,
        "height_ascent_seconds_per_m": 5.0,
        "height_ascent_time_reserve": 2.0,
        "height_ascent_arrival_tolerance": 0.05,
        "height_ascent_arrival_speed": 0.06,
        "height_ascent_max_speed": 0.5,
        "height_ascent_arrival_hold_seconds": 0.5,
        "height_ascent_hold_seconds": 5.0,
        "height_base_throttle_rc": 1610,
        "throttle_landing_start_rc": 1620,
        "throttle_landing_end_rc": 1400,
        "throttle_landing_rc_per_second": 10.0,
        "throttle_landing_speed_limit": 0.3,
        "throttle_landing_ground_tolerance": 0.05,
        "throttle_landing_ground_speed": 0.04,
        "throttle_landing_ground_hold_seconds": 1.0,
        "throttle_landing_velocity_window": 0.25,
        "height_descent_step": 0.05,
        "height_descent_step_timeout_seconds": 2.5,
        "height_descent_settle_seconds": 0.6,
        "height_descent_reach_tolerance": 0.02,
        "height_descent_max_command_gap": 0.15,
        "height_descent_max_target_undershoot": 0.10,
        "height_descent_total_timeout_seconds": 120.0,
        "height_descent_max_speed": 0.3,
        "yaw_step_deg": 35.0,
        "velocity_step": 0.15,
        "velocity_target_speed": 0.15,
        "velocity_target_min_speed": 0.135,
        "velocity_target_max_speed": 0.165,
        "velocity_confirmation_speeds": [0.10, 0.25, 1.0],
        "velocity_validation_speeds": [0.10, 0.25, 1.0],
        "velocity_p_target_arrival_seconds": 1.0,
        "velocity_p_time_tolerance_seconds": 0.20,
        "velocity_p_arrival_speed_tolerance": 0.01,
        "velocity_repeated_mode": True,
        "velocity_repeats": 3,
        "velocity_p_targets": [{"speed": 0.15, "arrival_seconds": 1.0,
                                "tolerance_seconds": 0.20}],
        "velocity_p_high_speed_tolerance": 0.04,
        "velocity_d_plateau_band_fraction": 0.15,
        "velocity_d_min_plateau_fraction": 0.50,
        "velocity_d_settle_target_seconds": 3.0,
        "velocity_i_mean_band_fraction": 0.20,
        "velocity_i_mean_required_fraction": 0.85,
        "velocity_trial_seconds": 5.0,
        "velocity_zero_cross_timeout_seconds": 5.0,
        "velocity_brake_seconds": 3.0,
        "velocity_i_hold_seconds": 8.0,
        "velocity_validation_hold_seconds": 5.0,
        "velocity_i_hold_band_fraction": 0.10,
        "velocity_i_required_hold_fraction": 0.70,
        "velocity_d_trial_seconds": 7.0,
        "velocity_d_hold_band_fraction": 0.15,
        "velocity_d_required_hold_fraction": 0.70,
        "velocity_d_brake_speed": 0.05,
        "velocity_p_brake_stop_speed": 0.05,
        "velocity_p_brake_pwm_per_second": 15.0,
        "velocity_p_brake_max_offset": 60.0,
        "velocity_p_brake_timeout_seconds": 12.0,
        # After an open-loop cleanup, observe a fresh neutral window before
        # allowing the reverse excitation.  This prevents a momentary zero
        # crossing from being mistaken for a usable stopped state.
        "velocity_p_brake_neutral_seconds": 0.4,
        "velocity_rest_seconds": 1.5,
        "velocity_rest_timeout_seconds": 10.0,
        "velocity_rest_tolerance": 0.03,
        "velocity_measurement_window_seconds": 0.2,
        "velocity_score_hz": 10.0,
        "velocity_settling_band_fraction": 0.10,
        "velocity_settling_seconds": 1.0,
        "velocity_rise_time_min": 0.6,
        "velocity_rise_time_max": 2.0,
        "velocity_required_in_band_fraction": 0.50,
        "velocity_required_tail_fraction": 0.75,
        "velocity_tail_seconds": 2.0,
        "velocity_max_acceleration": 0.50,
        "velocity_max_jerk": 3.0,
        "velocity_p_start": 0.25,
        "velocity_p_min_multiplier": 1.15,
        "velocity_p_max_multiplier": 2.5,
        "velocity_p_after_d_factor": 1.20,
        "velocity_d_start": 0.5,
        "velocity_d_multiplier": 2.0,
        "velocity_d_min_exploration_trials": 7,
        "velocity_d_trials": 15,
        "velocity_i_start": 0.0005,
        "velocity_i_multiplier": 2.0,
        "velocity_i_trials": 15,
        "velocity_fine_factor": 0.10,
        "velocity_fine_trials": 4,
        "velocity_trials_per_coefficient": 15,
        # Kept only to load reports/configurations made by earlier releases.
        "velocity_max_trials_per_axis": 20,
        "velocity_repeat_tolerance_fraction": 0.25,
        "xy_trial_center": None,
        "xy_trial_radius": 8.0,
        "ground_return_radius_m": 1.0,
        "xy_trial_wait_seconds": 60.0,
        "position_step": 0.5,
        "position_repeated_mode": True,
        "position_repeats": 3,
        "position_target_distance": 0.25,
        "position_p_arrival_seconds": 2.5,
        "position_p_time_tolerance_seconds": 0.5,
        "position_p_arrival_tolerance_m": 0.02,
        "position_trial_seconds": 7.0,
        "position_d_trial_seconds": 8.0,
        "position_i_trial_seconds": 8.0,
        "position_d_band_m": 0.07,
        "position_d_required_fraction": 0.85,
        "position_d_min_progress_fraction": 0.5,
        "position_i_required_fraction": 0.85,
        "position_p_after_d_factor": 0.95,
        "position_p_start": 0.15,
        "position_p_multiplier": 2.0,
        "position_d_start": 0.1,
        "position_d_multiplier": 2.0,
        "position_i_start": 0.001,
        "position_i_multiplier": 2.0,
        "position_trials_per_coefficient": 15,
        "position_validation_distances": [2.0, 1.0, 0.5, 0.125],
        "position_validation_reserve_seconds": 5.0,
        "position_calibration_distances": [0.5, 1.0],
        "position_calibration_tolerance_m": 0.08,
        "position_calibration_max_overshoot_m": 0.15,
        "position_calibration_settle_seconds": 1.0,
        "position_calibration_time_reserve_seconds": 3.0,
        "xy_broad_gain_factors": [0.65, 1.5],
        "xy_gain_ceiling_factor": 4.0,
        "joint_step_seconds": 6.0,
        "joint_return_seconds": 4.0,
        "joint_gain_factors": [0.9, 1.1],
        "step_seconds": 6.0,
        "return_seconds": 4.0,
        "recovery_seconds": 8.0,
        "takeoff_seconds": 12.0,
        "landing_seconds": 60.0,
        "landing_height_tolerance": 0.35,
        "gain_factors": [0.8, 1.25],
        "search_rounds": 3,
        "min_improvement": 0.01,
        "oscillation_window_seconds": 2.5,
        "oscillation_weight": 3.0,
        "oscillation_extra_rounds": 2,
        "oscillation_tolerances": {
            "height": 0.03, "yaw": 0.03,
            "acceleration": 0.05, "velocity": 0.05, "position": 0.05,
        },
        "jump_xy_m": 0.75,
        "jump_z_m": 0.45,
        "jump_max_gap_seconds": 0.25,
        "max_intervention_retries": 2,
        "auto_height_ki_factors": [0.01, 0.03, 0.08],
        "integral_trials": {},
        "validation_tolerances": {"xy_m": 0.18, "height_m": 0.12, "yaw_rad": 0.12},
        "safety": {
            "min_airborne_height": 0.35,
            "max_height": 3.5,
            "max_xy_radius": 2.5,
            "max_yaw_rate": 2.0,
            "max_tilt_deg": 40.0,
            "max_sample_gap": 0.45,
        },
    }


def load_config(path: Path) -> dict[str, Any]:
    config = default_config()
    user = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(user, dict):
        raise ValueError("Configuration must be a JSON object")
    unknown = set(user) - set(config)
    if unknown:
        raise ValueError(f"Unknown configuration fields: {sorted(unknown)}")
    for key, value in user.items():
        if key in ("safety", "direction", "validation_tolerances",
                   "oscillation_tolerances"):
            if not isinstance(value, dict):
                raise ValueError(f"{key} must be an object")
            extra = set(value) - set(config[key])
            if extra:
                raise ValueError(f"Unknown {key} fields: {sorted(extra)}")
            config[key].update(value)
        else:
            config[key] = value
    validate_config(config)
    return config


def validate_config(config: dict[str, Any]) -> None:
    get_drone_pid_setup(config["drone_name"])
    positive = (
        "sim_port", "msp_port", "max_xy_speed", "max_xy_acceleration",
        "kinematics_hz", "acceleration_control_hz", "velocity_control_hz",
        "position_control_hz",
        "max_accel_rc_offset", "acceleration_i_max_rc_offset",
        "acceleration_step", "acceleration_trial_seconds",
        "acceleration_settling_band_fraction", "acceleration_plateau_band_fraction",
        "acceleration_plateau_score_weight", "acceleration_d_min_plateau_fraction",
        "acceleration_min_tolerance",
        "acceleration_settling_seconds", "acceleration_max_jerk",
        "acceleration_p_coarse_step", "acceleration_i_coarse_step",
        "acceleration_p_target_arrival_seconds",
        "acceleration_p_time_tolerance_seconds", "acceleration_p_min_bracket_width",
        "acceleration_d_coarse_step", "acceleration_d_hold_fraction",
        "acceleration_i_success_hold_fraction",
        "acceleration_search_reverse_fraction", "acceleration_search_final_fraction",
        "acceleration_p_after_d_factor", "acceleration_d_arrival_seconds",
        "acceleration_i_earliest_score_seconds",
        "acceleration_d_hold_seconds",
        "acceleration_validation_trial_seconds", "acceleration_gain_ceiling_factor",
        "kinematics_duplicate_max_fraction",
        "max_yaw_rc_offset", "max_height_rc_offset", "hover_height",
        "height_step", "height_descent_step", "height_descent_step_timeout_seconds",
        "height_stage2_tolerance", "height_stage2_max_overshoot_m",
        "height_stage2_settle_seconds",
        "height_stage2_gain_ceiling_factor", "yaw_stage_tolerance_deg",
        "yaw_stage_max_overshoot_deg", "yaw_stage_settle_seconds",
        "yaw_stage_height_tolerance_m", "yaw_stage_gain_ceiling_factor",
        "height_pid_target_m", "height_pid_trial_seconds",
        "height_pid_hold_seconds", "height_p_arrival_seconds",
        "height_p_time_tolerance_seconds", "height_p_arrival_tolerance_m",
        "height_liftoff_threshold_m",
        "height_p_step", "height_d_step", "height_i_step",
        "height_d_band_m", "height_d_required_fraction",
        "height_i_band_m", "height_i_required_fraction",
        "yaw_pid_target_deg", "yaw_pid_trial_seconds", "yaw_pid_hold_seconds",
        "yaw_p_arrival_seconds", "yaw_p_time_tolerance_seconds",
        "yaw_p_arrival_tolerance_deg", "yaw_p_step", "yaw_d_step",
        "yaw_i_step", "yaw_d_band_deg", "yaw_d_required_fraction",
        "yaw_i_band_deg", "yaw_i_required_fraction", "yaw_brake_rate_deg_s",
        "yaw_brake_observe_seconds", "yaw_brake_neutral_seconds",
        "yaw_brake_pwm_per_second", "yaw_brake_max_offset",
        "yaw_brake_timeout_seconds", "manual_continue_seconds",
        "vertical_search_reverse_fraction",
        "vertical_search_final_fraction",
        "height_descent_settle_seconds", "height_descent_reach_tolerance",
        "height_descent_max_command_gap", "height_descent_max_target_undershoot",
        "height_descent_total_timeout_seconds",
        "height_descent_max_speed", "yaw_step_deg", "velocity_step", "position_step",
        "position_target_distance", "position_p_arrival_seconds",
        "position_p_time_tolerance_seconds", "position_p_arrival_tolerance_m",
        "position_trial_seconds", "position_d_trial_seconds",
        "position_i_trial_seconds", "position_d_band_m",
        "position_d_required_fraction", "position_d_min_progress_fraction",
        "position_i_required_fraction",
        "position_p_after_d_factor", "position_p_start", "position_p_multiplier",
        "position_d_start", "position_d_multiplier", "position_i_start",
        "position_i_multiplier", "position_validation_reserve_seconds",
        "position_calibration_tolerance_m", "position_calibration_max_overshoot_m",
        "position_calibration_settle_seconds",
        "position_calibration_time_reserve_seconds",
        "xy_gain_ceiling_factor",
        "step_seconds", "return_seconds", "recovery_seconds", "takeoff_seconds",
        "landing_seconds", "landing_height_tolerance",
        "joint_step_seconds", "joint_return_seconds",
        "velocity_rest_seconds", "velocity_rest_timeout_seconds",
        "velocity_rest_tolerance", "velocity_measurement_window_seconds",
        "velocity_score_hz", "velocity_settling_band_fraction",
        "velocity_settling_seconds", "velocity_target_speed",
        "velocity_p_target_arrival_seconds", "velocity_p_time_tolerance_seconds",
        "velocity_p_arrival_speed_tolerance",
        "velocity_p_high_speed_tolerance", "velocity_d_plateau_band_fraction",
        "velocity_d_min_plateau_fraction", "velocity_d_settle_target_seconds",
        "velocity_i_mean_band_fraction", "velocity_i_mean_required_fraction",
        "velocity_target_min_speed", "velocity_target_max_speed",
        "velocity_trial_seconds", "velocity_zero_cross_timeout_seconds",
        "velocity_brake_seconds", "velocity_i_hold_seconds",
        "velocity_validation_hold_seconds",
        "velocity_i_hold_band_fraction", "velocity_i_required_hold_fraction",
        "velocity_d_trial_seconds", "velocity_d_hold_band_fraction",
        "velocity_d_required_hold_fraction", "velocity_d_brake_speed",
        "velocity_p_brake_stop_speed", "velocity_p_brake_pwm_per_second",
        "velocity_p_brake_max_offset", "velocity_p_brake_timeout_seconds",
        "velocity_rise_time_min",
        "velocity_rise_time_max", "velocity_required_in_band_fraction",
        "velocity_required_tail_fraction", "velocity_tail_seconds",
        "velocity_max_acceleration", "velocity_max_jerk",
        "velocity_p_start", "velocity_p_min_multiplier",
        "velocity_p_max_multiplier",
        "velocity_p_after_d_factor",
        "velocity_d_start",
        "velocity_d_multiplier", "velocity_i_start", "velocity_i_multiplier",
        "velocity_fine_factor", "velocity_repeat_tolerance_fraction",
        "xy_trial_radius", "xy_trial_wait_seconds", "ground_return_radius_m",
        "height_ascent_min_m", "height_ascent_max_m",
        "height_ascent_speed_min", "height_ascent_speed_max",
        "height_ascent_seconds_per_m", "height_ascent_time_reserve",
        "height_ascent_arrival_tolerance", "height_ascent_arrival_speed",
        "height_ascent_max_speed",
        "height_ascent_liftoff_height",
        "height_ascent_arrival_hold_seconds", "height_ascent_hold_seconds",
        "throttle_landing_rc_per_second", "throttle_landing_speed_limit",
        "throttle_landing_ground_tolerance", "throttle_landing_ground_speed",
        "throttle_landing_ground_hold_seconds", "throttle_landing_velocity_window",
    )
    for key in positive:
        value = config[key]
        if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"{key} must be a finite positive number")
    if config["max_yaw_rc_offset"] > 500 or config["max_height_rc_offset"] > 300:
        raise ValueError("RC offsets exceed calibration limits")
    if not isinstance(config["vertical_repeated_mode"], bool):
        raise ValueError("vertical_repeated_mode must be boolean")
    for key in ("vertical_repeats", "vertical_trials_per_coefficient",
                "height_p_trials",
                "vertical_search_worse_streak"):
        if (not isinstance(config[key], int) or isinstance(config[key], bool)
                or config[key] < 1):
            raise ValueError(f"{key} must be a positive integer")
    if config["vertical_repeats"] < 3:
        raise ValueError("vertical_repeats must be at least 3")
    if not 0 < config["vertical_search_final_fraction"] < \
            config["vertical_search_reverse_fraction"] < 1:
        raise ValueError("vertical search fractions must satisfy 0 < final < reverse < 1")
    for key in ("height_d_required_fraction", "height_i_required_fraction",
                "yaw_d_required_fraction", "yaw_i_required_fraction"):
        if config[key] > 1:
            raise ValueError(f"{key} must be at most 1")
    if (config["height_p_arrival_seconds"] +
            config["height_p_time_tolerance_seconds"] >=
            config["height_pid_trial_seconds"]):
        raise ValueError("height P arrival window must fit inside its trial")
    if config["height_liftoff_threshold_m"] >= config["height_pid_target_m"]:
        raise ValueError("height_liftoff_threshold_m must be below height_pid_target_m")
    if (config["yaw_p_arrival_seconds"] +
            config["yaw_p_time_tolerance_seconds"] >=
            config["yaw_pid_trial_seconds"]):
        raise ValueError("yaw P arrival window must fit inside its trial")
    validation_heights = config["height_validation_targets_m"]
    if (not isinstance(validation_heights, list) or not validation_heights or
            any(not isinstance(value, (int, float)) or not math.isfinite(value)
                or value <= 0
                for value in validation_heights)):
        raise ValueError("height_validation_targets_m must contain positive heights")
    if not (0 < config["height_ascent_min_m"] < config["height_ascent_max_m"]
            <= config["safety"]["max_height"]):
        raise ValueError("ascent target range must fit inside safety.max_height")
    if not (config["height_ascent_speed_min"] <=
            config["height_ascent_speed_max"] <=
            config["height_ascent_max_speed"]):
        raise ValueError("ascent speed range must fit inside height_ascent_max_speed")
    if config["height_ascent_liftoff_height"] >= config["height_ascent_min_m"]:
        raise ValueError("height_ascent_liftoff_height must be below minimum target")
    targets = config["height_stage2_targets"]
    if (not isinstance(targets, list) or len(targets) < 1 or
            any(not isinstance(value, (int, float)) or not math.isfinite(value)
                or value <= 0 or value > config["safety"]["max_height"]
                for value in targets) or
            any(left >= right for left, right in zip(targets, targets[1:]))):
        raise ValueError("height_stage2_targets must be increasing positive heights within safety.max_height")
    if config["height_stage2_settle_seconds"] >= config["step_seconds"]:
        raise ValueError("height_stage2_settle_seconds must fit inside step_seconds")
    if config["yaw_stage_settle_seconds"] >= config["step_seconds"]:
        raise ValueError("yaw_stage_settle_seconds must fit inside step_seconds")
    angles = config["yaw_stage_targets_deg"]
    if (not isinstance(angles, list) or len(angles) < 2 or
            any(not isinstance(angle, (int, float)) or not math.isfinite(angle)
                or abs(angle) > 150 for angle in angles) or
            not any(angle > 0 for angle in angles) or
            not any(angle < 0 for angle in angles)):
        raise ValueError("yaw_stage_targets_deg must include positive and negative angles within ±150°")
    broad_factors = config["height_stage2_broad_gain_factors"]
    if (not isinstance(broad_factors, list) or not broad_factors or
            any(not isinstance(factor, (int, float)) or not math.isfinite(factor)
                or factor <= 0 or factor > 2 for factor in broad_factors)):
        raise ValueError("height_stage2_broad_gain_factors must be numbers in (0, 2]")
    yaw_factors = config["yaw_stage_broad_gain_factors"]
    if (not isinstance(yaw_factors, list) or not yaw_factors or
            any(not isinstance(factor, (int, float)) or not math.isfinite(factor)
                or factor <= 0 or factor > 2 for factor in yaw_factors)):
        raise ValueError("yaw_stage_broad_gain_factors must be numbers in (0, 2]")
    xy_factors = config["xy_broad_gain_factors"]
    if (not isinstance(xy_factors, list) or not xy_factors or
            any(not isinstance(factor, (int, float)) or not math.isfinite(factor)
                or factor <= 0 or factor > 2 for factor in xy_factors)):
        raise ValueError("xy_broad_gain_factors must be numbers in (0, 2]")
    base_offsets = config["height_stage2_base_rc_offsets"]
    if (not isinstance(base_offsets, list) or any(
            not isinstance(offset, int) or isinstance(offset, bool)
            or abs(offset) > 100 for offset in base_offsets)):
        raise ValueError("height_stage2_base_rc_offsets must be integer RC offsets within ±100")
    if (not isinstance(config["height_ascent_targets"], int)
            or not 1 <= config["height_ascent_targets"] <= 8):
        raise ValueError("height_ascent_targets must be an integer from 1 to 8")
    if (not isinstance(config["height_ascent_seed"], int)
            or not 0 <= config["height_ascent_seed"] <= 2**32 - 1):
        raise ValueError("height_ascent_seed must be a nonnegative 32-bit integer")
    if (not isinstance(config["height_ascent_search_rounds"], int)
            or not 0 <= config["height_ascent_search_rounds"] <= 5):
        raise ValueError("height_ascent_search_rounds must be an integer from 0 to 5")
    if (not isinstance(config["height_ascent_extra_rounds"], int)
            or not 0 <= config["height_ascent_extra_rounds"] <= 5):
        raise ValueError("height_ascent_extra_rounds must be an integer from 0 to 5")
        if (not isinstance(config["height_base_throttle_rc"], int)
                or not 1200 <= config["height_base_throttle_rc"] <= 1700):
            raise ValueError("height_base_throttle_rc must be between 1200 and 1700")
        if (not isinstance(config["throttle_landing_start_rc"], int)
                or not isinstance(config["throttle_landing_end_rc"], int)
                or not 1200 <= config["throttle_landing_end_rc"] <
                config["throttle_landing_start_rc"] <= 1800):
            raise ValueError("landing RC must descend within [1200, 1800]")
    if config["height_ascent_arrival_hold_seconds"] >= config["height_ascent_hold_seconds"]:
        raise ValueError("arrival hold must be shorter than ascent hold")
    if config["throttle_landing_ground_hold_seconds"] >= config["landing_seconds"]:
        raise ValueError("ground hold must be shorter than landing timeout")
    if not isinstance(config["output_dir"], str) or not config["output_dir"].strip():
        raise ValueError("output_dir must be a nonempty path")
    if config["velocity_step"] > config["max_xy_speed"]:
        raise ValueError("velocity_step must not exceed max_xy_speed")
    if not math.isclose(config["velocity_step"], config["velocity_target_speed"],
                        rel_tol=0.0, abs_tol=1e-9):
        raise ValueError("velocity_step must equal velocity_target_speed")
    if not (0 < config["velocity_target_min_speed"] <
            config["velocity_target_speed"] <
            config["velocity_target_max_speed"] <= config["max_xy_speed"]):
        raise ValueError("velocity target corridor must fit inside max_xy_speed")
    if (config["velocity_p_target_arrival_seconds"] +
            config["velocity_p_time_tolerance_seconds"] >=
            config["velocity_trial_seconds"]):
        raise ValueError("P arrival target and tolerance must fit inside the trial")
    if config["velocity_p_arrival_speed_tolerance"] >= config["velocity_target_speed"]:
        raise ValueError("P arrival speed tolerance must be below target speed")
    if not isinstance(config["velocity_repeated_mode"], bool):
        raise ValueError("velocity_repeated_mode must be boolean")
    if not isinstance(config["velocity_repeats"], int) or config["velocity_repeats"] < 3:
        raise ValueError("velocity_repeats must be at least 3")
    targets = config["velocity_p_targets"]
    if (not isinstance(targets, list) or len(targets) < 1 or
            any(not isinstance(item, dict) or
                any(key not in item or not isinstance(item[key], (int, float)) or
                    not math.isfinite(item[key]) or item[key] <= 0
                    for key in ("speed", "arrival_seconds", "tolerance_seconds")) or
                item["speed"] > config["max_xy_speed"] or
                item["arrival_seconds"] + item["tolerance_seconds"] >=
                config["velocity_trial_seconds"]
                for item in targets)):
        raise ValueError("velocity_p_targets must fit speed and trial limits")
    for key in ("velocity_d_plateau_band_fraction", "velocity_d_min_plateau_fraction",
                "velocity_i_mean_band_fraction", "velocity_i_mean_required_fraction"):
        if not 0 < config[key] <= 1:
            raise ValueError(f"{key} must be in (0, 1]")
    distances = config["position_calibration_distances"]
    if (not isinstance(distances, list) or len(distances) < 2
            or any(not isinstance(distance, (int, float)) or
                   not math.isfinite(distance) or distance <= 0
                   for distance in distances)
            or any(left >= right for left, right in zip(distances, distances[1:]))):
        raise ValueError("position_calibration_distances must be increasing positive distances")
    if config["position_calibration_settle_seconds"] >= config["step_seconds"]:
        raise ValueError("position settle duration must fit inside step_seconds")
    if not isinstance(config["position_repeated_mode"], bool):
        raise ValueError("position_repeated_mode must be boolean")
    if not isinstance(config["position_repeats"], int) or config["position_repeats"] < 3:
        raise ValueError("position_repeats must be at least 3")
    if (not isinstance(config["position_trials_per_coefficient"], int) or
            config["position_trials_per_coefficient"] < 1):
        raise ValueError("position_trials_per_coefficient must be positive")
    if (config["position_p_arrival_seconds"] +
            config["position_p_time_tolerance_seconds"] >=
            config["position_trial_seconds"]):
        raise ValueError("position P arrival window must fit inside its trial")
    if config["position_p_arrival_tolerance_m"] >= config["position_target_distance"]:
        raise ValueError("position arrival tolerance must be below target distance")
    for key in ("position_d_required_fraction", "position_d_min_progress_fraction",
                "position_i_required_fraction"):
        if config[key] > 1:
            raise ValueError(f"{key} must be at most 1")
    for key in ("position_p_multiplier", "position_d_multiplier",
                "position_i_multiplier"):
        if config[key] <= 1:
            raise ValueError(f"{key} must exceed 1")
    validation_distances = config["position_validation_distances"]
    if (not isinstance(validation_distances, list) or not validation_distances or
            any(not isinstance(distance, (int, float)) or
                not math.isfinite(distance) or distance <= 0
                for distance in validation_distances) or
            len(set(validation_distances)) != len(validation_distances)):
        raise ValueError("position_validation_distances must be distinct positive distances")
    if config["height_descent_reach_tolerance"] >= config["height_descent_step"]:
        raise ValueError("height_descent_reach_tolerance must be below height_descent_step")
    if config["height_descent_max_command_gap"] < config["height_descent_step"]:
        raise ValueError("height_descent_max_command_gap must be at least height_descent_step")
    if config["height_descent_max_target_undershoot"] < config["height_descent_step"]:
        raise ValueError("height_descent_max_target_undershoot must be at least height_descent_step")
    if config["height_descent_settle_seconds"] >= config["height_descent_step_timeout_seconds"]:
        raise ValueError("height_descent_settle_seconds must fit inside the step timeout")
    speeds = config["velocity_confirmation_speeds"]
    if (not isinstance(speeds, list) or not speeds or any(
            not isinstance(speed, (int, float)) or not math.isfinite(speed)
            or speed <= 0 or speed > config["max_xy_speed"] for speed in speeds)
            or any(left >= right for left, right in zip(speeds, speeds[1:]))):
        raise ValueError("velocity_confirmation_speeds must increase within (0, max_xy_speed]")
    validation_speeds = config["velocity_validation_speeds"]
    if (not isinstance(validation_speeds, list) or not validation_speeds or any(
            not isinstance(speed, (int, float)) or not math.isfinite(speed)
            or speed <= 0 or speed > config["max_xy_speed"] for speed in validation_speeds)
            or any(left >= right for left, right in zip(validation_speeds,
                                                        validation_speeds[1:]))):
        raise ValueError("velocity_validation_speeds must increase within (0, max_xy_speed]")
    if config["velocity_rest_timeout_seconds"] < config["velocity_rest_seconds"]:
        raise ValueError("velocity_rest_timeout_seconds must be at least velocity_rest_seconds")
    if config["velocity_measurement_window_seconds"] >= min(
            config["velocity_brake_seconds"], config["joint_return_seconds"]):
        raise ValueError("velocity measurement window must fit inside stop segments")
    if config["velocity_settling_band_fraction"] >= 1:
        raise ValueError("velocity_settling_band_fraction must be below 1")
    if config["velocity_settling_seconds"] >= min(
            config["velocity_trial_seconds"], config["joint_step_seconds"]):
        raise ValueError("velocity settling time must fit inside speed steps")
    if not (config["velocity_rise_time_min"] < config["velocity_rise_time_max"] <
            config["velocity_trial_seconds"]):
        raise ValueError("velocity rise-time interval must fit inside the trial")
    if config["velocity_tail_seconds"] >= config["velocity_trial_seconds"]:
        raise ValueError("velocity_tail_seconds must be shorter than the trial")
    if config["velocity_i_hold_seconds"] < config["velocity_trial_seconds"]:
        raise ValueError("velocity_i_hold_seconds must be at least velocity_trial_seconds")
    if not 0 < config["velocity_i_hold_band_fraction"] < 1:
        raise ValueError("velocity_i_hold_band_fraction must be in (0, 1)")
    if not 0 < config["velocity_i_required_hold_fraction"] <= 1:
        raise ValueError("velocity_i_required_hold_fraction must be in (0, 1]")
    if config["velocity_d_trial_seconds"] < config["velocity_trial_seconds"]:
        raise ValueError("velocity_d_trial_seconds must be at least velocity_trial_seconds")
    if config["velocity_p_brake_stop_speed"] <= 0:
        raise ValueError("velocity_p_brake_stop_speed must be positive")
    if config["velocity_zero_cross_timeout_seconds"] <= 0:
        raise ValueError("velocity_zero_cross_timeout_seconds must be positive")
    if config["velocity_p_brake_neutral_seconds"] < config[
            "velocity_measurement_window_seconds"]:
        raise ValueError("velocity_p_brake_neutral_seconds must cover the measurement window")
    if not 0 < config["velocity_d_hold_band_fraction"] < 1:
        raise ValueError("velocity_d_hold_band_fraction must be in (0, 1)")
    if not 0 < config["velocity_d_required_hold_fraction"] <= 1:
        raise ValueError("velocity_d_required_hold_fraction must be in (0, 1]")
    if config["velocity_d_brake_speed"] <= 0:
        raise ValueError("velocity_d_brake_speed must be positive")
    for key in ("velocity_required_in_band_fraction",
                "velocity_required_tail_fraction", "velocity_fine_factor"):
        if not 0 < config[key] < 1:
            raise ValueError(f"{key} must be in (0, 1)")
    if not 0 < config["velocity_repeat_tolerance_fraction"] < 1:
        raise ValueError("velocity_repeat_tolerance_fraction must be in (0, 1)")
    for key in ("velocity_d_trials", "velocity_i_trials",
                "velocity_fine_trials", "velocity_max_trials_per_axis",
                "velocity_trials_per_coefficient"):
        if not isinstance(config[key], int) or isinstance(config[key], bool) or config[key] < 1:
            raise ValueError(f"{key} must be a positive integer")
    if config["velocity_max_trials_per_axis"] < 4:
        raise ValueError("velocity_max_trials_per_axis must be at least 4")
    if config["velocity_trials_per_coefficient"] < 1:
        raise ValueError("velocity_trials_per_coefficient must be positive")
    if (not isinstance(config["velocity_d_min_exploration_trials"], int) or
            isinstance(config["velocity_d_min_exploration_trials"], bool) or
            not 1 <= config["velocity_d_min_exploration_trials"] <=
            config["velocity_trials_per_coefficient"]):
        raise ValueError("velocity_d_min_exploration_trials must fit inside velocity_trials_per_coefficient")
    for key in ("velocity_p_min_multiplier", "velocity_p_max_multiplier",
                "velocity_d_multiplier", "velocity_i_multiplier"):
        if config[key] <= 1:
            raise ValueError(f"{key} must be greater than 1")
    if config["velocity_p_min_multiplier"] > config["velocity_p_max_multiplier"]:
        raise ValueError("velocity_p_min_multiplier must not exceed its maximum")
    if config["velocity_p_after_d_factor"] < 1:
        raise ValueError("velocity_p_after_d_factor must be at least 1")
    center = config["xy_trial_center"]
    if (center is not None and
            (not isinstance(center, list) or len(center) != 2 or
             any(not isinstance(value, (int, float)) or not math.isfinite(value)
                 for value in center))):
        raise ValueError("xy_trial_center must be null or [x, y]")
    if not 0 <= config["velocity_filter_alpha"] <= 1:
        raise ValueError("velocity_filter_alpha must be between 0 and 1")
    if not 0 <= config["acceleration_filter_alpha"] <= 1:
        raise ValueError("acceleration_filter_alpha must be between 0 and 1")
    if config["max_accel_rc_offset"] > 500:
        raise ValueError("max_accel_rc_offset must not exceed 500")
    if config["acceleration_i_max_rc_offset"] > config["max_accel_rc_offset"]:
        raise ValueError("acceleration_i_max_rc_offset must fit max_accel_rc_offset")
    if not math.isclose(config["kinematics_hz"], config["acceleration_control_hz"],
                        rel_tol=0.0, abs_tol=1e-9):
        raise ValueError("acceleration_control_hz must equal kinematics_hz")
    if not (config["acceleration_control_hz"] > config["velocity_control_hz"] >
            config["position_control_hz"]):
        raise ValueError("control frequencies must descend acceleration > velocity > position")
    if config["acceleration_step"] > config["max_xy_acceleration"]:
        raise ValueError("acceleration_step must not exceed max_xy_acceleration")
    if config["acceleration_settling_seconds"] >= config["acceleration_trial_seconds"]:
        raise ValueError("acceleration settling time must fit inside the trial")
    if not 0 < config["acceleration_settling_band_fraction"] < 1:
        raise ValueError("acceleration_settling_band_fraction must be in (0, 1)")
    if not 0 < config["acceleration_plateau_band_fraction"] < 1:
        raise ValueError("acceleration_plateau_band_fraction must be in (0, 1)")
    if not 0 < config["acceleration_d_min_plateau_fraction"] < 1:
        raise ValueError("acceleration_d_min_plateau_fraction must be in (0, 1)")
    if config["acceleration_p_target_arrival_seconds"] >= config["acceleration_trial_seconds"]:
        raise ValueError("acceleration P arrival target must fit inside its trial")
    if config["acceleration_p_time_tolerance_seconds"] >= config["acceleration_p_target_arrival_seconds"]:
        raise ValueError("acceleration P time tolerance must be below its arrival target")
    if config["acceleration_p_min_bracket_width"] >= config["acceleration_p_coarse_step"]:
        raise ValueError("acceleration P minimum bracket width must be below its coarse step")
    if not 0 < config["acceleration_d_hold_fraction"] <= 1:
        raise ValueError("acceleration_d_hold_fraction must be in (0, 1]")
    if not 0 < config["acceleration_i_success_hold_fraction"] <= 1:
        raise ValueError("acceleration_i_success_hold_fraction must be in (0, 1]")
    if not 0 < config["acceleration_search_reverse_fraction"] < 1:
        raise ValueError("acceleration_search_reverse_fraction must be in (0, 1)")
    if not 0 < config["acceleration_search_final_fraction"] < config["acceleration_search_reverse_fraction"]:
        raise ValueError("acceleration_search_final_fraction must be smaller than reverse fraction")
    if config["acceleration_p_after_d_factor"] <= 0:
        raise ValueError("acceleration_p_after_d_factor must be positive")
    for key in ("acceleration_p_max_trials", "acceleration_search_coarse_trials",
                "acceleration_i_search_coarse_trials",
                "acceleration_search_refine_trials",
                "acceleration_search_worse_streak", "acceleration_repeats"):
        if not isinstance(config[key], int) or isinstance(config[key], bool) or config[key] < 1:
            raise ValueError(f"{key} must be a positive integer")
    if config["acceleration_repeats"] < 3:
        raise ValueError("acceleration_repeats must be at least 3")
    if config["acceleration_p_max_trials"] < 3:
        raise ValueError("acceleration_p_max_trials must be at least 3")
    if config["acceleration_d_arrival_seconds"] + config["acceleration_d_hold_seconds"] < 1:
        raise ValueError("acceleration D observation window is too short")
    validation_steps = config["acceleration_validation_steps"]
    if (not isinstance(validation_steps, list) or not validation_steps or any(
            not isinstance(value, (int, float)) or not math.isfinite(value) or
            value <= 0 or value > config["max_xy_acceleration"]
            for value in validation_steps)):
        raise ValueError("acceleration_validation_steps must fit max_xy_acceleration")
    factors = config["acceleration_broad_gain_factors"]
    if (not isinstance(factors, list) or not factors or any(
            not isinstance(factor, (int, float)) or not math.isfinite(factor)
            or factor <= 0 or factor > 2 for factor in factors)):
        raise ValueError("acceleration_broad_gain_factors must be numbers in (0, 2]")
    for key in ("kinematics_duplicate_window", "kinematics_duplicate_limit"):
        if (not isinstance(config[key], int) or isinstance(config[key], bool)
                or config[key] < 2):
            raise ValueError(f"{key} must be an integer of at least 2")
    if config["kinematics_duplicate_limit"] > config["kinematics_duplicate_window"]:
        raise ValueError("kinematics_duplicate_limit must fit its window")
    if not 0 <= config["kinematics_duplicate_max_fraction"] <= 1:
        raise ValueError("kinematics_duplicate_max_fraction must be in [0, 1]")
    if not 0 <= config["min_improvement"] < 1:
        raise ValueError("min_improvement must be in [0, 1)")
    if not isinstance(config["search_rounds"], int) or not 0 <= config["search_rounds"] <= 8:
        raise ValueError("search_rounds must be an integer from 0 to 8")
    if (not isinstance(config["max_intervention_retries"], int)
            or not 0 <= config["max_intervention_retries"] <= 10):
        raise ValueError("max_intervention_retries must be an integer from 0 to 10")
    if (not isinstance(config["oscillation_extra_rounds"], int)
            or not 0 <= config["oscillation_extra_rounds"] <= 4):
        raise ValueError("oscillation_extra_rounds must be an integer from 0 to 4")
    for key in ("oscillation_window_seconds", "oscillation_weight", "jump_xy_m",
                "jump_z_m", "jump_max_gap_seconds"):
        value = config[key]
        if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"{key} must be a finite positive number")
    if config["oscillation_window_seconds"] >= config["step_seconds"]:
        raise ValueError("oscillation_window_seconds must be below step_seconds")
    if config["oscillation_window_seconds"] >= min(config["joint_step_seconds"],
                                                    config["joint_return_seconds"]):
        raise ValueError("oscillation_window_seconds must be below joint segment durations")
    if (not isinstance(config["auto_height_ki_factors"], list)
            or any(not isinstance(value, (int, float)) or not math.isfinite(value)
                   or value <= 0 or value > 0.2
                   for value in config["auto_height_ki_factors"])):
        raise ValueError("auto_height_ki_factors must contain numbers in (0, 0.2]")
    integral_trials = config["integral_trials"]
    if not isinstance(integral_trials, dict) or set(integral_trials) - set(PID_NAMES):
        raise ValueError("integral_trials must map PID names to trial lists")
    for name, trials in integral_trials.items():
        if not isinstance(trials, list):
            raise ValueError(f"integral_trials.{name} must be a list")
        for trial in trials:
            if (not isinstance(trial, dict) or set(trial) != {"ki", "i_limit"}
                    or any(not isinstance(value, (int, float)) or not math.isfinite(value)
                           or value <= 0 for value in trial.values())):
                raise ValueError(f"Each integral_trials.{name} item needs positive ki and i_limit")
    factors = config["gain_factors"]
    if not isinstance(factors, list) or not factors or any(
        not isinstance(f, (int, float)) or not math.isfinite(f) or f <= 0 or f > 2
        for f in factors
    ):
        raise ValueError("gain_factors must contain numbers in (0, 2]")
    joint_factors = config["joint_gain_factors"]
    if (not isinstance(joint_factors, list) or not joint_factors
            or any(not isinstance(f, (int, float)) or not math.isfinite(f)
                   or f <= 0 or f > 2 for f in joint_factors)):
        raise ValueError("joint_gain_factors must contain numbers in (0, 2]")
    if any(value not in (-1, 1) for value in config["direction"].values()):
        raise ValueError("direction values must be +1 or -1")
    output_dir = Path(config["output_dir"]).expanduser().resolve()
    if any(not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0
           for value in config["validation_tolerances"].values()):
        raise ValueError("validation_tolerances must be finite positive numbers")
    if any(not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0
           for value in config["oscillation_tolerances"].values()):
        raise ValueError("oscillation_tolerances must be finite positive numbers")
    safety = config["safety"]
    if any(not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0
           for value in safety.values()):
        raise ValueError("safety values must be finite positive numbers")
    if config["yaw_step_deg"] >= 170:
        raise ValueError("yaw_step_deg must be below 170 degrees")


def _kinematics_signature(sample: Sample) -> tuple[float | None, ...]:
    """Exact simulator state used only to diagnose stale RPC replies."""
    return (sample.x, sample.y, sample.z, sample.yaw, sample.roll, sample.pitch,
            sample.vx_world, sample.vy_world, sample.vz_world)


def _duplicate_kinematics_window(samples: list[Sample], config: dict[str, Any]) -> bool:
    """Whether the latest short window contains repeated simulator states."""
    window = config["kinematics_duplicate_window"]
    recent = samples[-window:]
    if len(recent) < window:
        return False
    counts: dict[tuple[float, float, float, float, float, float], int] = {}
    for sample in recent:
        key = _kinematics_signature(sample)
        counts[key] = counts.get(key, 0) + 1
    return max(counts.values()) >= config["kinematics_duplicate_limit"]


class InterventionDetected(Exception):
    """A manual move or simulator reset made the current trial incomparable."""


class TrialInvalid(Exception):
    """The drone was not in a comparable flying state for this trial."""


class LandingIncomplete(Exception):
    """The next ground-start trial cannot begin until landing is confirmed."""


class SimulatorLink:
    """Owns MSP and simulator RPC connections; no high-level client instance."""

    def __init__(self, config: dict[str, Any]):
        self.config = config
        self.transmitter = None
        self.control = None
        self.sim = None
        self.stream_thread = None
        self.stream_stop = threading.Event()
        self.frame_lock = threading.Lock()
        self.io_lock = threading.Lock()
        self.frame = HOLD_FRAME
        self.frame_updated = 0.0
        self.stream_error: Exception | None = None

    def connect(self) -> None:
        from inavmspapi import MultirotorControl
        from inavmspapi.transmitter import TCPTransmitter
        import msgpackrpc

        self.transmitter = TCPTransmitter((self.config["host"], int(self.config["msp_port"])))
        self.transmitter.connect()
        self.control = MultirotorControl(self.transmitter)
        self.sim = msgpackrpc.Client(
            msgpackrpc.Address(self.config["host"], int(self.config["sim_port"])),
            timeout=10, pack_encoding="utf-8", unpack_encoding="utf-8",
        )
        time.sleep(2)
        self._ensure_althold_range()
        self._send_frame(RESET_FRAME)
        time.sleep(0.5)
        self.read_sample()

    def _msp(self, name: str, data: list[int]) -> None:
        from inavmspapi.msp_codes import MSPCodes

        with self.io_lock:
            if not self.control.send_RAW_msg(MSPCodes[name], data=data):
                raise ConnectionError(f"Could not send {name}")
            response = self.control.receive_msg()
            if response is None or response.get("crcError") or response.get("packet_error"):
                raise ConnectionError(f"Bad {name} response: {response}")
            decoded = self.control.process_recv_data(response)
            if decoded is None or decoded < 0:
                raise ConnectionError(f"Could not decode {name}: {decoded}")

    def _ensure_althold_range(self) -> None:
        self._msp("MSP_MODE_RANGES", [])
        ranges = self.control.MODE_RANGES
        if any(r["id"] == 3 and r["auxChannelIndex"] == 2
               and r["range"] == {"start": 1250, "end": 1350} for r in ranges):
            return
        empty = next((i for i, r in enumerate(ranges)
                      if r["id"] == 0 and r["range"] == {"start": 900, "end": 900}), None)
        if empty is None:
            raise RuntimeError("No free MSP mode range for NAV ALTHOLD")
        self._msp("MSP_SET_MODE_RANGE", [empty, 3, 2, 14, 18])
        time.sleep(0.3)
        self._msp("MSP_MODE_RANGES", [])
        if not any(r["id"] == 3 and r["auxChannelIndex"] == 2
                   and r["range"] == {"start": 1250, "end": 1350}
                   for r in self.control.MODE_RANGES):
            raise RuntimeError("NAV ALTHOLD range was not verified")

    def _send_frame(self, frame: tuple[int, ...] | list[int]) -> None:
        values = [int(clamp(v, 1000, 2000)) for v in frame]
        with self.io_lock:
            if not self.control.send_RAW_RC(values):
                raise ConnectionError("MSP rejected RC frame")
            response = self.control.receive_msg()
            if response is None or response.get("crcError") or response.get("packet_error"):
                raise ConnectionError(f"Bad RC response: {response}")

    def arm(self) -> None:
        self.stream_error = None
        self._send_frame(RESET_FRAME)
        time.sleep(1)
        self._send_frame(ARM_SWITCH_FRAME)
        self._send_frame(ARMED_NEUTRAL_FRAME)
        deadline = time.monotonic() + 1
        while time.monotonic() < deadline:
            self._send_frame(ARMED_NEUTRAL_FRAME)
            time.sleep(0.05)
        self._send_frame((1500, 1500, 1000, 1500, 2000, 1000, 1300))
        self.set_frame(HOLD_FRAME)
        self.stream_stop.clear()
        self.stream_thread = threading.Thread(target=self._stream, name="calibration_rc", daemon=True)
        self.stream_thread.start()

    def disarm(self) -> None:
        self.stream_stop.set()
        if self.stream_thread is not None:
            self.stream_thread.join(timeout=2)
            self.stream_thread = None
        for _ in range(3):
            self._send_frame((1500, 1500, 1000, 1500, 1000, 1000, 1000))
            time.sleep(0.05)

    def set_frame(self, frame: tuple[int, ...] | list[int]) -> None:
        with self.frame_lock:
            self.frame = tuple(frame)
            self.frame_updated = time.monotonic()

    def _stream(self) -> None:
        period = 1 / self.config["kinematics_hz"]
        while not self.stream_stop.is_set():
            start = time.monotonic()
            with self.frame_lock:
                frame = self.frame if start - self.frame_updated < 0.35 else HOLD_FRAME
            try:
                self._send_frame(frame)
            except Exception as exc:
                self.stream_error = exc
                self.stream_stop.set()
                return
            self.stream_stop.wait(max(0.0, period - (time.monotonic() - start)))

    def read_sample(self) -> Sample:
        kin = self.sim.call("getKinematicsData")
        if not isinstance(kin, dict):
            raise ConnectionError("Missing simulator kinematics")
        return sample_from_kinematics(kin, time.monotonic())

    def close(self) -> None:
        self.stream_stop.set()
        if self.stream_thread is not None:
            self.stream_thread.join(timeout=2)
        if self.control is not None:
            for _ in range(3):
                try:
                    self._send_frame((1500, 1500, 1000, 1500, 1000, 1000, 1000))
                except Exception:
                    break
                time.sleep(0.05)
        if self.transmitter is not None:
            try:
                self.transmitter.disconnect()
            except Exception:
                pass
        if self.sim is not None:
            try:
                self.sim.close()
            except Exception:
                pass


def printable_pids(configs: dict[str, dict[str, Any]]) -> dict[str, dict[str, Any]]:
    result = {}
    for name in PID_NAMES:
        pid = configs[name]
        result[name] = {
            key: (value.__name__ if callable(value) else value)
            for key, value in pid.items()
        }
    return result


def _oscillation_metrics(rows: list[dict[str, Any]], stage: str, axis: str,
                         amplitude: float, window_seconds: float) -> tuple[float, float, float]:
    """Measure motion around a linear settling trend in each segment's tail."""
    segments: dict[str, list[dict[str, Any]]] = {}
    for row in rows:
        segments.setdefault(row["segment"], []).append(row)
    rms_values = []
    ranges = []
    cycles = []
    for segment_rows in segments.values():
        end = segment_rows[-1]["t"]
        tail = [row for row in segment_rows if row["t"] >= end - window_seconds]
        bins: dict[int, list[float]] = {}
        start = tail[0]["t"]
        for row in tail:
            if stage == "height":
                error = row["target_z"] - row["z"]
            elif stage == "yaw":
                error = wrap_pi(row["target_yaw"] - row["yaw"])
            elif stage == "velocity":
                error = row[f"command_v{axis}"] - row[f"v{axis}_body"]
            else:
                error = row[f"target_{axis}"] - row[axis]
            bins.setdefault(int((row["t"] - start) / 0.1), []).append(error)
        values = [mean(bin_values) for bin_values in bins.values()]
        if len(values) < 5:
            continue
        x = list(range(len(values)))
        center_x, center_y = mean(x), mean(values)
        denominator = sum((value - center_x) ** 2 for value in x)
        slope = sum((a - center_x) * (b - center_y)
                    for a, b in zip(x, values)) / denominator
        residuals = [value - center_y - slope * (index - center_x)
                     for index, value in enumerate(values)]
        rms_values.append(math.sqrt(mean(value * value for value in residuals)))
        ranges.append(max(residuals) - min(residuals))
        deadband = 0.1 * amplitude
        signs = [1 if value > deadband else -1 if value < -deadband else 0
                 for value in residuals]
        nonzero = [sign for sign in signs if sign]
        cycles.append(sum(a != b for a, b in zip(nonzero, nonzero[1:])) / 2)
    return (mean(rms_values) if rms_values else 0.0,
            mean(ranges) if ranges else 0.0,
            mean(cycles) if cycles else 0.0)


def _physical_velocity_bins(rows: list[dict[str, Any]], segment: str,
                            window: float, hz: float) -> list[dict[str, float]]:
    """Return measured body velocity, preferring simulator linear_velocity.

    Older CSV runs predate the direct velocity fields, so they retain the
    wider position-difference estimate as an offline compatibility fallback.
    """
    segment_rows = [row for row in rows if row["segment"] == segment]
    if not segment_rows:
        return []
    start = segment_rows[0]["t"]
    groups: dict[int, list[dict[str, float]]] = {}
    previous = 0
    for row in segment_rows:
        t = row["t"]
        direct_velocity = (row.get("vx_world"), row.get("vy_world"))
        if all(isinstance(value, (int, float)) and math.isfinite(value)
               for value in direct_velocity):
            vx_world, vy_world = direct_velocity
        else:
            while (previous + 1 < len(segment_rows)
                   and segment_rows[previous + 1]["t"] <= t - window):
                previous += 1
            older = segment_rows[previous]
            dt = t - older["t"]
            if dt < window * 0.8:
                continue
            vx_world = (row["x"] - older["x"]) / dt
            vy_world = (row["y"] - older["y"]) / dt
        cs, sn = math.cos(row["yaw"]), math.sin(row["yaw"])
        item = {
            "t": t - start,
            "vx": vx_world * cs - vy_world * sn,
            "vy": vx_world * sn + vy_world * cs,
            "x": row["x"], "y": row["y"],
        }
        groups.setdefault(int((t - start) * hz), []).append(item)
    return [{key: mean(item[key] for item in group)
             for key in ("t", "vx", "vy", "x", "y")}
            for group in groups.values()]


def _invalid_xy_trial(rows: list[dict[str, Any]], stage: str,
                      config: dict[str, Any]) -> str | None:
    if stage not in ("acceleration", "velocity", "position") or not rows:
        return None
    scored = [row for row in rows if not row["segment"].startswith("pre_")]
    if not scored:
        return "No scored telemetry"
    safety = config["safety"]
    if mean(row["z"] - row.get("ground_z", 0.0) <
            safety["min_airborne_height"] for row in scored) > 0.1:
        return "Drone was below the airborne height during the trial"
    if mean(max(abs(row["roll"]), abs(row["pitch"])) >
            math.radians(safety["max_tilt_deg"]) for row in scored) > 0.2:
        return "Excessive tilt during the trial"
    moving = [row for row in scored if row["segment"] in ("positive", "negative")]
    if moving and not (stage == "position" and any(
            "position_cycle" in row for row in moving)) and (
            max(row["x"] for row in moving) - min(row["x"] for row in moving) < 0.01
                   and max(row["y"] for row in moving) - min(row["y"] for row in moving) < 0.01):
        return "Drone did not move during the commanded steps"
    if stage == "acceleration":
        scored = [row for row in scored if row["segment"] in ("positive", "negative")]
        duplicate_fraction = (mean(bool(row.get("kinematics_duplicate", False))
                                   for row in scored) if scored else 0.0)
        if duplicate_fraction > config["kinematics_duplicate_max_fraction"]:
            return ("Simulator returned repeated kinematics in "
                    f"{duplicate_fraction:.0%} of acceleration ticks")
    return None


def score_acceleration_steps(rows: list[dict[str, Any]], axis: str,
                             config: dict[str, Any],
                             magnitude: float | None = None) -> dict[str, Any]:
    """Score direct body-acceleration steps from controller telemetry."""
    target_magnitude = (config["acceleration_step"] if magnitude is None else magnitude)
    tolerance = max(target_magnitude * config["acceleration_settling_band_fraction"],
                    config["acceleration_min_tolerance"])
    acceleration_period = 1 / config["acceleration_control_hz"]
    required = max(2, math.ceil(config["acceleration_settling_seconds"] /
                                acceleration_period))
    parts: dict[str, dict[str, Any]] = {}
    for label, sign in (("positive", 1), ("negative", -1)):
        samples = [row for row in rows if row.get("segment") == label]
        values = [float(row[f"a{axis}_body"]) for row in samples
                  if isinstance(row.get(f"a{axis}_body"), (int, float))]
        targets = [float(row[f"target_a{axis}_body"]) for row in samples
                   if isinstance(row.get(f"target_a{axis}_body"), (int, float))]
        if len(values) < required or not targets:
            raise TrialInvalid(f"Insufficient acceleration telemetry for {label}")
        target = targets[-1]
        errors = [abs(value - target) for value in values]
        in_band = [error <= tolerance for error in errors]
        times = [float(row["segment_elapsed"]) for row in samples
                 if isinstance(row.get(f"a{axis}_body"), (int, float))]
        duration = max(float(row["segment_seconds"]) for row in samples)
        rise = duration
        for index in range(len(values) - required + 1):
            if all(in_band[index:index + required]):
                rise = times[index]
                break
        # P only asks whether the measured acceleration crosses the target,
        # and when. It does not require an in-band hold.
        p_arrival = next((elapsed for elapsed, value in zip(times, values)
                          if sign * value >= target_magnitude), None)
        tail = values[-max(required, len(values) // 3):]
        jerk = [abs((right - left) / max(acceleration_period,
                                         times[index + 1] - times[index]))
                for index, (left, right) in enumerate(zip(values, values[1:]))]
        parts[label] = {
            "target": target, "reached": rise < duration, "rise_time": rise,
            "p_reached": p_arrival is not None,
            "p_arrival_time": p_arrival,
            "mae": mean(errors), "terminal_mae": mean(abs(value - target) for value in tail),
            "in_band_fraction": mean(in_band),
            "oscillation_rms": math.sqrt(mean((value - mean(tail)) ** 2 for value in tail)),
            "peak_signed_acceleration": max(sign * value for value in values),
            "peak_overshoot": max(0.0, max(sign * value for value in values) -
                                  sign * target),
            "p95_jerk": _percentile(jerk, 0.95), "duration": duration,
        }
    values = list(parts.values())
    metrics = {
        "method": "physical_acceleration_step_response",
        "positive": parts["positive"], "negative": parts["negative"],
        "mae": mean(part["mae"] for part in values),
        "terminal_mae": max(part["terminal_mae"] for part in values),
        "oscillation_rms": max(part["oscillation_rms"] for part in values),
        "p95_jerk": max(part["p95_jerk"] for part in values),
        "in_band_fraction": min(part["in_band_fraction"] for part in values),
        "reached_both": all(part["reached"] for part in values),
        "p_reached_directions": sum(part["p_reached"] for part in values),
        "max_peak_overshoot": max(part["peak_overshoot"] for part in values),
        "samples": len(rows),
        "kinematics_duplicate_fraction": mean(
            bool(row.get("kinematics_duplicate", False)) for row in rows
            if row.get("segment") in ("positive", "negative")),
        "scheduler_late_p95": _percentile([
            float(row.get("scheduler_late_seconds", 0.0)) for row in rows
            if row.get("segment") in ("positive", "negative")], 0.95),
    }
    # Reaching the requested acceleration is a prerequisite, not a soft
    # preference.  The former score could select a quiet but weak controller
    # which stayed near zero acceleration for an entire step.
    unreached_directions = sum(not part["reached"] for part in values)
    terminal_miss = max(0.0, metrics["terminal_mae"] / tolerance - 1.0)
    metrics["reached_directions"] = 2 - unreached_directions
    metrics["terminal_miss"] = terminal_miss
    p_target_time = config["acceleration_p_target_arrival_seconds"]
    p_miss_penalty = 1 + config["acceleration_trial_seconds"] / p_target_time
    metrics["p_score"] = (
        p_miss_penalty * sum(not part["p_reached"] for part in values) +
        mean(abs((part["p_arrival_time"] if part["p_arrival_time"] is not None
                  else part["duration"]) - p_target_time) / p_target_time
             for part in values))
    metrics["score"] = (
        metrics["mae"] / target_magnitude +
        2 * metrics["terminal_mae"] / target_magnitude +
        max(part["rise_time"] / part["duration"] for part in values) +
        2 * metrics["oscillation_rms"] / target_magnitude +
        metrics["p95_jerk"] / config["acceleration_max_jerk"] +
        2 * (1 - metrics["in_band_fraction"]) +
        6 * unreached_directions +
        4 * terminal_miss)
    return metrics


def _acceleration_plateau_metrics(part: list[dict[str, Any]], axis: str,
                                  magnitude: float,
                                  config: dict[str, Any]) -> dict[str, Any]:
    """Find when acceleration stays near its own final level, even if biased."""
    samples = [(float(row["segment_elapsed"]), float(row[f"a{axis}_body"]))
               for row in part]
    duration = samples[-1][0]
    final_values = [value for elapsed, value in samples
                    if elapsed >= duration - 1.0]
    center = median(final_values)
    band = max(magnitude * config["acceleration_plateau_band_fraction"],
               config["acceleration_min_tolerance"])
    within = [abs(value - center) <= band for _, value in samples]
    settled_at: float | None = None
    for index, (start, _) in enumerate(samples):
        if duration - start < 1.0:
            break
        remaining = within[index:]
        if mean(remaining) < 0.85:
            continue
        # A long quiet tail must not hide another burst of oscillation.
        window_start = start
        windows_stable = True
        while window_start < duration:
            window = [inside for (elapsed, _), inside in
                      zip(samples[index:], remaining)
                      if window_start <= elapsed < window_start + 0.5]
            if len(window) >= 5 and mean(window) < 0.70:
                windows_stable = False
                break
            window_start += 0.5
        if windows_stable:
            settled_at = start
            break
    cutoff = duration if settled_at is None else settled_at
    transient = [value for elapsed, value in samples if elapsed < cutoff]
    transient_rms = (math.sqrt(mean((value - center) ** 2 for value in transient))
                     if transient else 0.0)
    return {
        "center": center, "band": band,
        "settling_time": cutoff, "settled": settled_at is not None,
        "transient_rms": transient_rms,
    }


def score_acceleration_repeats(rows: list[dict[str, Any]], axis: str,
                               config: dict[str, Any], phase: str,
                               magnitude: float | None = None) -> dict[str, Any]:
    """Score each out/back pair separately, then average repeated flights."""
    magnitude = config["acceleration_step"] if magnitude is None else magnitude
    cycles = sorted({int(row.get("acceleration_cycle", 1)) for row in rows
                     if row.get("segment") in ("positive", "negative")})
    if not cycles:
        raise TrialInvalid("No acceleration steps were recorded")
    tolerance = max(magnitude * config["acceleration_settling_band_fraction"],
                    config["acceleration_min_tolerance"])
    scored = []
    for cycle in cycles:
        cycle_rows = [row for row in rows
                      if int(row.get("acceleration_cycle", 1)) == cycle]
        base = score_acceleration_steps(cycle_rows, axis, config, magnitude)
        for direction, sign in (("positive", 1), ("negative", -1)):
            part = [row for row in cycle_rows if row.get("segment") == direction]
            arrival = next((float(row["segment_elapsed"]) for row in part
                            if sign * float(row[f"a{axis}_body"]) >= magnitude), None)
            arrived = (arrival is not None and
                       arrival <= config["acceleration_d_arrival_seconds"])
            if (phase != "p" and arrived and
                    float(part[-1]["segment_elapsed"]) +
                    2 / config["acceleration_control_hz"] <
                    arrival + config["acceleration_d_hold_seconds"]):
                raise TrialInvalid("Acceleration hold ended before the observation window")
            hold = ([row for row in part if arrival is not None and
                     arrival <= float(row["segment_elapsed"]) <=
                     arrival + config["acceleration_d_hold_seconds"]]
                    if arrived else [])
            hold_fraction = (mean(abs(float(row[f"a{axis}_body"]) - sign * magnitude)
                                  <= tolerance for row in hold) if hold else 0.0)
            plateau = (_acceleration_plateau_metrics(part, axis, magnitude, config)
                       if phase != "p" else None)
            scored.append({
                "cycle": cycle, "direction": direction, "arrived": arrived,
                "arrival_time": arrival if arrived else None,
                "hold_fraction": hold_fraction, "plateau": plateau,
                "base": base[direction],
            })
    by_direction = {
        direction: [item for item in scored if item["direction"] == direction]
        for direction in ("positive", "negative")
    }
    filtered_by_direction = {}
    if phase != "p":
        for direction, sign in (("positive", 1), ("negative", -1)):
            series = {
                cycle: [(float(row["segment_elapsed"]),
                         float(row[f"a{axis}_body"]))
                        for row in rows if row.get("segment") == direction and
                        int(row.get("acceleration_cycle", 1)) == cycle]
                for cycle in cycles
            }
            trace = smoothed_mean(align_repeats(series))
            filtered_by_direction[direction] = settled_band_response(
                trace, sign * magnitude, tolerance,
                max(magnitude * config["acceleration_plateau_band_fraction"],
                    config["acceleration_min_tolerance"]),
                config["acceleration_i_earliest_score_seconds"])
    arrival_limit = (config["acceleration_trial_seconds"] if phase == "p" else
                     config["acceleration_d_arrival_seconds"])
    arrival_fraction = mean(
        (item["arrival_time"] if item["arrived"] else
         arrival_limit) / arrival_limit for item in scored)
    p_target_time = config["acceleration_p_target_arrival_seconds"]
    p_time_error = mean(
        abs((item["arrival_time"] if item["arrived"] else arrival_limit) -
            p_target_time) for item in scored)
    raw_hold_fraction = mean(item["hold_fraction"] for item in scored)
    filtered_hold_fraction = (mean(item["hold_fraction"] for item in
                                   filtered_by_direction.values())
                              if filtered_by_direction else raw_hold_fraction)
    hold_fraction = (filtered_hold_fraction if phase == "i" else raw_hold_fraction)
    plateau_fraction = (mean(item["plateau"]["settling_time"] /
                             item["base"]["duration"] for item in scored)
                        if phase != "p" else 0.0)
    transient_rms = (mean(item["plateau"]["transient_rms"] for item in scored)
                     if phase != "p" else 0.0)
    plateau_levels = ({
        direction: mean((1 if direction == "positive" else -1) *
                        item["plateau"]["center"] / magnitude for item in items)
        for direction, items in by_direction.items()
    } if phase != "p" else None)
    min_plateau = config["acceleration_d_min_plateau_fraction"]
    weak_plateau_penalty = (mean(
        max(0.0, min_plateau - level) / min_plateau
        for level in plateau_levels.values()) if plateau_levels else 0.0)
    reached_fraction = mean(item["arrived"] for item in scored)
    # A single missed step among three repeats should not decide an entire
    # direction; two or more misses show a reproducible response problem.
    reached_directions = sum(
        mean(item["arrived"] for item in items) >= 2 / 3
        for items in by_direction.values())
    metrics = score_acceleration_steps(
        [row for row in rows if int(row.get("acceleration_cycle", 1)) == cycles[-1]],
        axis, config, magnitude)
    metrics.update({
        "phase": phase, "repeat_count": len(cycles), "repeat_steps": scored,
        "mae": mean(item["base"]["mae"] for item in scored),
        "terminal_mae": mean(item["base"]["terminal_mae"] for item in scored),
        "in_band_fraction": mean(item["base"]["in_band_fraction"] for item in scored),
        "oscillation_rms": mean(item["base"]["oscillation_rms"] for item in scored),
        "p95_jerk": mean(item["base"]["p95_jerk"] for item in scored),
        "p_reached_directions": reached_directions,
        "reached_directions": reached_directions,
        "reached_both": reached_directions == 2,
        "reached_steps": sum(item["arrived"] for item in scored),
        "arrival_fraction": arrival_fraction,
        "arrival_target_seconds": p_target_time,
        "arrival_time_error_seconds": p_time_error,
        "mean_arrival_seconds": mean(
            item["arrival_time"] for item in scored if item["arrived"])
            if any(item["arrived"] for item in scored) else None,
        "hold_fraction": hold_fraction,
        "raw_hold_fraction": raw_hold_fraction,
        "i_filtered_hold_fraction": filtered_hold_fraction,
        "i_filtered_response_by_direction": filtered_by_direction,
        "plateau_settling_time": (mean(item["plateau"]["settling_time"]
                                       for item in scored) if phase != "p" else None),
        "plateau_settling_fraction": plateau_fraction,
        "plateau_settled_steps": (sum(item["plateau"]["settled"] for item in scored)
                                  if phase != "p" else None),
        "plateau_time_by_direction": ({
            direction: mean(item["plateau"]["settling_time"] for item in items)
            for direction, items in by_direction.items()
        } if phase != "p" else None),
        "plateau_center_by_direction": ({
            direction: mean(item["plateau"]["center"] for item in items)
            for direction, items in by_direction.items()
        } if phase != "p" else None),
        "plateau_level_fraction_by_direction": plateau_levels,
        "weak_plateau_penalty": weak_plateau_penalty,
        "transient_rms": transient_rms,
        "hold_fraction_by_direction": {
            direction: (filtered_by_direction[direction]["hold_fraction"]
                        if phase == "i" else
                        mean(item["hold_fraction"] for item in items))
            for direction, items in by_direction.items()
        },
        "p_score": ((1 + arrival_limit / p_target_time) *
                    sum(not item["arrived"] for item in scored) +
                    p_time_error / p_target_time),
        # D reduces the transient; target-band dwell and steady-state bias
        # belong to I. A very weak plateau still needs a penalty, otherwise
        # excessive damping would win by keeping acceleration near zero.
        "d_score": (8 * (1 - reached_fraction) +
                    arrival_fraction + 8 * weak_plateau_penalty +
                    config["acceleration_plateau_score_weight"] * plateau_fraction +
                    0.5 * min(transient_rms / magnitude, 4.0)),
        "i_score": (8 * (1 - reached_fraction) +
                    4 * (1 - filtered_hold_fraction) + arrival_fraction +
                    (mean(item["mae"] for item in filtered_by_direction.values())
                     if filtered_by_direction else
                     mean(item["base"]["terminal_mae"] for item in scored)) /
                    tolerance +
                    (2 * mean(not item["settled"] for item in
                              filtered_by_direction.values())
                     if filtered_by_direction else 0.0)),
        "max_peak_overshoot": max(
            item["base"]["peak_overshoot"] for item in scored),
    })
    metrics["score"] = metrics[f"{phase}_score"]
    return metrics


def _acceleration_response_stable(metrics: dict[str, Any], config: dict[str, Any],
                                  magnitude: float | None = None) -> bool:
    if "hold_fraction_by_direction" in metrics:
        return (metrics["reached_directions"] == 2 and
                all(fraction >= config["acceleration_d_hold_fraction"]
                    for fraction in metrics["hold_fraction_by_direction"].values()))
    magnitude = config["acceleration_step"] if magnitude is None else magnitude
    tolerance = max(magnitude * config["acceleration_settling_band_fraction"],
                    config["acceleration_min_tolerance"])
    return (metrics.get("reached_both", False)
            and metrics["terminal_mae"] <= tolerance
            and metrics["oscillation_rms"] <= config["oscillation_tolerances"]["acceleration"]
            and metrics["p95_jerk"] <= config["acceleration_max_jerk"])


def _percentile(values: list[float], fraction: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, math.ceil(fraction * len(ordered)) - 1))
    return ordered[index]


def score_velocity_steps(rows: list[dict[str, Any]], axis: str, amplitude: float,
                         config: dict[str, Any], *, p_mode: bool = False) -> dict[str, Any]:
    """Score speed increments relative to the measured pre-step drift."""
    window = config["velocity_measurement_window_seconds"]
    hz = config["velocity_score_hz"]
    band = amplitude * config["velocity_settling_band_fraction"]
    required = max(2, math.ceil(config["velocity_settling_seconds"] * hz))
    directions: dict[str, dict[str, Any]] = {}
    for label, sign in (("positive", 1), ("negative", -1)):
        segment_rows = [row for row in rows if row["segment"] == label]
        preparation = _physical_velocity_bins(rows, f"pre_{label}", window, hz)
        if not preparation and label == "negative":
            preparation = _physical_velocity_bins(rows, "positive_stop", window, hz)
        speed = _physical_velocity_bins(rows, label, window, hz)
        stopping = _physical_velocity_bins(rows, f"{label}_stop", window, hz)
        if len(speed) < required or len(stopping) < 3:
            raise TrialInvalid(f"Insufficient physical speed data for {label}")
        baseline_values = [row.get("baseline_axis_speed") for row in segment_rows
                           if isinstance(row.get("baseline_axis_speed"), (int, float))]
        delta_values = [row.get("requested_delta_speed") for row in segment_rows
                        if isinstance(row.get("requested_delta_speed"), (int, float))]
        command_values = [row.get("commanded_axis_speed") for row in segment_rows
                          if isinstance(row.get("commanded_axis_speed"), (int, float))]
        baseline = baseline_values[-1] if baseline_values else 0.0
        requested_delta = delta_values[-1] if delta_values else sign * amplitude
        step_amplitude = abs(requested_delta)
        if step_amplitude < 1e-4:
            raise TrialInvalid(f"Speed increment was too small for {label}")
        step_sign = 1 if requested_delta > 0 else -1
        commanded_speed = (command_values[-1] if command_values else
                           baseline + requested_delta)
        values = [sample[f"v{axis}"] for sample in speed]
        progress = [step_sign * (value - baseline) for value in values]
        errors = [abs(step_amplitude - value) for value in progress]
        duration = max(row["segment_seconds"] for row in rows if row["segment"] == label)
        stop_duration = max(row["segment_seconds"] for row in rows
                            if row["segment"] == f"{label}_stop")
        reference = config["velocity_target_speed"]
        corridor_low = (step_amplitude * config["velocity_target_min_speed"] /
                        reference)
        corridor_high = (step_amplitude * config["velocity_target_max_speed"] /
                         reference)
        in_band = [corridor_low <= value <= corridor_high for value in progress]
        # P is judged by the first physical arrival at the signed command,
        # not by later hold quality.  This remains meaningful with any
        # non-zero initial drift because commanded_speed is absolute.
        arrival_tolerance = (config["velocity_p_arrival_speed_tolerance"] if p_mode
                             else config["velocity_target_max_speed"] -
                             config["velocity_target_speed"])
        direct_errors = [abs(value - commanded_speed) for value in values]
        arrival_time = next((sample["t"] for sample, error in zip(speed, direct_errors)
                             if error <= arrival_tolerance), duration)
        arrived = arrival_time < duration
        d_hold_band = abs(commanded_speed) * config["velocity_d_hold_band_fraction"]
        after_arrival = [abs(value - commanded_speed) <= d_hold_band
                         for sample, value in zip(speed, values)
                         if sample["t"] >= arrival_time]
        d_hold_fraction = (mean(after_arrival) if arrived and after_arrival else 0.0)
        i_hold_band = abs(commanded_speed) * config["velocity_i_hold_band_fraction"]
        i_after_arrival = [abs(value - commanded_speed) <= i_hold_band
                           for sample, value in zip(speed, values)
                           if sample["t"] >= arrival_time]
        i_hold_fraction = (mean(i_after_arrival)
                           if arrived and i_after_arrival else 0.0)
        reach_required = max(2, math.ceil(0.3 * hz))
        rise = duration
        for index in range(len(speed) - reach_required + 1):
            if all(in_band[index:index + reach_required]):
                rise = speed[index]["t"]
                break
        reached = rise < duration
        settling = duration
        for index in range(len(speed) - required + 1):
            if (all(error <= band for error in errors[index:index + required])
                    and mean(error <= band for error in errors[index:]) >= 0.8):
                settling = speed[index]["t"]
                break
        final = [value for sample, value in zip(speed, progress)
                 if sample["t"] >= speed[-1]["t"] - min(1.5, duration * 0.25)]
        oscillation_values = [value for sample, value in zip(speed, progress)
                              if sample["t"] >= speed[-1]["t"] -
                              min(4.0, duration - config["velocity_settling_seconds"])]
        tail_start = max(0.0, speed[-1]["t"] - config["velocity_tail_seconds"])
        tail_flags = [flag for sample, flag in zip(speed, in_band)
                      if sample["t"] >= tail_start]
        accelerations = []
        acceleration_times = []
        for previous, current in zip(speed, speed[1:]):
            dt = current["t"] - previous["t"]
            if dt > 1e-6:
                accelerations.append((step_sign * current[f"v{axis}"] -
                                      step_sign * previous[f"v{axis}"]) / dt)
                acceleration_times.append(current["t"])
        jerks = []
        for previous, current, t0, t1 in zip(
                accelerations, accelerations[1:],
                acceleration_times, acceleration_times[1:]):
            if t1 - t0 > 1e-6:
                jerks.append((current - previous) / (t1 - t0))
        # Braking is assessed against physical zero speed, rather than the
        # frozen pre-step drift: the controller receives an actual zero-speed
        # command during this segment.
        stop_values = [abs(sample[f"v{axis}"]) for sample in stopping]
        stop_time = stop_duration
        stop_required = max(2, math.ceil(0.7 * hz))
        for index in range(len(stopping) - stop_required + 1):
            if all(value <= config["velocity_rest_tolerance"]
                   for value in stop_values[index:index + stop_required]):
                stop_time = stopping[index]["t"]
                break
        stop_rows = [row for row in rows if row["segment"] == f"{label}_stop"]
        cutoff = stop_rows[0]["t"] + stop_time
        travelled = [row for row in stop_rows if row["t"] <= cutoff]
        stop_path = sum(abs(b[axis] - a[axis]) for a, b in zip(
            travelled, travelled[1:]))
        stop_terminal_speed = mean(stop_values[-max(2, int(hz)):])
        d_brake_time = stop_duration
        for index, value in enumerate(stop_values):
            if value <= config["velocity_d_brake_speed"]:
                d_brake_time = stopping[index]["t"]
                break
        d_brake_reached = d_brake_time < stop_duration
        d_brake_reversed = any(
            sample[f"v{axis}"] * commanded_speed < -config["velocity_d_brake_speed"]
            for sample in stopping)
        directions[label] = {
            "initial_axis_speed": abs(preparation[-1][f"v{axis}"])
            if preparation else None,
            "initial_cross_speed": abs(preparation[-1]["vy" if axis == "x" else "vx"])
            if preparation else None,
            "baseline_axis_speed": baseline,
            "commanded_axis_speed": commanded_speed,
            "requested_delta_speed": requested_delta,
            "requested_delta_magnitude": step_amplitude,
            "reached": reached,
            "rise_time": rise,
            "arrived": arrived,
            "arrival_time": arrival_time,
            "arrival_error_tolerance": arrival_tolerance,
            "d_hold_fraction_after_arrival": d_hold_fraction,
            "d_hold_band": d_hold_band,
            "i_hold_fraction_after_arrival": i_hold_fraction,
            "i_hold_band": i_hold_band,
            "settled": settling < duration,
            "settling_time": settling,
            "mae": mean(errors),
            "terminal_mae": mean(abs(step_amplitude - value) for value in final),
            "terminal_tracking_bias": mean(step_amplitude - value for value in final),
            "terminal_progress": mean(final),
            "max_speed": max(progress),
            "overshoot": max(0.0, max(progress) - step_amplitude),
            "in_band_fraction": mean(in_band),
            "tail_in_band_fraction": mean(tail_flags) if tail_flags else 0.0,
            "p95_acceleration": _percentile([abs(value) for value in accelerations], 0.95),
            "p95_jerk": _percentile([abs(value) for value in jerks], 0.95),
            "oscillation_rms": math.sqrt(mean(
                (value - mean(oscillation_values)) ** 2
                for value in oscillation_values)),
            "oscillation_peak_to_peak": max(oscillation_values) - min(oscillation_values),
            "stopped": (stop_time < stop_duration and
                        stop_terminal_speed <= config["velocity_rest_tolerance"]),
            "brake_initial_speed": stop_values[0],
            "brake_minimum_abs_speed": min(stop_values),
            "d_brake_reached": d_brake_reached,
            "d_brake_time": d_brake_time,
            "d_brake_reversed": d_brake_reversed,
            "stopping_time": stop_time,
            "stopping_distance": stop_path,
            "stop_terminal_speed": stop_terminal_speed,
            "duration": duration, "stop_duration": stop_duration,
        }
    parts = list(directions.values())
    # A reverse step can be clipped by the absolute ±target-speed limit when
    # a small residual drift remains. Its physical size is then close to zero,
    # but it must not turn every normalized score component into an outlier.
    score_reference_speed = config["velocity_target_speed"]
    metrics: dict[str, Any] = {
        "method": "physical_relative_step_response",
        "positive": directions["positive"], "negative": directions["negative"],
        "mae": mean(part["mae"] for part in parts),
        "terminal_mae": max(part["terminal_mae"] for part in parts),
        "max_overshoot": max(part["overshoot"] for part in parts),
        "oscillation_rms": max(part["oscillation_rms"] for part in parts),
        "oscillation_peak_to_peak": max(part["oscillation_peak_to_peak"] for part in parts),
        "in_band_fraction": min(part["in_band_fraction"] for part in parts),
        "tail_in_band_fraction": min(part["tail_in_band_fraction"] for part in parts),
        "p95_acceleration": max(part["p95_acceleration"] for part in parts),
        "p95_jerk": max(part["p95_jerk"] for part in parts),
        "terminal_tracking_bias": max(part["terminal_tracking_bias"] for part in parts),
        "minimum_terminal_tracking_bias": min(
            part["terminal_tracking_bias"] for part in parts),
        "saturation_fraction": mean(row["saturated"] for row in rows
                                    if not row["segment"].startswith("pre_")),
        "samples": len(rows),
        "score_reference_speed": score_reference_speed,
        "reached_both": all(part["reached"] for part in parts),
        "settled_both": all(part["settled"] for part in parts),
        "stopped_both": all(part["stopped"] for part in parts),
    }
    metrics["score"] = (
        mean(part["mae"] / score_reference_speed for part in parts)
        + 2 * max(part["terminal_mae"] / score_reference_speed
                  for part in parts)
        + 0.5 * max(part["overshoot"] / score_reference_speed
                    for part in parts)
        + 2 * max(part["oscillation_rms"] / score_reference_speed
                  for part in parts)
        + 2 * (1 - metrics["in_band_fraction"])
        + 2 * (1 - metrics["tail_in_band_fraction"])
        + 0.5 * mean(part["rise_time"] / part["duration"] for part in parts)
        + mean(part["settling_time"] / part["duration"] for part in parts)
        + 0.5 * mean(part["stopping_time"] / part["stop_duration"] for part in parts)
        + 0.5 * mean(part["stopping_distance"] /
                     (score_reference_speed * part["stop_duration"])
                     for part in parts)
        + 0.25 * metrics["saturation_fraction"]
        + 0.1 * metrics["p95_acceleration"] / config["velocity_max_acceleration"]
        + 0.05 * metrics["p95_jerk"] / config["velocity_max_jerk"]
    )
    return metrics


def score_velocity_p_arrival(metrics: dict[str, Any], config: dict[str, Any]) -> float:
    """Judge pure P only by first arrival at the commanded speed in both directions."""
    target = config["velocity_p_target_arrival_seconds"]
    penalties = []
    for direction in ("positive", "negative"):
        part = metrics[direction]
        arrived = part.get("arrived", False)
        elapsed = part.get("arrival_time", part.get("duration", target + 2.0))
        penalties.append(abs(elapsed - target) if arrived else
                         part["duration"] - target + 2.0)
    return mean(penalties)


def score_velocity_repeats(rows: list[dict[str, Any]], axis: str,
                           config: dict[str, Any], phase: str,
                           i_score_start: float | None = None,
                           requested_speed: float | None = None) -> dict[str, Any]:
    """Score the mean of three flights; retain individual flights as diagnostics."""
    if phase not in ("p", "d", "i", "validation"):
        raise ValueError(f"Unknown velocity phase {phase}")
    plans = (config["velocity_p_targets"] if phase == "p" else
             [{"speed": (config["velocity_target_speed"] if requested_speed is None
                         else requested_speed),
               "arrival_seconds": config["velocity_p_target_arrival_seconds"],
               "tolerance_seconds": config["velocity_p_time_tolerance_seconds"]}])
    repeats = config["velocity_repeats"]
    samples: dict[tuple[float, str, int], list[tuple[float, float]]] = {}
    for row in rows:
        if row.get("segment") not in ("positive", "negative"):
            continue
        speed = float(row["velocity_requested_speed"])
        cycle = int(row["velocity_cycle"])
        elapsed = float(row["segment_elapsed"])
        world_x, world_y = row.get("vx_world"), row.get("vy_world")
        if (isinstance(world_x, (int, float)) and
                isinstance(world_y, (int, float))):
            cs, sn = math.cos(row["yaw"]), math.sin(row["yaw"])
            velocity = (world_x * cs - world_y * sn if axis == "x" else
                        world_x * sn + world_y * cs)
        else:
            velocity = float(row[f"v{axis}_body"])
        if math.isfinite(elapsed) and math.isfinite(velocity):
            samples.setdefault((speed, row["segment"], cycle), []).append(
                (elapsed, velocity))
    zero_cross_failures = [row for row in rows
                           if row.get("velocity_zero_cross_failed")]
    if zero_cross_failures:
        # A velocity response that never crossed zero is a failed candidate,
        # not a missing/invalid recording. Do not time arrival from the
        # negative starting velocity or retry the same weak P indefinitely.
        duration = (config["velocity_trial_seconds"] if phase == "p" else
                    config["velocity_validation_hold_seconds"] if phase == "validation"
                    else config["velocity_i_hold_seconds"] if phase == "i" else
                    config["velocity_d_trial_seconds"])
        mean_steps = [{
            "speed": float(plan["speed"]), "direction": direction,
            "cycle": "mean", "arrived": False, "arrival_time": None,
            "duration": duration, "settled": False,
            "settling_time": duration, "plateau_fraction": 0.0,
            "transient_rms": float(plan["speed"]),
            "terminal_mae": float(plan["speed"]),
        } for plan in plans for direction in ("positive", "negative")]
        empty_response = {direction: {"score_window_valid": False,
                                      "hold_fraction": 0.0,
                                      "mae": float(plans[0]["speed"])}
                          for direction in ("positive", "negative")}
        return {
            "phase": phase, "score": 10.0 + len(zero_cross_failures),
            "repeat_count": repeats, "repeat_steps": [],
            "mean_steps": mean_steps,
            "scoring_basis": "zero_cross_not_reached",
            "zero_cross_failures": len(zero_cross_failures),
            "reached_by_target": {f"{float(plan['speed']):g}": 0 for plan in plans},
            "reached_steps": 0, "reached_both": False,
            "mean_arrival_seconds": None,
            "plateau_settling_time": duration,
            "plateau_settled_fraction": 0.0,
            "transient_rms": float(plans[0]["speed"]),
            "i_mean_response_by_direction": empty_response,
            "i_score_start_seconds": i_score_start if phase == "i" else None,
            "validation_target_speed": float(plans[0]["speed"]) if phase == "validation" else None,
            "validation_passed": False if phase == "validation" else None,
            "hold_fraction": 0.0,
            "hold_fraction_by_direction": {direction: 0.0 for direction in
                                           ("positive", "negative")},
            "terminal_mae": float(plans[0]["speed"]),
            "oscillation_rms": float(plans[0]["speed"]),
            "positive": {"arrival_time": None},
            "negative": {"arrival_time": None},
        }
    hz = config["velocity_score_hz"]

    def measure_step(trace: list[tuple[float, float]], speed: float,
                     direction: str, cycle: int | str) -> dict[str, Any]:
        sign = 1 if direction == "positive" else -1
        groups: dict[int, list[float]] = {}
        times: dict[int, list[float]] = {}
        for elapsed, value in trace:
            bin_index = int(elapsed * hz)
            groups.setdefault(bin_index, []).append(value)
            times.setdefault(bin_index, []).append(elapsed)
        binned = [(mean(times[index]), mean(groups[index]))
                  for index in sorted(groups)]
        tolerance = (config["velocity_p_arrival_speed_tolerance"]
                     if speed <= config["velocity_target_speed"] else
                     config["velocity_p_high_speed_tolerance"])
        arrival = next((elapsed for elapsed, value in binned
                        if sign * value >= speed - tolerance), None)
        terminal = [value for elapsed, value in binned
                    if elapsed >= binned[-1][0] - 1.0]
        center = median(terminal)
        band = speed * config["velocity_d_plateau_band_fraction"]
        settled_at = None
        for index, (elapsed, _) in enumerate(binned):
            if binned[-1][0] - elapsed < 1.0:
                break
            remainder = binned[index:]
            if mean(abs(value - center) <= band
                    for _, value in remainder) < 0.85:
                continue
            if any(mean(abs(value - center) <= band
                        for moment, value in remainder
                        if start <= moment < start + 0.5) < 0.70
                   for start in (elapsed + tick * 0.5 for tick in
                                 range(math.ceil((binned[-1][0] - elapsed) / 0.5)))
                   if sum(start <= moment < start + 0.5
                          for moment, _ in remainder) >= 2):
                continue
            settled_at = elapsed
            break
        duration = binned[-1][0]
        transient = [value for elapsed, value in binned
                     if elapsed < (settled_at if settled_at is not None else duration)]
        return {
            "speed": speed, "direction": direction, "cycle": cycle,
            "arrived": arrival is not None, "arrival_time": arrival,
            "duration": duration, "plateau_center": center,
            "plateau_fraction": sign * center / speed,
            "settled": settled_at is not None,
            "settling_time": settled_at if settled_at is not None else duration,
            "transient_rms": (math.sqrt(mean((value - center) ** 2
                                              for value in transient))
                              if transient else 0.0),
            "terminal_mae": mean(abs(value - sign * speed) for value in terminal),
        }

    steps = []
    mean_steps = []
    mean_traces: dict[tuple[float, str], list[tuple[float, float]]] = {}
    for plan in plans:
        speed = float(plan["speed"])
        for direction in ("positive", "negative"):
            series = {}
            for cycle in range(1, repeats + 1):
                trace = samples.get((speed, direction, cycle), [])
                if len(trace) < 4:
                    raise TrialInvalid(f"Missing velocity repeat {speed:g} {direction} #{cycle}")
                series[cycle] = trace
                steps.append(measure_step(trace, speed, direction, cycle))
            averaged = smoothed_mean(align_repeats(series))
            if len(averaged) < 4:
                raise TrialInvalid(f"Cannot align velocity repeats {speed:g} {direction}")
            mean_traces[(speed, direction)] = averaged
            mean_steps.append(measure_step(averaged, speed, direction, "mean"))
    mean_by_direction = {part["direction"]: part for part in mean_steps}
    repeated_mean = {}
    if phase in ("i", "validation"):
        if (phase == "i" and (i_score_start is None or
                              not math.isfinite(i_score_start) or i_score_start < 0)):
            raise ValueError("I scoring requires a settling time from the selected D trial")
        speed = float(plans[0]["speed"])
        for direction, sign in (("positive", 1), ("negative", -1)):
            trace = mean_traces[(speed, direction)]
            tolerance = speed * config["velocity_i_mean_band_fraction"]
            if phase == "validation":
                response = settled_band_response(
                    trace, sign * speed, tolerance,
                    max(speed * config["velocity_d_plateau_band_fraction"],
                        config["velocity_p_arrival_speed_tolerance"]), 0.75)
                repeated_mean[direction] = {
                    **response, "score_window_valid": response["settled"]}
            else:
                scored = [value for elapsed, value in trace if elapsed >= i_score_start]
                window_valid = bool(scored and trace[-1][0] - i_score_start >= 0.5)
                repeated_mean[direction] = {
                    "score_start": i_score_start,
                    "score_window_valid": window_valid,
                    "hold_fraction": (mean(abs(value - sign * speed) <= tolerance
                                           for value in scored) if window_valid else 0.0),
                    "mae": (mean(abs(value - sign * speed) for value in scored)
                            if window_valid else speed),
                    "bias": (mean(value - sign * speed for value in scored)
                             if window_valid else -sign * speed),
                }
    if phase == "p":
        penalties = []
        reached_by_target = {}
        for plan in plans:
            speed = float(plan["speed"])
            selected = [step for step in mean_steps if step["speed"] == speed]
            reached_by_target[f"{speed:g}"] = sum(step["arrived"] for step in selected)
            penalties.extend(
                (abs(step["arrival_time"] - plan["arrival_seconds"]) /
                 plan["arrival_seconds"] if step["arrived"] else 3.0)
                for step in selected)
        score = mean(penalties)
    elif phase == "d":
        reached_by_target = {}
        speed = float(plans[0]["speed"])
        score = mean((step["settling_time"] / config[
                          "velocity_d_settle_target_seconds"] +
                      0.5 * step["transient_rms"] / speed +
                      3.0 * (not step["settled"]) +
                      3.0 * max(0.0, config["velocity_d_min_plateau_fraction"] -
                                step["plateau_fraction"])) for step in mean_steps)
    else:
        reached_by_target = {}
        speed = float(plans[0]["speed"])
        score = mean(3.0 * (1 - part["hold_fraction"]) +
                     part["mae"] / (speed * config["velocity_i_mean_band_fraction"]) +
                     3.0 * (not part["score_window_valid"])
                     for part in repeated_mean.values())
    return {
        "phase": phase, "score": score, "repeat_count": repeats,
        "repeat_steps": steps, "mean_steps": mean_steps,
        "scoring_basis": "time_aligned_smoothed_mean_of_repeats",
        "reached_by_target": reached_by_target,
        "reached_steps": sum(step["arrived"] for step in mean_steps),
        "reached_both": all(part["arrived"] for part in mean_steps),
        "mean_arrival_seconds": (mean(step["arrival_time"] for step in mean_steps
                                      if step["arrived"])
                                 if any(step["arrived"] for step in mean_steps) else None),
        "plateau_settling_time": mean(step["settling_time"] for step in mean_steps),
        "plateau_settled_fraction": mean(step["settled"] for step in mean_steps),
        "transient_rms": mean(step["transient_rms"] for step in mean_steps),
        "i_mean_response_by_direction": repeated_mean,
        "i_score_start_seconds": i_score_start if phase == "i" else None,
        "validation_target_speed": float(plans[0]["speed"]) if phase == "validation" else None,
        "validation_passed": (all(part["arrived"] for part in mean_steps) and
                              all(part["score_window_valid"] and
                                  part["hold_fraction"] >=
                                  config["velocity_i_mean_required_fraction"]
                                  for part in repeated_mean.values()))
        if phase == "validation" else None,
        "hold_fraction": (mean(part["hold_fraction"] for part in
                               repeated_mean.values()) if repeated_mean else 0.0),
        "hold_fraction_by_direction": {
            direction: part["hold_fraction"] for direction, part in
            repeated_mean.items()},
        "terminal_mae": mean(step["terminal_mae"] for step in mean_steps),
        "oscillation_rms": mean(step["transient_rms"] for step in mean_steps),
        "positive": {"arrival_time": mean_by_direction["positive"]["arrival_time"]},
        "negative": {"arrival_time": mean_by_direction["negative"]["arrival_time"]},
    }


def _velocity_response_stable(metrics: dict[str, Any], amplitude: float,
                              config: dict[str, Any]) -> bool:
    parts = (metrics["positive"], metrics["negative"])
    return (metrics.get("preparations_stable", True)
            and metrics.get("reached_both", False)
            and metrics.get("stopped_both", False)
            and all(config["velocity_rise_time_min"] <= part["rise_time"] <=
                    config["velocity_rise_time_max"]
                    for part in parts)
            and metrics["in_band_fraction"] >=
            config["velocity_required_in_band_fraction"]
            and metrics["tail_in_band_fraction"] >=
            config["velocity_required_tail_fraction"]
            and metrics["p95_acceleration"] <= config["velocity_max_acceleration"]
            and metrics["p95_jerk"] <= config["velocity_max_jerk"]
            and all(part.get("terminal_mae", metrics["terminal_mae"]) <=
                    config["velocity_target_speed"] *
                    config["velocity_settling_band_fraction"]
                    for part in parts))


def score_position_repeats(rows: list[dict[str, Any]], axis: str,
                           config: dict[str, Any], phase: str,
                           i_score_start: float | None = None,
                           distance: float | None = None) -> dict[str, Any]:
    """Score position relative to each post-braking origin on mean curves."""
    if phase not in ("p", "d", "i", "validation"):
        raise ValueError(f"Unknown position phase {phase}")
    requested = (config["position_target_distance"] if distance is None else distance)
    repeats = config["position_repeats"]
    traces: dict[tuple[str, int], list[tuple[float, float]]] = {}
    for row in rows:
        direction = row.get("segment")
        if direction not in ("positive", "negative"):
            continue
        try:
            cycle = int(row["position_cycle"])
            elapsed = float(row["segment_elapsed"])
            displacement = float(row[axis]) - float(row["position_origin_axis"])
        except (KeyError, TypeError, ValueError):
            continue
        if math.isfinite(elapsed) and math.isfinite(displacement):
            traces.setdefault((direction, cycle), []).append((elapsed, displacement))

    band = config["position_d_band_m"]
    steps = []
    mean_traces = {}
    for direction, sign in (("positive", 1), ("negative", -1)):
        series = {}
        for cycle in range(1, repeats + 1):
            trace = traces.get((direction, cycle), [])
            if len(trace) < 4:
                raise TrialInvalid(f"Missing position repeat {direction} #{cycle}")
            series[cycle] = trace
        averaged = smoothed_mean(align_repeats(series))
        if len(averaged) < 4:
            raise TrialInvalid(f"Cannot align position repeats {direction}")
        mean_traces[direction] = averaged
        target = sign * requested
        arrival = next((t for t, value in averaged
                        if sign * value >= requested -
                        config["position_p_arrival_tolerance_m"]), None)
        response = settled_band_response(averaged, target, band, band, 0.0)
        duration = averaged[-1][0]
        tail = [value for t, value in averaged if t >= duration - 1.0]
        center = median(tail)
        plateau = [value for t, value in averaged
                   if t >= response["settling_time"]]
        hold = (mean(abs(value - center) <= band for value in plateau)
                if response["settled"] and plateau else 0.0)
        steps.append({"direction": direction, "target": target,
                      "arrived": arrival is not None, "arrival_time": arrival,
                      "duration": duration, "settled": response["settled"],
                      "settling_time": response["settling_time"],
                      "plateau_center": center, "plateau_hold_fraction": hold,
                      "terminal_mae": mean(abs(value - target) for value in tail)})
    by_direction = {step["direction"]: step for step in steps}
    mean_response = {}
    if phase in ("i", "validation"):
        if phase == "i" and (i_score_start is None or not
                             math.isfinite(i_score_start) or i_score_start < 0):
            raise ValueError("Position I scoring needs the selected D settling time")
        for direction, sign in (("positive", 1), ("negative", -1)):
            trace = mean_traces[direction]
            if phase == "validation":
                start = by_direction[direction]["settling_time"]
                window_valid = bool(by_direction[direction]["settled"])
            else:
                start = i_score_start
                window_valid = trace[-1][0] - start >= 0.5
            scored = [value for t, value in trace if t >= start] if window_valid else []
            target = sign * requested
            mean_response[direction] = {
                "score_start": start, "score_window_valid": bool(scored),
                "hold_fraction": (mean(abs(value - target) <= band for value in scored)
                                  if scored else 0.0),
                "mae": (mean(abs(value - target) for value in scored)
                        if scored else requested),
                "bias": (mean(value - target for value in scored)
                         if scored else -target),
            }
    if phase == "p":
        score = mean((abs(step["arrival_time"] -
                          config["position_p_arrival_seconds"]) /
                      config["position_p_arrival_seconds"]
                      if step["arrived"] else 3.0) for step in steps)
    elif phase == "d":
        score = mean(step["settling_time"] / config["position_d_trial_seconds"] +
                     2 * (1 - step["plateau_hold_fraction"]) +
                     3 * (not step["settled"]) +
                     3 * (sign * step["plateau_center"] < requested *
                          config["position_d_min_progress_fraction"])
                     for step, sign in zip(steps, (1, -1)))
    else:
        score = mean(3 * (1 - part["hold_fraction"]) +
                     part["mae"] / band + 3 * (not part["score_window_valid"])
                     for part in mean_response.values())
    residuals = []
    for step in steps:
        trace = mean_traces[step["direction"]]
        start = (step["settling_time"] if step["settled"] else
                 max(0.0, trace[-1][0] - 1.0))
        residuals.extend((value - step["plateau_center"]) ** 2
                         for t, value in trace if t >= start)
    return {
        "method": "position_repeated_response", "phase": phase,
        "score": score, "repeat_count": repeats, "distance": requested,
        "mean_steps": steps, "mean_response_by_direction": mean_response,
        "reached_steps": sum(step["arrived"] for step in steps),
        "mean_arrival_seconds": (mean(step["arrival_time"] for step in steps
                                      if step["arrived"])
                                 if any(step["arrived"] for step in steps) else None),
        "plateau_settling_time": mean(step["settling_time"] for step in steps),
        "plateau_settled_fraction": mean(step["settled"] for step in steps),
        "hold_fraction": (mean(part["hold_fraction"] for part in
                               mean_response.values()) if mean_response else 0.0),
        "hold_fraction_by_direction": {
            direction: part["hold_fraction"] for direction, part in
            mean_response.items()},
        "i_score_start_seconds": i_score_start if phase == "i" else None,
        "terminal_mae": mean(step["terminal_mae"] for step in steps),
        "oscillation_rms": math.sqrt(mean(residuals)) if residuals else 0.0,
        "validation_passed": (all(step["arrived"] for step in steps) and
                              all(part["score_window_valid"] and
                                  part["hold_fraction"] >=
                                  config["position_i_required_fraction"]
                                  for part in mean_response.values()))
        if phase == "validation" else None,
    }


def _position_p_ready(metrics: dict[str, Any], config: dict[str, Any]) -> bool:
    return (len(metrics.get("mean_steps", [])) == 2 and all(
        step["arrived"] and abs(step["arrival_time"] -
                                config["position_p_arrival_seconds"]) <=
        config["position_p_time_tolerance_seconds"]
        for step in metrics["mean_steps"]))


def _position_d_ready(metrics: dict[str, Any], config: dict[str, Any]) -> bool:
    return (len(metrics.get("mean_steps", [])) == 2 and all(
        step["settled"] and step["plateau_hold_fraction"] >=
        config["position_d_required_fraction"] and
        (1 if step["direction"] == "positive" else -1) *
        step["plateau_center"] >= config["position_target_distance"] *
        config["position_d_min_progress_fraction"]
        for step in metrics["mean_steps"]))


def _position_i_ready(metrics: dict[str, Any], config: dict[str, Any]) -> bool:
    parts = metrics.get("mean_response_by_direction", {})
    return (len(parts) == 2 and all(part["score_window_valid"] and
            part["hold_fraction"] >= config["position_i_required_fraction"]
            for part in parts.values()))


def _velocity_repeated_stable(metrics: dict[str, Any], config: dict[str, Any]) -> bool:
    parts = metrics.get("i_mean_response_by_direction", {})
    return (metrics.get("phase") == "i" and len(parts) == 2 and
            all(part["score_window_valid"] and part["hold_fraction"] >=
                config["velocity_i_mean_required_fraction"]
                for part in parts.values()))


def _velocity_repeated_p_ready(metrics: dict[str, Any],
                               config: dict[str, Any]) -> bool:
    """Require the mean response in each direction to arrive in the P window."""
    for plan in config["velocity_p_targets"]:
        for direction in ("positive", "negative"):
            averaged = [part for part in metrics.get("mean_steps", [])
                        if part["speed"] == plan["speed"] and
                        part["direction"] == direction]
            if (len(averaged) != 1 or not averaged[0]["arrived"] or
                    abs(averaged[0]["arrival_time"] - plan["arrival_seconds"]) >
                    plan["tolerance_seconds"]):
                return False
    return True


def _velocity_repeated_d_ready(metrics: dict[str, Any],
                               config: dict[str, Any]) -> bool:
    steps = metrics.get("mean_steps", [])
    return (len(steps) == 2 and
            all(part["settled"] and
                part["settling_time"] <= config["velocity_d_settle_target_seconds"] and
                part["plateau_fraction"] >= config["velocity_d_min_plateau_fraction"]
                for part in steps))


def _prefer_trial(candidate: dict[str, Any], current: dict[str, Any],
                  stage: str, amplitude: float, config: dict[str, Any],
                  *, joint: bool = False) -> bool:
    limit = config["oscillation_tolerances"][stage]

    def stable(metrics: dict[str, Any]) -> bool:
        axes = (metrics["x"], metrics["y"]) if joint else (metrics,)
        return all((_velocity_response_stable(axis, amplitude, config)
                    if stage == "velocity" else
                     _acceleration_response_stable(axis, config)
                     if stage == "acceleration" else
                     _height_stage2_stable(axis, config)
                     if stage == "height" and "waypoints" in axis else
                     _yaw_stage_stable(axis, config)
                     if stage == "yaw" and "waypoints" in axis else
                     _position_waypoints_stable(axis, config)
                     if stage == "position" and "waypoints" in axis else
                     axis["oscillation_rms"] <= limit and
                     (stage != "height" or
                      (axis.get("waypoints_reached_fraction", 1.0) == 1.0 and
                       axis.get("descent_reached_fraction", 1.0) == 1.0 and
                      axis.get("peak_descent_speed", 0.0) <=
                      config["height_descent_max_speed"] and
                      axis.get("descent_oscillation_rms", 0.0) <= limit)))
                   for axis in axes)

    new_stable, old_stable = stable(candidate), stable(current)
    if old_stable and not new_stable:
        return False
    if new_stable and not old_stable:
        if (stage == "acceleration" or
                stage in ("height", "yaw", "position") and "waypoints" in candidate):
            return True
        return candidate["score"] <= current["score"] * (1 + config["min_improvement"])
    if stage == "acceleration" and not (new_stable and old_stable):
        def response_rank(metrics: dict[str, Any]) -> tuple[int, float, float]:
            # Until a PID is stable, first seek a response that reaches both
            # polarities and ends nearer the requested acceleration. Only
            # then use oscillation and jerk penalties in the scalar score.
            return (
                int(metrics.get("reached_directions", sum(
                    int(metrics.get(part, {}).get("reached", False))
                    for part in ("positive", "negative")))),
                -metrics["terminal_mae"],
                metrics["in_band_fraction"],
            )
        candidate_rank = response_rank(candidate)
        current_rank = response_rank(current)
        if candidate_rank != current_rank:
            return candidate_rank > current_rank
    return candidate["score"] < current["score"] * (1 - config["min_improvement"])


def _height_stage2_stable(metrics: dict[str, Any],
                          config: dict[str, Any]) -> bool:
    return (metrics["waypoints_reached_fraction"] == 1.0
            and metrics["max_overshoot"] <=
            config["height_stage2_max_overshoot_m"]
            and all(point["settling_time"] is not None
                    for point in metrics["waypoints"])
            and metrics["oscillation_rms"] <=
            config["oscillation_tolerances"]["height"]
            and metrics["peak_descent_speed"] <=
            config["height_descent_max_speed"]
            and metrics["descent_oscillation_rms"] <=
            config["oscillation_tolerances"]["height"])


def _yaw_stage_stable(metrics: dict[str, Any],
                      config: dict[str, Any]) -> bool:
    return (metrics["waypoints_reached_fraction"] == 1.0
            and metrics["max_overshoot"] <=
            math.radians(config["yaw_stage_max_overshoot_deg"])
            and all(point["settling_time"] is not None
                    for point in metrics["waypoints"])
            and metrics["oscillation_rms"] <=
            config["oscillation_tolerances"]["yaw"]
            and metrics["peak_yaw_rate_windowed"] <=
            config["safety"]["max_yaw_rate"]
            and metrics["max_height_terminal_error"] <=
            config["yaw_stage_height_tolerance_m"])


def _plateau_after_arrival(samples: list[tuple[float, float]], arrival: float | None,
                           band: float, required_fraction: float
                           ) -> dict[str, Any]:
    """Find the first quiet suffix around the response's own final level."""
    if not samples:
        return {"center": 0.0, "settled": False, "settling_time": 0.0,
                "hold_fraction": 0.0, "oscillation_rms": 10.0 * band}
    duration = samples[-1][0]
    tail_start = max(0.0, duration - min(1.5, duration / 3))
    tail = [value for elapsed, value in samples if elapsed >= tail_start]
    center = median(tail or [samples[-1][1]])
    eligible = [(elapsed, value) for elapsed, value in samples
                if arrival is None or elapsed >= arrival]
    if len(eligible) < 3:
        return {"center": center, "settled": False, "settling_time": duration,
                "hold_fraction": 0.0, "oscillation_rms": 10.0 * band}
    settled_at: float | None = None
    hold_fraction = 0.0
    for index, (start, _) in enumerate(eligible):
        remaining = eligible[index:]
        if remaining[-1][0] - start < 1.0:
            break
        flags = [abs(value - center) <= band for _, value in remaining]
        fraction = mean(flags)
        if fraction < required_fraction:
            continue
        windows_ok = True
        cursor = start
        while cursor <= remaining[-1][0]:
            window = [abs(value - center) <= band for elapsed, value in remaining
                      if cursor <= elapsed < cursor + 0.5]
            if len(window) >= 3 and mean(window) < 0.70:
                windows_ok = False
                break
            cursor += 0.5
        if windows_ok:
            settled_at = start
            hold_fraction = fraction
            break
    score_start = duration if settled_at is None else settled_at
    quiet = [value for elapsed, value in samples if elapsed >= score_start]
    rms = (math.sqrt(mean((value - center) ** 2 for value in quiet))
           if quiet else math.inf)
    return {"center": center, "settled": settled_at is not None,
            "settling_time": score_start, "hold_fraction": hold_fraction,
            "oscillation_rms": rms}


def _height_repeated_p_ready(metrics: dict[str, Any],
                             config: dict[str, Any]) -> bool:
    return (metrics.get("reached_trials") == config["vertical_repeats"] and
            metrics.get("mean_arrival_seconds") is not None and
            abs(metrics["mean_arrival_seconds"] -
                config["height_p_arrival_seconds"]) <=
            config["height_p_time_tolerance_seconds"])


def _height_repeated_d_ready(metrics: dict[str, Any],
                             config: dict[str, Any]) -> bool:
    return (metrics.get("reached_trials") == config["vertical_repeats"] and
            metrics.get("settled_trials") == config["vertical_repeats"] and
            metrics.get("plateau_hold_fraction", 0.0) >=
            config["height_d_required_fraction"] and
            metrics.get("oscillation_rms", math.inf) <=
            config["height_d_band_m"])


def _height_repeated_i_ready(metrics: dict[str, Any],
                             config: dict[str, Any]) -> bool:
    return (metrics.get("reached_trials") == config["vertical_repeats"] and
            metrics.get("target_hold_fraction", 0.0) >=
            config["height_i_required_fraction"])


def _yaw_repeated_p_ready(metrics: dict[str, Any],
                          config: dict[str, Any]) -> bool:
    parts = metrics.get("mean_response_by_direction", {})
    return (len(parts) == 2 and all(
        part["reached_count"] == config["vertical_repeats"] and
        abs(part["mean_arrival_seconds"] - config["yaw_p_arrival_seconds"]) <=
        config["yaw_p_time_tolerance_seconds"] for part in parts.values()))


def _yaw_repeated_d_ready(metrics: dict[str, Any],
                          config: dict[str, Any]) -> bool:
    parts = metrics.get("mean_response_by_direction", {})
    return (len(parts) == 2 and all(
        part["reached_count"] == config["vertical_repeats"] and
        part["settled_count"] == config["vertical_repeats"] and
        part["plateau_hold_fraction"] >= config["yaw_d_required_fraction"] and
        part["oscillation_rms"] <= math.radians(config["yaw_d_band_deg"])
        for part in parts.values()))


def _yaw_repeated_i_ready(metrics: dict[str, Any],
                          config: dict[str, Any]) -> bool:
    parts = metrics.get("mean_response_by_direction", {})
    return (len(parts) == 2 and all(
        part["reached_count"] == config["vertical_repeats"] and
        part["target_hold_fraction"] >= config["yaw_i_required_fraction"]
        for part in parts.values()))


def _height_descent_metrics(rows: list[dict[str, Any]]) -> dict[str, float]:
    steps: dict[int, list[dict[str, Any]]] = {}
    for row in rows:
        index = row.get("height_step_index")
        if isinstance(index, int) and index > 0:
            steps.setdefault(index, []).append(row)
    peak_speed = 0.0
    peak_oscillation = 0.0
    for step_rows in steps.values():
        for row in step_rows:
            older = next((past for past in reversed(step_rows)
                          if 0.16 <= row["t"] - past["t"] <= 0.4), None)
            if older is not None:
                peak_speed = max(peak_speed,
                                 max(0.0, older["z"] - row["z"]) /
                                 (row["t"] - older["t"]))
        oscillation, _, _ = _oscillation_metrics(
            step_rows, "height", "z", 0.05, 1.0)
        peak_oscillation = max(peak_oscillation, oscillation)
    return {
        "descent_steps": len(steps),
        "descent_reached_fraction": (mean(bool(group[-1]["height_step_reached"])
                                            for group in steps.values()) if steps else 1.0),
        "descent_progressed_fraction": (mean(
            (group[-1].get("height_step_progressed") in (1, True, "1"))
            if group[-1].get("height_step_progressed") not in (None, "")
            else bool(group[-1]["height_step_reached"])
            for group in steps.values()) if steps else 1.0),
        "peak_descent_speed": peak_speed,
        "descent_oscillation_rms": peak_oscillation,
    }


def score_rows(rows: list[dict[str, Any]], stage: str, axis: str,
                amplitude: float, *, oscillation_window_seconds: float = 2.5,
                oscillation_weight: float = 3.0,
                height_descent_max_speed: float = 0.3,
                height_descent_oscillation_limit: float = 0.03) -> dict[str, float]:
    if not rows:
        raise ValueError("No telemetry samples to score")
    errors = []
    overshoots = []
    terminal = []
    chatter = []
    previous_rc = None
    previous_row = None
    peak_xy_speed = 0.0
    peak_yaw_rate = 0.0
    for row in rows:
        if previous_row is not None:
            dt = row["t"] - previous_row["t"]
            if dt > 1e-3:
                peak_xy_speed = max(
                    peak_xy_speed,
                    math.hypot(row["x"] - previous_row["x"],
                               row["y"] - previous_row["y"]) / dt,
                )
                peak_yaw_rate = max(
                    peak_yaw_rate,
                    abs(wrap_pi(row["yaw"] - previous_row["yaw"])) / dt,
                )
        previous_row = row
        if stage == "height":
            target, actual, origin = row["target_z"], row["z"], row["origin_z"]
        elif stage == "yaw":
            target, actual, origin = row["target_yaw"], row["yaw"], row["origin_yaw"]
        elif stage == "velocity":
            target = row[f"command_v{axis}"]
            actual = row[f"v{axis}_body"]
            origin = 0.0
        elif stage == "position":
            target, actual, origin = row[f"target_{axis}"], row[axis], row[f"origin_{axis}"]
        else:
            raise ValueError(stage)
        error = wrap_pi(target - actual) if stage == "yaw" else target - actual
        errors.append(abs(error))
        direction = wrap_pi(target - origin) if stage == "yaw" else target - origin
        if abs(direction) > 1e-6:
            beyond = wrap_pi(actual - target) if stage == "yaw" else actual - target
            overshoots.append(max(0.0, beyond * math.copysign(1.0, direction)))
        if row["segment_elapsed"] >= max(0.0, row["segment_seconds"] - 1.0):
            terminal.append(abs(error))
        rc = (row["rc_roll"], row["rc_pitch"], row["rc_throttle"], row["rc_yaw"])
        if previous_rc is not None:
            chatter.append(sum(abs(a - b) for a, b in zip(rc, previous_rc)) / 400)
        previous_rc = rc
    normalizer = max(amplitude, 1e-6)
    oscillation_rms, oscillation_range, oscillation_cycles = _oscillation_metrics(
        rows, stage, axis, amplitude, oscillation_window_seconds
    )
    metrics = {
        "mae": mean(errors),
        "terminal_mae": mean(terminal or errors),
        "max_overshoot": max(overshoots, default=0.0),
        "saturation_fraction": mean(row["saturated"] for row in rows),
        "rc_chatter": mean(chatter) if chatter else 0.0,
        "oscillation_rms": oscillation_rms,
        "oscillation_peak_to_peak": oscillation_range,
        "oscillation_cycles": oscillation_cycles,
        "peak_xy_speed": peak_xy_speed,
        "peak_yaw_rate": peak_yaw_rate,
        "max_xy_drift": max(
            math.hypot(row["x"] - row["origin_x"],
                       row["y"] - row["origin_y"]) for row in rows
        ),
        "warning_fraction": mean(bool(row["warnings"]) for row in rows),
        "max_sample_gap": max(row["sample_gap"] for row in rows),
        "samples": len(rows),
    }
    metrics["score"] = (
        metrics["mae"] / normalizer
        + 3.0 * metrics["terminal_mae"] / normalizer
        + 0.4 * metrics["max_overshoot"] / normalizer
        + oscillation_weight * metrics["oscillation_rms"] / normalizer
        + 0.25 * metrics["saturation_fraction"]
        + 0.1 * metrics["rc_chatter"]
    )
    if stage == "height":
        metrics.update(_height_descent_metrics(rows))
        metrics["score"] += (
            2.0 * (1.0 - metrics["descent_reached_fraction"])
            + 2.0 * max(0.0, metrics["peak_descent_speed"] /
                        height_descent_max_speed - 1.0)
            + max(0.0, metrics["descent_oscillation_rms"] /
                  height_descent_oscillation_limit - 1.0)
        )
    return metrics


def score_height_waypoints(rows: list[dict[str, Any]],
                           config: dict[str, Any],
                           targets: list[float] | None = None) -> dict[str, Any]:
    """Give every commanded height equal weight, independent of descent duration."""
    targets = config["height_stage2_targets"] if targets is None else targets
    metrics = score_rows(
        rows, "height", "z", max(targets) - min(targets),
        oscillation_window_seconds=config["oscillation_window_seconds"],
        oscillation_weight=config["oscillation_weight"],
        height_descent_max_speed=config["height_descent_max_speed"],
        height_descent_oscillation_limit=config["oscillation_tolerances"]["height"])
    waypoint_metrics = []
    tolerance = config["height_stage2_tolerance"]
    osc_limit = config["oscillation_tolerances"]["height"]
    for index, height in enumerate(targets, 1):
        segment = [row for row in rows if row.get("waypoint_index") == index]
        hold = [row for row in segment if row["segment"] == f"height_wp_{index:02d}_hold"]
        if not hold:
            raise TrialInvalid(f"No hold telemetry for height waypoint {index}")
        requested = hold[-1]["requested_target_z"]
        tail = [row for row in hold if row["t"] >= hold[-1]["t"] - 1.0]
        terminal_error = mean(abs(row["z"] - requested) for row in tail)
        hold_error = mean(abs(row["z"] - requested) for row in hold)
        oscillation, peak_to_peak, _ = _oscillation_metrics(
            hold, "height", "z", tolerance,
            config["oscillation_window_seconds"])
        direction = 1 if requested >= segment[0]["z"] else -1
        overshoot = max(max(0.0, (row["z"] - requested) * direction)
                        for row in segment)
        reached = (max(abs(row["z"] - requested) for row in tail) <= tolerance)
        arrival_time = next((row["t"] - hold[0]["t"] for row in hold
                             if abs(row["z"] - requested) <= tolerance), None)
        settling_time = None
        inside_since = True
        for row in reversed(hold):
            inside_since &= abs(row["z"] - requested) <= tolerance
            if (inside_since and hold[-1]["t"] - row["t"] >=
                    config["height_stage2_settle_seconds"]):
                settling_time = row["t"] - hold[0]["t"]
        hold_duration = hold[-1]["t"] - hold[0]["t"]
        waypoint_metrics.append({
            "height": height, "target_raw_z": requested,
            "reached": reached, "hold_mae": hold_error,
            "terminal_mae": terminal_error, "oscillation_rms": oscillation,
            "oscillation_peak_to_peak": peak_to_peak,
            "max_overshoot": overshoot, "arrival_time": arrival_time,
            "settling_time": settling_time, "hold_duration": hold_duration,
            "terminal_bias": mean(row["z"] - requested for row in tail),
        })
    metrics["waypoints"] = waypoint_metrics
    metrics["waypoints_reached_fraction"] = mean(
        item["reached"] for item in waypoint_metrics)
    metrics["mae"] = mean(item["hold_mae"] for item in waypoint_metrics)
    metrics["terminal_mae"] = mean(item["terminal_mae"] for item in waypoint_metrics)
    metrics["oscillation_rms"] = max(item["oscillation_rms"] for item in waypoint_metrics)
    metrics["max_overshoot"] = max(item["max_overshoot"] for item in waypoint_metrics)
    metrics["max_settling_time"] = max(
        item["settling_time"] if item["settling_time"] is not None
        else item["hold_duration"] + config["height_stage2_settle_seconds"]
        for item in waypoint_metrics)
    max_overshoot = config["height_stage2_max_overshoot_m"]
    metrics["score"] = mean(
        2.0 * item["terminal_mae"] / tolerance +
        item["hold_mae"] / tolerance +
        item["oscillation_rms"] / osc_limit +
        2.0 * min(item["max_overshoot"] / max_overshoot, 1.0) +
        5.0 * max(0.0, item["max_overshoot"] / max_overshoot - 1.0) +
        1.5 * ((item["settling_time"] if item["settling_time"] is not None
                else item["hold_duration"] + config["height_stage2_settle_seconds"])
               / max(item["hold_duration"], 1e-6)) +
        (0.0 if item["reached"] else 4.0)
        for item in waypoint_metrics)
    metrics["score"] += (
        2.0 * (1.0 - metrics["descent_progressed_fraction"]) +
        2.0 * max(0.0, metrics["peak_descent_speed"] /
                  config["height_descent_max_speed"] - 1.0) +
        max(0.0, metrics["descent_oscillation_rms"] / osc_limit - 1.0) +
        0.25 * metrics["saturation_fraction"] +
        0.1 * metrics["rc_chatter"])
    return metrics


def score_yaw_waypoints(rows: list[dict[str, Any]],
                        config: dict[str, Any]) -> dict[str, Any]:
    """Score wrapped heading response at each angle, including returns to zero."""
    angles = config["yaw_stage_targets_deg"]
    tolerance = math.radians(config["yaw_stage_tolerance_deg"])
    overshoot_limit = math.radians(config["yaw_stage_max_overshoot_deg"])
    osc_limit = config["oscillation_tolerances"]["yaw"]
    metrics = score_rows(rows, "yaw", "yaw", math.radians(max(abs(a) for a in angles)),
                         oscillation_window_seconds=config["oscillation_window_seconds"],
                         oscillation_weight=config["oscillation_weight"])
    points = []
    peak_rate = 0.0
    for index, angle in enumerate(angles, 1):
        hold = [row for row in rows if row.get("waypoint_index") == index and
                row["segment"] == f"yaw_wp_{index:02d}_hold"]
        if not hold:
            raise TrialInvalid(f"No telemetry for yaw waypoint {index}")
        target = hold[-1]["requested_target_yaw"]
        errors = [abs(wrap_pi(row["yaw"] - target)) for row in hold]
        end = hold[-1]["t"]
        tail = [row for row in hold if row["t"] >= end - 1.0]
        terminal = [abs(wrap_pi(row["yaw"] - target)) for row in tail]
        direction = wrap_pi(target - hold[0]["yaw"])
        overshoot = (max(max(0.0, wrap_pi(row["yaw"] - target) *
                             math.copysign(1.0, direction)) for row in hold)
                     if abs(direction) > 1e-6 else 0.0)
        osc, peak_to_peak, _ = _oscillation_metrics(
            hold, "yaw", "yaw", abs(direction), config["oscillation_window_seconds"])
        arrival = next((row["t"] - hold[0]["t"] for row in hold
                        if abs(wrap_pi(row["yaw"] - target)) <= tolerance), None)
        settling = None
        inside_since = True
        for row in reversed(hold):
            inside_since &= abs(wrap_pi(row["yaw"] - target)) <= tolerance
            if inside_since and end - row["t"] >= config["yaw_stage_settle_seconds"]:
                settling = row["t"] - hold[0]["t"]
        for pos, row in enumerate(hold):
            older = next((past for past in reversed(hold[:pos])
                          if 0.16 <= row["t"] - past["t"] <= 0.4), None)
            if older is not None:
                peak_rate = max(peak_rate, abs(wrap_pi(row["yaw"] - older["yaw"])) /
                                (row["t"] - older["t"]))
        points.append({
            "angle_deg": angle, "target_yaw": target,
            "reached": max(terminal) <= tolerance,
            "hold_mae": mean(errors), "terminal_mae": mean(terminal),
            "terminal_bias": mean(wrap_pi(row["yaw"] - target) for row in tail),
            "max_overshoot": overshoot, "oscillation_rms": osc,
            "oscillation_peak_to_peak": peak_to_peak,
            "arrival_time": arrival, "settling_time": settling,
            "hold_duration": end - hold[0]["t"],
            "height_hold_mae": mean(abs(row["z"] - row["target_z"]) for row in hold),
            "height_terminal_mae": mean(abs(row["z"] - row["target_z"]) for row in tail),
        })
    metrics["waypoints"] = points
    metrics["waypoints_reached_fraction"] = mean(point["reached"] for point in points)
    metrics["mae"] = mean(point["hold_mae"] for point in points)
    metrics["terminal_mae"] = mean(point["terminal_mae"] for point in points)
    metrics["max_overshoot"] = max(point["max_overshoot"] for point in points)
    metrics["oscillation_rms"] = max(point["oscillation_rms"] for point in points)
    metrics["peak_yaw_rate_windowed"] = peak_rate
    metrics["max_height_terminal_error"] = max(
        point["height_terminal_mae"] for point in points)
    metrics["max_settling_time"] = max(
        point["settling_time"] if point["settling_time"] is not None
        else point["hold_duration"] + config["yaw_stage_settle_seconds"]
        for point in points)
    metrics["score"] = mean(
        2.0 * point["terminal_mae"] / tolerance +
        point["hold_mae"] / tolerance +
        point["oscillation_rms"] / osc_limit +
        2.0 * min(point["max_overshoot"] / overshoot_limit, 1.0) +
        5.0 * max(0.0, point["max_overshoot"] / overshoot_limit - 1.0) +
        1.5 * ((point["settling_time"] if point["settling_time"] is not None
                else point["hold_duration"] + config["yaw_stage_settle_seconds"])
               / max(point["hold_duration"], 1e-6)) +
        (0.0 if point["reached"] else 4.0)
        for point in points)
    metrics["score"] += (
        2.0 * max(0.0, peak_rate / config["safety"]["max_yaw_rate"] - 1.0) +
        2.0 * max(0.0, metrics["max_height_terminal_error"] /
                  config["yaw_stage_height_tolerance_m"] - 1.0) +
        0.25 * metrics["saturation_fraction"] + 0.1 * metrics["rc_chatter"])
    return metrics


def score_position_waypoints(rows: list[dict[str, Any]], axis: str,
                             config: dict[str, Any]) -> dict[str, Any]:
    """Score every move and return in one isolated or joint position trial."""
    tolerance = config["position_calibration_tolerance_m"]
    overshoot_limit = config["position_calibration_max_overshoot_m"]
    osc_limit = config["oscillation_tolerances"]["position"]
    points = []
    indices = sorted({row["waypoint_index"] for row in rows
                      if isinstance(row.get("waypoint_index"), int)})
    if not indices:
        raise TrialInvalid("No position waypoints were recorded")
    for index in indices:
        hold = [row for row in rows if row.get("waypoint_index") == index]
        target = hold[-1][f"target_{axis}"]
        direction = target - hold[0][axis]
        end = hold[-1]["t"]
        tail = [row for row in hold if row["t"] >= end - 1.0]
        errors = [abs(row[axis] - target) for row in hold]
        overshoot = (max(max(0.0, (row[axis] - target) *
                             math.copysign(1.0, direction)) for row in hold)
                     if abs(direction) > 1e-6 else 0.0)
        osc, peak_to_peak, _ = _oscillation_metrics(
            hold, "position", axis, abs(direction),
            config["oscillation_window_seconds"])
        arrival = next((row["t"] - hold[0]["t"] for row in hold
                        if abs(row[axis] - target) <= tolerance), None)
        settling = None
        inside_since = True
        for row in reversed(hold):
            inside_since &= abs(row[axis] - target) <= tolerance
            if inside_since and end - row["t"] >= config["position_calibration_settle_seconds"]:
                settling = row["t"] - hold[0]["t"]
        points.append({
            "distance": hold[-1]["requested_distance"],
            "target": target, "return": hold[-1]["waypoint_return"],
            "reached": max(abs(row[axis] - target) for row in tail) <= tolerance,
            "hold_mae": mean(errors),
            "terminal_mae": mean(abs(row[axis] - target) for row in tail),
            "terminal_bias": mean(row[axis] - target for row in tail),
            "max_overshoot": overshoot,
            "oscillation_rms": osc, "oscillation_peak_to_peak": peak_to_peak,
            "arrival_time": arrival, "settling_time": settling,
            "hold_duration": end - hold[0]["t"],
            "height_terminal_mae": mean(abs(row["z"] - row["target_z"])
                                        for row in tail),
        })
    score = mean(
        point["hold_mae"] / tolerance +
        2.0 * point["terminal_mae"] / tolerance +
        point["oscillation_rms"] / osc_limit +
        2.0 * min(point["max_overshoot"] / overshoot_limit, 1.0) +
        5.0 * max(0.0, point["max_overshoot"] / overshoot_limit - 1.0) +
        (point["settling_time"] if point["settling_time"] is not None
         else point["hold_duration"] + config["position_calibration_settle_seconds"])
        / max(point["hold_duration"], 1e-6) +
        (0.0 if point["reached"] else 4.0)
        for point in points)
    metrics = {
        "method": "position_waypoint_response", "waypoints": points,
        "waypoints_reached_fraction": mean(point["reached"] for point in points),
        "mae": mean(point["hold_mae"] for point in points),
        "terminal_mae": mean(point["terminal_mae"] for point in points),
        "max_overshoot": max(point["max_overshoot"] for point in points),
        "oscillation_rms": max(point["oscillation_rms"] for point in points),
        "max_height_terminal_error": max(point["height_terminal_mae"]
                                         for point in points),
        "saturation_fraction": mean(row["saturated"] for row in rows),
        "samples": len(rows),
    }
    metrics["score"] = score + 0.25 * metrics["saturation_fraction"]
    return metrics


def _position_waypoints_stable(metrics: dict[str, Any],
                               config: dict[str, Any]) -> bool:
    return (metrics["waypoints_reached_fraction"] == 1.0
            and metrics["max_overshoot"] <=
            config["position_calibration_max_overshoot_m"]
            and all(point["settling_time"] is not None
                    for point in metrics["waypoints"])
            and metrics["oscillation_rms"] <=
            config["oscillation_tolerances"]["position"])


class CalibrationRunner:
    """Sequentially evaluates height, yaw, XY velocity, then XY position."""

    def __init__(self, config: dict[str, Any], link: SimulatorLink | None = None):
        self.config = config
        self.started_at = datetime.now(timezone.utc).isoformat()
        self.seed = get_drone_pid_setup(config["drone_name"])
        self.seed.update(acceleration_pid_defaults(config))
        self.seed["pid_yaw"]["max_control"] = min(
            self.seed["pid_yaw"]["max_control"], config["max_yaw_rc_offset"] / 100
        )
        for name in ("pid_pos_x", "pid_pos_y"):
            self.seed[name]["max_control"] = min(
                self.seed[name]["max_control"], config["max_xy_speed"]
            )
        self.best = copy.deepcopy(self.seed)
        self.best_height_base_rc = config["height_base_throttle_rc"]
        self.height_waypoints_mode = False
        self.yaw_waypoints_mode = False
        self.xy_waypoints_mode = False
        self.xy_reference_yaw: float | None = None
        self.controller = CalibrationController(config, self.best)
        self.link = link or SimulatorLink(config)
        self.run_dir = Path(config["output_dir"]).expanduser().resolve() / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        self.initial_sample: Sample | None = None
        # Fixed across stages in one named calibration session. Height trials
        # use their own takeoff origin; yaw and XY keep this original origin.
        self.session_start_z: float | None = None
        self.takeoff_start_z: float | None = None
        self.launch_xy: tuple[float, float] | None = None
        self.anchor: Sample | None = None
        self.last_sample: Sample | None = None
        self.records: list[dict[str, Any]] = []
        self.recoveries: list[dict[str, Any]] = []
        self.interventions: list[dict[str, Any]] = []
        self.stage_summary: dict[str, Any] = {}
        self.status = "created"
        self.failure: str | None = None
        self.resume_sources: dict[str, str] = {}
        self.file_number = 0
        self.last_csv: str | None = None

    def _save_summary(self) -> None:
        summary = {
            "status": self.status,
            "started_at": self.started_at,
            "updated_at": datetime.now(timezone.utc).isoformat(),
            "process_id": os.getpid(),
            "failure": self.failure,
            "last_csv": self.last_csv,
            "z_bias": self.controller.z_bias,
            "session_start_z": self.session_start_z,
            "takeoff_start_z": self.takeoff_start_z,
            "config": self.config,
            "tool_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            "pid_source_sha256": hashlib.sha256(
                (PACKAGE_ROOT / "pid.py").read_bytes()
            ).hexdigest(),
            "preset_source_sha256": hashlib.sha256(
                (PACKAGE_ROOT / "utils" / "drone_setups.py").read_bytes()
            ).hexdigest(),
            "initial_pids": printable_pids(self.seed),
            "recommended_pids": printable_pids(self.best),
            "recommended_height_base_throttle_rc": self.best_height_base_rc,
            "xy_reference_origin": self.launch_xy,
            "xy_reference_yaw": self.xy_reference_yaw,
            "resume_sources": self.resume_sources,
            "stages": self.stage_summary,
            "recoveries": self.recoveries,
            "interventions": self.interventions,
            "evaluations": self.records,
            "note": "Review and transfer coefficients manually; no preset was edited.",
        }
        destination = self.run_dir / "summary.json"
        temporary = self.run_dir / "summary.json.tmp"
        temporary.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
        for attempt in range(5):
            try:
                temporary.replace(destination)
                break
            except PermissionError:
                if attempt == 4:
                    raise
                # A viewer or file indexer can briefly hold summary.json on
                # Windows. Keep the completed trial and retry the replace.
                time.sleep(0.05 * (attempt + 1))

    def _write_csv(self, label: str, rows: list[dict[str, Any]]) -> str:
        self.file_number += 1
        name = f"{self.file_number:04d}_{label}.csv"
        if rows:
            with (self.run_dir / name).open("w", newline="", encoding="utf-8") as handle:
                fieldnames = list(dict.fromkeys(
                    key for row in rows for key in row))
                writer = csv.DictWriter(handle, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(rows)
        self.last_csv = name
        return name

    def _check_sample(self, sample: Sample, stage: str, *, allow_low: bool = False) -> str:
        safety = self.config["safety"]
        if self.link.stream_error is not None:
            raise ConnectionError(f"RC stream failed: {self.link.stream_error}")
        warnings = []
        if self.last_sample is not None:
            gap = sample.t - self.last_sample.t
            if gap > safety["max_sample_gap"]:
                warnings.append("sample_gap")
            if gap > 1e-3:
                yaw_rate = abs(wrap_pi(sample.yaw - self.last_sample.yaw) / gap)
                if yaw_rate > safety["max_yaw_rate"]:
                    warnings.append("yaw_rate")
        if stage in ("acceleration", "velocity", "position", "validation") and self.initial_sample is not None:
            radius = math.hypot(sample.x - self.initial_sample.x, sample.y - self.initial_sample.y)
            if radius > safety["max_xy_radius"]:
                warnings.append("xy_radius")
        if sample.z > safety["max_height"]:
            warnings.append("high_altitude")
        if not allow_low and sample.z < safety["min_airborne_height"]:
            warnings.append("low_altitude")
        if max(abs(sample.roll), abs(sample.pitch)) > math.radians(safety["max_tilt_deg"]):
            warnings.append("tilt")
        self.last_sample = sample
        return ",".join(warnings)

    def _target(self, *, height: float | None = None, height_relative: bool | None = None,
                yaw: float | None = None,
                position: tuple[float, float] | None = None,
                velocity: tuple[float, float] = (0.0, 0.0),
                acceleration: tuple[float, float] = (0.0, 0.0),
                active_axis: str | None = None) -> dict[str, Any]:
        anchor = self.anchor
        assert anchor is not None
        return {
            "height": anchor.z if height is None else height,
            "height_relative": ((self.yaw_waypoints_mode or self.xy_waypoints_mode)
                                if height_relative is None
                                else height_relative),
            "yaw": anchor.yaw if yaw is None else wrap_pi(yaw),
            "position": (anchor.x, anchor.y) if position is None else position,
            "velocity": velocity,
            "acceleration": acceleration,
            "active_axis": active_axis,
        }

    def _xy_heading(self) -> float:
        """Return the fixed heading selected at calibration start."""
        if self.xy_reference_yaw is not None:
            return self.xy_reference_yaw
        sample = self.initial_sample or self.anchor or self.last_sample
        if sample is None:
            raise RuntimeError("XY reference heading is unavailable")
        return sample.yaw

    def _intervention_reason(self, sample: Sample) -> str | None:
        previous = self.last_sample
        if previous is None:
            return None
        gap = sample.t - previous.t
        if gap <= 0:
            return None
        multiplier = 1 if gap <= self.config["jump_max_gap_seconds"] else 4
        xy_jump = math.hypot(sample.x - previous.x, sample.y - previous.y)
        z_jump = abs(sample.z - previous.z)
        if xy_jump > self.config["jump_xy_m"] * multiplier:
            return f"XY jump {xy_jump:.2f} m in {gap:.3f} s"
        if z_jump > self.config["jump_z_m"] * multiplier:
            return f"Z jump {z_jump:.2f} m in {gap:.3f} s"
        return None

    def _segment(self, stage: str, target: dict[str, Any], seconds: float,
                  label: str, rows: list[dict[str, Any]], *, allow_low: bool = False,
                  scored: bool = False, stop_when=None, target_update=None,
                  rc_override=None) -> None:
        begin = time.monotonic()
        end = begin + seconds
        first_row = len(rows)
        period = 1 / self.config["kinematics_hz"]
        next_tick = begin
        kinematics_history: list[Sample] = []
        while True:
            before_tick = time.monotonic()
            if before_tick >= end:
                break
            if before_tick < next_tick:
                time.sleep(next_tick - before_tick)
            tick = time.monotonic()
            if tick >= end:
                break
            scheduler_late = max(0.0, tick - next_tick)
            scheduler_skipped = int(scheduler_late // period)
            try:
                sample = self.link.read_sample()
            except Exception as exc:
                raise ConnectionError(f"Kinematics read failed: {exc}") from exc
            kinematics_history.append(sample)
            if len(kinematics_history) > self.config["kinematics_duplicate_window"]:
                kinematics_history.pop(0)
            duplicate_kinematics = _duplicate_kinematics_window(
                kinematics_history, self.config)
            sample_gap = sample.t - self.last_sample.t if self.last_sample is not None else 0.0
            reason = self._intervention_reason(sample)
            warnings = self._check_sample(sample, stage, allow_low=allow_low)
            if duplicate_kinematics:
                warnings = (f"kinematics_duplicate,{warnings}" if warnings
                            else "kinematics_duplicate")
            if reason is not None:
                self.interventions.append({
                    "stage": stage, "segment": label, "reason": reason,
                    "t": sample.t, "x": sample.x, "y": sample.y, "z": sample.z,
                })
                self.initial_sample = sample
                self.anchor = sample
                self.controller.reset(sample)
                self.link.set_frame(HOLD_FRAME)
                if scored:
                    raise InterventionDetected(reason)
                target["yaw"] = sample.yaw
                target["position"] = (sample.x, sample.y)
                warnings = f"manual_reposition,{warnings}" if warnings else "manual_reposition"
            if (scored and not allow_low and stage in
                    ("yaw", "height", "acceleration", "velocity", "position") and
                    sample.z < self.controller.z_bias +
                    self.config["safety"]["min_airborne_height"]):
                self.link.set_frame(HOLD_FRAME)
                raise TrialInvalid("Drone was below the airborne height during the trial")
            if target_update is not None:
                target_update(sample, sample.t - begin)
            output = self.controller.step(sample, stage, target)
            if rc_override is not None:
                output = {**output, **rc_override(sample, sample.t - begin)}
            self.link.set_frame((output["rc_roll"], output["rc_pitch"],
                                 output["rc_throttle"], output["rc_yaw"], 2000, 1000, 1300))
            anchor = self.anchor or sample
            row = {
                "t": sample.t, "sample_gap": sample_gap, "warnings": warnings,
                "scheduler_late_seconds": scheduler_late,
                "scheduler_skipped_ticks": scheduler_skipped,
                "kinematics_duplicate": int(duplicate_kinematics),
                "kinematics_duplicate_window": len(kinematics_history),
                "segment": label,
                "segment_elapsed": sample.t - begin, "segment_seconds": seconds,
                "stage": stage,
                "x": sample.x, "y": sample.y, "z": sample.z, "yaw": sample.yaw,
                "roll": sample.roll, "pitch": sample.pitch,
                "vx_world": sample.vx_world, "vy_world": sample.vy_world,
                "vz_world": sample.vz_world,
                "origin_x": anchor.x, "origin_y": anchor.y,
                "origin_z": anchor.z, "origin_yaw": anchor.yaw,
                "ground_z": self.controller.z_bias,
                "target_x": target["position"][0], "target_y": target["position"][1],
                "target_z": target["height"] + (
                    self.controller.z_bias if target.get("height_relative")
                    or target["height"] < 0.3 else 0.0),
                "target_z_command": target["height"],
                "height_relative": target.get("height_relative", False),
                "height_step_index": target.get("height_step_index", ""),
                "height_step_reached": "",
                "height_step_progressed": "",
                "target_yaw": target["yaw"],
                "command_vx": target["velocity"][0], "command_vy": target["velocity"][1],
                **output,
            }
            rows.append(row)
            if stop_when is not None and stop_when(rows[first_row:]):
                elapsed = sample.t - begin
                for step_row in rows[first_row:]:
                    step_row["segment_seconds"] = elapsed
                break
            # Schedule from the original time grid. If RPC work overran one
            # or more periods, advance to a future slot instead of issuing
            # catch-up reads back-to-back.
            next_tick += (scheduler_skipped + 1) * period
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)

    def _descend_height(self, goal_raw: float, label: str,
                        rows: list[dict[str, Any]], *, mode: str = "height",
                        yaw: float | None = None, relative: bool = False,
                        scored: bool = False, allow_low: bool = False,
                        total_seconds: float | None = None,
                        compensate_stall: bool = False) -> list[dict[str, Any]]:
        """Descend in steps; cautiously deepen a stalled command in stage 2."""
        c = self.config
        started = time.monotonic()
        if total_seconds is None and compensate_stall:
            total_seconds = c["height_descent_total_timeout_seconds"]
        results = []
        previous_command: float | None = None
        previous_actual: float | None = None
        while (self.last_sample is not None and
               self.last_sample.z > goal_raw + c["height_descent_reach_tolerance"]):
            remaining = (math.inf if total_seconds is None else
                         total_seconds - (time.monotonic() - started))
            if remaining <= 0:
                break
            actual = self.last_sample.z
            if compensate_stall and previous_command is not None:
                step_raw = min(previous_command, max(
                    goal_raw - c["height_descent_max_target_undershoot"],
                    actual - c["height_descent_max_command_gap"],
                    min(actual - c["height_descent_step"],
                        previous_command - c["height_descent_step"]),
                ))
                if (abs(step_raw - previous_command) < 1e-6 and
                        previous_actual is not None and
                        abs(actual - previous_actual) <
                        c["height_descent_reach_tolerance"] / 2):
                    break
            else:
                step_raw = max(goal_raw, actual - c["height_descent_step"])
            target = self._target(
                height=step_raw - self.controller.z_bias if relative else step_raw,
                height_relative=relative, yaw=yaw)
            index = 1 + max((row["height_step_index"] for row in rows
                             if isinstance(row.get("height_step_index"), int)),
                            default=0)
            target["height_step_index"] = index
            segment_label = f"{label}_step_{index:02d}"
            self.controller.pids["pid_height"].reset()
            first_row = len(rows)
            reached = False
            progressed = False

            def settled(step_rows: list[dict[str, Any]]) -> bool:
                nonlocal reached, progressed
                if not step_rows:
                    return False
                end_t = step_rows[-1]["t"]
                tail = [row for row in step_rows if
                        row["t"] >= end_t - c["height_descent_settle_seconds"]]
                if (end_t - step_rows[0]["t"] < c["height_descent_settle_seconds"]
                        or len(tail) < 2
                        or tail[-1]["t"] - tail[0]["t"] <
                        0.8 * c["height_descent_settle_seconds"]):
                    return False
                reached = (all(abs(row["z"] - step_raw) <=
                               c["height_descent_reach_tolerance"] for row in tail)
                           and abs(tail[-1]["z"] - tail[0]["z"]) /
                           (tail[-1]["t"] - tail[0]["t"]) <=
                           c["height_descent_max_speed"] / 3)
                if compensate_stall:
                    descent_speed = (tail[0]["z"] - tail[-1]["z"]) / (
                        tail[-1]["t"] - tail[0]["t"])
                    progressed = (tail[-1]["z"] <= actual -
                                  min(c["height_descent_step"] * 0.6,
                                      actual - goal_raw)
                                  and 0 <= descent_speed <=
                                  c["height_descent_max_speed"])
                return reached or progressed

            self._segment(mode, target,
                          min(c["height_descent_step_timeout_seconds"], remaining),
                          segment_label, rows, allow_low=allow_low,
                          scored=scored, stop_when=settled)
            step_rows = rows[first_row:]
            for row in step_rows:
                row["height_step_reached"] = int(reached)
                row["height_step_progressed"] = int(progressed or reached)
            results.append({
                "index": index, "command_z": target["height"],
                "target_raw_z": step_raw, "final_raw_z": self.last_sample.z,
                "reached": reached, "progressed": progressed,
                "duration": step_rows[-1]["t"] - step_rows[0]["t"] if step_rows else 0.0,
            })
            previous_command = step_raw
            previous_actual = actual
            if not reached and not compensate_stall:
                break
        return results

    def _wait_velocity_rest(self, axis: str, rows: list[dict[str, Any]],
                            label: str) -> bool:
        c = self.config
        started = time.monotonic()
        heading_target = self._xy_heading()
        target = self._target(height=c["hover_height"], yaw=heading_target,
                              active_axis=None if axis == "xy" else axis)
        while True:
            target["yaw"] = heading_target
            self._segment("velocity", target, c["velocity_rest_seconds"],
                          label, rows)
            measured = _physical_velocity_bins(
                rows, label, c["velocity_measurement_window_seconds"],
                c["velocity_score_hz"])
            recent = [item for item in measured if item["t"] >= measured[-1]["t"] - 0.8] if measured else []
            sample = self.last_sample
            airborne = sample is not None and sample.z >= c["safety"]["min_airborne_height"]
            level = sample is not None and max(abs(sample.roll), abs(sample.pitch)) <= math.radians(c["safety"]["max_tilt_deg"])
            heading = (sample is not None and
                       abs(wrap_pi(sample.yaw - heading_target)) <= 0.1)
            center = c["xy_trial_center"] if c["xy_trial_center"] is not None else self.launch_xy
            in_zone = (self.xy_waypoints_mode or center is None or sample is not None and
                       math.hypot(sample.x - center[0], sample.y - center[1])
                       <= c["xy_trial_radius"])
            if axis == "xy":
                speeds = [math.hypot(item["vx"], item["vy"]) for item in recent]
            else:
                speeds = [abs(item[f"v{axis}"]) for item in recent]
            resting = (len(recent) >= 3 and
                       max(speeds) <= c["velocity_rest_tolerance"])
            if airborne and level and heading and resting and in_zone:
                return True
            if time.monotonic() - started >= c["velocity_rest_timeout_seconds"]:
                if self.xy_waypoints_mode:
                    return False
                raise TrialInvalid("Drone did not reach airborne, level, in-zone rest before speed step")

    def _velocity_preparation_state(
            self, rows: list[dict[str, Any]], label: str, axis: str,
            rested: bool) -> dict[str, Any]:
        """Freeze a usable moving baseline after the bounded stop attempt."""
        c = self.config
        raw = [row for row in rows if row.get("segment") == label]
        measured = (_physical_velocity_bins(
            rows, label, c["velocity_measurement_window_seconds"],
            c["velocity_score_hz"])
            if raw and all(all(key in row for key in ("t", "x", "y", "yaw"))
                           for row in raw) else [])
        if not measured:
            # This fallback is primarily useful for mocked profiles. A real
            # flight without physical samples is rejected later by the scorer.
            return {"speed": 0.0, "acceleration": 0.0, "spread": 0.0,
                    "stable": bool(rested), "at_rest": bool(rested),
                    "measured": False}
        tail_start = measured[-1]["t"] - min(1.0, measured[-1]["t"])
        tail = [item for item in measured if item["t"] >= tail_start]
        values = [item[f"v{axis}"] for item in tail]
        baseline = mean(values)
        center_t = mean(item["t"] for item in tail)
        center_v = baseline
        denominator = sum((item["t"] - center_t) ** 2 for item in tail)
        acceleration = (sum((item["t"] - center_t) *
                            (item[f"v{axis}"] - center_v) for item in tail) /
                        denominator if denominator > 1e-9 else 0.0)
        spread = math.sqrt(mean((value - baseline) ** 2 for value in values))
        stable = (len(tail) >= 3 and
                  abs(acceleration) <= c["velocity_max_acceleration"] and
                  spread <= c["velocity_rest_tolerance"])
        return {"speed": baseline, "acceleration": acceleration,
                "spread": spread, "stable": stable,
                "at_rest": bool(rested), "measured": True}

    def _brake_velocity_vector(self, label: str, rows: list[dict[str, Any]],
                               *, active_axis: str | None = None,
                               zero_cross_sign: int | None = None) -> bool:
        """Bleed horizontal speed with ramped, open-loop opposing PWM.

        This is a state-cleanup manoeuvre for pure-P trials, not a PID test.
        Height and yaw stay enabled; both XY PID loops stay disabled.
        """
        c = self.config
        history: list[Sample] = []
        stopped = False
        brake_vector: tuple[float, float] | None = None
        window = c["velocity_measurement_window_seconds"]

        def override(sample: Sample, elapsed: float) -> dict[str, Any]:
            nonlocal stopped, brake_vector
            history.append(sample)
            while len(history) > 1 and sample.t - history[0].t > window:
                history.pop(0)
            older = history[0]
            dt = sample.t - older.t
            if dt < window * 0.8:
                return {"rc_roll": 1500, "rc_pitch": 1500,
                        "brake_vx_body": 0.0, "brake_vy_body": 0.0,
                        "brake_speed": 0.0, "brake_pwm_offset": 0.0,
                        "brake_complete": 0}
            if sample.vx_world is not None and sample.vy_world is not None:
                vx_world, vy_world = sample.vx_world, sample.vy_world
            else:
                vx_world = (sample.x - older.x) / dt
                vy_world = (sample.y - older.y) / dt
            cs, sn = math.cos(sample.yaw), math.sin(sample.yaw)
            vx = vx_world * cs - vy_world * sn
            vy = vx_world * sn + vy_world * cs
            speed = math.hypot(vx, vy)
            axis_velocity = vx if active_axis == "x" else vy
            if zero_cross_sign is not None and zero_cross_sign * axis_velocity <= 0:
                stopped = True
                return {"rc_roll": 1500, "rc_pitch": 1500,
                        "brake_vx_body": vx, "brake_vy_body": vy,
                        "brake_speed": speed, "brake_stop_reason": "axis_zero_cross",
                        "brake_pwm_offset": 0.0, "brake_complete": 1}
            if zero_cross_sign is None and speed <= c["velocity_p_brake_stop_speed"]:
                stopped = True
                return {"rc_roll": 1500, "rc_pitch": 1500,
                        "brake_vx_body": vx, "brake_vy_body": vy,
                        "brake_speed": speed, "brake_longitudinal_speed": speed,
                        "brake_lateral_speed": 0.0, "brake_stop_reason": "total_speed",
                        "brake_pwm_offset": 0.0, "brake_complete": 1}
            # The direction is captured once, at the start of the braking
            # manoeuvre.  Recomputing it on every telemetry tick made the
            # open-loop PWM swap axes around zero crossings and excite a new
            # horizontal oscillation.
            if brake_vector is None:
                brake_vector = (vx, vy)
            brake_vx, brake_vy = brake_vector
            denominator = brake_vx * brake_vx + brake_vy * brake_vy
            brake_norm = math.sqrt(denominator)
            longitudinal = (vx * brake_vx + vy * brake_vy) / brake_norm
            lateral = (vx * -brake_vy + vy * brake_vx) / brake_norm
            # A cross-axis drift can keep the vector magnitude above the
            # threshold after the regulated component has already reached
            # zero. Continuing fixed PWM at that point accelerates the drone
            # along the opposite direction. Stop on the signed projection;
            # a negative projection is also a confirmed reversal.
            axis_speed = abs(vx if active_axis == "x" else vy)
            axis_stopped = (active_axis is not None and
                            axis_speed <= c["velocity_p_brake_stop_speed"])
            if zero_cross_sign is None and (longitudinal <= c["velocity_p_brake_stop_speed"]
                                            or axis_stopped):
                stopped = True
                return {"rc_roll": 1500, "rc_pitch": 1500,
                        "brake_vx_body": vx, "brake_vy_body": vy,
                        "brake_speed": speed,
                        "brake_longitudinal_speed": longitudinal,
                        "brake_lateral_speed": lateral,
                        "brake_stop_reason": ("axis_stop" if axis_stopped else
                                              "longitudinal_stop"),
                        "brake_pwm_offset": 0.0, "brake_complete": 1}
            wx = 2 * brake_vx * brake_vx / denominator
            wy = 2 * brake_vy * brake_vy / denominator
            base = min(c["velocity_p_brake_max_offset"],
                       c["velocity_p_brake_pwm_per_second"] * elapsed)
            pitch = int(clamp(
                1500 - math.copysign(wx * base, brake_vx) * c["direction"]["pitch"],
                1000, 2000))
            roll = int(clamp(
                1500 - math.copysign(wy * base, brake_vy) * c["direction"]["roll"],
                1000, 2000))
            return {"rc_roll": roll, "rc_pitch": pitch,
                    "brake_vx_body": vx, "brake_vy_body": vy,
                    "brake_speed": speed,
                    "brake_longitudinal_speed": longitudinal,
                    "brake_lateral_speed": lateral,
                    "brake_stop_reason": "",
                    "brake_pwm_offset": base, "brake_complete": 0}

        target = self._target(height=c["hover_height"], yaw=self._xy_heading())
        self._segment("braking", target, c["velocity_p_brake_timeout_seconds"],
                      label, rows, scored=False,
                      stop_when=lambda _: stopped, rc_override=override)
        return stopped

    def _d_brake_reached(self, rows: list[dict[str, Any]], label: str,
                          axis: str) -> bool:
        measured = _physical_velocity_bins(
            rows, label, self.config["velocity_measurement_window_seconds"],
            self.config["velocity_score_hz"])
        return any(abs(item[f"v{axis}"]) <= self.config["velocity_d_brake_speed"]
                   for item in measured)

    def _profile_velocity_range(
            self, axis: str, label: str, *,
            step_seconds: float | None = None,
            p_mode: bool = False,
            d_mode: bool = False) -> tuple[list[dict[str, Any]], str]:
        """Measure forward step, braking, reverse step, and braking in one run.

        Each speed command is an increment from the measured velocity that
        remains after the preceding zero-speed interval.  This preserves the
        bounded drift-compensation policy while making the two excitations
        explicitly opposite: ``+dV``, then ``-dV``.  The second baseline is
        taken after the first three-second brake, not after a fresh unrelated
        recovery attempt.
        """
        c = self.config
        rows: list[dict[str, Any]] = []
        speed = c["velocity_target_speed"]
        duration = c["velocity_trial_seconds"] if step_seconds is None else step_seconds

        def annotate(items: list[dict[str, Any]], preparation: dict[str, Any],
                     *, commanded: float | None = None,
                     requested_delta: float | None = None) -> None:
            for row in items:
                row["preparation_rest"] = preparation["at_rest"]
                row["preparation_stable"] = preparation["stable"]
                row["baseline_axis_speed"] = preparation["speed"]
                row["baseline_axis_acceleration"] = preparation["acceleration"]
                row["baseline_axis_spread"] = preparation["spread"]
                if commanded is not None:
                    row["commanded_axis_speed"] = commanded
                    row["requested_delta_speed"] = requested_delta
                    row["requested_speed"] = commanded

        try:
            pre_start = len(rows)
            rested = (self._brake_velocity_vector("pre_positive", rows, active_axis=axis)
                      if p_mode else
                      self._wait_velocity_rest(axis, rows, "pre_positive"))
            preparation = self._velocity_preparation_state(
                rows, "pre_positive", axis, bool(rested))
            annotate(rows[pre_start:], preparation)
            self.controller.reset(self.last_sample)

            for index, (direction, desired_delta) in enumerate(
                    (("positive", speed), ("negative", -speed))):
                # The preceding brake defines the baseline for the reverse
                # step.  It is deliberately not replaced by a second, long
                # rest wait: the stop itself is part of the experiment.
                if index:
                    stop_label = ("positive_service_brake" if d_mode and any(
                        row["segment"] == "positive_service_brake" for row in rows)
                                  else "positive_stop")
                    preparation = self._velocity_preparation_state(
                        rows, stop_label, axis, rested=False)
                    preparation["at_rest"] = bool(
                        preparation["stable"] and
                        abs(preparation["speed"]) <= c["velocity_rest_tolerance"])
                    # A reverse step from a substantial remaining axial drift
                    # can be clamped to an almost-zero increment. That makes
                    # the trial incomparable and corrupts score normalization.
                    # One last open-loop cleanup is allowed, but never start
                    # the reverse command until the active axis is genuinely
                    # slow enough.
                    if abs(preparation["speed"]) > c["velocity_p_brake_stop_speed"]:
                        cleanup_start = len(rows)
                        self._brake_velocity_vector(
                            "positive_cleanup_brake", rows, active_axis=axis)
                        # Do not reuse the braking history as the baseline.
                        # Its final tick can cross zero while inertia still
                        # carries the drone.  Hold neutral for a short,
                        # independently measurable interval, then require a
                        # stable active-axis speed before the reverse step.
                        neutral_start = len(rows)
                        neutral = self._target(
                            height=c["hover_height"], yaw=self._xy_heading(),
                            active_axis=axis)
                        self._segment(
                            "braking", neutral,
                            c["velocity_p_brake_neutral_seconds"],
                            "positive_cleanup_neutral", rows, scored=False)
                        preparation = self._velocity_preparation_state(
                            rows, "positive_cleanup_neutral", axis, rested=False)
                        annotate(rows[cleanup_start:neutral_start], preparation,
                                 commanded=0.0, requested_delta=0.0)
                        annotate(rows[neutral_start:], preparation, commanded=0.0,
                                 requested_delta=0.0)
                    if ((preparation["measured"] and not preparation["stable"]) or
                            abs(preparation["speed"]) >
                            c["velocity_p_brake_stop_speed"]):
                        raise TrialInvalid(
                            f"Active {axis.upper()} speed was not stable below "
                            f"{c['velocity_p_brake_stop_speed']:.3f} m/s after braking "
                            f"({preparation['speed']:.3f} m/s)")

                baseline = preparation["speed"]
                # Pure P is measured against an absolute ±speed target.
                # D/I retain the established bounded increment from drift.
                commanded = (desired_delta if p_mode else
                             clamp(baseline + desired_delta, -speed, speed))
                requested_delta = commanded - baseline
                velocity = ((commanded, 0.0) if axis == "x" else
                            (0.0, commanded))
                target = self._target(height=c["hover_height"],
                                      yaw=self._xy_heading(),
                                      velocity=velocity, active_axis=axis)
                step_start = len(rows)
                self._segment("velocity", target, duration, direction, rows, scored=True)
                annotate(rows[step_start:], preparation, commanded=commanded,
                         requested_delta=requested_delta)

                stop_start = len(rows)
                if p_mode:
                    self._brake_velocity_vector(f"{direction}_stop", rows,
                                                active_axis=axis)
                else:
                    stop = self._target(height=c["hover_height"],
                                        yaw=self._xy_heading(), active_axis=axis)
                    self._segment("velocity", stop, c["velocity_brake_seconds"],
                                  f"{direction}_stop", rows, scored=True)
                    if (d_mode and not self._d_brake_reached(
                            rows, f"{direction}_stop", axis)):
                        self._brake_velocity_vector(
                            f"{direction}_service_brake", rows, active_axis=axis)
                annotate(rows[stop_start:], preparation, commanded=0.0,
                         requested_delta=-commanded)
        finally:
            csv_name = self._write_csv(label, rows)
        return rows, csv_name

    def _profile_acceleration_range(self, axis: str, label: str, *,
                                    magnitude: float | None = None,
                                    duration: float | None = None,
                                    phase: str = "d") -> tuple[list[dict[str, Any]], str]:
        """Run repeated ± acceleration steps with upstream XY loops disabled."""
        c = self.config
        magnitude = c["acceleration_step"] if magnitude is None else magnitude
        duration = c["acceleration_trial_seconds"] if duration is None else duration
        rows: list[dict[str, Any]] = []
        try:
            for cycle in range(1, c["acceleration_repeats"] + 1):
                for direction, sign in (("positive", 1), ("negative", -1)):
                    self.controller.reset(self.last_sample)
                    target_acceleration = ((sign * magnitude, 0.0)
                                           if axis == "x" else
                                           (0.0, sign * magnitude))
                    target = self._target(height=c["hover_height"], yaw=self._xy_heading(),
                                          acceleration=target_acceleration, active_axis=axis)
                    start = len(rows)
                    if phase == "p":
                        self._segment("acceleration", target, duration,
                                      direction, rows, scored=True)
                    else:
                        arrival_limit = c["acceleration_d_arrival_seconds"]
                        hold_seconds = c["acceleration_d_hold_seconds"]

                        def observation_complete(part: list[dict[str, Any]]) -> bool:
                            arrival = next((row["segment_elapsed"] for row in part
                                            if sign * row[f"a{axis}_body"] >= magnitude
                                            and row["segment_elapsed"] <= arrival_limit), None)
                            elapsed = part[-1]["segment_elapsed"]
                            return (elapsed >= arrival_limit if arrival is None else
                                    elapsed >= arrival + hold_seconds)

                        self._segment("acceleration", target,
                                      arrival_limit + hold_seconds + 0.1,
                                      direction, rows, scored=True,
                                      stop_when=observation_complete)
                    for row in rows[start:]:
                        row["acceleration_cycle"] = cycle
                    brake_start = len(rows)
                    self._brake_velocity_vector(f"{direction}_service_brake", rows,
                                                active_axis=axis)
                    for row in rows[brake_start:]:
                        row["acceleration_cycle"] = cycle
        finally:
            csv_name = self._write_csv(label, rows)
        return rows, csv_name

    def _profile_velocity_repeats(self, axis: str, label: str, phase: str,
                                  magnitude: float | None = None
                                  ) -> tuple[list[dict[str, Any]], str]:
        """Run signed flights; time each response only after crossing axis zero."""
        c = self.config
        plans = (c["velocity_p_targets"] if phase == "p" else
                 [{"speed": (c["velocity_target_speed"] if magnitude is None
                             else magnitude)}])
        rows: list[dict[str, Any]] = []

        def axis_speed(source: Sample | dict[str, Any]) -> float:
            world_x = (source.vx_world if isinstance(source, Sample) else
                       source.get("vx_world"))
            world_y = (source.vy_world if isinstance(source, Sample) else
                       source.get("vy_world"))
            if world_x is not None and world_y is not None:
                yaw = source.yaw if isinstance(source, Sample) else source["yaw"]
                cs, sn = math.cos(yaw), math.sin(yaw)
                return (world_x * cs - world_y * sn if axis == "x" else
                        world_x * sn + world_y * cs)
            if isinstance(source, Sample):
                return self.controller.last_actual_body_velocity[0 if axis == "x" else 1]
            return float(source[f"v{axis}_body"])

        try:
            self._brake_velocity_vector("pre_first_service_brake", rows,
                                        active_axis=axis)
            for cycle in range(1, c["velocity_repeats"] + 1):
                for plan in plans:
                    speed = float(plan["speed"])
                    duration = (c["velocity_trial_seconds"] if phase == "p" else
                                c["velocity_validation_hold_seconds"] if phase ==
                                "validation" else
                                c["velocity_i_hold_seconds"] if phase == "i" else
                                c["velocity_d_trial_seconds"])
                    for direction, sign in (("positive", 1), ("negative", -1)):
                        # The ordinary service brake stops near zero. If it
                        # leaves velocity already pointing toward the target,
                        # cross zero with open-loop braking first; otherwise
                        # the measured arrival would start partway through.
                        zero_brake_ok = True
                        if (self.last_sample is not None and
                                sign * axis_speed(self.last_sample) > 0):
                            zero_brake_ok = self._brake_velocity_vector(
                                f"pre_{direction}_axis_zero_brake", rows,
                                active_axis=axis, zero_cross_sign=sign)
                        self.controller.reset(self.last_sample)
                        velocity = ((sign * speed, 0.0) if axis == "x" else
                                    (0.0, sign * speed))
                        target = self._target(
                            height=c["hover_height"], yaw=self._xy_heading(),
                            velocity=velocity, active_axis=axis)
                        start = len(rows)
                        if zero_brake_ok:
                            self._segment(
                                "velocity", target,
                                c["velocity_zero_cross_timeout_seconds"],
                                f"pre_{direction}_zero_cross", rows,
                                scored=True,
                                stop_when=lambda recent: sign * axis_speed(recent[-1]) >= 0)
                        crossed = (zero_brake_ok and len(rows) > start and
                                   sign * axis_speed(rows[-1]) >= 0)
                        if crossed:
                            # Keep the PID state from the zero-crossing tick.
                            # The scored segment gets its own full duration.
                            self._segment("velocity", target, duration,
                                          direction, rows, scored=True)
                        else:
                            # This is a bad response to this candidate, not
                            # an invalid trial. Record it so the tuner can
                            # increase P instead of retrying indefinitely.
                            rows[-1].update(
                                velocity_zero_cross_failed=1,
                                velocity_cycle=cycle,
                                velocity_requested_speed=speed,
                                velocity_zero_cross_direction=direction)
                        for row in rows[start:]:
                            row["velocity_cycle"] = cycle
                            row["velocity_requested_speed"] = speed
                            row["commanded_axis_speed"] = sign * speed
                        brake_start = len(rows)
                        self._brake_velocity_vector(
                            f"{direction}_service_brake", rows, active_axis=axis)
                        for row in rows[brake_start:]:
                            row["velocity_cycle"] = cycle
                            row["velocity_requested_speed"] = speed
        finally:
            csv_name = self._write_csv(label, rows)
        return rows, csv_name

    def _profile_position_waypoints(self, axis: str, label: str,
                                    *, joint: bool = False) -> tuple[list[dict[str, Any]], str]:
        """Move out and back on each axis separately, then optionally diagonally."""
        c = self.config
        a = self.anchor
        assert a is not None
        rows: list[dict[str, Any]] = []
        distances = ([c["position_step"]] if joint else
                     c["position_calibration_distances"])
        point_index = 0
        try:
            for distance in distances:
                component = distance / math.sqrt(2) if joint else distance
                for sign in (1, -1):
                    move = ((a.x + sign * component, a.y + sign * component)
                            if joint else
                            (a.x + sign * component, a.y) if axis == "x" else
                            (a.x, a.y + sign * component))
                    for returning, destination in ((False, move),
                                                   (True, (a.x, a.y))):
                        point_index += 1
                        start = len(rows)
                        target = self._target(
                            height=c["hover_height"], yaw=self._xy_heading(),
                            position=destination,
                            active_axis=None if joint else axis)
                        duration = max(c["step_seconds"],
                                       component / c["max_xy_speed"] +
                                       c["position_calibration_time_reserve_seconds"])
                        self._segment("position", target, duration,
                                      f"position_wp_{point_index:02d}_hold",
                                      rows, scored=True)
                        for row in rows[start:]:
                            row["waypoint_index"] = point_index
                            row["requested_distance"] = component
                            row["waypoint_return"] = returning
        finally:
            csv_name = self._write_csv(label, rows)
        return rows, csv_name

    def _profile_position_repeats(self, axis: str, label: str, phase: str,
                                  distance: float | None = None
                                  ) -> tuple[list[dict[str, Any]], str]:
        """Three signed position steps, each relative to its post-braking start."""
        c = self.config
        requested = c["position_target_distance"] if distance is None else distance
        duration = (c["position_trial_seconds"] if phase == "p" else
                    c["position_d_trial_seconds"] if phase == "d" else
                    c["position_i_trial_seconds"] if phase == "i" else
                    max(c["position_i_trial_seconds"],
                        requested / c["max_xy_speed"] +
                        c["position_validation_reserve_seconds"]))
        rows: list[dict[str, Any]] = []
        try:
            # Position axes are world-frame axes. Stop the full horizontal
            # vector, independent of the launch heading.
            self._brake_velocity_vector("pre_first_service_brake", rows)
            for cycle in range(1, c["position_repeats"] + 1):
                for direction, sign in (("positive", 1), ("negative", -1)):
                    if self.last_sample is None:
                        raise TrialInvalid("No sample after position service brake")
                    origin = self.last_sample.x if axis == "x" else self.last_sample.y
                    destination = ((origin + sign * requested, self.last_sample.y)
                                   if axis == "x" else
                                   (self.last_sample.x, origin + sign * requested))
                    self.controller.reset(self.last_sample)
                    target = self._target(
                        height=c["hover_height"], yaw=self._xy_heading(),
                        position=destination, active_axis=axis)
                    start = len(rows)
                    self._segment("position", target, duration, direction,
                                  rows, scored=True)
                    for row in rows[start:]:
                        row["position_cycle"] = cycle
                        row["position_origin_axis"] = origin
                        row["position_requested_distance"] = requested
                        row["position_target_offset"] = sign * requested
                    self._brake_velocity_vector(
                        f"{direction}_service_brake", rows)
        finally:
            csv_name = self._write_csv(label, rows)
        return rows, csv_name

    def _profile_height_waypoints(
            self, label: str, targets: list[float] | None = None,
            *, allow_low: bool = True
            ) -> tuple[list[dict[str, Any]], str]:
        """Measure each requested ground-relative height in one continuous flight."""
        rows: list[dict[str, Any]] = []
        try:
            heights = (self.config["height_stage2_targets"]
                       if targets is None else targets)
            for index, height in enumerate(heights, 1):
                segment_allow_low = allow_low and index == 1
                requested_raw = self.controller.z_bias + height
                start = len(rows)
                name = f"height_wp_{index:02d}"
                hold_height = height
                distance = (abs(requested_raw - self.last_sample.z)
                            if self.last_sample is not None else 0.0)
                descending = (self.last_sample is not None and
                              self.last_sample.z > requested_raw +
                              self.config["height_descent_reach_tolerance"])
                if descending:
                    steps = self._descend_height(
                        requested_raw, f"{name}_descent", rows,
                        relative=True, scored=True, allow_low=segment_allow_low,
                        compensate_stall=True)
                    if (steps and self.last_sample.z > requested_raw +
                            self.config["height_descent_reach_tolerance"]):
                        hold_height = steps[-1]["command_z"]
                duration = (self.config["step_seconds"] if descending else
                            max(self.config["step_seconds"],
                                self.config["height_ascent_seconds_per_m"] * distance +
                                self.config["height_ascent_time_reserve"],
                                self.config["height_pid_trial_seconds"]
                                if targets is not None else 0.0))
                self._segment(
                    "height", self._target(height=hold_height,
                                           height_relative=True),
                    duration, f"{name}_hold", rows,
                    scored=True, allow_low=segment_allow_low)
                for row in rows[start:]:
                    row["waypoint_index"] = index
                    row["requested_height"] = height
                    row["requested_target_z"] = requested_raw
        finally:
            file_name = self._write_csv(label, rows)
        return rows, file_name

    def _profile_yaw_waypoints(self, label: str) -> tuple[list[dict[str, Any]], str]:
        """Turn around the recovered heading while holding launch-relative height."""
        rows: list[dict[str, Any]] = []
        assert self.anchor is not None
        starting_yaw = self.anchor.yaw
        try:
            for index, angle in enumerate(self.config["yaw_stage_targets_deg"], 1):
                requested = wrap_pi(starting_yaw + math.radians(angle))
                first = len(rows)
                self._segment(
                    "yaw", self._target(height=self.config["hover_height"],
                                         yaw=requested),
                    self.config["step_seconds"], f"yaw_wp_{index:02d}_hold",
                    rows, scored=True)
                for row in rows[first:]:
                    row["waypoint_index"] = index
                    row["requested_yaw_deg"] = angle
                    row["requested_target_yaw"] = requested
        finally:
            file_name = self._write_csv(label, rows)
        return rows, file_name

    def _profile(self, stage: str, axis: str, label: str,
                  *, joint: bool = False,
                  velocity_speed: float | None = None,
                  velocity_hold_seconds: float | None = None,
                  velocity_p_mode: bool = False,
                  velocity_d_mode: bool = False) -> tuple[list[dict[str, Any]], str]:
        if stage == "height" and self.height_waypoints_mode:
            return self._profile_height_waypoints(label)
        if stage == "yaw" and self.yaw_waypoints_mode:
            return self._profile_yaw_waypoints(label)
        if (self.xy_waypoints_mode and stage == "velocity" and not joint and
                velocity_speed is None):
            return self._profile_velocity_range(
                axis, label, step_seconds=velocity_hold_seconds,
                p_mode=velocity_p_mode, d_mode=velocity_d_mode)
        if stage == "acceleration" and not joint:
            return self._profile_acceleration_range(axis, label)
        if self.xy_waypoints_mode and stage == "position":
            return self._profile_position_waypoints(axis, label, joint=joint)
        c = self.config
        a = self.anchor
        assert a is not None
        if stage == "height":
            step = c["height_step"]
            commands = [
                ("positive", self._target(height=c["hover_height"] + step)),
                ("negative", self._target(height=c["hover_height"] - step)),
            ]
        elif stage == "yaw":
            step = math.radians(c["yaw_step_deg"])
            commands = [
                ("positive", self._target(height=c["hover_height"], yaw=a.yaw + step)),
                ("negative", self._target(height=c["hover_height"], yaw=a.yaw - step)),
            ]
        elif stage == "velocity":
            requested = c["velocity_step"] if velocity_speed is None else velocity_speed
            step = requested / math.sqrt(2) if joint else requested
            velocity = ((step, step) if joint else
                        (step, 0) if axis == "x" else (0, step))
            commands = [
                ("positive", self._target(height=c["hover_height"], yaw=self._xy_heading(),
                                          velocity=velocity,
                                          active_axis=None if joint else axis)),
                ("positive_stop", self._target(height=c["hover_height"], yaw=self._xy_heading(),
                                               active_axis=None if joint else axis)),
                ("negative", self._target(height=c["hover_height"], yaw=self._xy_heading(),
                                          velocity=(-velocity[0], -velocity[1]),
                                          active_axis=None if joint else axis)),
                ("negative_stop", self._target(height=c["hover_height"], yaw=self._xy_heading(),
                                               active_axis=None if joint else axis)),
            ]
        else:
            step = c["position_step"] / math.sqrt(2) if joint else c["position_step"]
            pos = ((a.x + step, a.y + step) if joint else
                   (a.x + step, a.y) if axis == "x" else (a.x, a.y + step))
            neg = ((a.x - step, a.y - step) if joint else
                   (a.x - step, a.y) if axis == "x" else (a.x, a.y - step))
            commands = [
                ("positive", self._target(height=c["hover_height"], yaw=self._xy_heading(),
                                          position=pos,
                                          active_axis=None if joint else axis)),
                ("negative", self._target(height=c["hover_height"], yaw=self._xy_heading(),
                                          position=neg,
                                          active_axis=None if joint else axis)),
                ("return", self._target(height=c["hover_height"], yaw=self._xy_heading(),
                                        active_axis=None if joint else axis)),
            ]
        rows: list[dict[str, Any]] = []
        candidate_configs = copy.deepcopy(self.controller.pid_configs)
        try:
            for index, (name, target) in enumerate(commands):
                if stage == "velocity" and name in ("positive", "negative"):
                    self._wait_velocity_rest(axis, rows, f"pre_{name}")
                    self.controller.reset(self.last_sample)
                    if self.last_sample is not None:
                        self.anchor = self.last_sample
                if index and stage in ("height", "yaw"):
                    # Restore only the already available stage loops between
                    # scored steps. XY stays disabled for height and yaw.
                    interventions_before = len(self.interventions)
                    self._recover(stage)
                    if len(self.interventions) != interventions_before:
                        raise InterventionDetected("Manual move during interstitial recovery")
                    self.controller.set_configs(candidate_configs)
                    self.controller.reset(self.last_sample)
                if joint:
                    duration = (c["joint_return_seconds"] if name in ("return", "positive_stop", "negative_stop")
                                else c["joint_step_seconds"])
                else:
                    duration = (c["return_seconds"] if name in ("return", "positive_stop", "negative_stop")
                                else c["step_seconds"])
                if (stage == "height" and name == "negative" and
                        self.last_sample is not None and
                        target["height"] < self.last_sample.z):
                    steps = self._descend_height(target["height"], name, rows,
                                                 scored=True)
                    hold_height = steps[-1]["target_raw_z"] if steps else target["height"]
                    self._segment(stage, self._target(height=hold_height), duration,
                                  "negative_hold", rows, scored=True)
                else:
                    self._segment(stage, target, duration, name, rows, scored=True)
        finally:
            file_name = self._write_csv(label, rows)
        return rows, file_name

    def _recover(self, stage: str) -> None:
        mode = "height" if stage == "height" else "yaw"
        relative_height = ((stage == "height" and self.height_waypoints_mode) or
                           (stage == "yaw" and self.yaw_waypoints_mode) or
                           (stage in ("acceleration", "velocity", "position") and
                            self.xy_waypoints_mode))
        if relative_height:
            self.controller.base_throttle_rc = self.best_height_base_rc
        self.controller.set_configs(self.best)
        self.controller.reset(self.last_sample)
        rows: list[dict[str, Any]] = []
        target = self._target(
            height=self.config["hover_height"],
            height_relative=relative_height,
            yaw=self._xy_heading() if stage in ("acceleration", "velocity", "position") else None)
        target_raw = (target["height"] + self.controller.z_bias
                      if relative_height else target["height"])
        try:
            # The staged descent helper belongs to height calibration and
            # landing. During XY recovery it can chase a transient altitude
            # overshoot with successive lower targets. Keep one fixed hover
            # target here so the height PID recovers normally.
            if (stage not in ("acceleration", "velocity", "position") and self.last_sample is not None and
                    self.last_sample.z > target_raw +
                    self.config["height_descent_reach_tolerance"]):
                steps = self._descend_height(target_raw, "recovery", rows,
                                              mode=mode, yaw=target["yaw"],
                                              relative=relative_height,
                                              compensate_stall=relative_height)
                if (steps and self.last_sample.z > target_raw +
                        self.config["height_descent_reach_tolerance"]):
                    target["height"] = steps[-1]["command_z"]
            self._segment(mode, target, self.config["recovery_seconds"],
                          "recovery", rows)
        finally:
            file_name = self._write_csv("recovery", rows)
        if rows:
            tail = [row for row in rows if row["t"] >= rows[-1]["t"] - 1.0]
            recent_tail = [row for row in tail if
                           row["t"] >= rows[-1]["t"] - 0.25]
            height_error = mean(abs(row["z"] - target_raw) for row in tail)
            recent_height_error = mean(abs(row["z"] - target_raw)
                                       for row in recent_tail)
            height_reached = (height_error <= self.config["height_stage2_tolerance"]
                              and recent_height_error <=
                              self.config["height_stage2_tolerance"])
            self.recoveries.append({
                "stage": stage,
                "active_loops": mode,
                "telemetry_csv": file_name,
                "height_error": height_error,
                "recent_height_error": recent_height_error,
                "height_bias": mean(row["z"] - target_raw for row in tail),
                "mean_throttle_rc": (mean(row["rc_throttle"] for row in tail)
                                     if all("rc_throttle" in row for row in tail)
                                     else None),
                "height_reached": height_reached if relative_height else None,
                "yaw_error": mean(abs(wrap_pi(row["yaw"] - target["yaw"]))
                for row in tail),
            })
            self._save_summary()
            if relative_height and not self.recoveries[-1]["height_reached"]:
                raise TrialInvalid(
                    f"Recovery stayed {self.recoveries[-1]['height_error']:.3f} m "
                    f"from launch-relative {self.config['hover_height']:.2f} m")
            if self.yaw_waypoints_mode and stage == "yaw" and self.recoveries[-1][
                    "yaw_error"] > math.radians(self.config["yaw_stage_tolerance_deg"]):
                raise TrialInvalid("Yaw did not return to the starting heading")
            if self.xy_waypoints_mode and stage in ("acceleration", "velocity", "position") and self.recoveries[-1][
                    "yaw_error"] > math.radians(self.config["yaw_stage_tolerance_deg"]):
                raise TrialInvalid("Yaw did not return to the XY calibration heading")

    def _ensure_xy_zone(self, stage: str) -> None:
        if stage not in ("acceleration", "velocity", "position"):
            return
        if self.xy_waypoints_mode:
            return
        center = self.config["xy_trial_center"]
        if center is None:
            center = self.launch_xy
        if center is None:
            return
        radius = self.config["xy_trial_radius"]
        def ready(sample: Sample | None) -> bool:
            return (sample is not None
                    and math.hypot(sample.x - center[0], sample.y - center[1]) <= radius
                    and sample.z >= self.config["safety"]["min_airborne_height"]
                    and abs(wrap_pi(sample.yaw - self._xy_heading())) <= 0.1
                    and max(abs(sample.roll), abs(sample.pitch)) <=
                    math.radians(self.config["safety"]["max_tilt_deg"]))

        if ready(self.last_sample):
            return
        print(f"{stage}: waiting for manual return within {radius:g} m of "
              f"({center[0]:.2f}, {center[1]:.2f})")
        self.controller.set_configs(self.best)
        self.controller.reset(self.last_sample)
        rows: list[dict[str, Any]] = []
        started = time.monotonic()
        try:
            while time.monotonic() - started < self.config["xy_trial_wait_seconds"]:
                target = self._target(height=self.config["hover_height"],
                                      yaw=self._xy_heading())
                self._segment("yaw", target, min(1.0, self.config["xy_trial_wait_seconds"]),
                              "waiting_for_xy_zone", rows)
                if ready(self.last_sample):
                    return
        finally:
            self._write_csv("waiting_for_xy_zone", rows)
        raise TrialInvalid("Drone did not return to the XY calibration zone")

    def _xy_drone_is_down(self, stage: str) -> bool:
        return (stage in ("acceleration", "velocity", "position") and
                self.last_sample is not None and
                (self.last_sample.z < (
                    self.takeoff_start_z if self.takeoff_start_z is not None
                    else self.controller.z_bias) +
                 self.config["safety"]["min_airborne_height"] or
                 max(abs(self.last_sample.roll), abs(self.last_sample.pitch)) >
                 math.radians(self.config["safety"]["max_tilt_deg"])))

    def _wait_for_xy_respawn(self, stage: str) -> None:
        """Pause a crashed trial until the drone is returned to stable ground."""
        previous_status = self.status
        self.status = f"{stage}_waiting_for_respawn"
        self._save_summary()
        self.link.disarm()
        center = self.config["xy_trial_center"] or self.launch_xy
        prior = self.last_sample
        manual_reset_seen = False
        stable_since: float | None = None
        wait_started = time.monotonic()
        print(f"{stage}: drone is down; waiting for manual respawn on the ground "
              "(Ctrl+C to stop)")
        while True:
            sample = self.link.read_sample()
            if prior is not None:
                manual_reset_seen |= (
                    math.hypot(sample.x - prior.x, sample.y - prior.y) >
                    4 * self.config["jump_xy_m"] or
                    abs(sample.z - prior.z) > 4 * self.config["jump_z_m"])
            prior = sample
            self.last_sample = sample
            at_start = (center is not None and
                        math.hypot(sample.x - center[0], sample.y - center[1]) <=
                        self.config["xy_trial_radius"])
            velocities = (sample.vx_world, sample.vy_world, sample.vz_world)
            still = (all(value is not None and abs(value) <= 0.15
                         for value in velocities) and
                     max(abs(sample.roll), abs(sample.pitch)) <=
                     math.radians(self.config["safety"]["max_tilt_deg"]))
            if (at_start or manual_reset_seen) and still:
                if stable_since is None:
                    stable_since = sample.t
                if sample.t - stable_since >= 0.5:
                    self.initial_sample = self.anchor = sample
                    self.launch_xy = (sample.x, sample.y)
                    # Preserve the heading captured before the first takeoff.
                    # A respawn can briefly report a different yaw.
                    self.controller.z_bias = (self.session_start_z
                                              if self.session_start_z is not None
                                              else sample.z)
                    self.controller.base_throttle_rc = self.best_height_base_rc
                    self.controller.set_configs(self.best)
                    self._save_summary()
                    self.link.arm()
                    self._takeoff()
                    if self.stage_summary["takeoff"]["reached_height"]:
                        self.status = previous_status
                        self._save_summary()
                        print(f"{stage}: respawn complete; repeating the same trial")
                        return
                    self.link.disarm()
                    manual_reset_seen = False
                    stable_since = None
                    print(f"{stage}: takeoff did not reach hover height; "
                          "waiting for another manual respawn")
            else:
                stable_since = None
            if (time.monotonic() - wait_started >=
                    self.config["manual_continue_seconds"]):
                try:
                    input(f"{stage}: возврат на землю не подтверждён за "
                          f"{self.config['manual_continue_seconds']:.0f} с. "
                          "Поставьте дрон на землю и нажмите Enter для продолжения "
                          "(Ctrl+C — остановить): ")
                except EOFError:
                    wait_started = time.monotonic()
                    time.sleep(0.5)
                    continue
                sample = self.link.read_sample()
                self.last_sample = self.initial_sample = self.anchor = sample
                self.launch_xy = (sample.x, sample.y)
                self.controller.z_bias = (self.session_start_z
                                          if self.session_start_z is not None
                                          else sample.z)
                self.controller.base_throttle_rc = self.best_height_base_rc
                self.controller.set_configs(self.best)
                self.controller.reset(sample)
                self._save_summary()
                self.link.arm()
                self._takeoff()
                if self.stage_summary.get("takeoff", {}).get(
                        "reached_height", True):
                    self.status = previous_status
                    self._save_summary()
                    print(f"{stage}: ручное подтверждение принято; "
                          "повторяем ту же пробу")
                    return
                self.link.disarm()
                wait_started = time.monotonic()
                print(f"{stage}: после ручного подтверждения взлёт не выполнен; "
                      "ожидаем повторного подтверждения")
            time.sleep(0.5)

    def _vertical_drone_is_down(self, stage: str) -> bool:
        return (stage in ("height", "yaw") and self.last_sample is not None and
                self.last_sample.z < (self.takeoff_start_z if self.takeoff_start_z is not None
                                      else self.controller.z_bias) +
                self.config["safety"]["min_airborne_height"])

    def _wait_for_ground_return(self, stage: str) -> None:
        """Wait for a stable ground start after an invalid height/yaw flight."""
        previous_status = self.status
        self.status = f"{stage}_waiting_for_ground_return"
        self._save_summary()
        self.link.disarm()
        center = self.launch_xy
        stable_since: float | None = None
        wait_started = time.monotonic()
        height_local_ground = stage == "height"
        if height_local_ground:
            print("height: waiting for a stable landing at the current position "
                  "(Ctrl+C to stop)")
        else:
            print(f"{stage}: waiting for return to the launch area on the ground "
                  "(Ctrl+C to stop)")
        while True:
            sample = self.link.read_sample()
            self.last_sample = sample
            at_start = (height_local_ground or
                        (center is not None and
                         math.hypot(sample.x-center[0], sample.y-center[1]) <=
                         self.config["ground_return_radius_m"]))
            if height_local_ground:
                # A height trial deliberately has no XY controller. Horizontal
                # drift and terrain elevation therefore cannot define whether
                # the drone has landed. Use only a stable vertical rate and an
                # upright attitude, then make this Z the next trial's origin.
                still = (sample.vz_world is not None and
                         abs(sample.vz_world) <=
                         self.config["throttle_landing_ground_speed"] and
                         max(abs(sample.roll), abs(sample.pitch)) <=
                         math.radians(self.config["safety"]["max_tilt_deg"]))
            else:
                velocities = (sample.vx_world, sample.vy_world, sample.vz_world)
                still = (all(value is not None and abs(value) <= 0.15
                             for value in velocities) and
                         abs(sample.vz_world) <=
                         self.config["throttle_landing_ground_speed"] and
                         max(abs(sample.roll), abs(sample.pitch)) <=
                         math.radians(self.config["safety"]["max_tilt_deg"]))
            if at_start and still:
                if stable_since is None:
                    stable_since = sample.t
                if sample.t - stable_since >= 0.5:
                    self.initial_sample = self.anchor = sample
                    if stage in ("height", "height_ascent"):
                        self.controller.z_bias = sample.z
                    else:
                        self.controller.z_bias = (self.session_start_z
                                                  if self.session_start_z is not None
                                                  else sample.z)
                    self.controller.base_throttle_rc = self.best_height_base_rc
                    self.controller.set_configs(self.best)
                    self.controller.reset(sample)
                    self.status = previous_status
                    self._save_summary()
                    if height_local_ground:
                        print(f"height: stable landing confirmed at Z={sample.z:.3f} m; "
                              "using it as the next takeoff origin")
                    else:
                        print(f"{stage}: ground return confirmed; resuming calibration")
                    return
            else:
                stable_since = None
            if (time.monotonic() - wait_started >=
                    self.config["manual_continue_seconds"]):
                try:
                    input(f"{stage}: возврат на землю не подтверждён за "
                          f"{self.config['manual_continue_seconds']:.0f} с. "
                          "Поставьте дрон на землю и нажмите Enter для продолжения "
                          "(Ctrl+C — остановить): ")
                except EOFError:
                    # A non-interactive process has no stdin. Keep automatic
                    # recovery active and offer the prompt again later.
                    wait_started = time.monotonic()
                    time.sleep(0.5)
                    continue
                sample = self.link.read_sample()
                self.last_sample = self.initial_sample = self.anchor = sample
                self.controller.z_bias = (
                    sample.z if stage in ("height", "height_ascent") else
                    self.session_start_z if self.session_start_z is not None else
                    sample.z)
                self.controller.base_throttle_rc = self.best_height_base_rc
                self.controller.set_configs(self.best)
                self.controller.reset(sample)
                self.status = previous_status
                self._save_summary()
                print(f"{stage}: ручное подтверждение принято; продолжаем калибровку")
                return
            time.sleep(0.5)

    def _resume_vertical_after_ground(self, stage: str) -> None:
        """Wait and take off before replaying the interrupted height/yaw trial."""
        while True:
            self._wait_for_ground_return(stage)
            self.link.arm()
            self._takeoff()
            if self.stage_summary.get("takeoff", {}).get("reached_height", True):
                print(f"{stage}: takeoff complete; repeating the same trial")
                return
            print(f"{stage}: takeoff did not reach hover height; "
                  "waiting for another ground return")

    def _evaluate(self, stage: str, name: str, axis: str,
                   candidate: dict[str, dict[str, Any]], label: str,
                   *, velocity_speed: float | None = None,
                  acceleration_magnitude: float | None = None,
                  acceleration_duration: float | None = None,
                  acceleration_phase: str = "d",
                   velocity_hold_seconds: float | None = None,
                   velocity_p_mode: bool = False,
                   velocity_d_mode: bool = False,
                   velocity_phase: str | None = None,
                   velocity_magnitude: float | None = None,
                   velocity_i_score_start: float | None = None,
                   position_phase: str | None = None,
                   position_distance: float | None = None,
                   position_i_score_start: float | None = None,
                   height_base_rc: int | None = None) -> dict[str, Any] | None:
        attempt = 0
        while attempt <= self.config["max_intervention_retries"]:
            if self._vertical_drone_is_down(stage):
                self._resume_vertical_after_ground(stage)
                attempt = 0
            if self._xy_drone_is_down(stage):
                self._wait_for_xy_respawn(stage)
                attempt = 0
            try:
                self._recover(stage)
                self._ensure_xy_zone(stage)
                if self.last_sample is not None:
                    self.anchor = self.last_sample
                self.controller.set_configs(candidate)
                self.controller.reset(self.last_sample)
                if stage == "height" and self.height_waypoints_mode:
                    self.controller.base_throttle_rc = (
                        self.best_height_base_rc if height_base_rc is None
                        else height_base_rc)
                if stage == "velocity" and velocity_phase is not None:
                    rows, csv_name = self._profile_velocity_repeats(
                        axis, label, velocity_phase, velocity_magnitude)
                elif stage == "position" and position_phase is not None:
                    rows, csv_name = self._profile_position_repeats(
                        axis, label, position_phase, position_distance)
                elif stage == "acceleration":
                    rows, csv_name = self._profile_acceleration_range(
                        axis, label, magnitude=acceleration_magnitude,
                        duration=acceleration_duration, phase=acceleration_phase)
                elif velocity_speed is None:
                    profile_kwargs = ({
                        "velocity_hold_seconds": velocity_hold_seconds,
                        "velocity_p_mode": velocity_p_mode,
                        "velocity_d_mode": velocity_d_mode,
                    } if stage == "velocity" and
                    (velocity_hold_seconds is not None or velocity_p_mode or
                     velocity_d_mode) else {})
                    rows, csv_name = self._profile(stage, axis, label, **profile_kwargs)
                else:
                    rows, csv_name = self._profile(stage, axis, label,
                                                   velocity_speed=velocity_speed)
            except (InterventionDetected, TrialInvalid) as exc:
                self.records.append({
                    "stage": stage, "pid": name, "axis": axis, "label": label,
                    "attempt": attempt + 1, "valid": False,
                    "reason": str(exc), "telemetry_csv": self.last_csv,
                })
                self._save_summary()
                print(f"{stage}/{name} {label}: ignored invalid trial "
                      f"(attempt {attempt + 1}: {exc})")
                if (stage in ("height", "yaw") and
                        (isinstance(exc, TrialInvalid) or
                         self._vertical_drone_is_down(stage))):
                    # A vertical trial cannot establish a comparable starting
                    # state by retrying in place. Wait for a stable ground
                    # return, take off with the selected best controller, and
                    # replay this exact candidate without spending its retry
                    # budget. This also covers an airborne recovery that never
                    # reached the requested start height.
                    self._resume_vertical_after_ground(stage)
                    attempt = 0
                    continue
                if self._xy_drone_is_down(stage):
                    self._wait_for_xy_respawn(stage)
                    attempt = 0
                    continue
                attempt += 1
                continue
            invalid = _invalid_xy_trial(rows, stage, self.config)
            if invalid is not None:
                self.records.append({
                    "stage": stage, "pid": name, "axis": axis, "label": label,
                    "attempt": attempt + 1, "valid": False,
                    "reason": invalid, "telemetry_csv": csv_name,
                })
                self._save_summary()
                print(f"{stage}/{name} {label}: ignored invalid trial ({invalid})")
                if stage in ("height", "yaw"):
                    self._resume_vertical_after_ground(stage)
                    attempt = 0
                    continue
                if self._xy_drone_is_down(stage):
                    self._wait_for_xy_respawn(stage)
                    attempt = 0
                    continue
                attempt += 1
                continue
            amplitude = {
                "height": self.config["height_step"],
                "yaw": math.radians(self.config["yaw_step_deg"]),
                "acceleration": self.config["acceleration_step"],
                "velocity": self.config["velocity_step"] if velocity_speed is None else velocity_speed,
                "position": self.config["position_step"],
            }[stage]
            try:
                metrics = (score_velocity_repeats(
                               rows, axis, self.config, velocity_phase,
                               velocity_i_score_start, velocity_magnitude)
                           if stage == "velocity" and velocity_phase is not None else
                           score_position_repeats(
                               rows, axis, self.config, position_phase,
                               position_i_score_start, position_distance)
                           if stage == "position" and position_phase is not None else
                           score_acceleration_repeats(
                               rows, axis, self.config, acceleration_phase,
                               acceleration_magnitude)
                           if stage == "acceleration" else
                           score_velocity_steps(rows, axis, amplitude, self.config,
                                                p_mode=velocity_p_mode)
                           if stage == "velocity" else
                           score_height_waypoints(rows, self.config)
                           if stage == "height" and self.height_waypoints_mode else
                           score_yaw_waypoints(rows, self.config)
                           if stage == "yaw" and self.yaw_waypoints_mode else
                           score_position_waypoints(rows, axis, self.config)
                           if stage == "position" and self.xy_waypoints_mode else
                           score_rows(
                               rows, stage, axis, amplitude,
                               oscillation_window_seconds=self.config["oscillation_window_seconds"],
                               oscillation_weight=self.config["oscillation_weight"],
                               height_descent_max_speed=self.config["height_descent_max_speed"],
                               height_descent_oscillation_limit=self.config[
                                   "oscillation_tolerances"]["height"],
                           ))
                if stage == "velocity" and velocity_phase is None:
                    preparation_stable = []
                    preparation_at_rest = []
                    for direction in ("positive", "negative"):
                        pre = [row for row in rows
                               if row["segment"] == f"pre_{direction}"]
                        # The reverse step starts immediately after the first
                        # braking phase.  Its preparation fields live on the
                        # moving rows rather than in a redundant delay segment.
                        if not pre:
                            pre = [row for row in rows if row["segment"] == direction]
                        preparation_stable.append(bool(pre) and bool(
                            pre[-1].get("preparation_stable",
                                        pre[-1].get("preparation_rest", True))))
                        preparation_at_rest.append(bool(pre) and bool(
                            pre[-1].get("preparation_rest", True)))
                    metrics["preparations_stable"] = all(preparation_stable)
                    metrics["preparations_at_rest"] = all(preparation_at_rest)
                    metrics["score"] += 3.0 * sum(
                        not ready for ready in preparation_stable)
                    if velocity_p_mode:
                        metrics["response_score"] = metrics["score"]
                        metrics["p_arrival_target_seconds"] = self.config[
                            "velocity_p_target_arrival_seconds"]
                        metrics["score"] = score_velocity_p_arrival(metrics, self.config)
            except TrialInvalid as exc:
                self.records.append({
                    "stage": stage, "pid": name, "axis": axis, "label": label,
                    "attempt": attempt + 1, "valid": False,
                    "reason": str(exc), "telemetry_csv": csv_name,
                })
                self._save_summary()
                attempt += 1
                continue
            record = {
                "stage": stage, "pid": name, "axis": axis, "label": label,
                "attempt": attempt + 1, "valid": True,
                "test_speed": velocity_speed if stage == "velocity" else None,
                "test_distance": position_distance if stage == "position" else None,
                "height_base_throttle_rc": (
                    self.controller.base_throttle_rc if stage == "height" and
                    self.height_waypoints_mode else None),
                "gains": {k: candidate[name][k] for k in ("kp", "ki", "kd")},
                "metrics": metrics, "telemetry_csv": csv_name,
            }
            self.records.append(record)
            self._save_summary()
            if stage == "velocity" and velocity_phase is not None:
                if velocity_phase == "p":
                    print(f"{stage}/{name} {label}: score={metrics['score']:.3f}, "
                          f"mean curves reached={metrics['reached_steps']}/{len(metrics['mean_steps'])}, "
                          f"mean arrival={metrics['mean_arrival_seconds'] if metrics['mean_arrival_seconds'] is not None else float('nan'):.2f}s, "
                          f"repeats={metrics['repeat_count']}")
                elif velocity_phase == "d":
                    print(f"{stage}/{name} {label}: score={metrics['score']:.3f}, "
                          f"settled={metrics['plateau_settled_fraction']:.0%}, "
                          f"settle={metrics['plateau_settling_time']:.2f}s, "
                          f"repeats={metrics['repeat_count']}")
                elif velocity_phase == "i":
                    print(f"{stage}/{name} {label}: score={metrics['score']:.3f}, "
                          f"mean hold={metrics['hold_fraction']:.0%}, "
                          f"hold +/-={metrics['hold_fraction_by_direction']['positive']:.0%}/"
                          f"{metrics['hold_fraction_by_direction']['negative']:.0%}, "
                          f"repeats={metrics['repeat_count']}")
                else:
                    print(f"{stage}/{name} {label}: validation "
                          f"{metrics['validation_target_speed']:.2f} m/s, "
                          f"{'pass' if metrics['validation_passed'] else 'needs review'}, "
                          f"arrival +/-={metrics['positive']['arrival_time']}/"
                          f"{metrics['negative']['arrival_time']} s, "
                          f"mean hold +/-={metrics['hold_fraction_by_direction']['positive']:.0%}/"
                          f"{metrics['hold_fraction_by_direction']['negative']:.0%}")
            elif stage == "position" and position_phase is not None:
                if position_phase == "p":
                    print(f"{stage}/{name} {label}: score={metrics['score']:.3f}, "
                          f"reached={metrics['reached_steps']}/2, "
                          f"arrival={metrics['mean_arrival_seconds'] if metrics['mean_arrival_seconds'] is not None else float('nan'):.2f}s, "
                          f"repeats={metrics['repeat_count']}")
                elif position_phase == "d":
                    print(f"{stage}/{name} {label}: score={metrics['score']:.3f}, "
                          f"settled={metrics['plateau_settled_fraction']:.0%}, "
                          f"settle={metrics['plateau_settling_time']:.2f}s, "
                          f"repeats={metrics['repeat_count']}")
                else:
                    print(f"{stage}/{name} {label}: "
                          f"{'validation' if position_phase == 'validation' else 'I'} "
                          f"{metrics['distance']:.3f}m, score={metrics['score']:.3f}, "
                          f"mean hold +/-={metrics['hold_fraction_by_direction']['positive']:.0%}/"
                          f"{metrics['hold_fraction_by_direction']['negative']:.0%}")
            elif stage == "velocity":
                def seconds(value: float | None) -> str:
                    return f"{value:.2f}" if value is not None else "n/a"
                print(f"{stage}/{name} {label}: score={metrics['score']:.3f}, "
                      f"V0={metrics['positive']['baseline_axis_speed']:.3f}/"
                      f"{metrics['negative']['baseline_axis_speed']:.3f}, "
                      f"dV={metrics['positive']['requested_delta_speed']:.3f}/"
                      f"{metrics['negative']['requested_delta_speed']:.3f}, "
                      f"peak dV={metrics['positive']['max_speed']:.3f}/"
                      f"{metrics['negative']['max_speed']:.3f}, "
                      f"arrival={seconds(metrics['positive'].get('arrival_time'))}/"
                      f"{seconds(metrics['negative'].get('arrival_time'))}s, "
                      f"rise={seconds(metrics['positive']['rise_time'])}/"
                      f"{seconds(metrics['negative']['rise_time'])}s, "
                      f"in-band={metrics['in_band_fraction']:.0%}, "
                      f"tail={metrics['tail_in_band_fraction']:.0%}, "
                      f"a95={metrics['p95_acceleration']:.3f}, "
                      f"j95={metrics['p95_jerk']:.3f}, "
                      f"brake={seconds(metrics['positive']['stopping_time'])}/"
                      f"{seconds(metrics['negative']['stopping_time'])}s, "
                      f"min|v|={metrics['positive']['brake_minimum_abs_speed']:.3f}/"
                      f"{metrics['negative']['brake_minimum_abs_speed']:.3f}, "
                      f"stopped={metrics['stopped_both']}, "
                      f"osc={metrics['oscillation_rms']:.4f}")
            elif stage == "acceleration":
                if metrics["phase"] == "p":
                    print(f"{stage}/{name} {label}: score={metrics['score']:.3f}, "
                          f"reached={metrics['reached_steps']}/{2 * metrics['repeat_count']}, "
                          f"arrival={metrics['mean_arrival_seconds'] if metrics['mean_arrival_seconds'] is not None else float('nan'):.2f}s, "
                          f"target={metrics['arrival_target_seconds']:.2f}s, "
                          f"time error={metrics['arrival_time_error_seconds']:.2f}s, "
                          f"repeats={metrics['repeat_count']}")
                elif metrics["phase"] == "d":
                    levels = metrics["plateau_level_fraction_by_direction"]
                    print(f"{stage}/{name} {label}: score={metrics['score']:.3f}, "
                          f"reached={metrics['reached_directions']}/2, "
                          f"plateau={metrics['plateau_settling_time']:.2f}s, "
                          f"level +/-={levels['positive']:.0%}/{levels['negative']:.0%}, "
                          f"transient RMS={metrics['transient_rms']:.3f}, "
                          f"repeats={metrics['repeat_count']}")
                else:
                    print(f"{stage}/{name} {label}: score={metrics['score']:.3f}, "
                          f"reached={metrics['reached_directions']}/2, "
                          f"hold={metrics['hold_fraction']:.0%}, "
                          f"hold +/-={metrics['hold_fraction_by_direction']['positive']:.0%}/"
                          f"{metrics['hold_fraction_by_direction']['negative']:.0%}, "
                          f"MAE={metrics['mae']:.4f}, final={metrics['terminal_mae']:.4f}, "
                          f"repeats={metrics['repeat_count']}")
            elif stage == "yaw" and self.yaw_waypoints_mode:
                print(f"{stage}/{name} {label}: score={metrics['score']:.3f}, "
                      f"MAE={metrics['mae']:.4f}, final={metrics['terminal_mae']:.4f}, "
                      f"overshoot={math.degrees(metrics['max_overshoot']):.2f} deg, "
                      f"settle={metrics['max_settling_time']:.2f}s, "
                      f"osc={math.degrees(metrics['oscillation_rms']):.2f} deg")
            elif stage == "position" and self.xy_waypoints_mode:
                print(f"{stage}/{name} {label}: score={metrics['score']:.3f}, "
                      f"reached={metrics['waypoints_reached_fraction']:.0%}, "
                      f"final={metrics['terminal_mae']:.3f} m, "
                      f"overshoot={metrics['max_overshoot']:.3f} m, "
                      f"osc={metrics['oscillation_rms']:.3f} m")
            else:
                descent = (f", descent={metrics['descent_reached_fraction']:.0%} "
                           f"({metrics['descent_steps']} steps), "
                           f"vz={metrics['peak_descent_speed']:.3f} m/s, "
                           f"step osc={metrics['descent_oscillation_rms']:.4f}"
                           if stage == "height" else "")
                print(f"{stage}/{name} {label}: score={metrics['score']:.3f}, "
                      f"MAE={metrics['mae']:.4f}, final={metrics['terminal_mae']:.4f}, "
                      f"osc={metrics['oscillation_rms']:.4f}{descent}")
            return record
        return None

    def _confirm_velocity_range(self) -> None:
        """Check chosen inner gains at low and high speeds without retuning them."""
        outcomes = []
        for name, axis in (("pid_vel_pitch", "x"), ("pid_vel_roll", "y")):
            for speed in self.config["velocity_confirmation_speeds"]:
                label = f"velocity_confirm_{axis}_{speed:g}"
                result = self._evaluate("velocity", name, axis, self.best, label,
                                        velocity_speed=speed)
                metrics = None if result is None else result["metrics"]
                passed = (metrics is not None and _velocity_response_stable(
                    metrics, speed, self.config))
                outcomes.append({
                    "pid": name, "axis": axis, "speed": speed,
                    "passed": passed, "metrics": metrics,
                    "telemetry_csv": result["telemetry_csv"] if result else self.last_csv,
                })
        self.stage_summary["velocity_range"] = {
            "passed": all(item["passed"] for item in outcomes),
            "tests": outcomes,
        }
        self._save_summary()

    def _tune_acceleration_pid(self, name: str, axis: str,
                               *, initial_source: str = "zero_p") -> bool:
        """Bracket P arrival time, then search D and I in three passes."""
        c = self.config
        amplitude = c["acceleration_step"]
        initial_gains = {key: self.best[name][key] for key in ("kp", "ki", "kd")}
        working = copy.deepcopy(self.best)
        trials_by_gain: dict[str, list[dict[str, Any]]] = {}
        chosen: dict[str, dict[str, Any]] = {}

        def evaluate(gain: str, value: float, phase: str, label: str,
                     *, magnitude: float = amplitude) -> dict[str, Any] | None:
            candidate = copy.deepcopy(working)
            candidate[name][gain] = value
            return self._evaluate(
                "acceleration", name, axis, candidate, label,
                acceleration_magnitude=magnitude, acceleration_phase=phase)

        def search_p() -> tuple[dict[str, Any] | None, dict[str, Any]]:
            target_time = c["acceleration_p_target_arrival_seconds"]
            tolerance = c["acceleration_p_time_tolerance_seconds"]
            max_trials = c["acceleration_p_max_trials"]
            step = c["acceleration_p_coarse_step"]
            best: dict[str, Any] | None = None
            best_value = 0.0
            history: list[dict[str, Any]] = []
            trials = 0
            lower: float | None = None
            upper: float | None = None
            target_met = False

            def probe(value: float, phase: str) -> dict[str, Any] | None:
                nonlocal best, best_value, trials, target_met
                label = f"acceleration_{name}_kp_{phase}_{trials:02d}"
                trials += 1
                result = evaluate("kp", value, "p", label)
                if result is None:
                    return None
                metrics = result["metrics"]
                history.append({"pass": phase, "gain": value,
                                "score": metrics["score"], "metrics": metrics,
                                "telemetry_csv": result["telemetry_csv"]})
                score_resolution = 1 / (c["acceleration_control_hz"] * target_time)
                full_response = metrics["reached_steps"] == 2 * c["acceleration_repeats"]
                if best is None:
                    best, best_value = result, value
                else:
                    best_score = best["metrics"]["score"]
                    best_full = (best["metrics"]["reached_steps"] ==
                                 2 * c["acceleration_repeats"])
                    if (metrics["score"] < best_score - score_resolution or
                            (abs(metrics["score"] - best_score) <= score_resolution and
                             ((full_response and best_full and value < best_value) or
                              (not full_response and not best_full and value > best_value)))):
                        best, best_value = result, value
                if (metrics["reached_steps"] == 2 * c["acceleration_repeats"] and
                        metrics["arrival_time_error_seconds"] <= tolerance):
                    target_met = True
                return result

            for index in range(min(c["acceleration_search_coarse_trials"], max_trials)):
                value = round(index * step, 10)
                result = probe(value, "bracket")
                if result is None:
                    continue
                metrics = result["metrics"]
                if (metrics["reached_steps"] == 2 * c["acceleration_repeats"] and
                        metrics["mean_arrival_seconds"] <= target_time):
                    upper = value
                    if lower is None:
                        lower = max(0.0, value - step)
                    break
                lower = value

            while (lower is not None and upper is not None and
                   upper - lower > c["acceleration_p_min_bracket_width"] and
                   trials < max_trials):
                middle = round((lower + upper) / 2, 10)
                result = probe(middle, "refine")
                if result is None:
                    break
                metrics = result["metrics"]
                if (metrics["reached_steps"] == 2 * c["acceleration_repeats"] and
                        metrics["mean_arrival_seconds"] <= target_time):
                    upper = middle
                else:
                    lower = middle

            trials_by_gain["kp"] = history
            if best is not None:
                working[name]["kp"] = best_value
                chosen["kp"] = best["metrics"]
            return best, {
                "target_met": target_met,
                "target_seconds": target_time,
                "tolerance_seconds": tolerance,
                "trials": trials,
                "bracket": [lower, upper],
                "chosen_kp": best_value if best is not None else None,
            }

        def search(gain: str, step: float, phase: str) -> dict[str, Any] | None:
            best: dict[str, Any] | None = None
            best_value = 0.0
            history: list[dict[str, Any]] = []
            success = False
            coarse_limit = (c["acceleration_i_search_coarse_trials"] if phase == "i"
                            else c["acceleration_search_coarse_trials"])
            for pass_number, (direction, fraction, limit) in enumerate((
                    (1, 1.0, coarse_limit),
                    (-1, c["acceleration_search_reverse_fraction"],
                     c["acceleration_search_refine_trials"]),
                    (1, c["acceleration_search_final_fraction"],
                     c["acceleration_search_refine_trials"]))):
                value = 0.0 if pass_number == 0 else best_value
                worse_streak = 0
                for attempt in range(limit):
                    if pass_number or attempt:
                        value = round(value + direction * step * fraction, 10)
                    if value < 0:
                        break
                    label = f"acceleration_{name}_{gain}_pass{pass_number}_{attempt:02d}"
                    result = evaluate(gain, value, phase, label)
                    if result is None:
                        continue
                    metrics = result["metrics"]
                    score = metrics["score"]
                    history.append({"pass": pass_number, "gain": value,
                                    "score": score, "metrics": metrics,
                                    "telemetry_csv": result["telemetry_csv"]})
                    best_score = best["metrics"]["score"] if best is not None else math.inf
                    if score < best_score:
                        best, best_value = result, value
                        worse_streak = 0
                    elif score > best_score:
                        worse_streak += 1
                    else:
                        worse_streak = 0
                    if (phase == "i" and metrics.get("hold_fraction", 0.0) >
                            c["acceleration_i_success_hold_fraction"] and
                            metrics["reached_directions"] == 2 and
                            len(metrics.get("i_filtered_response_by_direction", {})) == 2 and
                            all(part["settled"] for part in
                                metrics.get("i_filtered_response_by_direction", {}).values())):
                        # First qualifying response is enough; retain that
                        # tested set even if an earlier, low-hold score won.
                        best, best_value = result, value
                        success = True
                        break
                    if worse_streak >= c["acceleration_search_worse_streak"]:
                        break
                    if (phase == "d" and value > 0 and
                            metrics["reached_directions"] < 2):
                        # A larger D suppresses acceleration; refine below it.
                        break
                if success:
                    break
            trials_by_gain[gain] = history
            if best is not None:
                working[name][gain] = best_value
                chosen[gain] = best["metrics"]
            return best

        working[name].update(kp=0.0, kd=0.0, ki=0.0)
        p_result, p_search = search_p()
        if p_result is None:
            self.stage_summary[name] = {"stage": "acceleration", "status": "unscored",
                                        "reason": "No valid P trial"}
            self._save_summary()
            return False
        p_only = copy.deepcopy(working[name])
        working[name]["kp"] *= c["acceleration_p_after_d_factor"]
        if not p_search["target_met"]:
            print(f"acceleration/{name}: P arrival target not met; "
                  f"best P={p_search['chosen_kp']}, "
                  f"bracket={p_search['bracket']}")
        d_result = search("kd", c["acceleration_d_coarse_step"], "d")
        i_result = search("ki", c["acceleration_i_coarse_step"], "i")
        if d_result is None and i_result is None:
            # The P reserve is intended only for a tested D/I controller. If
            # every later probe is invalid, retain the best actually flown P.
            working[name] = p_only
        self.best = copy.deepcopy(working)
        self.controller.set_configs(self.best)
        validation = []
        for magnitude in c["acceleration_validation_steps"]:
            label = f"acceleration_{name}_validate_{magnitude:g}"
            result = self._evaluate(
                "acceleration", name, axis, self.best, label,
                acceleration_magnitude=magnitude, acceleration_phase="d")
            metrics = result["metrics"] if result else None
            validation.append({
                "magnitude": magnitude, "metrics": metrics,
                "passed": (metrics is not None and
                           _acceleration_response_stable(metrics, c, magnitude)),
                "telemetry_csv": result["telemetry_csv"] if result else self.last_csv,
            })
        stable = all(item["passed"] for item in validation)
        best_result = i_result or d_result or p_result
        final_phase = "ki" if i_result else "kd" if d_result else "kp"
        self.stage_summary[name] = {
            "stage": "acceleration", "stable": stable,
            "baseline_score": trials_by_gain[final_phase][0]["score"],
            "best_score": best_result["metrics"]["score"],
            "best_metrics": best_result["metrics"],
            "initial_source": initial_source,
            "initial_gains": initial_gains,
            "recommended_gains": {key: self.best[name][key]
                                  for key in ("kp", "ki", "kd")},
            "p_result": chosen.get("kp"), "d_result": chosen.get("kd"),
            "p_search": p_search,
            "i_result": chosen.get("ki"),
            "search_passes": trials_by_gain,
            "validation": validation,
        }
        self._save_summary()
        return True

    def _tune_position_pid(self, name: str, axis: str,
                           *, initial_source: str = "zero_gains_start") -> bool:
        """Tune position P, D, I once from three repeated signed flights."""
        c = self.config
        initial = {key: self.best[name][key] for key in ("kp", "ki", "kd")}
        limit = c["position_trials_per_coefficient"]
        history: dict[str, list[dict[str, Any]]] = {phase: [] for phase in "pdi"}
        d_score_start: float | None = None

        def evaluate(phase: str, value: float,
                     base: dict[str, dict[str, Any]]) -> dict[str, Any] | None:
            candidate = copy.deepcopy(base)
            candidate[name][{"p": "kp", "d": "kd", "i": "ki"}[phase]] = value
            label = f"position_{name}_{phase}_{len(history[phase]) + 1:02d}"
            result = self._evaluate(
                "position", name, axis, candidate, label,
                position_phase=phase,
                position_i_score_start=d_score_start if phase == "i" else None)
            if result is not None:
                history[phase].append({"value": value, "candidate": candidate,
                                       "result": result})
            return result

        p_base = copy.deepcopy(self.best)
        p_base[name].update(kp=0.0, kd=0.0, ki=0.0)
        kp = 0.0
        slow, fast = None, None
        for _ in range(limit):
            result = evaluate("p", kp, p_base)
            if result is None:
                break
            if _position_p_ready(result["metrics"], c):
                break
            if kp == 0:
                slow = 0.0
                kp = c["position_p_start"]
                continue
            steps = result["metrics"]["mean_steps"]
            too_fast = (all(step["arrived"] for step in steps) and
                        mean(step["arrival_time"] for step in steps) <
                        c["position_p_arrival_seconds"])
            if too_fast:
                fast = kp
                kp = (slow + fast) / 2 if slow is not None else kp / c[
                    "position_p_multiplier"]
            else:
                slow = kp
                kp = (slow + fast) / 2 if fast is not None else kp * c[
                    "position_p_multiplier"]
            if kp <= 0 or (slow is not None and fast is not None and
                           fast - slow < 1e-5):
                break
        if not history["p"]:
            self.stage_summary[name] = {
                "stage": "position", "status": "unscored", "stable": False,
                "reason": "Every repeated P trial was invalid or interrupted",
                "recommended_gains": initial}
            self._save_summary()
            return False
        ready_p = [item for item in history["p"] if
                   _position_p_ready(item["result"]["metrics"], c)]
        selected_p = min(ready_p or history["p"],
                         key=lambda item: item["result"]["metrics"]["score"])
        p_met = _position_p_ready(selected_p["result"]["metrics"], c)
        d_base = copy.deepcopy(selected_p["candidate"])
        d_base[name]["kp"] *= c["position_p_after_d_factor"]

        def search(phase: str, base: dict[str, dict[str, Any]],
                   start: float, multiplier: float,
                   ready: Callable[[dict[str, Any], dict[str, Any]], bool]
                   ) -> dict[str, Any] | None:
            values = [0.0, start]
            worse = 0
            best_score = math.inf
            index = 0
            while len(history[phase]) < limit:
                if index >= len(values):
                    values.append(values[-1] * multiplier)
                value = values[index]
                index += 1
                result = evaluate(phase, value, base)
                if result is None:
                    break
                score = result["metrics"]["score"]
                if score < best_score:
                    best_score = score
                    worse = 0
                else:
                    worse += 1
                if value == 0.0 and ready(result["metrics"], c):
                    return history[phase][-1]
                if phase == "i" and ready(result["metrics"], c):
                    break
                if worse >= 2 and len(history[phase]) >= 4:
                    break
            passing = [item for item in history[phase] if
                       ready(item["result"]["metrics"], c)]
            pool = passing or history[phase]
            return (min(pool, key=lambda item: item["result"]["metrics"]["score"])
                    if pool else None)

        selected_d = search("d", d_base, c["position_d_start"],
                            c["position_d_multiplier"], _position_d_ready)
        if selected_d is None:
            self.stage_summary[name] = {
                "stage": "position", "status": "unscored", "stable": False,
                "reason": "No valid repeated D trial",
                "recommended_gains": initial}
            self._save_summary()
            return False
        d_score_start = mean(step["settling_time"] for step in
                             selected_d["result"]["metrics"]["mean_steps"])
        selected_i = search("i", selected_d["candidate"], c["position_i_start"],
                            c["position_i_multiplier"], _position_i_ready)
        chosen = selected_i or selected_d
        self.best = copy.deepcopy(chosen["candidate"])
        self.controller.set_configs(self.best)
        metrics = chosen["result"]["metrics"]
        d_met = _position_d_ready(selected_d["result"]["metrics"], c)
        i_met = bool(selected_i and _position_i_ready(metrics, c))
        stable = p_met and d_met and i_met
        self.stage_summary[name] = {
            "stage": "position", "baseline_score": history["p"][0]["result"]["metrics"]["score"],
            "best_score": metrics["score"], "best_metrics": metrics,
            "oscillation_limit": c["position_d_band_m"], "stable": stable,
            "initial_source": initial_source, "initial_gains": initial,
            "recommended_gains": {key: self.best[name][key]
                                  for key in ("kp", "ki", "kd")},
            "p_criteria_met": p_met, "d_criteria_met": d_met,
            "i_criteria_met": i_met,
            "d_needed": selected_d["value"] > 0,
            "i_needed": bool(selected_i and selected_i["value"] > 0),
            "d_settling_reference_seconds": d_score_start,
            "d_reference_trial": selected_d["result"]["label"],
            "d_reference_telemetry_csv": selected_d["result"]["telemetry_csv"],
            "phase_trial_counts": {key: len(items) for key, items in history.items()},
            "trial_count": sum(len(items) for items in history.values()),
            "trial_limit": 3 * limit,
        }
        self._save_summary()
        return stable

    def _validate_position_distances(self, name: str, axis: str) -> bool:
        results = []
        for distance in self.config["position_validation_distances"]:
            result = self._evaluate(
                "position", name, axis, self.best,
                f"position_{name}_validate_{distance:g}",
                position_phase="validation", position_distance=distance)
            metrics = None if result is None else result["metrics"]
            results.append({"distance": distance,
                            "passed": bool(metrics and metrics["validation_passed"]),
                            "metrics": metrics,
                            "telemetry_csv": None if result is None else result[
                                "telemetry_csv"]})
            self.stage_summary[name]["distance_validation"] = results
            self._save_summary()
        passed = all(item["passed"] for item in results)
        self.stage_summary[name]["distance_validation_passed"] = passed
        self._save_summary()
        return passed

    def _tune_pid(self, stage: str, name: str, axis: str,
                  *, baseline: dict[str, Any] | None = None,
                  initial_source: str = "preset_or_resume") -> bool:
        initial_gains = {k: self.best[name][k] for k in ("kp", "ki", "kd")}
        if baseline is None:
            baseline = self._evaluate(
                stage, name, axis, self.best, f"{stage}_{name}_baseline")
        if baseline is None:
            self.stage_summary[name] = {
                "stage": stage, "status": "unscored",
                "reason": "Every baseline attempt was invalid or interrupted",
                "recommended_gains": {k: self.best[name][k] for k in ("kp", "ki", "kd")},
            }
            self._save_summary()
            return False
        best_score = baseline["metrics"]["score"]
        best_result = baseline
        initial_score = best_score
        main_rounds = self.config["search_rounds"]
        oscillation_limit = self.config["oscillation_tolerances"][stage]
        height_stage2 = stage == "height" and self.height_waypoints_mode
        yaw_stage = stage == "yaw" and self.yaw_waypoints_mode
        xy_stage = stage in ("acceleration", "velocity", "position") and self.xy_waypoints_mode

        def is_stable(metrics: dict[str, Any]) -> bool:
            if stage == "velocity":
                return _velocity_response_stable(
                    metrics, self.config["velocity_step"], self.config)
            if stage == "acceleration":
                return _acceleration_response_stable(metrics, self.config)
            if height_stage2:
                return _height_stage2_stable(metrics, self.config)
            if yaw_stage:
                return _yaw_stage_stable(metrics, self.config)
            if stage == "position" and self.xy_waypoints_mode:
                return _position_waypoints_stable(metrics, self.config)
            return (metrics["oscillation_rms"] <= oscillation_limit and
                    (stage != "height" or
                     (metrics.get("descent_reached_fraction", 1.0) == 1.0 and
                      metrics.get("peak_descent_speed", 0.0) <=
                      self.config["height_descent_max_speed"] and
                      metrics.get("descent_oscillation_rms", 0.0) <=
                      oscillation_limit)))

        def consider(candidate: dict[str, dict[str, Any]], label: str,
                     *, base_rc: int | None = None) -> None:
            nonlocal best_score, best_result
            result = (self._evaluate(stage, name, axis, candidate, label)
                      if base_rc is None else
                      self._evaluate(stage, name, axis, candidate, label,
                                     height_base_rc=base_rc))
            if result is None:
                return
            if _prefer_trial(result["metrics"], best_result["metrics"],
                             stage, self.config[f"{stage}_step"] if stage != "yaw"
                             else math.radians(self.config["yaw_step_deg"]),
                             self.config):
                best_score = result["metrics"]["score"]
                best_result = result
                self.best = candidate
                if base_rc is not None:
                    self.best_height_base_rc = base_rc
                self._save_summary()

        if (height_stage2 or yaw_stage or xy_stage) and main_rounds:
            prefix = ("height_stage2" if height_stage2 else
                      "yaw_stage" if yaw_stage else "xy")
            if stage == "acceleration":
                prefix = "acceleration"
            broad_anchor = copy.deepcopy(self.best)
            for gain_name in ("kp", "kd"):
                for index, factor in enumerate(
                        self.config[f"{prefix}_broad_gain_factors"]):
                    candidate = copy.deepcopy(broad_anchor)
                    candidate[name][gain_name] *= factor
                    if (candidate[name][gain_name] <
                            self.seed[name][gain_name] * 0.1 or
                            candidate[name][gain_name] >
                            self.seed[name][gain_name] *
                            self.config[f"{prefix}_gain_ceiling_factor"]):
                        continue
                    consider(candidate, f"{stage}_{name}_broad_{gain_name}_{index}")

        def search_gain_round(round_number: int) -> None:
            for gain_name in ("kp", "kd"):
                candidates = []
                for factor in self.config["gain_factors"]:
                    candidate = copy.deepcopy(self.best)
                    candidate[name][gain_name] *= factor ** (1 / (round_number + 1))
                    ceiling = (self.config["height_stage2_gain_ceiling_factor"]
                               if height_stage2 else
                               self.config["yaw_stage_gain_ceiling_factor"]
                               if yaw_stage else
                               self.config["acceleration_gain_ceiling_factor"]
                               if stage == "acceleration" else
                               self.config["xy_gain_ceiling_factor"]
                               if xy_stage else 2)
                    if (candidate[name][gain_name] < self.seed[name][gain_name] * 0.1
                            or candidate[name][gain_name] >
                            self.seed[name][gain_name] * ceiling):
                        continue
                    candidates.append(candidate)
                for index, candidate in enumerate(candidates):
                    label = f"{stage}_{name}_r{round_number}_{gain_name}_{index}"
                    consider(candidate, label)

        for round_number in range(main_rounds):
            search_gain_round(round_number)
        integral_trials = list(self.config["integral_trials"].get(name, []))
        if (stage == "height" and self.config["search_rounds"] > 0
                and self.best[name].get("i_limit") is not None):
            integral_trials.extend({
                "ki": self.seed[name]["kp"] * factor,
                "i_limit": self.best[name]["i_limit"],
            } for factor in self.config["auto_height_ki_factors"])
        for index, trial in enumerate(integral_trials):
            candidate = copy.deepcopy(self.best)
            candidate[name]["ki"] = trial["ki"]
            candidate[name]["i_limit"] = trial["i_limit"]
            label = f"{stage}_{name}_ki_{index}"
            consider(candidate, label)
        if height_stage2 and main_rounds:
            initial_base_rc = self.best_height_base_rc
            for offset in self.config["height_stage2_base_rc_offsets"]:
                base_rc = initial_base_rc + offset
                if not 1200 <= base_rc <= 1700:
                    continue
                consider(copy.deepcopy(self.best),
                         f"{stage}_{name}_base_{base_rc}", base_rc=base_rc)
        if main_rounds:
            for round_number in range(main_rounds,
                                      main_rounds + self.config["oscillation_extra_rounds"]):
                if is_stable(best_result["metrics"]):
                    break
                search_gain_round(round_number)
        stable = is_stable(best_result["metrics"])
        # consider() can finish on a worse probe. Keep the selected candidate
        # active for landing and for every downstream control stage.
        self.controller.set_configs(self.best)
        self.stage_summary[name] = {
            "stage": stage, "baseline_score": initial_score,
            "best_score": best_score,
            "best_metrics": best_result["metrics"],
            "oscillation_limit": oscillation_limit,
            "stable": stable,
            "initial_source": initial_source,
            "initial_gains": initial_gains,
            "recommended_gains": {k: self.best[name][k] for k in ("kp", "ki", "kd")},
        }
        self._save_summary()
        return True

    def _tune_velocity_pid_cycle_repeated(
            self, name: str, axis: str, *,
            initial_source: str = "small_p_start",
            baseline: dict[str, Any] | None = None) -> bool:
        """Tune P arrival, D transient, then I bias from three repeated flights."""
        c = self.config
        initial_gains = {key: self.best[name][key] for key in ("kp", "ki", "kd")}
        maximum = c["velocity_trials_per_coefficient"]
        history: dict[str, list[dict[str, Any]]] = {"p": [], "d": [], "i": []}
        d_score_start: float | None = None

        def evaluate(phase: str, value: float,
                     base: dict[str, dict[str, Any]]) -> dict[str, Any] | None:
            candidate = copy.deepcopy(base)
            candidate[name][{"p": "kp", "d": "kd", "i": "ki"}[phase]] = value
            label = f"velocity_{name}_{phase}_{len(history[phase]) + 1:02d}"
            result = self._evaluate(
                "velocity", name, axis, candidate, label,
                velocity_phase=phase,
                velocity_i_score_start=d_score_start if phase == "i" else None)
            if result is not None:
                history[phase].append({"value": value, "candidate": candidate,
                                       "result": result})
            return result

        def p_ready(metrics: dict[str, Any]) -> bool:
            return _velocity_repeated_p_ready(metrics, c)

        def d_ready(metrics: dict[str, Any]) -> bool:
            return _velocity_repeated_d_ready(metrics, c)

        def i_ready(metrics: dict[str, Any]) -> bool:
            parts = metrics["i_mean_response_by_direction"]
            return (len(parts) == 2 and
                    all(part["score_window_valid"] and part["hold_fraction"] >=
                        c["velocity_i_mean_required_fraction"]
                        for part in parts.values()))

        p_base = copy.deepcopy(self.best)
        p_base[name].update(kp=0.0, ki=0.0, kd=0.0)
        # A later cycle starts around its previous complete controller while
        # isolating P; remove the reserve added before D on the prior cycle.
        kp = (0.0 if initial_source == "zero_gains_start" else
              initial_gains["kp"] / c["velocity_p_after_d_factor"]
              if initial_source != "small_p_start" and initial_gains["kp"] > 0
              else c["velocity_p_start"])
        slow, fast = None, None
        for _ in range(maximum):
            result = evaluate("p", kp, p_base)
            if result is None:
                break
            metrics = result["metrics"]
            if p_ready(metrics):
                break
            if kp == 0.0:
                slow = 0.0
                kp = c["velocity_p_start"]
                continue
            signed = []
            for plan in c["velocity_p_targets"]:
                signed.extend((part["arrival_time"] - plan["arrival_seconds"])
                              if part["arrived"] else plan["arrival_seconds"]
                              for part in metrics["mean_steps"]
                              if part["speed"] == plan["speed"])
            too_fast = all(part["arrived"] for part in metrics["mean_steps"]) and mean(signed) < 0
            if too_fast:
                fast = kp
                kp = (slow + fast) / 2 if slow is not None else kp / c[
                    "velocity_p_max_multiplier"]
            else:
                slow = kp
                kp = (slow + fast) / 2 if fast is not None else kp * c[
                    "velocity_p_max_multiplier"]
            if kp <= 0 or (slow is not None and fast is not None and
                           fast - slow < 1e-5):
                break
        if not history["p"]:
            self.stage_summary[name] = {
                "stage": "velocity", "status": "unscored", "stable": False,
                "reason": "Every repeated P trial was invalid or interrupted",
                "recommended_gains": initial_gains}
            self._save_summary()
            return False
        ready_p = [item for item in history["p"]
                   if p_ready(item["result"]["metrics"])]
        selected_p = min(ready_p or history["p"],
                         key=lambda item: item["result"]["metrics"]["score"])
        p_met = p_ready(selected_p["result"]["metrics"])
        d_base = copy.deepcopy(selected_p["candidate"])
        d_base[name]["kp"] *= c["velocity_p_after_d_factor"]

        def search_phase(phase: str, base: dict[str, dict[str, Any]],
                         start: float, multiplier: float,
                         ready: Callable[[dict[str, Any]], bool]
                         ) -> dict[str, Any] | None:
            values = [0.0]
            if start > 0:
                values.append(start)
            best = None
            best_score = math.inf
            worse = 0
            index = 0
            while len(history[phase]) < maximum:
                if index >= len(values):
                    values.append(values[-1] * multiplier)
                value = values[index]
                index += 1
                result = evaluate(phase, value, base)
                if result is None:
                    break
                score = result["metrics"]["score"]
                if score < best_score:
                    best = history[phase][-1]
                    best_score = score
                    worse = 0
                else:
                    worse += 1
                if value == 0.0 and phase in ("d", "i") and ready(result["metrics"]):
                    return history[phase][-1]
                if phase == "i" and ready(result["metrics"]):
                    return history[phase][-1]
                enough_exploration = (phase != "d" or len(history[phase]) >=
                                      c["velocity_d_min_exploration_trials"])
                if ready(result["metrics"]) and worse >= 1 and enough_exploration:
                    break
                if worse >= 2 and len(history[phase]) >= 4 and enough_exploration:
                    break
            # Refine the best neighbourhood after geometric exploration.
            ordered = sorted(history[phase], key=lambda item: item["value"])
            if best is not None and len(history[phase]) < maximum:
                position = ordered.index(best)
                neighbours = [ordered[i]["value"] for i in
                              (position - 1, position + 1) if 0 <= i < len(ordered)]
                for neighbour in neighbours:
                    if len(history[phase]) >= maximum:
                        break
                    middle = (best["value"] + neighbour) / 2
                    if middle == best["value"] or middle == neighbour:
                        continue
                    result = evaluate(phase, middle, base)
                    if result is not None and result["metrics"]["score"] < best_score:
                        best = history[phase][-1]
                        best_score = result["metrics"]["score"]
            passing = [item for item in history[phase]
                       if ready(item["result"]["metrics"])]
            return min(passing, key=lambda item: item["result"]["metrics"]["score"]) if passing else best

        d_start = (initial_gains["kd"] if initial_source != "small_p_start" and
                   initial_gains["kd"] > 0 else c["velocity_d_start"])
        selected_d = search_phase("d", d_base, d_start,
                                  c["velocity_d_multiplier"], d_ready)
        if selected_d is None:
            self.stage_summary[name] = {
                "stage": "velocity", "status": "unscored", "stable": False,
                "reason": "No valid repeated D trial",
                "recommended_gains": initial_gains}
            self._save_summary()
            return False
        d_steps = selected_d["result"]["metrics"]["mean_steps"]
        d_score_start = mean(part["settling_time"] for part in d_steps)
        i_base = copy.deepcopy(selected_d["candidate"])
        i_start = (initial_gains["ki"] if initial_source != "small_p_start" and
                   initial_gains["ki"] > 0 else c["velocity_i_start"])
        selected_i = search_phase("i", i_base, i_start,
                                  c["velocity_i_multiplier"], i_ready)
        chosen = selected_i or selected_d
        self.best = copy.deepcopy(chosen["candidate"])
        metrics = chosen["result"]["metrics"]
        # The P arrival and D settling criteria complete tuning. I remains
        # measured and reported, but its hold corridor is diagnostic here.
        stable = p_met and d_ready(selected_d["result"]["metrics"])
        self.stage_summary[name] = {
            "stage": "velocity", "baseline_score": history["p"][0]["result"]["metrics"]["score"],
            "best_score": metrics["score"], "best_metrics": metrics,
            "stable": stable, "initial_source": initial_source,
            "initial_gains": initial_gains,
            "recommended_gains": {key: self.best[name][key]
                                  for key in ("kp", "ki", "kd")},
            "p_criteria_met": p_met,
            "d_criteria_met": d_ready(selected_d["result"]["metrics"]),
            "d_settling_reference_seconds": d_score_start,
            "d_reference_trial": selected_d["result"]["label"],
            "d_reference_telemetry_csv": selected_d["result"]["telemetry_csv"],
            "d_reference_steps": [
                {key: part[key] for key in ("direction", "cycle", "settled",
                                            "settling_time")}
                for part in d_steps],
            "i_needed": True, "i_criteria_met": bool(selected_i and i_ready(metrics)),
            "repeatable": True, "trial_count": sum(len(items) for items in history.values()),
            "trial_limit": 3 * maximum, "coefficient_trial_limit": maximum,
            "phase_trial_counts": {key: len(items) for key, items in history.items()},
            "phase_trials": [{"phase": phase, "label": item["result"]["label"],
                              "gains": item["result"]["gains"],
                              "score": item["result"]["metrics"]["score"],
                              "telemetry_csv": item["result"]["telemetry_csv"]}
                             for phase, items in history.items() for item in items],
        }
        self._save_summary()
        return stable

    def _tune_velocity_pid_cycle(
            self, name: str, axis: str, *,
            initial_source: str = "small_p_start",
            baseline: dict[str, Any] | None = None) -> bool:
        """Tune pure PID at the configured speed in ordered P, D, optional I phases."""
        c = self.config
        initial = copy.deepcopy(self.best)
        initial_gains = {key: initial[name][key] for key in ("kp", "ki", "kd")}
        retaining_prior_gains = initial_source != "small_p_start"
        maximum = c["velocity_trials_per_coefficient"]
        signatures: set[tuple[float, float, float]] = set()
        trials: list[tuple[str, dict[str, Any], dict[str, dict[str, Any]]]] = []
        attempted = 0

        if baseline is not None:
            signatures.add(tuple(initial[name][key] for key in ("kp", "ki", "kd")))
            trials.append(("transfer", baseline, copy.deepcopy(initial)))
            attempted = 1

        def run(candidate: dict[str, dict[str, Any]], phase: str,
                label: str, *, hold_seconds: float | None = None) -> dict[str, Any] | None:
            nonlocal attempted
            signature = tuple(candidate[name][key] for key in ("kp", "ki", "kd"))
            if signature in signatures:
                return None
            signatures.add(signature)
            attempted += 1
            d_mode = phase in ("d_base", "d", "i", "fine")
            profile_seconds = (hold_seconds if hold_seconds is not None else
                               c["velocity_d_trial_seconds"] if d_mode else None)
            result = self._evaluate(
                "velocity", name, axis, candidate, label,
                velocity_hold_seconds=profile_seconds,
                velocity_p_mode=(phase == "p"), velocity_d_mode=d_mode)
            if result is not None:
                trials.append((phase, result, copy.deepcopy(candidate)))
            return result

        def p_ready(metrics: dict[str, Any]) -> bool:
            parts = (metrics["positive"], metrics["negative"])
            target = c["velocity_p_target_arrival_seconds"]
            tolerance = c["velocity_p_time_tolerance_seconds"]
            return (all(part.get("arrived", part.get("reached",
                                                       metrics.get("reached_both", False)))
                        for part in parts)
                    and all(abs(part.get("arrival_time", part["rise_time"]) -
                                target) <= tolerance
                            for part in parts))

        def p_quality(item: tuple[dict[str, Any], dict[str, dict[str, Any]]]) -> tuple:
            result, _ = item
            metrics = result["metrics"]
            return (not p_ready(metrics), metrics["score"] if "score" in metrics
                    else score_velocity_p_arrival(metrics, c))

        def response_and_brake_ready(metrics: dict[str, Any]) -> bool:
            parts = (metrics["positive"], metrics["negative"])
            return (
                all(part.get("arrived", part.get("reached",
                                                    metrics.get("reached_both", False)))
                    for part in parts)
                and all(c["velocity_rise_time_min"] <=
                        part.get("arrival_time", part["rise_time"]) <=
                        c["velocity_rise_time_max"] for part in parts)
                and all(part.get("d_brake_reached", metrics.get("stopped_both", False))
                        for part in parts)
                and not any(part.get("d_brake_reversed", False) for part in parts)
                and metrics["oscillation_rms"] <= c["oscillation_tolerances"]["velocity"]
            )

        def d_ready(metrics: dict[str, Any]) -> bool:
            return (response_and_brake_ready(metrics) and all(
                part.get("d_hold_fraction_after_arrival",
                         metrics.get("tail_in_band_fraction", 0.0)) >=
                c["velocity_d_required_hold_fraction"]
                for part in (metrics["positive"], metrics["negative"])))

        def i_ready(metrics: dict[str, Any]) -> bool:
            return (response_and_brake_ready(metrics) and all(
                part.get("i_hold_fraction_after_arrival",
                         part.get("d_hold_fraction_after_arrival",
                                  metrics.get("tail_in_band_fraction", 0.0))) >=
                c["velocity_i_required_hold_fraction"]
                for part in (metrics["positive"], metrics["negative"])))

        def final_quality(item: tuple[dict[str, Any], dict[str, dict[str, Any]]]) -> tuple:
            result, _ = item
            metrics = result["metrics"]
            parts = (metrics["positive"], metrics["negative"])
            # Selection is lexicographic rather than a blind descent over one
            # aggregate score.  A candidate that happens to have a lower
            # score must not win if it cannot brake or does not hold the
            # corridor; score is retained only as the final tie-breaker.
            return (
                not d_ready(metrics),
                not all(part.get("d_brake_reached", metrics["stopped_both"])
                        for part in parts),
                max(0.0, c["velocity_d_required_hold_fraction"] -
                    min(part.get("d_hold_fraction_after_arrival",
                                 metrics["tail_in_band_fraction"])
                        for part in parts)),
                max(0.0, c["velocity_required_in_band_fraction"] -
                    metrics["in_band_fraction"]),
                max(part.get("terminal_mae", metrics["terminal_mae"]) /
                    c["velocity_target_speed"]
                    for part in parts),
                max(part.get("stop_terminal_speed",
                             0.0 if metrics["stopped_both"] else float("inf"))
                    for part in parts),
                metrics["oscillation_rms"],
                metrics["score"],
            )

        def i_quality(item: tuple[dict[str, Any], dict[str, dict[str, Any]]]) -> tuple:
            """Rank I candidates without allowing a different P/D pair to enter."""
            result, _ = item
            metrics = result["metrics"]
            parts = (metrics["positive"], metrics["negative"])
            strict_hold = min(
                part.get("i_hold_fraction_after_arrival",
                         part.get("d_hold_fraction_after_arrival",
                                  metrics["tail_in_band_fraction"]))
                for part in parts)
            return (
                not i_ready(metrics),
                max(0.0, c["velocity_i_required_hold_fraction"] - strict_hold),
                max(part.get("terminal_mae", metrics["terminal_mae"]) /
                    c["velocity_target_speed"]
                    for part in parts),
                metrics["oscillation_rms"],
                metrics["score"],
            )

        def guided_gain_search(
                phase: str, start: float, multiplier: float, limit: int,
                make_candidate: Callable[[float], dict[str, dict[str, Any]]],
                quality: Callable[[tuple[dict[str, Any], dict[str, dict[str, Any]]]], tuple],
                ready: Callable[[dict[str, Any]], bool],
                *, hold_seconds: float | None = None
        ) -> list[tuple[dict[str, Any], dict[str, dict[str, Any]]]]:
            """Bracket a 1-D gain minimum, then refine it in log space.

            Three initial probes establish whether the useful direction is
            lower or higher. Later probes either expand only toward the best
            boundary or bisect the more promising adjacent interval.
            """
            records: list[tuple[float, dict[str, Any], dict[str, dict[str, Any]]]] = []
            visited: set[float] = set()

            def probe(value: float) -> bool:
                value = max(value, 1e-12)
                key = round(math.log(value), 12)
                if key in visited or len(visited) >= limit:
                    return False
                visited.add(key)
                candidate = make_candidate(value)
                result = run(candidate, phase,
                             f"velocity_{name}_{phase}_{len(visited):02d}",
                             hold_seconds=hold_seconds)
                if result is not None:
                    records.append((value, result, candidate))
                return True

            for value in (start, start / multiplier, start * multiplier):
                probe(value)
            while records and len(visited) < limit:
                best_value, best_result, best_candidate = min(
                    records, key=lambda item: quality((item[1], item[2])))
                if len(visited) >= 3 and ready(best_result["metrics"]):
                    break
                ordered = sorted(records, key=lambda item: item[0])
                index = next(i for i, item in enumerate(ordered)
                             if item[0] == best_value)
                if index == 0:
                    next_value = best_value / multiplier
                elif index == len(ordered) - 1:
                    next_value = best_value * multiplier
                else:
                    left, right = ordered[index - 1], ordered[index + 1]
                    neighbour = min((left, right),
                                    key=lambda item: quality((item[1], item[2])))
                    next_value = math.sqrt(best_value * neighbour[0])
                if not probe(next_value):
                    break
            return [(result, candidate) for _, result, candidate in records]

        def gain_candidate(base: dict[str, dict[str, Any]], gain: str,
                           value: float) -> dict[str, dict[str, Any]]:
            candidate = copy.deepcopy(base)
            candidate[name][gain] = value
            return candidate

        # P: bracket the gain that reaches the speed at the configured time;
        # early and late arrivals have equal cost. D and I stay at zero.
        p_results: list[tuple[dict[str, Any], dict[str, dict[str, Any]]]] = []
        # Stored gains belong to the D/I controller and already include the
        # 20% P reserve.  Remove that reserve for the isolated pure-P probe;
        # D will add it back before its own trials.  Without this conversion,
        # every large cycle would inflate P by another 20%.
        kp = (initial[name]["kp"] / c["velocity_p_after_d_factor"]
              if retaining_prior_gains else
              initial[name]["kp"] if baseline is not None else
              c["velocity_p_start"])
        slow_kp: float | None = None
        fast_kp: float | None = None
        p_trials = 0
        while p_trials < maximum:
            candidate = copy.deepcopy(initial)
            candidate[name]["kp"] = kp
            candidate[name]["ki"] = 0.0
            candidate[name]["kd"] = 0.0
            result = run(candidate, "p", f"velocity_{name}_p_{attempted + 1:02d}")
            p_trials += 1
            if result is None:
                # An invalid flight contains no direction for P.  In
                # particular, never turn a manual/interrupted sample into a
                # large arbitrary gain increase.
                break
            p_results.append((result, candidate))
            metrics = result["metrics"]
            if p_ready(metrics):
                break
            parts = (metrics["positive"], metrics["negative"])
            # The P phase reacts only to the first arrival time.  Hold,
            # braking, overshoot, and oscillation belong to D/I and final
            # validation, although they remain in telemetry for inspection.
            arrived_both = all(part.get("arrived", False) for part in parts)
            arrival_times = [part.get("arrival_time", part["rise_time"])
                             for part in parts]
            too_fast = (arrived_both and
                        mean(arrival_times) < c["velocity_p_target_arrival_seconds"])
            if too_fast:
                fast_kp = kp
                if slow_kp is not None:
                    kp = (slow_kp + fast_kp) / 2
                else:
                    kp /= c["velocity_p_max_multiplier"]
            else:
                slow_kp = kp
                if fast_kp is not None:
                    kp = (slow_kp + fast_kp) / 2
                else:
                    rise_factor = max(arrival_times) / c[
                        "velocity_p_target_arrival_seconds"]
                    factor = min(c["velocity_p_max_multiplier"], max(
                        c["velocity_p_min_multiplier"], rise_factor))
                    kp *= factor

        if not p_results:
            self.stage_summary[name] = {
                "stage": "velocity", "status": "unscored", "stable": False,
                "reason": "Every pure-P trial was invalid or interrupted",
                "initial_source": initial_source,
                "recommended_gains": initial_gains,
            }
            self._save_summary()
            return False

        p_result, p_candidate = min(p_results, key=p_quality)
        p_criteria_met = p_ready(p_result["metrics"])
        working_results = [(p_result, copy.deepcopy(p_candidate))]

        # Apply the configured P factor for the D/I phases.
        d_trials = 0
        # Even when P exhausted its attempts without becoming optimal, use
        # its best value as the starting point for D.
        d_base = copy.deepcopy(p_candidate)
        d_base[name]["kp"] *= c["velocity_p_after_d_factor"]
        transition = run(d_base, "d_base", f"velocity_{name}_d_base")
        if transition is not None:
            working_results = [(transition, d_base)]
            p_candidate = d_base

        # D brackets the better damping value rather than scanning a fixed grid.
        d_value = (initial[name]["kd"] if retaining_prior_gains and
                   initial[name]["kd"] > 0.0 else c["velocity_d_start"])
        d_results = guided_gain_search(
            "d", d_value, c["velocity_d_multiplier"],
            min(maximum, c["velocity_d_trials"]),
            lambda value: gain_candidate(p_candidate, "kd", value),
            final_quality, d_ready)
        d_trials = len(d_results)
        for result, candidate in d_results:
            working_results.append((result, candidate))

        best_result, best_candidate = min(working_results, key=final_quality)
        d_criteria_met = d_ready(best_result["metrics"])

        # I is a separate, stricter hold stage. If P or D still falls short,
        # I is nevertheless sampled during this
        # large cycle; the next cycle receives the best complete P/D/I set.
        i_needed = not i_ready(best_result["metrics"])
        i_value = (initial[name]["ki"] if retaining_prior_gains and
                   initial[name]["ki"] > 0.0 else c["velocity_i_start"])
        i_trials = 0
        i_results = [(best_result, copy.deepcopy(best_candidate))]
        i_results_found = (guided_gain_search(
            "i", i_value, c["velocity_i_multiplier"],
            min(maximum, c["velocity_i_trials"]),
            lambda value: gain_candidate(best_candidate, "ki", value),
            i_quality, i_ready, hold_seconds=c["velocity_i_hold_seconds"])
            if i_needed else [])
        i_trials = len(i_results_found)
        for result, candidate in i_results_found:
            # Every I candidate is a copy of the selected P/D pair: only Ki
            # is allowed to change in this stage.
            i_results.append((result, candidate))
            best_result, best_candidate = min(i_results, key=i_quality)
            if i_ready(best_result["metrics"]):
                break

        i_criteria_met = i_ready(best_result["metrics"])

        self.best = copy.deepcopy(best_candidate)
        repeat = None
        attempted += 1
        repeat = self._evaluate(
            "velocity", name, axis, self.best, f"velocity_{name}_repeat",
            velocity_hold_seconds=(c["velocity_i_hold_seconds"] if i_needed else
                                   c["velocity_d_trial_seconds"]),
            velocity_d_mode=True)
        repeatable = False
        if repeat is not None:
            high = max(best_result["metrics"]["score"], repeat["metrics"]["score"])
            low = max(min(best_result["metrics"]["score"], repeat["metrics"]["score"]),
                      1e-9)
            repeatable = (high / low - 1 <=
                          c["velocity_repeat_tolerance_fraction"])
        ready = i_ready if i_needed else d_ready
        stable = (ready(best_result["metrics"]) and repeat is not None and
                  ready(repeat["metrics"]) and repeatable)
        compact_trials = [{
            "phase": phase, "label": result["label"],
            "gains": result["gains"], "score": result["metrics"]["score"],
            "rise_time_positive": result["metrics"]["positive"]["rise_time"],
            "rise_time_negative": result["metrics"]["negative"]["rise_time"],
            "arrival_time_positive": result["metrics"]["positive"].get(
                "arrival_time", result["metrics"]["positive"]["rise_time"]),
            "arrival_time_negative": result["metrics"]["negative"].get(
                "arrival_time", result["metrics"]["negative"]["rise_time"]),
            "d_hold_fraction_positive": result["metrics"]["positive"].get(
                "d_hold_fraction_after_arrival",
                result["metrics"]["positive"].get("tail_in_band_fraction", 0.0)),
            "d_hold_fraction_negative": result["metrics"]["negative"].get(
                "d_hold_fraction_after_arrival",
                result["metrics"]["negative"].get("tail_in_band_fraction", 0.0)),
            "i_hold_fraction_positive": result["metrics"]["positive"].get(
                "i_hold_fraction_after_arrival",
                result["metrics"]["positive"].get(
                    "d_hold_fraction_after_arrival", 0.0)),
            "i_hold_fraction_negative": result["metrics"]["negative"].get(
                "i_hold_fraction_after_arrival",
                result["metrics"]["negative"].get(
                    "d_hold_fraction_after_arrival", 0.0)),
            "d_brake_reached_positive": result["metrics"]["positive"].get(
                "d_brake_reached", result["metrics"].get("stopped_both", False)),
            "d_brake_reached_negative": result["metrics"]["negative"].get(
                "d_brake_reached", result["metrics"].get("stopped_both", False)),
            "peak_speed": max(result["metrics"]["positive"]["max_speed"],
                              result["metrics"]["negative"]["max_speed"]),
            "in_band_fraction": result["metrics"]["in_band_fraction"],
            "tail_in_band_fraction": result["metrics"]["tail_in_band_fraction"],
            "p95_acceleration": result["metrics"]["p95_acceleration"],
            "p95_jerk": result["metrics"]["p95_jerk"],
            "telemetry_csv": result["telemetry_csv"],
        } for phase, result, _ in trials]
        self.stage_summary[name] = {
            "stage": "velocity",
            "baseline_score": p_results[0][0]["metrics"]["score"],
            "best_score": best_result["metrics"]["score"],
            "best_metrics": best_result["metrics"],
            "oscillation_limit": c["oscillation_tolerances"]["velocity"],
            "repeat_metrics": None if repeat is None else repeat["metrics"],
            "repeatable": repeatable,
            "stable": stable,
            "initial_source": initial_source,
            "initial_gains": initial_gains,
            "recommended_gains": {key: self.best[name][key]
                                  for key in ("kp", "ki", "kd")},
            "target_speed": c["velocity_target_speed"],
            "target_corridor": [c["velocity_target_min_speed"],
                                c["velocity_target_max_speed"]],
            "p_criteria_met": p_criteria_met,
            "d_criteria_met": d_criteria_met,
            "i_needed": i_needed,
            "i_criteria_met": i_criteria_met,
            "trial_count": attempted,
            "trial_limit": 3 * maximum + 1,
            "coefficient_trial_limit": maximum,
            "phase_trial_counts": {"p": p_trials, "d": d_trials, "i": i_trials},
            "phase_trials": compact_trials,
        }
        self._save_summary()
        # A valid but unstable X result must not seed Y.  It remains in the
        # report for inspection, then the staged run lands as needs_review.
        return stable

    def _confirm_velocity_axis(self, name: str, axis: str,
                               cycle: int) -> list[dict[str, Any]]:
        """Check one completed large cycle at every requested speed."""
        outcomes = []
        for speed in self.config["velocity_confirmation_speeds"]:
            label = f"velocity_{axis}_cycle_{cycle}_confirm_{speed:g}"
            result = self._evaluate("velocity", name, axis, self.best, label,
                                    velocity_speed=speed)
            metrics = None if result is None else result["metrics"]
            outcomes.append({
                "speed": speed,
                "passed": (metrics is not None and _velocity_response_stable(
                    metrics, speed, self.config)),
                "metrics": metrics,
                "telemetry_csv": result["telemetry_csv"] if result else self.last_csv,
            })
        return outcomes

    def _velocity_minimally_acceptable(self, metrics: dict[str, Any]) -> bool:
        """A weak but useful result: it reaches the command without violent motion."""
        if "repeat_steps" in metrics:
            return metrics["reached_both"] and metrics["plateau_settled_fraction"] >= 0.5
        parts = (metrics["positive"], metrics["negative"])
        return (all(part.get("arrived", part.get("reached", False)) for part in parts)
                and not any(part.get("d_brake_reversed", False) for part in parts)
                and metrics["oscillation_rms"] <=
                2 * self.config["oscillation_tolerances"]["velocity"])

    def _tune_velocity_pid(
            self, name: str, axis: str, *,
            initial_source: str = "small_p_start",
            baseline: dict[str, Any] | None = None) -> bool:
        """Run P→D→I once; leave further refinement to a new user run."""
        c = self.config
        cycles: list[dict[str, Any]] = []
        stop_reason: str | None = None
        tuner = (self._tune_velocity_pid_cycle_repeated
                 if c["velocity_repeated_mode"] else self._tune_velocity_pid_cycle)
        cycle_stable = tuner(name, axis, initial_source=initial_source,
                             baseline=baseline)
        # Both velocity tuners retain the selected candidate in self.best, but
        # their final physical probe may have used a different candidate.
        self.controller.set_configs(self.best)
        current = copy.deepcopy(self.stage_summary.get(name, {}))
        if current.get("status") == "unscored":
            stop_reason = current["reason"]
            final_stable = False
        else:
            metrics = current.get("best_metrics")
            confirmation = (self._confirm_velocity_axis(name, axis, 1)
                            if self.xy_waypoints_mode and not c["velocity_repeated_mode"]
                            else [])
            confirmed = bool(confirmation) and all(item["passed"] for item in confirmation)
            final_stable = cycle_stable and (confirmed if confirmation else True)
            minimally_acceptable = (metrics is not None and
                                    self._velocity_minimally_acceptable(metrics))
            oscillating_failure = bool(metrics is not None and
                                       metrics["oscillation_rms"] >
                                       c["oscillation_tolerances"]["velocity"] and
                                       not minimally_acceptable)
            cycles.append({
                "cycle": 1,
                "stable_at_target_speed": cycle_stable,
                "confirmation": confirmation,
                "confirmation_passed": confirmed if confirmation else None,
                "minimally_acceptable": minimally_acceptable,
                "oscillating_failure": oscillating_failure,
                "gains": {key: self.best[name][key] for key in ("kp", "ki", "kd")},
                "cycle_summary": current,
            })

        summary = self.stage_summary.get(name, {})
        summary.update({
            "stable": final_stable,
            "tuning_cycles": cycles,
            "completed_tuning_cycles": len(cycles),
            "tuning_cycle_limit": 1,
            "oscillation_stop_reason": stop_reason,
            "recommended_gains": {key: self.best[name][key]
                                  for key in ("kp", "ki", "kd")},
        })
        self.stage_summary[name] = summary
        self._save_summary()
        return final_stable

    def _validate_velocity_speeds(self, name: str, axis: str) -> bool:
        """Check fixed gains at each requested speed without changing them."""
        results = []
        for speed in self.config["velocity_validation_speeds"]:
            result = self._evaluate(
                "velocity", name, axis, self.best,
                f"velocity_{name}_validate_{speed:g}",
                velocity_phase="validation", velocity_magnitude=speed)
            metrics = None if result is None else result["metrics"]
            results.append({
                "speed": speed,
                "passed": bool(metrics and metrics["validation_passed"]),
                "metrics": metrics,
                "telemetry_csv": None if result is None else result["telemetry_csv"],
            })
            self.stage_summary[name]["speed_validation"] = results
            self._save_summary()
        passed = all(item["passed"] for item in results)
        self.stage_summary[name]["speed_validation_passed"] = passed
        # Cross-speed checks are reported for review. The requested completion
        # criterion for repeated velocity tuning is P arrival plus D settling.
        self._save_summary()
        return passed

    def _evaluate_joint(self, stage: str, names: tuple[str, str],
                        candidate: dict[str, dict[str, Any]],
                        label: str) -> dict[str, Any] | None:
        """Score a short diagonal flight with both XY axes active."""
        attempt = 0
        while attempt <= self.config["max_intervention_retries"]:
            if self._xy_drone_is_down(stage):
                self._wait_for_xy_respawn(stage)
                attempt = 0
            try:
                self._recover(stage)
                self._ensure_xy_zone(stage)
                if self.last_sample is not None:
                    self.anchor = self.last_sample
                self.controller.set_configs(candidate)
                self.controller.reset(self.last_sample)
                rows, csv_name = self._profile(stage, "xy", label, joint=True)
            except (InterventionDetected, TrialInvalid) as exc:
                self.records.append({
                    "stage": f"joint_{stage}", "pid": list(names), "axis": "xy",
                    "label": label, "attempt": attempt + 1, "valid": False,
                    "reason": str(exc), "telemetry_csv": self.last_csv,
                })
                self._save_summary()
                print(f"joint_{stage} {label}: ignored invalid trial (attempt {attempt + 1})")
                if self._xy_drone_is_down(stage):
                    self._wait_for_xy_respawn(stage)
                    attempt = 0
                    continue
                attempt += 1
                continue
            invalid = _invalid_xy_trial(rows, stage, self.config)
            if invalid is not None:
                self.records.append({
                    "stage": f"joint_{stage}", "pid": list(names), "axis": "xy",
                    "label": label, "attempt": attempt + 1, "valid": False,
                    "reason": invalid, "telemetry_csv": csv_name,
                })
                self._save_summary()
                if self._xy_drone_is_down(stage):
                    self._wait_for_xy_respawn(stage)
                    attempt = 0
                    continue
                attempt += 1
                continue
            amplitude = self.config[f"{stage}_step"] / math.sqrt(2)
            try:
                metrics = {
                    axis: (score_velocity_steps(rows, axis, amplitude, self.config)
                           if stage == "velocity" else
                           score_position_waypoints(rows, axis, self.config)
                           if stage == "position" and self.xy_waypoints_mode else
                           score_rows(
                               rows, stage, axis, amplitude,
                               oscillation_window_seconds=self.config["oscillation_window_seconds"],
                               oscillation_weight=self.config["oscillation_weight"],
                           )) for axis in ("x", "y")
                }
            except TrialInvalid as exc:
                self.records.append({
                    "stage": f"joint_{stage}", "pid": list(names), "axis": "xy",
                    "label": label, "attempt": attempt + 1, "valid": False,
                    "reason": str(exc), "telemetry_csv": csv_name,
                })
                self._save_summary()
                attempt += 1
                continue
            scores = [metrics[axis]["score"] for axis in ("x", "y")]
            score = max(scores) + 0.25 * mean(scores)
            record = {
                "stage": f"joint_{stage}", "pid": list(names), "axis": "xy",
                "label": label, "attempt": attempt + 1, "valid": True,
                "gains": {name: {k: candidate[name][k] for k in ("kp", "ki", "kd")}
                          for name in names},
                "metrics": {"score": score, "x": metrics["x"], "y": metrics["y"]},
                "telemetry_csv": csv_name,
            }
            self.records.append(record)
            self._save_summary()
            print(f"joint_{stage} {label}: score={score:.3f}, "
                  f"X osc={metrics['x']['oscillation_rms']:.4f}, "
                  f"Y osc={metrics['y']['oscillation_rms']:.4f}")
            return record
        return None

    def _tune_joint(self, stage: str, names: tuple[str, str]) -> bool:
        key = f"joint_{stage}"
        baseline = self._evaluate_joint(stage, names, self.best, f"{key}_baseline")
        if baseline is None:
            self.stage_summary[key] = {
                "status": "unscored", "stable": False,
                "reason": "Every joint baseline attempt was invalid or interrupted",
            }
            self._save_summary()
            return False
        best_result = baseline
        best_score = baseline["metrics"]["score"]
        for gain_name in ("kp", "kd"):
            for index, factor in enumerate(self.config["joint_gain_factors"]):
                candidate = copy.deepcopy(self.best)
                for name in names:
                    candidate[name][gain_name] *= factor
                label = f"{key}_{gain_name}_{index}"
                result = self._evaluate_joint(stage, names, candidate, label)
                if result is None:
                    continue
                score = result["metrics"]["score"]
                if _prefer_trial(result["metrics"], best_result["metrics"],
                                 stage, self.config[f"{stage}_step"] / math.sqrt(2),
                                 self.config, joint=True):
                    best_score = score
                    best_result = result
                    self.best = candidate
                    self._save_summary()
        limit = self.config["oscillation_tolerances"][stage]
        metrics = best_result["metrics"]
        joint_stable = (all(_velocity_response_stable(
            metrics[axis], self.config["velocity_step"] / math.sqrt(2), self.config)
            for axis in ("x", "y")) if stage == "velocity" else
            all(_position_waypoints_stable(metrics[axis], self.config)
                for axis in ("x", "y")) if self.xy_waypoints_mode else
            all(metrics[axis]["oscillation_rms"] <= limit for axis in ("x", "y")))
        self.stage_summary[key] = {
            "baseline_score": baseline["metrics"]["score"],
            "best_score": best_score,
            "best_metrics": metrics,
            "oscillation_limit": limit,
            "stable": joint_stable,
            "recommended_gains": {
                name: {k: self.best[name][k] for k in ("kp", "ki", "kd")}
                for name in names
            },
        }
        for name, axis in zip(names, ("x", "y")):
            self.stage_summary[name]["recommended_gains"] = {
                k: self.best[name][k] for k in ("kp", "ki", "kd")
            }
            self.stage_summary[name]["isolated_stable"] = self.stage_summary[name]["stable"]
            self.stage_summary[name]["final_joint_metrics"] = metrics[axis]
            self.stage_summary[name]["stable"] = (
                _velocity_response_stable(
                    metrics[axis], self.config["velocity_step"] / math.sqrt(2), self.config)
                if stage == "velocity" else
                _position_waypoints_stable(metrics[axis], self.config)
                if self.xy_waypoints_mode else
                metrics[axis]["oscillation_rms"] <= limit)
        self.controller.set_configs(self.best)
        self._save_summary()
        return True

    def _validation(self) -> bool:
        for attempt in range(self.config["max_intervention_retries"] + 1):
            try:
                return self._validation_once()
            except InterventionDetected as exc:
                self.stage_summary["validation"] = {
                    "passed": False, "attempt": attempt + 1,
                    "reason": f"Manual move during validation: {exc}",
                    "telemetry_csv": self.last_csv,
                }
                self._save_summary()
                print(f"Validation ignored after manual move (attempt {attempt + 1})")
        return False

    def _validation_once(self) -> bool:
        self._recover("position")
        if self.last_sample is not None:
            self.anchor = self.last_sample
        a = self.anchor
        assert a is not None
        self.controller.set_configs(self.best)
        self.controller.reset(self.last_sample)
        rows: list[dict[str, Any]] = []
        target = self._target(
            height=self.config["hover_height"] + self.config["height_step"] * 0.5,
            yaw=a.yaw + math.radians(self.config["yaw_step_deg"] * 0.6),
            position=(a.x + self.config["position_step"] * 0.7,
                      a.y + self.config["position_step"] * 0.7),
        )
        try:
            self._segment("validation", target, self.config["step_seconds"] + 2,
                          "diagonal", rows, scored=True)
            self._segment("validation", self._target(height=self.config["hover_height"]),
                          self.config["return_seconds"] + 2, "return", rows, scored=True)
        finally:
            csv_name = self._write_csv("validation", rows)
        tail = [row for row in rows if row["segment"] == "return"
                and row["t"] >= rows[-1]["t"] - 1.0]
        hold_rows = [row for row in rows if row["segment"] == "return"]
        window = self.config["oscillation_window_seconds"]
        oscillation = {
            "height": _oscillation_metrics(hold_rows, "height", "z",
                                           self.config["height_step"], window)[0],
            "yaw": _oscillation_metrics(hold_rows, "yaw", "yaw",
                                        math.radians(self.config["yaw_step_deg"]), window)[0],
            "velocity_x": _oscillation_metrics(hold_rows, "velocity", "x",
                                               self.config["velocity_step"], window)[0],
            "velocity_y": _oscillation_metrics(hold_rows, "velocity", "y",
                                               self.config["velocity_step"], window)[0],
            "position_x": _oscillation_metrics(hold_rows, "position", "x",
                                               self.config["position_step"], window)[0],
            "position_y": _oscillation_metrics(hold_rows, "position", "y",
                                               self.config["position_step"], window)[0],
        }
        limits = self.config["oscillation_tolerances"]
        stable = (
            oscillation["height"] <= limits["height"]
            and oscillation["yaw"] <= limits["yaw"]
            and all(oscillation[key] <= limits["velocity"]
                    for key in ("velocity_x", "velocity_y"))
            and all(oscillation[key] <= limits["position"]
                    for key in ("position_x", "position_y"))
        )
        self.stage_summary["validation"] = {
            "telemetry_csv": csv_name,
            "final_xy_error": mean(math.hypot(row["x"] - a.x, row["y"] - a.y)
                                   for row in tail),
            "final_height_error": mean(abs(row["z"] - self.config["hover_height"])
                                       for row in tail),
            "final_yaw_error": mean(abs(wrap_pi(row["yaw"] - a.yaw)) for row in tail),
            "oscillation_rms": oscillation,
            "stable": stable,
        }
        result = self.stage_summary["validation"]
        tolerances = self.config["validation_tolerances"]
        result["passed"] = (
            result["final_xy_error"] <= tolerances["xy_m"]
            and result["final_height_error"] <= tolerances["height_m"]
            and result["final_yaw_error"] <= tolerances["yaw_rad"]
            and stable
        )
        self._save_summary()
        return result["passed"]

    def _takeoff(self) -> None:
        self.last_sample = self.link.read_sample()
        self.takeoff_start_z = self.last_sample.z
        if self.height_waypoints_mode and not (self.yaw_waypoints_mode or
                                               self.xy_waypoints_mode):
            self.controller.z_bias = self.takeoff_start_z
        elif self.session_start_z is not None and (self.yaw_waypoints_mode or
                                                     self.xy_waypoints_mode):
            self.controller.z_bias = self.session_start_z
        self.controller.set_configs(self.best)
        self.controller.reset(self.last_sample)
        rows: list[dict[str, Any]] = []
        target = self._target(height=self.config["hover_height"] + 0.075,
                              height_relative=(self.height_waypoints_mode or
                                               self.xy_waypoints_mode))
        try:
            self._segment("height", target, self.config["takeoff_seconds"],
                          "takeoff", rows, allow_low=True)
        finally:
            self._write_csv("takeoff", rows)
        current = self.last_sample
        if current is not None:
            self.anchor = current
        self.stage_summary["takeoff"] = {
            "reached_height": current is not None
            and current.z >= self.config["hover_height"] - 0.15 +
            (self.controller.z_bias if (self.height_waypoints_mode or
                                        self.xy_waypoints_mode) else 0.0),
            "final_height": current.z if current is not None else None,
        }
        self._save_summary()

    def _land(self) -> bool:
        if self.last_sample is None or self.anchor is None:
            return False
        self.controller.set_configs(self.best)
        self.controller.reset(self.last_sample)
        rows: list[dict[str, Any]] = []
        started = time.monotonic()
        goal_raw = self.controller.z_bias
        steps: list[dict[str, Any]] = []
        try:
            steps = self._descend_height(
                goal_raw, "land", rows, relative=True, allow_low=True,
                total_seconds=self.config["landing_seconds"])
            remaining = self.config["landing_seconds"] - (time.monotonic() - started)
            if remaining > 0:
                hold_height = steps[-1]["command_z"] if steps else 0.0
                self._segment("height", self._target(height=hold_height,
                                                     height_relative=True),
                              min(2.0, remaining), "land_hold", rows, allow_low=True)
        finally:
            file_name = self._write_csv("landing", rows)
        final_height = self.last_sample.z if self.last_sample else math.inf
        relative_height = final_height - self.controller.z_bias
        step_metrics = _height_descent_metrics(rows)
        reached = (final_height <= goal_raw + self.config[
            "height_descent_reach_tolerance"] and
            (not steps or steps[-1]["reached"]))
        passed = (reached and relative_height <= self.config["landing_height_tolerance"]
                  and step_metrics["peak_descent_speed"] <=
                  self.config["height_descent_max_speed"]
                  and step_metrics["descent_oscillation_rms"] <=
                  self.config["oscillation_tolerances"]["height"])
        self.stage_summary["landing"] = {
            "telemetry_csv": file_name, "final_height": final_height,
            "height_above_launch": relative_height, "z_bias": self.controller.z_bias,
            "target_height_above_launch": 0.0, "target_raw_z": goal_raw,
            "descent_reached": reached, "steps": steps, **step_metrics,
            "passed": passed,
        }
        self._save_summary()
        return passed

    def _throttle_ramp_land(self, label: str) -> dict[str, Any]:
        """Phase-one landing: direct RC throttle, independent of trial PID gains."""
        c = self.config
        period = 1 / c["kinematics_hz"]
        begin = time.monotonic()
        previous_tick = begin
        throttle = float(c["throttle_landing_start_rc"])
        grounded_since: float | None = None
        rows: list[dict[str, Any]] = []
        brake_events = 0
        grounded = False
        try:
            while time.monotonic() - begin < c["landing_seconds"]:
                tick = time.monotonic()
                dt = max(0.0, tick - previous_tick)
                previous_tick = tick
                sample = self.link.read_sample()
                if self.link.stream_error is not None:
                    raise ConnectionError(f"RC stream failed: {self.link.stream_error}")
                self.last_sample = sample
                row = {"t": sample.t, "elapsed": tick - begin,
                       "x": sample.x, "y": sample.y, "z": sample.z,
                       "height_above_launch": sample.z - self.controller.z_bias,
                       "yaw": sample.yaw, "roll": sample.roll, "pitch": sample.pitch,
                        "throttle_rc": 0, "vertical_speed": "",
                       "grounded": False}
                rows.append(row)
                vz = vertical_rate(rows, c["throttle_landing_velocity_window"])
                if vz is not None:
                    row["vertical_speed"] = vz
                if vz is not None and vz < -c["throttle_landing_speed_limit"]:
                    # Restore thrust gradually when the sink rate is too high.
                    throttle = min(c["throttle_landing_start_rc"],
                                   throttle + 4 * c["throttle_landing_rc_per_second"] * dt)
                    brake_events += 1
                else:
                    throttle = max(c["throttle_landing_end_rc"],
                                   throttle - c["throttle_landing_rc_per_second"] * dt)
                row["throttle_rc"] = int(throttle)
                self.link.set_frame((1500, 1500, int(throttle), 1500,
                                     2000, 1000, 1300))
                at_ground = (vz is not None and
                             abs(vz) <= c["throttle_landing_ground_speed"] and
                             throttle <= c["throttle_landing_end_rc"] + 1)
                if at_ground:
                    if grounded_since is None:
                        grounded_since = sample.t
                    if sample.t - grounded_since >= c["throttle_landing_ground_hold_seconds"]:
                        row["grounded"] = True
                        grounded = True
                        break
                else:
                    grounded_since = None
                delay = tick + period - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
        finally:
            file_name = self._write_csv(label, rows)
        return {
            "grounded": grounded,
            "detected_ground_z": self.last_sample.z if grounded else None,
            "ground_offset_from_launch": (
                self.last_sample.z - self.controller.z_bias if grounded else None),
            "final_height_above_launch": (self.last_sample.z - self.controller.z_bias
                                          if self.last_sample is not None else None),
            "final_vertical_speed": (vertical_rate(rows, c["throttle_landing_velocity_window"])
                                     if rows else None),
            "final_throttle_rc": int(throttle),
            "brake_events": brake_events,
            "duration": time.monotonic() - begin,
            "telemetry_csv": file_name,
        }

    def _height_pid_flight(self, candidate: dict[str, dict[str, Any]],
                           phase: str, label: str,
                           score_start: float | None = None) -> dict[str, Any]:
        """Fly one fixed one-metre response from the current ground level."""
        c = self.config
        self._wait_for_ground_return("height")
        ground = self.last_sample
        if ground is None:
            raise TrialInvalid("No ground sample before height trial")
        self.takeoff_start_z = ground.z
        self.controller.z_bias = ground.z
        self.anchor = ground
        self.controller.base_throttle_rc = self.best_height_base_rc
        self.controller.set_configs(candidate)
        self.controller.reset(ground)
        self.link.arm()
        target_height = c["height_pid_target_m"]
        target_raw = ground.z + target_height
        target = self._target(height=target_height, height_relative=True,
                              yaw=ground.yaw)
        rows: list[dict[str, Any]] = []
        reached = False
        arrival: float | None = None
        liftoff_t: float | None = None
        airborne = False

        def observe(step_rows: list[dict[str, Any]]) -> bool:
            nonlocal reached, arrival, liftoff_t, airborne
            row = step_rows[-1]
            height_above_ground = row["z"] - ground.z
            if (liftoff_t is None and height_above_ground >=
                    c["height_liftoff_threshold_m"]):
                liftoff_t = row["t"]
            airborne |= row["z"] >= ground.z + c["safety"]["min_airborne_height"]
            if (row["segment_elapsed"] > 0.4 and
                    max(abs(row["roll"]), abs(row["pitch"])) >
                    math.radians(c["safety"]["max_tilt_deg"])):
                raise TrialInvalid("Drone tilted or overturned during height trial")
            if (airborne and row["z"] < ground.z + 0.12):
                raise TrialInvalid("Drone fell during height trial")
            if (liftoff_t is not None and arrival is None and
                    abs(row["z"] - target_raw) <=
                    c["height_p_arrival_tolerance_m"]):
                reached = True
                arrival = row["t"] - liftoff_t
                # P is scored only by liftoff-to-arrival time. Keep flying
                # until the normal trial boundary so the graph also shows
                # post-arrival behaviour before service landing begins.
                return phase != "p"
            return False

        try:
            self._segment("height", target, c["height_pid_trial_seconds"],
                          "height_response", rows, scored=True, allow_low=True,
                          stop_when=observe)
            if reached and phase != "p":
                self._segment("height", target, c["height_pid_hold_seconds"],
                              "height_hold", rows, scored=True, allow_low=True)
        except BaseException:
            self._write_csv(label, rows)
            self.link.set_frame(HOLD_FRAME)
            self.link.disarm()
            raise
        flight_csv = self._write_csv(label, rows)
        landing = self._throttle_ramp_land(f"{label}_throttle_landing")
        self.link.disarm()
        if not landing["grounded"]:
            raise LandingIncomplete(
                f"Throttle landing did not reach stable ground; {landing['telemetry_csv']}")

        start_t = (liftoff_t if liftoff_t is not None else
                   rows[0]["t"] if rows else 0.0)
        response = [(row["t"] - start_t, row["z"] - ground.z) for row in rows
                    if row["t"] >= start_t]
        peak = max((value for _, value in response), default=0.0)
        plateau = _plateau_after_arrival(
            response, arrival, c["height_d_band_m"],
            c["height_d_required_fraction"])
        i_start = (plateau["settling_time"] if score_start is None else score_start)
        i_values = [value for elapsed, value in response if elapsed >= i_start]
        target_hold = (mean(abs(value - target_height) <= c["height_i_band_m"]
                            for value in i_values) if i_values else 0.0)
        final_values = [value for elapsed, value in response
                        if elapsed >= max(0.0, response[-1][0] - 1.0)] if response else []
        return {
            "reached": reached, "arrival_time": arrival,
            "liftoff_detected": liftoff_t is not None,
            "peak_height": peak, "shortfall": max(0.0, target_height - peak),
            "final_error": (mean(abs(value - target_height) for value in final_values)
                            if final_values else target_height),
            "plateau": plateau, "target_hold_fraction": target_hold,
            "i_score_start_seconds": i_start, "flight_csv": flight_csv,
            "landing": landing,
        }

    def _evaluate_height_pid_candidate(
            self, candidate: dict[str, dict[str, Any]], phase: str, label: str,
            score_start: float | None = None) -> dict[str, Any]:
        """Average three ground-to-height flights; replay crashes indefinitely."""
        trials = []
        repeat = 0
        while repeat < self.config["vertical_repeats"]:
            try:
                trial = self._height_pid_flight(
                    candidate, phase, f"{label}_flight{repeat + 1}", score_start)
            except (InterventionDetected, TrialInvalid, LandingIncomplete) as exc:
                self.records.append({
                    "stage": "height", "pid": "pid_height", "label": label,
                    "attempt": repeat + 1, "valid": False, "reason": str(exc),
                    "telemetry_csv": self.last_csv,
                })
                self._save_summary()
                print(f"height/pid_height {label}: waiting for ground return "
                      f"after invalid flight ({exc})")
                self._wait_for_ground_return("height")
                continue
            trials.append(trial)
            repeat += 1
        arrivals = [trial["arrival_time"] for trial in trials if trial["reached"]]
        reached = len(arrivals)
        target_time = self.config["height_p_arrival_seconds"]
        mean_arrival = mean(arrivals) if arrivals else None
        if phase == "p":
            score = (mean(abs(value - target_time) for value in arrivals) /
                     target_time if arrivals else 0.0)
            score += 5.0 * (len(trials) - reached)
            score += mean(trial["shortfall"] for trial in trials) / \
                self.config["height_pid_target_m"]
        elif phase == "d":
            score = (5.0 * (len(trials) - reached) +
                     mean(trial["plateau"]["settling_time"] for trial in trials) /
                     (self.config["height_pid_trial_seconds"] +
                      self.config["height_pid_hold_seconds"]) +
                     mean(trial["plateau"]["oscillation_rms"]
                          for trial in trials) / self.config["height_d_band_m"])
        else:
            score = (5.0 * (len(trials) - reached) +
                     4.0 * (1.0 - mean(trial["target_hold_fraction"]
                                       for trial in trials)) +
                     mean(trial["final_error"] for trial in trials) /
                     self.config["height_i_band_m"])
        metrics = {
            "method": "height_repeated_pid_response", "phase": phase,
            "score": score, "repeat_count": len(trials), "trials": trials,
            "reached_trials": reached, "mean_arrival_seconds": mean_arrival,
            "plateau_settling_time": mean(
                trial["plateau"]["settling_time"] for trial in trials),
            "settled_trials": sum(trial["plateau"]["settled"] for trial in trials),
            "plateau_hold_fraction": mean(
                trial["plateau"]["hold_fraction"] for trial in trials),
            "oscillation_rms": mean(
                trial["plateau"]["oscillation_rms"] for trial in trials),
            "target_hold_fraction": mean(
                trial["target_hold_fraction"] for trial in trials),
            "terminal_mae": mean(trial["final_error"] for trial in trials),
            "i_score_start_seconds": score_start,
        }
        record = {
            "stage": "height", "pid": "pid_height", "axis": "z",
            "label": label, "valid": True,
            "gains": {key: candidate["pid_height"][key]
                      for key in ("kp", "ki", "kd")},
            "metrics": metrics,
            "telemetry_csv": trials[-1]["flight_csv"],
            "telemetry_csvs": [trial["flight_csv"] for trial in trials],
        }
        self.records.append(record)
        self._save_summary()
        if phase == "p":
            print(f"height/pid_height {label}: score={score:.3f}, "
                  f"reached={reached}/{len(trials)}, arrival="
                  f"{mean_arrival if mean_arrival is not None else float('nan'):.2f}s")
        elif phase == "d":
            print(f"height/pid_height {label}: score={score:.3f}, "
                  f"settled={metrics['settled_trials']}/{len(trials)}, "
                  f"settle={metrics['plateau_settling_time']:.2f}s, "
                  f"osc={metrics['oscillation_rms']:.4f}m")
        else:
            print(f"height/pid_height {label}: score={score:.3f}, "
                  f"hold={metrics['target_hold_fraction']:.0%}, "
                  f"MAE={metrics['terminal_mae']:.4f}m")
        return record

    def _yaw_rate_from_rows(self, rows: list[dict[str, Any]],
                            window: float = 0.25) -> float | None:
        if len(rows) < 2:
            return None
        latest = rows[-1]
        older = next((row for row in reversed(rows[:-1])
                      if latest["t"] - row["t"] >= window), None)
        if older is None:
            return None
        return wrap_pi(latest["yaw"] - older["yaw"]) / (latest["t"] - older["t"])

    def _brake_yaw_rotation(self, label: str,
                            rows: list[dict[str, Any]]) -> bool:
        """Open-loop yaw cleanup; the braking direction is captured once."""
        c = self.config
        neutral = self._target(height=c["hover_height"],
                               height_relative=True,
                               yaw=self.last_sample.yaw if self.last_sample else None)
        observe_start = len(rows)
        self._segment("yaw", neutral, c["yaw_brake_observe_seconds"],
                      f"{label}_observe", rows, scored=True,
                      rc_override=lambda sample, elapsed: {"rc_yaw": 1500})
        observed = rows[observe_start:]
        rate = self._yaw_rate_from_rows(observed)
        limit = math.radians(c["yaw_brake_rate_deg_s"])
        stopped = rate is not None and abs(rate) <= limit
        if not stopped and rate is not None:
            direction = 1 if rate > 0 else -1
            brake_rows: list[dict[str, Any]] = []

            def override(sample: Sample, elapsed: float) -> dict[str, Any]:
                nonlocal stopped
                offset = min(c["yaw_brake_max_offset"],
                             c["yaw_brake_pwm_per_second"] * elapsed)
                rc = int(clamp(1500 - direction * offset *
                               c["direction"]["yaw"], 1000, 2000))
                return {"rc_yaw": rc, "yaw_brake_pwm_offset": offset}

            def slow(recent: list[dict[str, Any]]) -> bool:
                nonlocal stopped
                local = recent[-max(2, int(c["kinematics_hz"] * 0.4)):]
                measured = self._yaw_rate_from_rows(local)
                stopped = measured is not None and abs(measured) <= limit
                return stopped

            first = len(rows)
            self._segment("yaw", neutral, c["yaw_brake_timeout_seconds"],
                          f"{label}_brake", rows, scored=True,
                          stop_when=slow, rc_override=override)
            brake_rows.extend(rows[first:])
        self._segment("yaw", neutral, c["yaw_brake_neutral_seconds"],
                      f"{label}_neutral", rows, scored=True,
                      rc_override=lambda sample, elapsed: {"rc_yaw": 1500})
        return stopped

    def _profile_yaw_pid_repeats(self, label: str) -> tuple[list[dict[str, Any]], str]:
        c = self.config
        rows: list[dict[str, Any]] = []
        magnitude = math.radians(c["yaw_pid_target_deg"])
        try:
            for cycle in range(1, c["vertical_repeats"] + 1):
                for direction, sign in (("positive", 1), ("negative", -1)):
                    if self._vertical_drone_is_down("yaw"):
                        raise TrialInvalid("Drone was below airborne height before yaw step")
                    self._brake_yaw_rotation(
                        f"cycle{cycle}_{direction}_service_brake", rows)
                    if self.last_sample is None:
                        raise TrialInvalid("No sample after yaw service brake")
                    origin = self.last_sample.yaw
                    requested = wrap_pi(origin + sign * magnitude)
                    self.controller.reset(self.last_sample)
                    target = self._target(height=c["hover_height"],
                                          height_relative=True, yaw=requested)
                    first = len(rows)
                    self._segment("yaw", target, c["yaw_pid_trial_seconds"],
                                  direction, rows, scored=True)
                    for row in rows[first:]:
                        row["yaw_cycle"] = cycle
                        row["yaw_origin"] = origin
                        row["yaw_target_offset"] = sign * magnitude
                        row["yaw_requested_target"] = requested
        finally:
            csv_name = self._write_csv(label, rows)
        return rows, csv_name

    def _score_yaw_pid_repeats(self, rows: list[dict[str, Any]], phase: str,
                               score_start: float | None = None) -> dict[str, Any]:
        c = self.config
        magnitude = math.radians(c["yaw_pid_target_deg"])
        arrival_band = math.radians(c["yaw_p_arrival_tolerance_deg"])
        plateau_band = math.radians(c["yaw_d_band_deg"])
        target_band = math.radians(c["yaw_i_band_deg"])
        trials = []
        for cycle in range(1, c["vertical_repeats"] + 1):
            for direction, sign in (("positive", 1), ("negative", -1)):
                part = [row for row in rows if row.get("yaw_cycle") == cycle and
                        row.get("segment") == direction]
                if not part:
                    raise TrialInvalid(f"No yaw telemetry for {direction} cycle {cycle}")
                origin = float(part[0]["yaw_origin"])
                target = float(part[0]["yaw_requested_target"])
                series = [(float(row["segment_elapsed"]),
                           sign * wrap_pi(float(row["yaw"]) - origin)) for row in part]
                arrival = next((elapsed for elapsed, value in series
                                if abs(value - magnitude) <= arrival_band), None)
                plateau = _plateau_after_arrival(
                    series, arrival, plateau_band, c["yaw_d_required_fraction"])
                start = plateau["settling_time"] if score_start is None else score_start
                scored = [row for row in part if row["segment_elapsed"] >= start]
                hold = (mean(abs(wrap_pi(float(row["yaw"]) - target)) <= target_band
                             for row in scored) if scored else 0.0)
                final = [abs(wrap_pi(float(row["yaw"]) - target)) for row in part
                         if row["segment_elapsed"] >=
                         max(0.0, c["yaw_pid_trial_seconds"] - 1.0)]
                trials.append({
                    "cycle": cycle, "direction": direction,
                    "reached": arrival is not None, "arrival_time": arrival,
                    "plateau": plateau, "target_hold_fraction": hold,
                    "final_error": mean(final) if final else math.pi,
                })
        by_direction = {}
        for direction in ("positive", "negative"):
            parts = [trial for trial in trials if trial["direction"] == direction]
            arrivals = [part["arrival_time"] for part in parts if part["reached"]]
            by_direction[direction] = {
                "reached_count": len(arrivals),
                "mean_arrival_seconds": mean(arrivals) if arrivals else None,
                "settled_count": sum(part["plateau"]["settled"] for part in parts),
                "settling_time": mean(part["plateau"]["settling_time"]
                                      for part in parts),
                "plateau_hold_fraction": mean(part["plateau"]["hold_fraction"]
                                               for part in parts),
                "oscillation_rms": mean(part["plateau"]["oscillation_rms"]
                                        for part in parts),
                "target_hold_fraction": mean(part["target_hold_fraction"]
                                             for part in parts),
                "terminal_mae": mean(part["final_error"] for part in parts),
            }
        if phase == "p":
            score = mean(
                (abs(part["mean_arrival_seconds"] - c["yaw_p_arrival_seconds"])
                 / c["yaw_p_arrival_seconds"] if part["mean_arrival_seconds"] is not None
                 else 5.0) + 5.0 * (c["vertical_repeats"] - part["reached_count"])
                for part in by_direction.values())
        elif phase == "d":
            score = mean(
                5.0 * (c["vertical_repeats"] - part["reached_count"]) +
                part["settling_time"] / c["yaw_pid_trial_seconds"] +
                part["oscillation_rms"] / plateau_band
                for part in by_direction.values())
        else:
            score = mean(
                5.0 * (c["vertical_repeats"] - part["reached_count"]) +
                4.0 * (1.0 - part["target_hold_fraction"]) +
                part["terminal_mae"] / target_band
                for part in by_direction.values())
        return {"method": "yaw_repeated_pid_response", "phase": phase,
                "score": score, "repeat_count": c["vertical_repeats"],
                "trials": trials, "mean_response_by_direction": by_direction,
                "i_score_start_seconds": score_start,
                "terminal_mae": mean(part["terminal_mae"]
                                     for part in by_direction.values()),
                "oscillation_rms": max(part["oscillation_rms"]
                                       for part in by_direction.values())}

    def _evaluate_yaw_pid_candidate(
            self, candidate: dict[str, dict[str, Any]], phase: str, label: str,
            score_start: float | None = None) -> dict[str, Any]:
        while True:
            if self._vertical_drone_is_down("yaw"):
                self._resume_vertical_after_ground("yaw")
            try:
                # Recover height with yaw disabled. The yaw reference for every
                # scored turn is deliberately captured after service braking.
                recovery: list[dict[str, Any]] = []
                self.controller.set_configs(self.best)
                self.controller.reset(self.last_sample)
                self._segment("height", self._target(
                    height=self.config["hover_height"], height_relative=True),
                    min(2.0, self.config["recovery_seconds"]),
                    "yaw_height_recovery", recovery, scored=True)
                self._write_csv(f"{label}_height_recovery", recovery)
                self.controller.set_configs(candidate)
                self.controller.reset(self.last_sample)
                rows, csv_name = self._profile_yaw_pid_repeats(label)
                metrics = self._score_yaw_pid_repeats(rows, phase, score_start)
                break
            except (InterventionDetected, TrialInvalid) as exc:
                self.records.append({
                    "stage": "yaw", "pid": "pid_yaw", "label": label,
                    "valid": False, "reason": str(exc),
                    "telemetry_csv": self.last_csv,
                })
                self._save_summary()
                print(f"yaw/pid_yaw {label}: waiting for restart ({exc})")
                self._resume_vertical_after_ground("yaw")
        record = {
            "stage": "yaw", "pid": "pid_yaw", "axis": "yaw",
            "label": label, "valid": True,
            "gains": {key: candidate["pid_yaw"][key]
                      for key in ("kp", "ki", "kd")},
            "metrics": metrics, "telemetry_csv": csv_name,
        }
        self.records.append(record)
        self._save_summary()
        parts = metrics["mean_response_by_direction"]
        if phase == "p":
            print(f"yaw/pid_yaw {label}: score={metrics['score']:.3f}, "
                  f"arrival +/-={parts['positive']['mean_arrival_seconds']}/"
                  f"{parts['negative']['mean_arrival_seconds']}s")
        elif phase == "d":
            print(f"yaw/pid_yaw {label}: score={metrics['score']:.3f}, "
                  f"settle +/-={parts['positive']['settling_time']:.2f}/"
                  f"{parts['negative']['settling_time']:.2f}s")
        else:
            print(f"yaw/pid_yaw {label}: score={metrics['score']:.3f}, "
                  f"hold +/-={parts['positive']['target_hold_fraction']:.0%}/"
                  f"{parts['negative']['target_hold_fraction']:.0%}")
        return record

    def _vertical_three_pass_search(
            self, stage: str, name: str, gain: str, phase: str, step: float,
            working: dict[str, dict[str, Any]], evaluate: Callable[..., dict[str, Any]],
            ready: Callable[[dict[str, Any], dict[str, Any]], bool],
            score_start: float | None = None, trial_limit: int | None = None
            ) -> tuple[dict[str, Any] | None, list[dict[str, Any]]]:
        """Forward, backward, forward search with one total trial budget."""
        c = self.config
        limit = (c["vertical_trials_per_coefficient"]
                 if trial_limit is None else trial_limit)
        coarse = max(1, math.ceil(limit * 0.6))
        reverse = max(1, math.ceil(limit * 0.2))
        budgets = (coarse, reverse, max(1, limit - coarse - reverse))
        passes = ((1, 1.0), (-1, c["vertical_search_reverse_fraction"]),
                  (1, c["vertical_search_final_fraction"]))
        history = []
        best = None
        best_value = 0.0
        total = 0
        for pass_number, ((direction, fraction), budget) in enumerate(zip(passes, budgets)):
            value = 0.0 if pass_number == 0 else best_value
            worse = 0
            for attempt in range(budget):
                if total >= limit:
                    break
                if pass_number or attempt:
                    value = round(value + direction * step * fraction, 10)
                if value < 0:
                    break
                label = f"{stage}_{name}_{gain}_pass{pass_number}_{attempt:02d}"
                result = evaluate(working, gain, value, phase, label, score_start)
                total += 1
                metrics = result["metrics"]
                history.append({"pass": pass_number, "gain": value,
                                "score": metrics["score"], "metrics": metrics,
                                "telemetry_csv": result["telemetry_csv"]})
                if best is None or metrics["score"] < best["metrics"]["score"]:
                    best, best_value, worse = result, value, 0
                elif metrics["score"] > best["metrics"]["score"]:
                    worse += 1
                if ready(metrics, c):
                    best, best_value = result, value
                    working[name][gain] = best_value
                    return best, history
                if worse >= c["vertical_search_worse_streak"]:
                    break
        if best is not None:
            working[name][gain] = best_value
        return best, history


    def _height_validation_repeated_pid(self) -> dict[str, Any]:
        """Validate the chosen height loop at the configured altitude sequence."""
        while True:
            try:
                self._wait_for_ground_return("height")
                self.height_waypoints_mode = True
                ground = self.last_sample
                if ground is None:
                    raise TrialInvalid("No ground sample before height validation")
                self.takeoff_start_z = ground.z
                self.controller.z_bias = ground.z
                self.anchor = ground
                self.controller.set_configs(self.best)
                self.controller.reset(ground)
                self.link.arm()
                rows, csv_name = self._profile_height_waypoints(
                    "height_pid_validation",
                    list(self.config["height_validation_targets_m"]),
                    allow_low=True)
                tilted = any(max(abs(row["roll"]), abs(row["pitch"])) >
                             math.radians(self.config["safety"]["max_tilt_deg"])
                             for row in rows)
                airborne = any(row["z"] >= ground.z +
                               self.config["safety"]["min_airborne_height"]
                               for row in rows)
                fell = airborne and any(
                    row["waypoint_index"] > 1 and row["z"] < ground.z + 0.12
                    for row in rows)
                if tilted or fell:
                    raise TrialInvalid("Drone fell or overturned during height validation")
                metrics = score_height_waypoints(
                    rows, self.config,
                    list(self.config["height_validation_targets_m"]))
                metrics["method"] = "height_waypoint_validation"
                metrics["phase"] = "validation"
                landing = self._throttle_ramp_land("height_pid_validation_landing")
                self.link.disarm()
                if not landing["grounded"]:
                    self._wait_for_ground_return("height")
                    landing = {**landing, "grounded": True,
                               "manual_ground_return": True}
                self.stage_summary["takeoff"] = {
                    "reached_height": bool(metrics["waypoints"][0]["reached"]),
                    "final_height": rows[-1]["z"] if rows else ground.z,
                }
                self.records.append({
                    "stage": "height", "pid": "pid_height", "axis": "z",
                    "label": "height_pid_validation", "valid": True,
                    "gains": {key: self.best["pid_height"][key]
                              for key in ("kp", "ki", "kd")},
                    "metrics": metrics, "telemetry_csv": csv_name,
                })
                self._save_summary()
                return {"metrics": metrics, "telemetry_csv": csv_name,
                        "landing": landing,
                        "targets": list(self.config["height_validation_targets_m"]),
                        "passed": _height_stage2_stable(metrics, self.config)}
            except (InterventionDetected, TrialInvalid, LandingIncomplete) as exc:
                self.records.append({"stage": "height", "pid": "pid_height",
                                     "label": "height_pid_validation",
                                     "valid": False, "reason": str(exc),
                                     "telemetry_csv": self.last_csv})
                self._save_summary()
                print(f"height validation: waiting for restart ({exc})")
                self._wait_for_ground_return("height")

    def _tune_height_pid_repeated(self) -> bool:
        c = self.config
        name = "pid_height"
        initial = {key: self.best[name][key] for key in ("kp", "ki", "kd")}
        working = copy.deepcopy(self.best)
        working[name].update(kp=0.0, ki=0.0, kd=0.0)
        searches: dict[str, list[dict[str, Any]]] = {}

        def evaluate(base: dict[str, dict[str, Any]], gain: str, value: float,
                     phase: str, label: str,
                     score_start: float | None = None) -> dict[str, Any]:
            candidate = copy.deepcopy(base)
            candidate[name][gain] = value
            return self._evaluate_height_pid_candidate(
                candidate, phase, label, score_start)

        p_best, p_history = self._vertical_three_pass_search(
            "height", name, "kp", "p", c["height_p_step"], working,
            evaluate, _height_repeated_p_ready,
            trial_limit=c["height_p_trials"])
        searches["kp"] = p_history
        if p_best is None:
            self.stage_summary[name] = {"stage": "height", "status": "unscored",
                                        "stable": False,
                                        "reason": "No valid repeated P trial"}
            self._save_summary()
            return False
        d_best, searches["kd"] = self._vertical_three_pass_search(
            "height", name, "kd", "d", c["height_d_step"], working,
            evaluate, _height_repeated_d_ready)
        d_reference = (d_best["metrics"]["plateau_settling_time"]
                       if d_best is not None else None)
        i_best, searches["ki"] = self._vertical_three_pass_search(
            "height", name, "ki", "i", c["height_i_step"], working,
            evaluate, _height_repeated_i_ready, d_reference)
        final_result = i_best or d_best or p_best
        self.best = copy.deepcopy(working)
        self.controller.set_configs(self.best)
        validation = self._height_validation_repeated_pid()
        p_ready = _height_repeated_p_ready(p_best["metrics"], c)
        d_ready = d_best is not None and _height_repeated_d_ready(
            d_best["metrics"], c)
        i_ready = i_best is not None and _height_repeated_i_ready(
            i_best["metrics"], c)
        stable = p_ready and d_ready and i_ready and validation["passed"]
        self.stage_summary[name] = {
            "stage": "height", "stable": stable,
            "baseline_score": p_history[0]["score"],
            "best_score": final_result["metrics"]["score"],
            "best_metrics": final_result["metrics"],
            "initial_gains": initial,
            "recommended_gains": {key: self.best[name][key]
                                  for key in ("kp", "ki", "kd")},
            "p_criteria_met": p_ready, "d_criteria_met": d_ready,
            "i_criteria_met": i_ready,
            "d_settling_reference_seconds": d_reference,
            "search_passes": searches, "validation": validation,
        }
        self.stage_summary["landing"] = {
            **validation["landing"], "passed": validation["landing"]["grounded"],
            "final_height": self.last_sample.z if self.last_sample else None,
            "height_above_launch": (self.last_sample.z - self.controller.z_bias
                                    if self.last_sample else None),
            "z_bias": self.controller.z_bias,
        }
        self._save_summary()
        return stable

    def _tune_yaw_pid_repeated(self) -> bool:
        c = self.config
        name = "pid_yaw"
        initial = {key: self.best[name][key] for key in ("kp", "ki", "kd")}
        working = copy.deepcopy(self.best)
        working[name].update(kp=0.0, ki=0.0, kd=0.0)
        searches: dict[str, list[dict[str, Any]]] = {}

        def evaluate(base: dict[str, dict[str, Any]], gain: str, value: float,
                     phase: str, label: str,
                     score_start: float | None = None) -> dict[str, Any]:
            candidate = copy.deepcopy(base)
            candidate[name][gain] = value
            return self._evaluate_yaw_pid_candidate(
                candidate, phase, label, score_start)

        p_best, p_history = self._vertical_three_pass_search(
            "yaw", name, "kp", "p", c["yaw_p_step"], working,
            evaluate, _yaw_repeated_p_ready)
        searches["kp"] = p_history
        if p_best is None:
            self.stage_summary[name] = {"stage": "yaw", "status": "unscored",
                                        "stable": False,
                                        "reason": "No valid repeated P trial"}
            self._save_summary()
            return False
        d_best, searches["kd"] = self._vertical_three_pass_search(
            "yaw", name, "kd", "d", c["yaw_d_step"], working,
            evaluate, _yaw_repeated_d_ready)
        d_reference = (mean(part["settling_time"] for part in
                            d_best["metrics"]["mean_response_by_direction"].values())
                       if d_best is not None else None)
        i_best, searches["ki"] = self._vertical_three_pass_search(
            "yaw", name, "ki", "i", c["yaw_i_step"], working,
            evaluate, _yaw_repeated_i_ready, d_reference)
        final_result = i_best or d_best or p_best
        self.best = copy.deepcopy(working)
        self.controller.set_configs(self.best)
        confirmation = self._evaluate_yaw_pid_candidate(
            self.best, "i", "yaw_pid_yaw_final_confirmation", d_reference)
        p_ready = _yaw_repeated_p_ready(p_best["metrics"], c)
        d_ready = d_best is not None and _yaw_repeated_d_ready(
            d_best["metrics"], c)
        i_ready = _yaw_repeated_i_ready(confirmation["metrics"], c)
        stable = p_ready and d_ready and i_ready
        self.stage_summary[name] = {
            "stage": "yaw", "stable": stable,
            "baseline_score": p_history[0]["score"],
            "best_score": final_result["metrics"]["score"],
            "best_metrics": confirmation["metrics"],
            "initial_gains": initial,
            "recommended_gains": {key: self.best[name][key]
                                  for key in ("kp", "ki", "kd")},
            "p_criteria_met": p_ready, "d_criteria_met": d_ready,
            "i_criteria_met": i_ready,
            "d_settling_reference_seconds": d_reference,
            "search_passes": searches,
            "confirmation_telemetry_csv": confirmation["telemetry_csv"],
        }
        self._save_summary()
        return stable

    def _ascent_targets(self) -> list[float]:
        c = self.config
        rng = random.Random(c["height_ascent_seed"])
        count = c["height_ascent_targets"]
        width = (c["height_ascent_max_m"] - c["height_ascent_min_m"]) / count
        targets = [c["height_ascent_min_m"] + (index + rng.random()) * width
                   for index in range(count)]
        rng.shuffle(targets)
        return targets

    def _ascent_speeds(self) -> list[float]:
        c = self.config
        rng = random.Random(c["height_ascent_seed"] ^ 0xA5A5A5A5)
        return [rng.uniform(c["height_ascent_speed_min"],
                            c["height_ascent_speed_max"])
                for _ in range(c["height_ascent_targets"])]

    def _ascent_trial(self, target_height: float, target_speed: float, label: str,
                       candidate: dict[str, dict[str, Any]],
                      base_throttle_rc: int) -> dict[str, Any]:
        c = self.config
        ground = self.link.read_sample()
        self.takeoff_start_z = ground.z
        self.controller.z_bias = ground.z
        self.last_sample = ground
        self.anchor = ground
        self.controller.base_throttle_rc = base_throttle_rc
        self.controller.set_configs(candidate)
        self.controller.reset(ground)
        deadline = (c["height_ascent_seconds_per_m"] * target_height +
                    c["height_ascent_time_reserve"])
        target_z = self.controller.z_bias + target_height
        target = self._target(height=max(0.0, ground.z - self.controller.z_bias),
                              height_relative=True)
        rows: list[dict[str, Any]] = []
        reached = False
        reach_time: float | None = None
        self.link.arm()
        def arrived(step_rows: list[dict[str, Any]]) -> bool:
            nonlocal reached, reach_time
            if len(step_rows) < 2:
                return False
            end = step_rows[-1]["t"]
            tail = [row for row in step_rows if
                    row["t"] >= end - c["height_ascent_arrival_hold_seconds"]]
            if (len(tail) < 2 or tail[-1]["t"] - tail[0]["t"] <
                    0.8 * c["height_ascent_arrival_hold_seconds"]):
                return False
            vz = (tail[-1]["z"] - tail[0]["z"]) / (tail[-1]["t"] - tail[0]["t"])
            reached = (all(abs(row["z"] - target_z) <=
                           c["height_ascent_arrival_tolerance"] for row in tail)
                       and abs(vz) <= c["height_ascent_arrival_speed"])
            if reached:
                reach_time = end - step_rows[0]["t"]
            return reached

        flight_error = None
        try:
            self._segment(
                "height", target, deadline, "ascent", rows,
                scored=True, allow_low=True, stop_when=arrived,
                target_update=lambda sample, elapsed: target.__setitem__(
                    "height", min(target_height, max(0.0, ground.z -
                    self.controller.z_bias) + target_speed * max(0.0, elapsed))))
            target["height"] = target_height
            self._segment("height", target, c["height_ascent_hold_seconds"],
                          "ascent_hold", rows, scored=True, allow_low=True)
        except Exception as exc:
            flight_error = exc
        finally:
            flight_csv = self._write_csv(label, rows)
        landing = self._throttle_ramp_land(f"{label}_throttle_landing")
        if not landing["grounded"]:
            self.stage_summary["height_ascent_landing_failure"] = landing
            self._save_summary()
            raise LandingIncomplete(
                f"Throttle landing did not reach stable ground; {landing['telemetry_csv']}")
        self.stage_summary.pop("height_ascent_landing_failure", None)
        self.link.disarm()
        if flight_error is not None:
            raise flight_error
        hold = [row for row in rows if row["segment"] == "ascent_hold"]
        tail = [row for row in hold if row["t"] >= hold[-1]["t"] - 1.5]
        final_error = mean(abs(row["z"] - target_z) for row in tail)
        bias = mean(row["z"] - target_z for row in tail)
        oscillation, oscillation_range, _ = _oscillation_metrics(
            hold, "height", "z", c["height_ascent_arrival_tolerance"],
            min(2.5, c["height_ascent_hold_seconds"]))
        ascent_rows = [row for row in rows if row["segment"] == "ascent"]
        liftoff = next((row for row in ascent_rows if
                        row["z"] - ground.z >= c["height_ascent_liftoff_height"]), None)
        liftoff_delay = (liftoff["segment_elapsed"] if liftoff is not None else deadline)
        initial_ascent = [row for row in ascent_rows if row["segment_elapsed"] <= 2.0]
        ceiling_rc = min(1800, base_throttle_rc + c["max_height_rc_offset"])
        near_ceiling_fraction = (sum(row["rc_throttle"] >= ceiling_rc - 10
                                     for row in initial_ascent) / len(initial_ascent)
                                 if initial_ascent else 0.0)
        speeds = []
        speed_errors = []
        for index, row in enumerate(rows):
            vz = vertical_rate(rows[:index + 1], 0.2)
            if vz is not None:
                speeds.append(vz)
                if (row["segment"] == "ascent" and
                        row["z"] - ground.z >= c["height_ascent_liftoff_height"] and
                        row["target_z"] <= target_z - 0.1):
                    speed_errors.append(abs(vz - target_speed))
        peak_climb = max(speeds, default=0.0)
        speed_tracking_mae = (mean(speed_errors) if speed_errors else target_speed)
        overshoot = max((row["z"] - target_z for row in rows), default=0.0)
        settled_after = c["height_ascent_hold_seconds"]
        for row in hold:
            window = [item for item in hold if
                      row["t"] - 1.0 <= item["t"] <= row["t"]]
            if (len(window) >= 5 and window[-1]["t"] - window[0]["t"] >= 0.8
                    and max(item["z"] for item in window) -
                    min(item["z"] for item in window) <=
                    2 * c["oscillation_tolerances"]["height"] and
                     mean(abs(item["z"] - target_z) for item in window) <=
                    c["height_ascent_arrival_tolerance"]):
                settled_after = row["t"] - hold[0]["t"]
                break
        stable = (reached and final_error <= c["height_ascent_arrival_tolerance"]
                  and oscillation <= c["oscillation_tolerances"]["height"]
                  and peak_climb <= c["height_ascent_max_speed"])
        score = (
            4 * final_error / c["height_ascent_arrival_tolerance"]
            + 2 * oscillation / c["oscillation_tolerances"]["height"]
            + 2 * max(0.0, overshoot) / c["height_ascent_arrival_tolerance"]
            + (reach_time if reach_time is not None else deadline) / deadline
            + settled_after / c["height_ascent_hold_seconds"]
            + 2 * max(0.0, peak_climb / c["height_ascent_max_speed"] - 1.0)
            + speed_tracking_mae / target_speed
            + 0.5 * liftoff_delay / deadline
            + (0.0 if reached else 10.0 +
               max(0.0, target_z - max(row["z"] for row in rows)) /
               c["height_ascent_arrival_tolerance"])
        )
        return {
            "target_height": target_height, "target_speed": target_speed,
            "target_raw_z": target_z,
            "ground_raw_z": ground.z,
            "start_height": 0.0,
            "reached": reached, "reach_time": reach_time,
            "deadline": deadline, "final_error": final_error,
            "final_bias": bias, "peak_climb_speed": peak_climb,
            "speed_tracking_mae": speed_tracking_mae,
            "liftoff_delay": liftoff_delay,
            "initial_near_ceiling_fraction": near_ceiling_fraction,
            "max_overshoot": max(0.0, overshoot),
            "oscillation_rms": oscillation,
            "oscillation_peak_to_peak": oscillation_range,
            "settled_after_seconds": settled_after,
            "stable": stable, "score": score,
            "flight_csv": flight_csv, "landing": landing,
        }

    def _evaluate_ascent_candidate(self, candidate: dict[str, dict[str, Any]],
                                   base_rc: int, label: str,
                                   plans: list[tuple[float, float]]) -> dict[str, Any]:
        trials = []
        for index, (target, speed) in enumerate(plans):
            attempt = 0
            while True:
                self.status = "height_ascent"
                self._save_summary()
                try:
                    trial = self._ascent_trial(
                        target, speed, f"{label}_{index + 1:02d}_try{attempt}",
                        candidate, base_rc)
                    break
                except (InterventionDetected, LandingIncomplete) as exc:
                    self.records.append({
                        "stage": "height_ascent", "label": label,
                        "valid": False, "target_height": target,
                        "reason": str(exc), "attempt": attempt + 1,
                        "telemetry_csv": self.last_csv,
                    })
                    self._save_summary()
                    print(f"height_ascent {label}: ignored incomplete trial; "
                          "waiting for ground return")
                    self._wait_for_ground_return("height_ascent")
                    attempt += 1
            trials.append(trial)
            print(f"height_ascent {label} {target:.2f} m @ {speed:.2f} m/s: "
                  f"score={trial['score']:.2f}, reached={trial['reached']}, "
                  f"bias={trial['final_bias']:.3f} m, "
                  f"osc={trial['oscillation_rms']:.3f} m, "
                  f"landed={trial['landing']['grounded']}")
        record = {
            "stage": "height_ascent", "label": label, "valid": True,
            "gains": {key: candidate["pid_height"][key] for key in ("kp", "ki", "kd")},
            "base_throttle_rc": base_rc,
            "targets": [{"height": height, "speed": speed}
                        for height, speed in plans],
            "trials": trials, "stable": all(item["stable"] for item in trials),
            "score": mean(item["score"] for item in trials),
        }
        self.records.append(record)
        self._save_summary()
        return record

    def _run_height_ascent_stage(self) -> bool:
        """Phase one: independent ascent/hold trials, each ending on the ground."""
        plans = list(zip(self._ascent_targets(), self._ascent_speeds()))
        best = self._evaluate_ascent_candidate(
            self.best, self.best_height_base_rc, "ascent_baseline", plans)
        baseline_score = best["score"]
        evaluated = {
            (self.best_height_base_rc, *(self.best["pid_height"][key]
                                         for key in ("kp", "ki", "kd", "i_limit")))
        }

        def evaluate(candidate: dict[str, dict[str, Any]], base: int,
                     label: str) -> dict[str, Any] | None:
            signature = (base, *(candidate["pid_height"][key]
                                 for key in ("kp", "ki", "kd", "i_limit")))
            if signature in evaluated:
                return None
            evaluated.add(signature)
            return self._evaluate_ascent_candidate(candidate, base, label, plans)

        def preferred(result: dict[str, Any]) -> bool:
            return ((result["stable"] and not best["stable"]) or
                    (result["stable"] == best["stable"] and
                     result["score"] < best["score"] *
                     (1 - self.config["min_improvement"])))

        initial_base_rc = self.best_height_base_rc
        for delta in (-20, 20):
            base = initial_base_rc + delta
            if 1200 <= base <= 1700:
                result = evaluate(self.best, base, f"ascent_base_{base}")
                if result is not None and preferred(result):
                    best, self.best_height_base_rc = result, base
        for round_number in range(self.config["height_ascent_search_rounds"]):
            for gain in ("kp", "kd"):
                anchor = copy.deepcopy(self.best)
                for index, factor in enumerate(self.config["gain_factors"]):
                    candidate = copy.deepcopy(anchor)
                    candidate["pid_height"][gain] *= factor ** (1 / (round_number + 1))
                    result = evaluate(candidate, self.best_height_base_rc,
                                      f"ascent_r{round_number}_{gain}_{index}")
                    if result is not None and preferred(result):
                        self.best, best = candidate, result
        for round_number in range(self.config["height_ascent_extra_rounds"]):
            if best["stable"]:
                break
            improved = False
            for gain, factors in (("kp", (0.65, 0.8)),
                                  ("kd", (0.8, 1.25))):
                anchor = copy.deepcopy(self.best)
                for index, factor in enumerate(factors):
                    candidate = copy.deepcopy(anchor)
                    candidate["pid_height"][gain] *= factor
                    result = evaluate(candidate, self.best_height_base_rc,
                                      f"ascent_extra{round_number}_{gain}_{index}")
                    if result is not None and preferred(result):
                        self.best, best = candidate, result
                        improved = True
            if not improved:
                break
        refined_base_rc = self.best_height_base_rc
        for delta in (-20, 20):
            base = refined_base_rc + delta
            if 1200 <= base <= 1700:
                result = evaluate(self.best, base, f"ascent_refine_base_{base}")
                if result is not None and preferred(result):
                    best, self.best_height_base_rc = result, base
        if self.best["pid_height"].get("i_limit") is not None:
            bias = mean(abs(trial["final_bias"]) for trial in best["trials"])
            controlled = all(
                trial["oscillation_rms"] <=
                self.config["oscillation_tolerances"]["height"] * 1.5 and
                trial["peak_climb_speed"] <=
                self.config["height_ascent_max_speed"] * 1.2
                for trial in best["trials"])
            integral_trials = ([
                {"ki": self.seed["pid_height"]["kp"] * factor}
                for factor in self.config["auto_height_ki_factors"]
            ] if controlled and bias > self.config["height_ascent_arrival_tolerance"]
               else [])
            integral_trials.extend(self.config["integral_trials"].get("pid_height", []))
            for index, trial in enumerate(integral_trials):
                candidate = copy.deepcopy(self.best)
                candidate["pid_height"]["ki"] = trial["ki"]
                if "i_limit" in trial:
                    candidate["pid_height"]["i_limit"] = trial["i_limit"]
                result = evaluate(candidate, self.best_height_base_rc,
                                  f"ascent_ki_{index}")
                if result is not None and preferred(result):
                    self.best, best = candidate, result
        self.controller.base_throttle_rc = self.best_height_base_rc
        # The last evaluated candidate is not necessarily the selected one.
        # Restore the lowest-score candidate before the continuous height phase.
        self.controller.set_configs(self.best)
        self.stage_summary["height_ascent"] = {
            "targets": [{"height": height, "speed": speed}
                        for height, speed in plans],
            "baseline_score": baseline_score,
            "best_score": best["score"], "stable": best["stable"],
            "recommended_gains": best["gains"],
            "recommended_height_base_throttle_rc": self.best_height_base_rc,
            "score_version": 2,
            "best_trials": best["trials"],
        }
        self._save_summary()
        return best["stable"]

    def _prepare_height_ascent(self) -> None:
        self.run_dir.mkdir(parents=True, exist_ok=False)
        self.status = "connecting"
        self._save_summary()
        self.link.connect()
        initial = self.link.read_sample()
        self.initial_sample = self.anchor = self.last_sample = initial
        self.launch_xy = (initial.x, initial.y)
        self.xy_reference_yaw = initial.yaw
        if self.session_start_z is None:
            self.session_start_z = initial.z
        self.controller.z_bias = self.session_start_z

    def _resume_summary(self, run_path: Path) -> dict[str, Any]:
        summary_path = run_path / "summary.json" if run_path.is_dir() else run_path
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
        if summary["config"]["drone_name"] != self.config["drone_name"]:
            raise ValueError("Resume run uses a different drone_name")
        for key in ("pid_source_sha256", "preset_source_sha256"):
            current = (hashlib.sha256(
                (PACKAGE_ROOT / ("pid.py" if key == "pid_source_sha256"
                         else "utils/drone_setups.py")).read_bytes()
            ).hexdigest())
            if summary.get(key) != current:
                raise ValueError(f"Resume run has a different {key}")
        return summary

    def _resume_pid(self, summary: dict[str, Any], name: str) -> None:
        source = summary["recommended_pids"][name]
        for key in ("kp", "ki", "kd"):
            value = source[key]
            if not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
                raise ValueError(f"Invalid resumed {name}.{key}")
            self.best[name][key] = float(value)
        i_limit = source.get("i_limit")
        if i_limit is not None:
            if (not isinstance(i_limit, (int, float)) or
                    not math.isfinite(i_limit) or i_limit <= 0):
                raise ValueError(f"Invalid resumed {name}.i_limit")
            self.best[name]["i_limit"] = float(i_limit)

    def resume_height_from(self, run_path: Path) -> None:
        summary = self._resume_summary(run_path)
        self._resume_pid(summary, "pid_height")
        base = summary["recommended_height_base_throttle_rc"]
        if not isinstance(base, int) or not 1200 <= base <= 1700:
            raise ValueError("Invalid resumed height base throttle")
        self.best_height_base_rc = base
        self.controller.base_throttle_rc = base
        self.controller.set_configs(self.best)
        self.resume_sources["height"] = str(run_path.resolve())

    def resume_yaw_from(self, run_path: Path) -> None:
        summary = self._resume_summary(run_path)
        self._resume_pid(summary, "pid_yaw")
        self.controller.set_configs(self.best)
        self.resume_sources["yaw"] = str(run_path.resolve())

    def resume_acceleration_from(self, run_path: Path) -> None:
        summary = self._resume_summary(run_path)
        for name in ("pid_accel_pitch", "pid_accel_roll"):
            self._resume_pid(summary, name)
            self.best[name]["i_limit"] = None
        self.controller.set_configs(self.best)
        self.resume_sources["acceleration"] = str(run_path.resolve())

    def resume_acceleration_pitch_from(self, run_path: Path) -> None:
        """Continue with the best completed I trial from an interrupted X run."""
        summary = self._resume_summary(run_path)
        trials = [record for record in summary.get("evaluations", [])
                  if record.get("valid") and record.get("stage") == "acceleration"
                  and record.get("pid") == "pid_accel_pitch"
                  and (record.get("metrics") or {}).get("phase") == "i"
                  and (record.get("metrics") or {}).get("reached_directions") == 2]
        if not trials:
            raise ValueError("Resume run has no valid pitch I trial")
        qualified = [record for record in trials
                     if record["metrics"]["hold_fraction"] >
                     self.config["acceleration_i_success_hold_fraction"]
                     and len(record["metrics"].get(
                         "i_filtered_response_by_direction", {})) == 2
                     and all(part["settled"] for part in record["metrics"]
                                 ["i_filtered_response_by_direction"].values())]
        if not qualified:
            raise ValueError("Resume run has no pitch I trial with successful hold")
        chosen = min(qualified, key=lambda record: record["metrics"]["score"])
        for key in ("kp", "ki", "kd"):
            value = chosen["gains"][key]
            if not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
                raise ValueError(f"Invalid resumed pitch {key}")
            self.best["pid_accel_pitch"][key] = float(value)
        self.best["pid_accel_pitch"]["i_limit"] = None
        self.controller.set_configs(self.best)
        self.stage_summary["pid_accel_pitch"] = {
            "stage": "acceleration", "stable": _acceleration_response_stable(
                chosen["metrics"], self.config),
            "baseline_score": chosen["metrics"]["score"],
            "best_score": chosen["metrics"]["score"],
            "best_metrics": chosen["metrics"],
            "recommended_gains": {key: self.best["pid_accel_pitch"][key]
                                  for key in ("kp", "ki", "kd")},
            "resumed_trial": chosen["label"],
            "resumed_telemetry_csv": chosen.get("telemetry_csv"),
        }
        self.resume_sources["acceleration_pitch"] = str(run_path.resolve())
        print(f"Acceleration pitch resumed from {chosen['label']}: "
              f"score={chosen['metrics']['score']:.3f}, "
              f"hold={chosen['metrics']['hold_fraction']:.0%}")

    def resume_velocity_from(self, run_path: Path) -> None:
        summary = self._resume_summary(run_path)
        for name in ("pid_vel_pitch", "pid_vel_roll"):
            self._resume_pid(summary, name)
        self.controller.set_configs(self.best)
        self.resume_sources["velocity"] = str(run_path.resolve())

    def run_ascent_only(self) -> Path:
        """Tune ascent/hold from ground, landing between trials by RC ramp."""
        try:
            self._prepare_height_ascent()
            stable = self._run_height_ascent_stage()
            self.status = "complete" if stable else "needs_review"
        except (LandingIncomplete, InterventionDetected) as exc:
            self.failure = str(exc)
            self.status = "needs_review"
        except (KeyboardInterrupt, Exception) as exc:
            self.failure = f"{type(exc).__name__}: {exc}"
            self.status = "aborted"
            print(f"Ascent calibration stopped: {self.failure}", file=sys.stderr)
        finally:
            self.link.close()
            self._save_summary()
        return self.run_dir

    def _run_height_continuous_stage(self) -> None:
        self.height_waypoints_mode = True
        if self.initial_sample is not None:
            self.controller.z_bias = self.initial_sample.z
        self.controller.base_throttle_rc = self.best_height_base_rc
        self.status = "height_continuous_takeoff"
        self._save_summary()
        self.link.arm()
        self._takeoff()
        if not self.stage_summary.get("takeoff", {}).get("reached_height", True):
            self._resume_vertical_after_ground("height")
        self.status = "height_continuous"
        self._save_summary()
        tuned = self._tune_pid("height", "pid_height", "z")
        self.status = "landing"
        self._save_summary()
        landing = self._throttle_ramp_land("height_continuous_throttle_landing")
        self.stage_summary["landing"] = {
            **landing,
            "passed": landing["grounded"],
            "final_height": (self.last_sample.z if self.last_sample else None),
            "height_above_launch": landing["final_height_above_launch"],
            "z_bias": self.controller.z_bias,
        }
        if landing["grounded"]:
            self.link.disarm()
        else:
            self._wait_for_ground_return("height")
            self.stage_summary["landing"]["ground_return_confirmed"] = True
        self.status = ("complete" if tuned and landing["grounded"] and
                       self.stage_summary["pid_height"].get("stable", False)
                       else "needs_review")

    def run_height_two_stage(self, *, continuous_only: bool = False) -> Path:
        """Run continuous height tuning, optionally after ascent calibration."""
        try:
            self._prepare_height_ascent()
            if self.config["vertical_repeated_mode"] and not continuous_only:
                self.height_waypoints_mode = True
                self.status = "height_pid"
                self._save_summary()
                stable = self._tune_height_pid_repeated()
                self.status = "complete" if stable else "needs_review"
                return self.run_dir
            ascent_stable = True
            if not continuous_only:
                ascent_stable = self._run_height_ascent_stage()
                if not ascent_stable:
                    print("Height ascent did not meet every stability threshold; "
                          "continuing with the lowest-score candidate.")
            self._run_height_continuous_stage()
            if not ascent_stable and self.status == "complete":
                self.status = "needs_review"
        except (LandingIncomplete, InterventionDetected) as exc:
            self.failure = str(exc)
            self.status = "needs_review"
        except (KeyboardInterrupt, Exception) as exc:
            self.failure = f"{type(exc).__name__}: {exc}"
            self.status = "aborted"
            print(f"Height calibration stopped: {self.failure}", file=sys.stderr)
        finally:
            self.link.close()
            self._save_summary()
        return self.run_dir

    def run_yaw_only(self) -> Path:
        """Tune yaw using a previously calibrated height loop in one flight."""
        try:
            self._prepare_height_ascent()
            self.height_waypoints_mode = True
            self.yaw_waypoints_mode = True
            self.controller.base_throttle_rc = self.best_height_base_rc
            self.status = "yaw_takeoff"
            self._save_summary()
            self.link.arm()
            self._takeoff()
            if not self.stage_summary.get("takeoff", {}).get("reached_height", True):
                self._resume_vertical_after_ground("yaw")
            self.status = "yaw"
            self._save_summary()
            tuned = (self._tune_yaw_pid_repeated()
                     if self.config["vertical_repeated_mode"] else
                     self._tune_pid("yaw", "pid_yaw", "yaw"))
            self.status = "landing"
            self._save_summary()
            landing = self._throttle_ramp_land("yaw_throttle_landing")
            self.stage_summary["landing"] = {
                **landing,
                "passed": landing["grounded"],
                "final_height": (self.last_sample.z if self.last_sample else None),
                "height_above_launch": landing["final_height_above_launch"],
                "z_bias": self.controller.z_bias,
            }
            if landing["grounded"]:
                self.link.disarm()
            else:
                self._wait_for_ground_return("yaw")
                self.stage_summary["landing"]["ground_return_confirmed"] = True
            self.status = ("complete" if tuned and landing["grounded"] and
                           self.stage_summary["pid_yaw"].get("stable", False)
                           else "needs_review")
        except (LandingIncomplete, InterventionDetected) as exc:
            self.failure = str(exc)
            self.status = "needs_review"
        except (KeyboardInterrupt, Exception) as exc:
            self.failure = f"{type(exc).__name__}: {exc}"
            self.status = "aborted"
            print(f"Yaw calibration stopped: {self.failure}", file=sys.stderr)
        finally:
            self.link.close()
            self._save_summary()
        return self.run_dir

    def _transfer_x_gains_to_y(self, stage: str, source_name: str,
                               target_name: str) -> bool:
        """Try X gains on Y; reuse the same flight as Y's tuning baseline."""
        original_target = copy.deepcopy(self.best[target_name])

        def tune_velocity_from_zero() -> bool:
            self.best[target_name] = copy.deepcopy(original_target)
            self.best[target_name].update(kp=0.0, ki=0.0, kd=0.0)
            self.controller.set_configs(self.best)
            return self._tune_velocity_pid(
                target_name, "y", initial_source=(
                    "zero_gains_start" if self.config["velocity_repeated_mode"]
                    else "small_p_start"))

        def tune_position_from_zero() -> bool:
            self.best[target_name] = copy.deepcopy(original_target)
            self.best[target_name].update(kp=0.0, ki=0.0, kd=0.0)
            self.controller.set_configs(self.best)
            return self._tune_position_pid(
                target_name, "y", initial_source="zero_gains_start")

        candidate = copy.deepcopy(self.best)
        for key in ("kp", "ki", "kd", "i_limit"):
            if key in candidate[source_name] and key in candidate[target_name]:
                candidate[target_name][key] = candidate[source_name][key]
        self.best = candidate
        label = f"{stage}_{target_name}_transfer_from_{source_name}"
        velocity_reference = None
        p_result = None
        d_result = None
        if stage == "position" and self.config["position_repeated_mode"]:
            pure_p = copy.deepcopy(self.best)
            pure_p[target_name]["kp"] /= self.config["position_p_after_d_factor"]
            pure_p[target_name]["kd"] = 0.0
            pure_p[target_name]["ki"] = 0.0
            p_result = self._evaluate(
                stage, target_name, "y", pure_p, f"{label}_p",
                position_phase="p")
            if p_result is None or not _position_p_ready(
                    p_result["metrics"], self.config):
                return tune_position_from_zero()
            d_candidate = copy.deepcopy(self.best)
            d_candidate[target_name]["ki"] = 0.0
            d_result = self._evaluate(
                stage, target_name, "y", d_candidate, f"{label}_d",
                position_phase="d")
            if d_result is None or not _position_d_ready(
                    d_result["metrics"], self.config):
                return tune_position_from_zero()
            reference = mean(step["settling_time"] for step in
                             d_result["metrics"]["mean_steps"])
            result = self._evaluate(
                stage, target_name, "y", self.best, f"{label}_i",
                position_phase="i", position_i_score_start=reference)
            if result is None or not _position_i_ready(
                    result["metrics"], self.config):
                return tune_position_from_zero()
            velocity_reference = reference
        elif stage == "velocity" and self.config["velocity_repeated_mode"]:
            pure_p = copy.deepcopy(self.best)
            pure_p[target_name]["kp"] /= self.config["velocity_p_after_d_factor"]
            pure_p[target_name]["kd"] = 0.0
            pure_p[target_name]["ki"] = 0.0
            p_result = self._evaluate(
                stage, target_name, "y", pure_p, f"{label}_p",
                velocity_phase="p")
            if p_result is None or not _velocity_repeated_p_ready(
                    p_result["metrics"], self.config):
                return tune_velocity_from_zero()
            # The transferred controller needs its own D settling reference on Y.
            d_result = self._evaluate(
                stage, target_name, "y", self.best, f"{label}_d",
                velocity_phase="d")
            if d_result is None or not _velocity_repeated_d_ready(
                    d_result["metrics"], self.config):
                return tune_velocity_from_zero()
            velocity_reference = mean(
                part["settling_time"] for part in d_result["metrics"]["mean_steps"])
            result = self._evaluate(
                stage, target_name, "y", self.best, f"{label}_i",
                velocity_phase="i", velocity_i_score_start=velocity_reference)
        else:
            result = self._evaluate(stage, target_name, "y", self.best, label)
        if result is None:
            if stage == "velocity" and self.config["velocity_repeated_mode"]:
                # I is diagnostic; unavailable I telemetry does not undo P/D.
                assert d_result is not None
                result = d_result
            elif stage == "position" and self.config["position_repeated_mode"]:
                return tune_position_from_zero()
            elif stage == "velocity":
                return tune_velocity_from_zero()
            elif stage == "acceleration":
                return self._tune_acceleration_pid(
                    target_name, "y", initial_source=f"{source_name}_gains")
            else:
                return self._tune_pid(stage, target_name, "y",
                                      initial_source=f"{source_name}_gains")
        metrics = result["metrics"]
        if ((stage == "position" and self.config["position_repeated_mode"]) or
                (stage == "velocity" and self.config["velocity_repeated_mode"])):
            stable = True  # P and D were checked above; position I as well.
        elif stage == "velocity":
            stable = _velocity_response_stable(
                metrics, self.config["velocity_step"], self.config)
        elif stage == "acceleration":
            stable = _acceleration_response_stable(metrics, self.config)
        else:
            stable = _position_waypoints_stable(metrics, self.config)
        if stable:
            gains = {key: self.best[target_name][key]
                     for key in ("kp", "ki", "kd")}
            self.stage_summary[target_name] = {
                "stage": stage,
                "baseline_score": metrics["score"],
                "best_score": metrics["score"],
                "best_metrics": metrics,
                "oscillation_limit": (self.config["position_d_band_m"]
                                      if stage == "position" and
                                      self.config["position_repeated_mode"] else
                                      self.config["oscillation_tolerances"][stage]),
                "stable": True,
                "initial_source": f"{source_name}_gains",
                "initial_gains": gains,
                "recommended_gains": gains,
                "transferred_from": source_name,
                "transfer_accepted": True,
            }
            if velocity_reference is not None:
                assert d_result is not None and p_result is not None
                self.stage_summary[target_name].update({
                    "p_criteria_met": True,
                    "p_reference_trial": p_result["label"],
                    "d_settling_reference_seconds": velocity_reference,
                    "d_reference_trial": d_result["label"],
                    "d_reference_telemetry_csv": d_result["telemetry_csv"],
                    "d_reference_steps": [
                        {key: part[key] for key in
                         (("direction", "settled", "settling_time")
                          if stage == "position" else
                          ("direction", "cycle", "settled", "settling_time"))}
                        for part in d_result["metrics"]["mean_steps"]],
                    "d_criteria_met": True,
                    "i_criteria_met": _velocity_repeated_stable(metrics,
                                                                 self.config)
                    if stage == "velocity" else _position_i_ready(metrics,
                                                                   self.config),
                })
                if stage == "position":
                    self.stage_summary[target_name].update({
                        "d_needed": gains["kd"] > 0,
                        "i_needed": gains["ki"] > 0,
                    })
            self._save_summary()
            print(f"{stage}/{target_name}: X gains accepted for Y; "
                  "separate Y search skipped")
            return True
        if stage == "velocity":
            return tune_velocity_from_zero()
        if stage == "position" and self.config["position_repeated_mode"]:
            return tune_position_from_zero()
        if stage == "acceleration":
            return self._tune_acceleration_pid(
                target_name, "y", initial_source=f"{source_name}_gains")
        return self._tune_pid(stage, target_name, "y", baseline=result,
                              initial_source=f"{source_name}_gains")

    def run_xy_only(self, stage: str, *, skip_x: bool = False) -> Path:
        """Tune X, then Y, then a short joint probe using loaded upstream PIDs."""
        if stage not in ("acceleration", "velocity", "position"):
            raise ValueError(f"Unknown XY stage {stage}")
        if skip_x and stage != "acceleration":
            raise ValueError("Skipping X is only available for acceleration")
        names = (("pid_accel_pitch", "pid_accel_roll") if stage == "acceleration" else
                 ("pid_vel_pitch", "pid_vel_roll") if stage == "velocity" else
                 ("pid_pos_x", "pid_pos_y"))
        try:
            self._prepare_height_ascent()
            self.xy_waypoints_mode = True
            self.controller.base_throttle_rc = self.best_height_base_rc
            self.status = f"{stage}_takeoff"
            self._save_summary()
            self.link.arm()
            self._takeoff()
            if skip_x:
                scored = bool(self.stage_summary.get(names[0], {}).get("best_metrics"))
            else:
                self.status = f"{stage}_x"
                self._save_summary()
                scored = (self._tune_position_pid(names[0], "x")
                          if stage == "position" and
                          self.config["position_repeated_mode"] else
                          self._tune_velocity_pid(names[0], "x")
                          if stage == "velocity" else
                          self._tune_acceleration_pid(names[0], "x")
                          if stage == "acceleration" else
                          self._tune_pid(stage, names[0], "x"))
            if (stage == "velocity" and self.config["velocity_repeated_mode"] and
                    self.stage_summary.get(names[0], {}).get("best_metrics")):
                self.status = "velocity_x_validation"
                self._save_summary()
                self._validate_velocity_speeds(names[0], "x")
            if (stage == "position" and self.config["position_repeated_mode"] and
                    self.stage_summary.get(names[0], {}).get("best_metrics")):
                self.status = "position_x_validation"
                self._save_summary()
                self._validate_position_distances(names[0], "x")
            if scored:
                self.status = f"{stage}_y_transfer"
                self._save_summary()
                scored = self._transfer_x_gains_to_y(stage, names[0], names[1])
            elif (stage == "velocity" and self.config["velocity_repeated_mode"] and
                  self.stage_summary.get(names[0], {}).get("best_metrics")):
                # A poor X result must not prevent an independent Y trial.
                # There are no trustworthy X gains to transfer to roll.
                self.best[names[1]].update(kp=0.0, ki=0.0, kd=0.0)
                self.controller.set_configs(self.best)
                self.status = "velocity_y"
                self._save_summary()
                scored = self._tune_velocity_pid(
                    names[1], "y", initial_source="zero_gains_start")
            elif (stage == "position" and self.config["position_repeated_mode"] and
                  self.stage_summary.get(names[0], {}).get("best_metrics")):
                self.best[names[1]].update(kp=0.0, ki=0.0, kd=0.0)
                self.controller.set_configs(self.best)
                self.status = "position_y"
                self._save_summary()
                scored = self._tune_position_pid(
                    names[1], "y", initial_source="zero_gains_start")
            if (stage == "velocity" and self.config["velocity_repeated_mode"] and
                    self.stage_summary.get(names[1], {}).get("best_metrics")):
                self.status = "velocity_y_validation"
                self._save_summary()
                self._validate_velocity_speeds(names[1], "y")
            if (stage == "position" and self.config["position_repeated_mode"] and
                    self.stage_summary.get(names[1], {}).get("best_metrics")):
                self.status = "position_y_validation"
                self._save_summary()
                self._validate_position_distances(names[1], "y")
            joint_scored = stage == "acceleration" and scored
            if scored:
                if stage != "acceleration" and not (
                        (stage == "velocity" and self.config["velocity_repeated_mode"]) or
                        (stage == "position" and self.config["position_repeated_mode"])):
                    self.status = f"joint_{stage}"
                    self._save_summary()
                    joint_scored = self._tune_joint(stage, names)
                elif stage in ("velocity", "position"):
                    joint_scored = True
            if joint_scored and not (stage == "position" and
                                     self.config["position_repeated_mode"]):
                self.status = f"{stage}_confirmation"
                self._save_summary()
                for name, axis in zip(names, ("x", "y")):
                    result = self._evaluate(
                        stage, name, axis, self.best,
                        f"{stage}_{name}_final_confirmation",
                        **({"velocity_phase": "i", "velocity_i_score_start":
                            self.stage_summary.get(name, {}).get(
                                "d_settling_reference_seconds")}
                           if stage == "velocity" and
                           self.config["velocity_repeated_mode"] else {}))
                    metrics = None if result is None else result["metrics"]
                    self.stage_summary[name]["final_isolated_metrics"] = metrics
                    isolated_stable = (
                        _velocity_repeated_stable(metrics, self.config)
                        if self.config["velocity_repeated_mode"] else
                        _velocity_response_stable(metrics,
                                                  self.config["velocity_step"], self.config)
                        if stage == "velocity" else
                        _acceleration_response_stable(metrics, self.config)
                        if stage == "acceleration" else
                        _position_waypoints_stable(metrics, self.config)
                    ) if metrics is not None else False
                    if stage == "velocity" and self.config["velocity_repeated_mode"]:
                        self.stage_summary[name]["final_isolated_i_criteria_met"] = (
                            isolated_stable)
                    else:
                        self.stage_summary[name]["stable"] = (
                            isolated_stable and
                            self.stage_summary[name].get("stable", False))
                    self._save_summary()
            self.status = "landing"
            self._save_summary()
            landing = self._throttle_ramp_land(f"{stage}_throttle_landing")
            self.stage_summary["landing"] = {
                **landing,
                "passed": landing["grounded"],
                "final_height": (self.last_sample.z if self.last_sample else None),
                "height_above_launch": landing["final_height_above_launch"],
                "z_bias": self.controller.z_bias,
            }
            if landing["grounded"]:
                self.link.disarm()
            required = (names if stage == "acceleration" or
                        (stage == "velocity" and self.config["velocity_repeated_mode"]) or
                        (stage == "position" and self.config["position_repeated_mode"])
                        else (*names, f"joint_{stage}"))
            stable = (joint_scored and
                      all(self.stage_summary.get(name, {}).get("stable", False)
                          for name in required) and
                      (stage != "position" or not self.config["position_repeated_mode"] or
                       all(self.stage_summary.get(name, {}).get(
                           "distance_validation_passed", False) for name in names)))
            self.status = "complete" if stable and landing["grounded"] else "needs_review"
        except (LandingIncomplete, InterventionDetected) as exc:
            self.failure = str(exc)
            self.status = "needs_review"
        except (KeyboardInterrupt, Exception) as exc:
            self.failure = f"{type(exc).__name__}: {exc}"
            self.status = "aborted"
            print(f"XY {stage} calibration stopped: {self.failure}", file=sys.stderr)
        finally:
            self.link.close()
            self._save_summary()
        return self.run_dir

    def run(self) -> Path:
        # Keep the original full-calibration control law independent of the
        # experimental ascent-mode hover throttle.
        self.controller.base_throttle_rc = 1500
        self.best_height_base_rc = 1500
        self.run_dir.mkdir(parents=True, exist_ok=False)
        self.status = "connecting"
        self._save_summary()
        armed = False
        try:
            self.link.connect()
            initial = self.link.read_sample()
            self.launch_xy = (initial.x, initial.y)
            self.initial_sample = initial
            self.anchor = initial
            self.last_sample = initial
            self.xy_reference_yaw = initial.yaw
            self.controller.z_bias = initial.z
            self.link.arm()
            armed = True
            self.status = "takeoff"
            self._save_summary()
            self._takeoff()
            plan = (
                ("height", (("pid_height", "z"),)),
                ("yaw", (("pid_yaw", "yaw"),)),
                ("acceleration", (("pid_accel_pitch", "x"), ("pid_accel_roll", "y"))),
                ("velocity", (("pid_vel_pitch", "x"), ("pid_vel_roll", "y"))),
                ("position", (("pid_pos_x", "x"), ("pid_pos_y", "y"))),
            )
            tuning_complete = True
            for stage, pids in plan:
                self.status = stage
                self._save_summary()
                for name, axis in pids:
                    tuned = (self._tune_position_pid(name, axis)
                             if stage == "position" and
                             self.config["position_repeated_mode"] else
                             self._tune_velocity_pid(name, axis)
                             if stage == "velocity" else
                             self._tune_acceleration_pid(name, axis)
                             if stage == "acceleration" else
                             self._tune_pid(stage, name, axis))
                    if (stage == "velocity" and self.config["velocity_repeated_mode"] and
                            self.stage_summary.get(name, {}).get("best_metrics")):
                        self._validate_velocity_speeds(name, axis)
                    if (stage == "position" and self.config["position_repeated_mode"] and
                            self.stage_summary.get(name, {}).get("best_metrics")):
                        self._validate_position_distances(name, axis)
                    if (not tuned and not
                            self.stage_summary.get(name, {}).get("best_metrics")):
                        tuning_complete = False
                        break
                if not tuning_complete:
                    break
                if stage in ("velocity", "position") and not (
                        (stage == "velocity" and self.config["velocity_repeated_mode"]) or
                        (stage == "position" and self.config["position_repeated_mode"])):
                    self.status = f"joint_{stage}"
                    self._save_summary()
                    if not self._tune_joint(stage, (pids[0][0], pids[1][0])):
                        tuning_complete = False
                        break
                    if stage == "velocity":
                        self.status = "velocity_range"
                        self._save_summary()
                        self._confirm_velocity_range()
            if tuning_complete:
                self.status = "validation"
                self._save_summary()
                validated = self._validation()
            else:
                validated = False
                self.stage_summary["validation"] = {
                    "passed": False,
                    "reason": "A required PID had no valid baseline trial",
                }
            self.status = "landing"
            self._save_summary()
            landed = self._land()
            required = (*PID_NAMES, "joint_position")
            if self.config["position_repeated_mode"]:
                required = PID_NAMES
            if not self.config["velocity_repeated_mode"]:
                required += ("joint_velocity",)
            stable = (all(self.stage_summary.get(name, {}).get("stable", False)
                          for name in required)
                      and (self.config["velocity_repeated_mode"] or
                           self.stage_summary.get("velocity_range", {}).get("passed", False))
                      and (not self.config["position_repeated_mode"] or
                           all(self.stage_summary.get(name, {}).get(
                               "distance_validation_passed", False)
                               for name in ("pid_pos_x", "pid_pos_y"))))
            self.status = "complete" if validated and landed and stable else "needs_review"
        except (KeyboardInterrupt, Exception) as exc:
            self.failure = f"{type(exc).__name__}: {exc}"
            self.status = "aborted"
            print(f"Calibration stopped: {self.failure}", file=sys.stderr)
        finally:
            if armed and self.status == "aborted":
                self.link.set_frame(HOLD_FRAME)
                time.sleep(0.2)
            self.link.close()
            self._save_summary()
        return self.run_dir


def inspect_run(path: Path) -> None:
    summary = json.loads((path / "summary.json").read_text(encoding="utf-8"))
    print(f"Status: {summary['status']}")
    origin = summary.get("xy_reference_origin")
    heading = summary.get("xy_reference_yaw")
    if origin is not None and heading is not None:
        print(f"XY reference: ({origin[0]:.3f}, {origin[1]:.3f}) m, "
              f"yaw={math.degrees(heading):.2f} deg")
    if summary.get("failure"):
        print(f"Failure: {summary['failure']}")
        if summary.get("last_csv"):
            print(f"Last telemetry: {summary['last_csv']}")
    for name, data in summary["stages"].items():
        if name == "takeoff":
            print(f"Takeoff ({'reached' if data['reached_height'] else 'below target'}): "
                  f"Z={data['final_height']}")
        elif name == "height_ascent":
            print(f"Height ascent: {data['baseline_score']:.3f} -> "
                  f"{data['best_score']:.3f}; "
                  f"{'stable' if data['stable'] else 'needs review'}; "
                  f"gains={data['recommended_gains']}; "
                  f"base RC={data['recommended_height_base_throttle_rc']}")
            for trial in data["best_trials"]:
                print(f"  target={trial['target_height']:.2f} m, "
                      f"reached={trial['reached']}, "
                      f"error={trial['final_error']:.3f} m, "
                      f"osc={trial['oscillation_rms']:.3f} m, "
                      f"landed={trial['landing']['grounded']}")
        elif name == "height_ascent_landing_failure":
            print(f"Throttle landing needs review: "
                  f"height={data['final_height_above_launch']}, "
                  f"vz={data['final_vertical_speed']}, "
                  f"telemetry={data['telemetry_csv']}")
        elif name == "validation":
            if "reason" in data:
                print(f"Validation needs review: {data['reason']}")
            else:
                print(f"Validation ({'pass' if data['passed'] else 'needs review'}): "
                      f"XY={data['final_xy_error']:.3f} m, "
                      f"Z={data['final_height_error']:.3f} m, "
                      f"yaw={data['final_yaw_error']:.3f} rad")
                if "oscillation_rms" in data:
                    print(f"Validation oscillation RMS: {data['oscillation_rms']}")
        elif name == "landing":
            speed = (data.get("final_vertical_speed") if "final_vertical_speed" in data
                     else data.get("peak_descent_speed", 0.0))
            print(f"Landing ({'pass' if data['passed'] else 'needs review'}): "
                  f"Z={data['final_height']:.3f} m, "
                  f"above launch={data.get('height_above_launch', data['final_height']):.3f} m, "
                  f"z_bias={data.get('z_bias', 0.0):.3f} m, "
                  f"vertical speed={speed if speed is not None else float('nan'):.3f} m/s")
        elif name in ("joint_velocity", "joint_position"):
            if data.get("status") == "unscored":
                print(f"{name}: unscored; {data['reason']}")
            else:
                print(f"{name}: {data['baseline_score']:.3f} -> "
                      f"{data['best_score']:.3f}; "
                      f"X osc={data['best_metrics']['x']['oscillation_rms']:.4f}, "
                      f"Y osc={data['best_metrics']['y']['oscillation_rms']:.4f}; "
                      f"{'stable' if data['stable'] else 'oscillating'}")
        elif name == "velocity_range":
            print(f"Velocity range: {'pass' if data['passed'] else 'needs review'}")
            for item in data["tests"]:
                metrics = item["metrics"]
                if metrics is None:
                    print(f"  {item['axis']} {item['speed']:.3f} m/s: invalid trial")
                else:
                    print(f"  {item['axis']} {item['speed']:.3f} m/s: "
                          f"{'pass' if item['passed'] else 'needs review'}, "
                          f"rise={metrics['positive']['rise_time']:.2f}/"
                          f"{metrics['negative']['rise_time']:.2f} s, "
                          f"osc={metrics['oscillation_rms']:.4f}")
        else:
            if data.get("status") == "unscored":
                print(f"{name}: unscored; {data['reason']}")
            else:
                metrics = (data.get("final_isolated_metrics") or
                           data.get("final_joint_metrics", data["best_metrics"]))
                oscillation = (f"osc={metrics['oscillation_rms']:.4f}"
                                f"/{data['oscillation_limit']:.4f}; "
                                f"{'stable' if data['stable'] else 'needs review'}; "
                               if "oscillation_rms" in metrics and "oscillation_limit" in data
                               else "osc=not measured in this run; ")
                score_label = ("isolated score" if "final_joint_metrics" in data else "score")
                print(f"{name}: {score_label} {data['baseline_score']:.3f} -> "
                      f"{data['best_score']:.3f}; "
                      f"final MAE={metrics['terminal_mae']:.4f}; {oscillation}"
                      f"{data['recommended_gains']}")
                if metrics.get("method") in (
                        "height_repeated_pid_response",
                        "yaw_repeated_pid_response"):
                    print(f"  P={'met' if data.get('p_criteria_met') else 'missed'}, "
                          f"D={'met' if data.get('d_criteria_met') else 'missed'}, "
                          f"I={'met' if data.get('i_criteria_met') else 'missed'}; "
                          f"3-flight repeats={metrics.get('repeat_count', 0)}; "
                          f"D reference={data.get('d_settling_reference_seconds', float('nan')):.2f}s")
                    if metrics["method"] == "height_repeated_pid_response":
                        validation = data.get("validation", {})
                        print(f"  height validation "
                              f"{validation.get('targets', [])}: "
                              f"{'pass' if validation.get('passed') else 'needs review'}")
                    else:
                        parts = metrics.get("mean_response_by_direction", {})
                        if len(parts) == 2:
                            print(f"  yaw hold +/-="
                                  f"{parts['positive']['target_hold_fraction']:.0%}/"
                                  f"{parts['negative']['target_hold_fraction']:.0%}")
                if data.get("transfer_accepted"):
                    print(f"  accepted directly from {data['transferred_from']}; "
                          "separate Y search was skipped")
                if name in ("pid_vel_pitch", "pid_vel_roll") and \
                        "trial_count" in data:
                    print(f"  pure PID trials={data['trial_count']}/"
                          f"{data['trial_limit']}; "
                          f"P target={'met' if data['p_criteria_met'] else 'missed'}; "
                          f"D target={'met' if data['d_criteria_met'] else 'missed'}; "
                          f"I={'used' if data['i_needed'] else 'not needed'}; "
                          f"repeatable={data['repeatable']}")
                    if "repeat_steps" in metrics:
                        print(f"  three-flight mean hold={metrics['hold_fraction']:.0%}, "
                              f"+/-={metrics['hold_fraction_by_direction'].get('positive', 0):.0%}/"
                              f"{metrics['hold_fraction_by_direction'].get('negative', 0):.0%}; "
                              f"service braking excluded from score")
                    else:
                        print(f"  in-band={metrics['in_band_fraction']:.0%}, "
                              f"tail={metrics['tail_in_band_fraction']:.0%}, "
                              f"a95={metrics['p95_acceleration']:.3f} m/s², "
                              f"j95={metrics['p95_jerk']:.3f} m/s³")
                if name in ("pid_vel_pitch", "pid_vel_roll"):
                    for check in data.get("speed_validation", []):
                        check_metrics = check.get("metrics")
                        if check_metrics is None:
                            print(f"  validation {check['speed']:.2f} m/s: invalid trial")
                        else:
                            holds = check_metrics["hold_fraction_by_direction"]
                            print(f"  validation {check['speed']:.2f} m/s: "
                                  f"{'pass' if check['passed'] else 'needs review'}, "
                                  f"mean hold +/-={holds['positive']:.0%}/"
                                  f"{holds['negative']:.0%}, "
                                  f"settle={check_metrics['plateau_settling_time']:.2f} s")
                if name in ("pid_pos_x", "pid_pos_y") and \
                        metrics.get("method") == "position_repeated_response":
                    print(f"  P={'met' if data.get('p_criteria_met') else 'missed'}, "
                          f"D={'met' if data.get('d_criteria_met') else 'missed'}, "
                          f"I={'met' if data.get('i_criteria_met') else 'missed'}; "
                          f"trials={data.get('trial_count', 'transfer')}; "
                          f"D reference={data.get('d_settling_reference_seconds', float('nan')):.2f}s")
                    for check in data.get("distance_validation", []):
                        check_metrics = check.get("metrics")
                        if check_metrics is None:
                            print(f"  validation {check['distance']:.3f} m: invalid trial")
                        else:
                            holds = check_metrics["hold_fraction_by_direction"]
                            print(f"  validation {check['distance']:.3f} m: "
                                  f"{'pass' if check['passed'] else 'needs review'}, "
                                  f"mean hold +/-={holds['positive']:.0%}/"
                                  f"{holds['negative']:.0%}, "
                                  f"settle={check_metrics['plateau_settling_time']:.2f}s")
                if name == "pid_height" and "descent_steps" in metrics:
                    progress = metrics.get("descent_progressed_fraction")
                    fraction = (metrics["descent_reached_fraction"] if
                                progress is None else progress)
                    verb = "reached" if progress is None else "progressed"
                    print(f"  descent: {fraction:.0%} "
                          f"of {metrics['descent_steps']} steps {verb}, "
                          f"peak vz={metrics['peak_descent_speed']:.3f} m/s, "
                          f"step osc={metrics['descent_oscillation_rms']:.4f} m")
                if name == "pid_height" and "waypoints" in metrics:
                    print(f"  hold MAE={metrics['mae']:.3f} m, "
                          f"max overshoot={metrics['max_overshoot']:.3f} m, "
                          f"base RC={summary['recommended_height_base_throttle_rc']}")
                    for point in metrics["waypoints"]:
                        settling_time = point.get("settling_time")
                        settle = (f"{settling_time:.2f}s" if settling_time is not None
                                  else "not measured")
                        print(f"  {point['height']:.2f} m: "
                               f"{'reached' if point['reached'] else 'missed'}, "
                               f"error={point['terminal_mae']:.3f} m, "
                               f"overshoot={point['max_overshoot']:.3f} m, "
                               f"settle={settle}, "
                               f"osc={point['oscillation_rms']:.3f} m")
                if name == "pid_yaw" and "waypoints" in metrics:
                    print(f"  max overshoot={math.degrees(metrics['max_overshoot']):.2f} deg, "
                          f"peak yaw rate={metrics['peak_yaw_rate_windowed']:.3f} rad/s, "
                          f"max height error={metrics['max_height_terminal_error']:.3f} m")
                    for point in metrics["waypoints"]:
                        settle = point["settling_time"]
                        print(f"  {point['angle_deg']:+.0f} deg: "
                              f"{'reached' if point['reached'] else 'missed'}, "
                              f"error={math.degrees(point['terminal_mae']):.2f} deg, "
                              f"overshoot={math.degrees(point['max_overshoot']):.2f} deg, "
                              f"settle={f'{settle:.2f}s' if settle is not None else 'not settled'}, "
                              f"osc={math.degrees(point['oscillation_rms']):.2f} deg")
                if metrics.get("method") == "physical_step_response":
                    for direction in ("positive", "negative"):
                        step = metrics[direction]
                        rise = (f"{step['rise_time']:.2f}" if
                                step.get("rise_time") is not None else "not reached")
                        settling = (f"{step['settling_time']:.2f}" if
                                    step.get("settling_time") is not None else "not settled")
                        stopping = (f"{step['stopping_time']:.2f}" if
                                    step.get("stopping_time") is not None else "not stopped")
                        print(f"  {direction}: peak={step['max_speed']:.3f} m/s, "
                              f"rise={rise} s, settling={settling} s, "
                              f"stop={stopping} s, "
                              f"braking={step['stopping_distance']:.3f} m")
                if metrics.get("method") == "position_waypoint_response":
                    print(f"  {metrics['waypoints_reached_fraction']:.0%} of "
                          f"{len(metrics['waypoints'])} waypoints reached, "
                          f"max overshoot={metrics['max_overshoot']:.3f} m")
                    for point in metrics["waypoints"]:
                        settle = point["settling_time"]
                        print(f"  {point['distance']:.2f} m "
                              f"{'return' if point['return'] else 'outbound'}: "
                              f"{'reached' if point['reached'] else 'missed'}, "
                              f"error={point['terminal_mae']:.3f} m, "
                              f"overshoot={point['max_overshoot']:.3f} m, "
                              f"settle={f'{settle:.2f}s' if settle is not None else 'not settled'}, "
                              f"osc={point['oscillation_rms']:.3f} m")
    valid = [item for item in summary["evaluations"] if item.get("metrics")]
    if valid:
        worst = max(valid, key=lambda item: item["metrics"]["score"])
        print(f"Worst trial: {worst['label']} ({worst['telemetry_csv']})")
    if summary.get("interventions"):
        print(f"Manual moves ignored: {len(summary['interventions'])}")
    print(f"Raw telemetry and summary: {path.resolve()}")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--init-config", type=Path, metavar="FILE",
                        help="write an editable JSON configuration and exit")
    parser.add_argument("--config", type=Path, help="JSON configuration")
    parser.add_argument("--fly", action="store_true", help="connect, arm and run calibration")
    height_mode = parser.add_mutually_exclusive_group()
    height_mode.add_argument(
        "--ascent-only", action="store_true",
        help="tune height ascent/hold only, landing by direct throttle between trials")
    height_mode.add_argument(
        "--height-two-stage", action="store_true",
        help="tune ascent from ground, then continuous up/down height control")
    height_mode.add_argument(
        "--height-stage2", action="store_true",
        help="test continuous up/down height control using gains from --resume-height")
    height_mode.add_argument(
        "--yaw-only", action="store_true",
        help="tune yaw with calibrated height gains from --resume-height; XY remains disabled")
    height_mode.add_argument(
        "--velocity-only", action="store_true",
        help="tune XY speed X then Y using loaded height, yaw and acceleration")
    height_mode.add_argument(
        "--acceleration-only", action="store_true",
        help="tune experimental XY acceleration X then Y, using loaded height and yaw")
    height_mode.add_argument(
        "--acceleration-roll-only", action="store_true",
        help="continue with the best pitch I trial, then verify/tune acceleration roll")
    height_mode.add_argument(
        "--position-only", action="store_true",
        help="tune XY position using loaded height, yaw and XY speed")
    parser.add_argument("--resume-height", type=Path, metavar="RUN_DIR",
                        help="load recommended height gains and base throttle from a previous run")
    parser.add_argument("--resume-yaw", type=Path, metavar="RUN_DIR",
                        help="load recommended yaw gains from a previous run")
    parser.add_argument("--resume-acceleration", type=Path, metavar="RUN_DIR",
                        help="load experimental XY acceleration gains before speed/position tuning")
    parser.add_argument("--resume-acceleration-pitch", type=Path, metavar="RUN_DIR",
                        help="load the best pitch I trial for --acceleration-roll-only")
    parser.add_argument("--resume-velocity", type=Path, metavar="RUN_DIR",
                        help="load recommended XY speed gains from a previous run")
    parser.add_argument("--inspect", type=Path, metavar="RUN_DIR",
                        help="summarize a previous run")
    args = parser.parse_args(argv)
    if args.init_config:
        config = default_config()
        config["hover_height"] = get_drone_takeoff_height(config["drone_name"])
        args.init_config.write_text(json.dumps(config, ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"Wrote {args.init_config}")
        return 0
    if args.inspect:
        inspect_run(args.inspect)
        return 0
    if args.fly and args.config is None:
        parser.error("--fly requires --config")
    if (args.ascent_only or args.height_two_stage or args.height_stage2
            or args.yaw_only or args.acceleration_only or args.acceleration_roll_only
            or args.velocity_only or args.position_only) and not args.fly:
        parser.error("calibration mode requires --fly")
    if (args.height_stage2 or args.yaw_only or args.acceleration_only or
            args.acceleration_roll_only or args.velocity_only or
            args.position_only) and args.resume_height is None:
        parser.error("selected mode requires --resume-height RUN_DIR")
    if (args.acceleration_only or args.acceleration_roll_only or
            args.velocity_only or args.position_only) and args.resume_yaw is None:
        parser.error("XY calibration requires --resume-yaw RUN_DIR")
    if args.acceleration_roll_only and args.resume_acceleration_pitch is None:
        parser.error("--acceleration-roll-only requires --resume-acceleration-pitch RUN_DIR")
    if (args.velocity_only or args.position_only) and args.resume_acceleration is None:
        parser.error("speed/position calibration requires --resume-acceleration RUN_DIR")
    if args.position_only and args.resume_velocity is None:
        parser.error("--position-only requires --resume-velocity RUN_DIR")
    if args.resume_height and not (args.ascent_only or args.height_two_stage
                                   or args.height_stage2 or args.yaw_only or
                                   args.acceleration_only or args.acceleration_roll_only or
                                   args.velocity_only or args.position_only):
        parser.error("--resume-height requires a staged calibration mode")
    if args.resume_yaw and not (args.acceleration_only or args.acceleration_roll_only or
                                args.velocity_only or args.position_only):
        parser.error("--resume-yaw requires an XY calibration mode")
    if args.resume_acceleration and not (args.velocity_only or args.position_only):
        parser.error("--resume-acceleration requires --velocity-only or --position-only")
    if args.resume_acceleration_pitch and not args.acceleration_roll_only:
        parser.error("--resume-acceleration-pitch requires --acceleration-roll-only")
    if args.resume_velocity and not args.position_only:
        parser.error("--resume-velocity requires --position-only")
    config = load_config(args.config) if args.config else default_config()
    if not args.fly:
        print(json.dumps(config, ensure_ascii=False, indent=2))
        print("Dry run only. Pass --config FILE --fly to connect to the simulator.")
        return 0
    runner = CalibrationRunner(config)
    if args.resume_height:
        runner.resume_height_from(args.resume_height)
    if args.resume_yaw:
        runner.resume_yaw_from(args.resume_yaw)
    if args.resume_acceleration:
        runner.resume_acceleration_from(args.resume_acceleration)
    if args.resume_acceleration_pitch:
        runner.resume_acceleration_pitch_from(args.resume_acceleration_pitch)
    if args.resume_velocity:
        runner.resume_velocity_from(args.resume_velocity)
    if args.ascent_only:
        path = runner.run_ascent_only()
    elif args.height_two_stage:
        path = runner.run_height_two_stage()
    elif args.height_stage2:
        path = runner.run_height_two_stage(continuous_only=True)
    elif args.yaw_only:
        path = runner.run_yaw_only()
    elif args.acceleration_only:
        path = runner.run_xy_only("acceleration")
    elif args.acceleration_roll_only:
        path = runner.run_xy_only("acceleration", skip_x=True)
    elif args.velocity_only:
        path = runner.run_xy_only("velocity")
    elif args.position_only:
        path = runner.run_xy_only("position")
    else:
        path = runner.run()
    inspect_run(path)
    return 0 if runner.status == "complete" else 1


if __name__ == "__main__":
    raise SystemExit(main())
