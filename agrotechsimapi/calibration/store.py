"""Versioned, portable JSON calibration profiles and an atomic named store."""
from __future__ import annotations

import copy
import json
import math
import os
import re
import tempfile
from datetime import datetime, timezone
from importlib import resources
from pathlib import Path
from typing import Any

from .paths import data_dir

PID_NAMES = ("pid_height", "pid_yaw", "pid_accel_pitch", "pid_accel_roll",
             "pid_vel_pitch", "pid_vel_roll", "pid_pos_x", "pid_pos_y")
DEFAULT_PRESET = "edu-ext"
BUILTIN_PRESETS = (DEFAULT_PRESET, "edu-constructor", "edu")
_BUILTIN_BY_NAME = {name.casefold(): name for name in BUILTIN_PRESETS}
_BUILTIN_BY_NAME["default"] = DEFAULT_PRESET
CONTROL_KEYS = ("kinematics_hz", "acceleration_control_hz", "velocity_control_hz",
                "position_control_hz", "velocity_filter_alpha", "acceleration_filter_alpha",
                "max_xy_speed", "max_xy_acceleration", "max_accel_rc_offset",
                "acceleration_i_max_rc_offset", "max_yaw_rc_offset",
                "max_height_rc_offset", "height_base_throttle_rc", "direction",
                "height_descent_step", "height_descent_reach_tolerance",
                "height_descent_settle_seconds", "height_descent_max_speed",
                "height_descent_step_timeout_seconds", "height_descent_max_target_undershoot",
                "height_descent_max_command_gap", "throttle_landing_start_rc",
                "throttle_landing_end_rc", "throttle_landing_rc_per_second",
                "throttle_landing_speed_limit", "throttle_landing_ground_tolerance",
                "throttle_landing_ground_speed", "throttle_landing_ground_hold_seconds",
                "landing_seconds")


def timestamp() -> str:
    return datetime.now(timezone.utc).isoformat()


def validate_name(name: str) -> str:
    if not isinstance(name, str) or not re.fullmatch(r"[\w][\w.-]{0,79}", name, re.UNICODE):
        raise ValueError("Calibration name: 1–80 letters, digits, underscores, dots or hyphens")
    if name.endswith('.') or name.split('.')[0].upper() in {
            "CON", "PRN", "AUX", "NUL", *(f"COM{i}" for i in range(1, 10)),
            *(f"LPT{i}" for i in range(1, 10))}:
        raise ValueError("Reserved calibration name")
    return name


def atomic_json(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    text = json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False)
    fd, temporary = tempfile.mkstemp(prefix=path.name + '.', suffix='.tmp', dir=path.parent)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as stream:
            stream.write(text)
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def _number(value: Any, label: str, *, minimum: float = 0, positive=False) -> None:
    if (isinstance(value, bool) or not isinstance(value, (int, float)) or
            not math.isfinite(value) or value < minimum or (positive and value == 0)):
        raise ValueError(f"Invalid calibration value: {label}")


def processing_functions() -> dict:
    from ..utils import drone_setups
    return {name: getattr(drone_setups, name) for name in (
        'default_height_pid_processing', 'edu_extended_height_pid_processing',
        'big_drone_height_pid_processing', 'multifunctional_height_pid_processing',
        't40_height_pid_processing', 'phantom_height_pid_processing')}


def validate_profile(profile: dict) -> dict:
    """Reject corrupt/incompatible profiles before they reach a controller."""
    if not isinstance(profile, dict) or profile.get('schema_version') != 1:
        raise ValueError("Unsupported calibration schema_version (expected 1)")
    validate_name(profile.get('name'))
    try:
        date = datetime.fromisoformat(profile['created_at'])
        if date.tzinfo is None:
            raise ValueError("Calibration created_at must include a timezone")
    except (KeyError, TypeError) as exc:
        raise ValueError("Missing calibration date") from exc
    if profile.get('status') not in ('complete', 'needs_review'):
        raise ValueError("Only finished calibrations can be loaded")
    pids = profile.get('pids', {})
    if not isinstance(pids, dict) or set(pids) != set(PID_NAMES):
        raise ValueError("Calibration must contain all eight PID controllers")
    allowed = {'kp','ki','kd','max_control','i_limit','is_exp','exp_factor','processing_func'}
    for name, values in pids.items():
        if not isinstance(values, dict) or set(values) - allowed:
            raise ValueError(f"Unsupported PID fields in {name}")
        for gain in ('kp', 'ki', 'kd'):
            _number(values.get(gain), f'{name}.{gain}')
        for field in ('max_control', 'i_limit', 'exp_factor'):
            if values.get(field) is not None:
                _number(values[field], f'{name}.{field}', positive=True)
        if 'is_exp' in values and not isinstance(values['is_exp'], bool):
            raise ValueError(f"Invalid {name}.is_exp")
        function = values.get('processing_func')
        if function is not None and (not isinstance(function, str) or
                                     function not in processing_functions()):
            raise ValueError(f"Unknown PID processing function: {function!r}")
    config = profile.get('config', {})
    if not isinstance(config, dict):
        raise ValueError("Missing calibration config")
    for key in CONTROL_KEYS:
        if key not in config:
            raise ValueError(f"Missing control setting: {key}")
        if key != 'direction':
            _number(config[key], key, positive=True)
    for key in ('velocity_filter_alpha', 'acceleration_filter_alpha'):
        if config[key] > 1:
            raise ValueError(f"{key} must be in (0, 1]")
    if not (config['kinematics_hz'] >= config['acceleration_control_hz'] >=
            config['velocity_control_hz'] >= config['position_control_hz']):
        raise ValueError("Expected kinematics >= acceleration >= velocity >= position frequency")
    if not 1200 <= config['height_base_throttle_rc'] <= 1800:
        raise ValueError("Invalid height base throttle")
    if not 1000 <= config['throttle_landing_end_rc'] <= config['throttle_landing_start_rc'] <= 1800:
        raise ValueError("Invalid landing throttle ramp")
    if config['height_descent_reach_tolerance'] >= config['height_descent_step']:
        raise ValueError("Descent tolerance must be smaller than the descent step")
    for key in ('max_accel_rc_offset', 'acceleration_i_max_rc_offset',
                'max_yaw_rc_offset', 'max_height_rc_offset'):
        if config[key] > 500:
            raise ValueError(f"{key} must be <= 500")
    direction = config['direction']
    if not isinstance(direction, dict) or any(direction.get(k) not in (-1, 1)
                                             for k in ('pitch', 'roll', 'yaw')):
        raise ValueError("Direction signs must be +1 or -1")
    # JSON portability also rejects NaN/Infinity hidden in metrics or extra fields.
    try:
        json.dumps(profile, allow_nan=False)
    except (ValueError, TypeError) as exc:
        raise ValueError("Calibration must contain finite JSON values") from exc
    return profile


def runtime_pids(profile: dict) -> dict:
    values = copy.deepcopy(validate_profile(profile)['pids'])
    functions = processing_functions()
    for pid in values.values():
        function = pid.get('processing_func')
        if function:
            pid['processing_func'] = functions[function]
        # null represents the PID's default unbounded output.
        if pid.get('max_control', 0) is None:
            pid.pop('max_control')
    return values


def finite_json(value: Any) -> Any:
    """Undefined diagnostics remain explicit nulls in portable JSON."""
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {k: finite_json(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [finite_json(v) for v in value]
    return value


class CalibrationStore:
    def __init__(self, root: str | Path | None = None):
        self.root = Path(root).expanduser().resolve() if root is not None else data_dir()

    def path(self, name: str) -> Path:
        return self.root / 'calibrations' / (validate_name(name) + '.json')

    def load(self, name: str = DEFAULT_PRESET) -> dict:
        validate_name(name)
        builtin = _BUILTIN_BY_NAME.get(name.casefold())
        if builtin is not None:
            text = resources.files(__package__).joinpath(
                f'data/{builtin}.json').read_text(encoding='utf-8')
        else:
            text = self.path(name).read_text(encoding='utf-8')
        return validate_profile(json.loads(text))

    def load_file(self, path: str | Path) -> dict:
        return validate_profile(json.loads(Path(path).read_text(encoding='utf-8')))

    def list(self) -> list[dict]:
        profiles = [self.load(name) for name in BUILTIN_PRESETS]
        for path in sorted((self.root / 'calibrations').glob('*.json')):
            profile = self.load_file(path)
            if profile['name'].casefold() not in _BUILTIN_BY_NAME:
                profiles.append(profile)
        return profiles

    def check_destination(self, name: str, *, replace: bool = False) -> Path:
        path = self.path(name)
        if name.casefold() in _BUILTIN_BY_NAME:
            names = ', '.join(BUILTIN_PRESETS)
            raise ValueError(f"{name} is bundled and read-only; choose a name other than {names}")
        if path.exists() and not replace:
            raise FileExistsError(f"Calibration {name!r} already exists; use --replace")
        return path

    def save(self, profile: dict, *, replace: bool = False) -> Path:
        validate_profile(profile)
        path = self.check_destination(profile['name'], replace=replace)
        atomic_json(path, profile)
        return path

    def export(self, name: str, destination: str | Path, *, replace: bool = False) -> Path:
        path = Path(destination).expanduser().resolve()
        if path.exists() and not replace:
            raise FileExistsError(f"{path} already exists; use --replace")
        profile = copy.deepcopy(self.load(name))
        # Local telemetry is not part of a coefficients-only JSON export.
        profile.pop('session_id', None)
        atomic_json(path, profile)
        return path

    def import_file(self, source: str | Path, *, name: str | None = None,
                    replace: bool = False) -> Path:
        profile = self.load_file(source)
        profile['name'] = name or profile['name']
        profile.pop('session_id', None)
        return self.save(profile, replace=replace)
