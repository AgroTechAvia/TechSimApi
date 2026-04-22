from copy import deepcopy
from typing import Dict


def default_height_pid_processing(x):
    """Apply default nonlinear scaling for altitude PID output."""
    return ((2 / (1 + (2.7 ** (-x * 4.125)))) - 1) * 1.7

def edu_extended_height_pid_processing(x):
    """Apply EDU_EXTENDED nonlinear scaling for altitude PID output."""
    return ((2 / (1 + (2.7 ** (-x * 4.0)))) - 1) * 1.68

def big_drone_height_pid_processing(x):
    """Apply EDU_EXTENDED nonlinear scaling for altitude PID output."""
    return ((2 / (1 + (2.7 ** (-x * 2.7)))) - 1) * 1.65


DRONE_PID_SETUPS: Dict[str, Dict[str, Dict]] = {
    "DEFAULT": {
        "pid_pos_x": {"kp": 1.85, "ki": 0.0, "kd": 1.5, "max_control": 0.2, "i_limit": 0.1},
        "pid_pos_y": {"kp": 1.85, "ki": 0.0, "kd": 1.5, "max_control": 0.2, "i_limit": 0.1},
        "pid_vel_pitch": {"kp": 3.15, "ki": 0.0, "kd": 3.4, "max_control": 1.5, "i_limit": 0.0033},
        "pid_vel_roll": {"kp": 3.15, "ki": 0.0, "kd": 3.4, "max_control": 1.5, "i_limit": 0.0033},
        "pid_yaw": {"kp": 20.0, "ki": 0.0, "kd": 0.4, "max_control": 1.0, "i_limit": None},
        "pid_height": {"kp": 4.5, "ki": 0.0, "kd": 3.5, "i_limit": 1.0, "processing_func": default_height_pid_processing}
    },
    "EDU_EXTENDED": {
        "pid_pos_x": {"kp": 1.85, "ki": 0.0, "kd": 1.75, "max_control": 0.25, "i_limit": 0.1},
        "pid_pos_y": {"kp": 1.85, "ki": 0.0, "kd": 1.75, "max_control": 0.25, "i_limit": 0.1},
        "pid_vel_pitch": {"kp": 2.0, "ki": 0.0, "kd": 3.4, "max_control": 2.0, "i_limit": 0.0033},
        "pid_vel_roll": {"kp": 2.0, "ki": 0.0, "kd": 3.4, "max_control": 2.0, "i_limit": 0.0033},
        "pid_yaw": {"kp": 45.0, "ki": 0.0, "kd": 0.4, "max_control": 1.0, "i_limit": None},
        "pid_height": {"kp": 3.6, "ki": 0.0, "kd": 3.3, "i_limit": 1.0, "processing_func": edu_extended_height_pid_processing}
    },
    "DJI_MATRICE_210": {
        "pid_pos_x": {"kp": 1.85, "ki": 0.0, "kd": 1.75, "max_control": 0.75, "i_limit": 0.1},
        "pid_pos_y": {"kp": 1.85, "ki": 0.0, "kd": 1.75, "max_control": 0.75, "i_limit": 0.1},
        "pid_vel_pitch": {"kp": 3.0, "ki": 0.0, "kd": 3.4, "max_control": 2.0, "i_limit": 0.0033},
        "pid_vel_roll": {"kp": 3.0, "ki": 0.0, "kd": 3.4, "max_control": 2.0, "i_limit": 0.0033},
        "pid_yaw": {"kp": 25.0, "ki": 0.0, "kd": 0.4, "max_control": 2.0, "i_limit": None},
        "pid_height": {"kp": 4.6, "ki": 0.0, "kd": 9.75, "i_limit": 1.0, "processing_func": big_drone_height_pid_processing}
    },

    "AGRO": {
        "pid_pos_x": {"kp": 1.85, "ki": 0.0, "kd": 1.95, "max_control": 0.5, "i_limit": 0.1},
        "pid_pos_y": {"kp": 1.85, "ki": 0.0, "kd": 1.95, "max_control": 0.5, "i_limit": 0.1},
        "pid_vel_pitch": {"kp": 3.0, "ki": 0.0, "kd": 3.7, "max_control": 2.15, "i_limit": 0.0033},
        "pid_vel_roll": {"kp": 3.0, "ki": 0.0, "kd": 3.7, "max_control": 2.15, "i_limit": 0.0033},
        "pid_yaw": {"kp": 40.0, "ki": 0.0, "kd": 0.4, "max_control": 2.0, "i_limit": None},
        "pid_height": {"kp": 4.6, "ki": 0.0, "kd": 9.75, "i_limit": 1.0, "processing_func": big_drone_height_pid_processing}
    },

}

DRONE_TAKEOFF_HEIGHTS: Dict[str, float] = {
    "DEFAULT": 1.0,
    "EDU_EXTENDED": 1.0,
    "DJI_MATRICE_210": 2.0,
    "AGRO": 2.0,
}


def get_drone_pid_setup(drone_name: str = "DEFAULT") -> Dict[str, Dict]:
    """Return a deep copy of PID preset configuration for selected drone."""
    normalized_name = (drone_name or "DEFAULT").upper()
    if normalized_name not in DRONE_PID_SETUPS:
        available = ", ".join(sorted(DRONE_PID_SETUPS.keys()))
        raise ValueError(f"Unknown drone preset '{drone_name}'. Available presets: {available}")
    return deepcopy(DRONE_PID_SETUPS[normalized_name])


def get_drone_takeoff_height(drone_name: str = "DEFAULT") -> float:
    """Return preset takeoff height in meters for selected drone."""
    normalized_name = (drone_name or "DEFAULT").upper()
    return DRONE_TAKEOFF_HEIGHTS.get(normalized_name, 1.0)
