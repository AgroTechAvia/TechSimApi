"""High-level simulator client with cascaded PID control loops."""
from inavmspapi import MultirotorControl
from inavmspapi.transmitter import TCPTransmitter
from inavmspapi.msp_codes import MSPCodes
from agrotechsimapi.client import PointCloud, SimClient

from agrotechsimapi.pid import PID, AdaptivePID
from typing import Dict, Iterable, Optional, Tuple, Literal, Union

from agrotechsimapi.utils.utils import LoopingTimer, sim_to_api_distance, vel_to_rc_signal
from agrotechsimapi.utils.drone_setups import get_drone_pid_setup, get_drone_takeoff_height
from agrotechsimapi.utils.vision import process_aruco, process_blob, resolution_changes

from transforms3d.euler import quat2euler

import time
import math
import threading
import logging
import asyncio
import copy
from pathlib import Path
from agrotechsimapi.control import CascadedController, Sample, sample_from_kinematics
from agrotechsimapi.calibration.store import DEFAULT_PRESET, CalibrationStore, runtime_pids

import numpy as np

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def main():
    """Module entry point used for local debugging."""
    pass


if __name__ == "__main__":
    main()


class HighLevelSimClient:
    """High-level API for drone control in the TechSim simulator."""

    ControlMode = Literal["position", "velocity"]

    _ARM_RESET_FRAME = (1000, 1000, 1000, 1000, 1000, 1000, 1000)
    _ARM_SWITCH_FRAME = (1000, 1000, 1000, 1000, 2000, 1000, 1000)
    _ARMED_NEUTRAL_FRAME = (1500, 1500, 1000, 1500, 2000, 1000, 1000)
    _ARM_FRAME_FREQUENCY = 20.0
    _ARM_SETTLE_SECONDS = 1.0

    _ALTHOLD_MODE_ID = 3
    _ALTHOLD_AUX_CHANNEL_INDEX = 2
    _ALTHOLD_RANGE_START = 1250
    _ALTHOLD_RANGE_END = 1350
    _ALTHOLD_AUX_VALUE = 1300

    _max_velocity = 0.2  # Maximum linear speed in m/s.
    _max_acceleration = 0.75  # Maximum linear acceleration in m/s^2.
    # =======================================================

    _z_bias = 0

    @staticmethod
    def _pid_to_config(pid: PID) -> Dict:
        """Convert a PID instance to a serializable configuration dictionary."""
        return {
            "kp": pid.kp,
            "ki": pid.ki,
            "kd": pid.kd,
            "max_control": pid.max_control,
            "i_limit": pid.i_limit,
            "is_exp": pid.is_exp,
            "exp_factor": pid.exp_factor,
            "processing_func": pid._processing_func,
        }

    @classmethod
    def _resolve_pid_config(
        cls,
        preset_pid: Dict,
        custom_pid: Optional[Union[PID, Dict]],
        pid_name: str,
    ) -> Dict:
        """Build final PID config from preset values and optional override."""
        if custom_pid is None:
            return dict(preset_pid)

        if isinstance(custom_pid, PID):
            return cls._pid_to_config(custom_pid)

        if isinstance(custom_pid, dict):
            merged = dict(preset_pid)
            merged.update(custom_pid)
            return merged

        raise TypeError(
            f"{pid_name} должен быть PID, dict или None, получен {type(custom_pid).__name__}"
        )

    def __init__(
        self,
        drone_name: str = "DEFAULT",
        pid_pos_x: Optional[Union[PID, Dict]] = None,
        pid_pos_y: Optional[Union[PID, Dict]] = None,
        pid_vel_pitch: Optional[Union[PID, Dict]] = None,
        pid_vel_roll: Optional[Union[PID, Dict]] = None,
        pid_yaw: Optional[Union[PID, Dict]] = None,
        pid_height: Optional[Union[PID, Dict]] = None,
        *,
        calibration: str = DEFAULT_PRESET,
        calibration_path: Optional[Union[str, Path]] = None,
        calibration_dir: Optional[Union[str, Path]] = None,
        pid_accel_pitch: Optional[Union[PID, Dict]] = None,
        pid_accel_roll: Optional[Union[PID, Dict]] = None,
    ):
        """Initialize client state, PID controllers, and runtime flags."""
        self.camera_id = 0

        store = CalibrationStore(calibration_dir)
        profile = store.load_file(calibration_path) if calibration_path is not None else store.load(calibration)
        self.calibration_name = profile["name"]
        self.calibration = copy.deepcopy(profile)
        self._control_config = copy.deepcopy(profile["config"])
        self._takeoff_height = get_drone_takeoff_height(drone_name)
        configs = runtime_pids(profile)
        overrides = dict(pid_pos_x=pid_pos_x, pid_pos_y=pid_pos_y,
                         pid_vel_pitch=pid_vel_pitch, pid_vel_roll=pid_vel_roll,
                         pid_yaw=pid_yaw, pid_height=pid_height,
                         pid_accel_pitch=pid_accel_pitch, pid_accel_roll=pid_accel_roll)
        for name, override in overrides.items():
            configs[name] = self._resolve_pid_config(configs[name], override, name)
        self._cascade = CascadedController(self._control_config, configs)
        for name, pid in self._cascade.pids.items():
            setattr(self, '_' + name, pid)
        self._calibrated_base_throttle = self._control_config['height_base_throttle_rc']
        self._base_throttle_hover = self._calibrated_base_throttle
        self._max_throttle = 1800
        self._min_throttle = 1200
        self._max_velocity = self._control_config['max_xy_speed']
        self._max_acceleration = self._control_config['max_xy_acceleration']
        self._roll_direction = self._control_config['direction']['roll']
        self._pitch_direction = self._control_config['direction']['pitch']
        self._yaw_direction = self._control_config['direction']['yaw']
        self._control_lock = threading.RLock()
        self._latest_sample = None
        self._last_control_sample_t = None
        self._control_telemetry = {}
        self._velocity_frame = 'base_link'
        self._descent_goal = None
        self._descent_command = None
        self._descent_step_started = None
        self._descent_step_origin = None
        self._landing_throttle = None
        self._z_bias = 0.0
        self._reference_initialized = False

        self._control_mode = "position"
        self._target_position = (0.0, 0.0)
        self._target_velocity = (0.0, 0.0)
        self._target_yaw = 0.0
        self._yaw_mode = "position"
        self._target_yaw_rate = 0.0

        self._motors_locked = True

        self._prev_x = 0.0
        self._prev_y = 0.0
        self._prev_z = 0.0
        self._prev_target_vx = 0.0
        self._prev_target_vy = 0.0
        self._prev_target_vz = 0.0

        self._vel_filter_alpha = self._control_config["velocity_filter_alpha"]
        self._filtered_vx_world = 0.0
        self._filtered_vy_world = 0.0
        self._vel_filter_initialized = False
        # =========================================

        self._odom = (0.0, 0.0)
        self._altitude = 0.0
        self._target_height = 0.0

        self._armed_flag = False
        self._poshold_flag = False
        self._althold_flag = False
        self._althold_range_configured = False
        self._height_timer_started = False

        self._sim_img = None
        self._blob_img = None
        self._aruco_img = None
        self._aruco_data = []
        self._camera_pose_aruco_data = []
        self._blob_data = []

        self._odom0_xy = (0.0, 0.0)

        self._sim_kinematics = None
        self._sim_ultrasonic = None

        self._client_lock = threading.Lock()
        self._msp_io_lock = threading.Lock()
        self._arming_rc_frame = None
        self._rc_timer_suspended = False

        self._consecutive_errors = 0
        self._error_threshold = 2
        self._simulator_alive = True
        self._on_death_callback = None
        # =================================================

        self._is_abort = False
        # =====================================

        self.initDrone()
    
    def connect(self, ip, port=5762, *, sim_port=8080):
        """Connect MSP on port and simulator RPC on sim_port; start control."""
        port, sim_port = int(port), int(sim_port)
        if not (1 <= port <= 65535 and 1 <= sim_port <= 65535):
            raise ValueError('MSP and simulator ports must be between 1 and 65535')
        self.__HOST = ip
        self.__SIM_PORT = sim_port
        self.__TCP_PORT = port
        self.__TCP_ADDRESS = (ip, port)
        self._simulator_alive = True
        self._consecutive_errors = 0
        self._reference_initialized = False
        self._latest_sample = None
        self._last_control_sample_t = None
        self._height_timer_started = False
        try:
            self.__tcp_transmitter = TCPTransmitter(self.__TCP_ADDRESS)
            self.__tcp_transmitter.connect()
            if not self.__tcp_transmitter.is_connect:
                raise ConnectionError(f'Could not connect to MSP at {ip}:{port}')
            self._control = MultirotorControl(self.__tcp_transmitter)
            time.sleep(2)
            self._althold_range_configured = self.add_range_for_althold()
            self._send_rc_frame(self._ARM_RESET_FRAME)
            time.sleep(0.5)
            self._client = SimClient(address=ip, port=sim_port)
            # Camera/range RPC must not delay the 55 Hz kinematics stream.
            self._kinematics_client = SimClient(address=ip, port=sim_port)
            self._sensors_client = SimClient(address=ip, port=sim_port)
            self.initDrone()
            period = 1 / self._control_config['kinematics_hz']
            self._rc_timer = LoopingTimer(period, self.transmit_rc_to_sim, name='rc_timer')
            self._sim_kinematics_timer = LoopingTimer(period, self.sim_kinematics_callback, name='kinematics')
            self._image_processing_timer = LoopingTimer(0.1, self._sensor_callback, name='sensors')
            # Compatibility timer attributes remain available. Control itself runs
            # once per fresh sample, with the shared cascade scheduling each loop.
            self._yaw_timer = LoopingTimer(1/25, self.yaw_callback, name='yaw')
            self._position_timer = LoopingTimer(1/self._control_config['position_control_hz'], self.position_callback, name='position')
            self._velocity_timer = LoopingTimer(1/self._control_config['velocity_control_hz'], self.velocity_callback, name='velocity')
            self._height_timer = LoopingTimer(period, self.height_callback, name='height')
            self.sim_kinematics_callback()
            if self._latest_sample is None:
                raise ConnectionError('No valid simulator kinematics')
            self._sim_kinematics_timer.start()
            self._rc_timer.start()
            self._image_processing_timer.start()
        except Exception:
            self._stop_all_timers()
            self._close_connections()
            raise

    def disconnect(self):
        """Disarm, stop every worker, and close all RPC/MSP connections."""
        self.disarmDrone()
        self._stop_all_timers()
        if hasattr(self, '_control'):
            try:
                self._send_rc_frame(self._current_rc_frame())
            except Exception as exc:
                logger.warning('Could not send final disarm: %s', exc)
        self._althold_flag = False
        self._height_timer_started = False
        self._close_connections()

    def _close_connections(self):
        for name in ('_client', '_kinematics_client', '_sensors_client'):
            client = getattr(self, name, None)
            if client is not None:
                try:
                    client.rpc_client.close()
                except Exception:
                    pass
                setattr(self, name, None)
        transmitter = getattr(self, '_HighLevelSimClient__tcp_transmitter', None)
        if transmitter is not None:
            transmitter.disconnect()

    def yaw_callback(self):
        """Compatibility callback; advance the cascade at most once per sample."""
        self._control_step()

    def position_callback(self):
        """Compatibility callback; advance the cascade at most once per sample."""
        self._control_step()

    def velocity_callback(self):
        """Compatibility callback; advance the cascade at most once per sample."""
        self._control_step()

    def height_callback(self):
        """Compatibility callback; advance the cascade at most once per sample."""
        self._control_step()

    def set_velocity_xy(self, vx: float, vy: float, frame: str = "base_link"):
        """Set XY speed in m/s; odom stays world-fixed while the drone turns."""
        if frame not in ('base_link', 'odom'):
            raise ValueError("frame must be 'odom' or 'base_link'")
        if not all(math.isfinite(v) for v in (vx, vy)):
            raise ValueError('Velocity must be finite')
        with self._control_lock:
            self._control_mode = 'velocity'
            self._velocity_frame = frame
            self._target_velocity = tuple(max(-self._max_velocity, min(self._max_velocity, v)) for v in (vx, vy))

    def set_position_mode(self):
        """Switch to position hold at the current location."""
        with self._control_lock:
            if self._control_mode != 'position':
                self._target_position = self._world_xy()
            self._control_mode = 'position'

    def set_velocity_mode(self):
        """Switch to velocity control with a zero-speed command."""
        self.set_velocity_xy(0.0, 0.0)

    def set_velocity_yaw(self, yaw_rate: float):
        """Set yaw rate command and switch yaw controller to rate mode."""
        print(f"\n[control] set yaw:{round(yaw_rate,2)}")
        self._yaw_mode = "velocity"
        '''max_yaw_rate = 1.5
        self._target_yaw_rate = max(-max_yaw_rate, min(max_yaw_rate, yaw_rate))'''
        yaw_pwm = int(vel_to_rc_signal(yaw_rate))
        r, p, _ = self._rpy_vel_data
        self._rpy_vel_data = (r, p, yaw_pwm)

    def set_yaw_position_mode(self):
        """Switch yaw controller to position mode."""
        self._yaw_mode = "position"

    def lock_motors(self):
        """Prevent motor command updates from control callbacks."""
        self._motors_locked = True

    def unlock_motors(self):
        """Allow motor command updates from control callbacks."""
        if self._motors_locked and hasattr(self, '_cascade'):
            with self._control_lock:
                self._cascade.reset(self._latest_sample)
                self._last_control_sample_t = None
        self._motors_locked = False
    
    def set_max_velocity(self, max_vel: float):
        """Set maximum horizontal velocity limit in m/s."""
        self._max_velocity = max(0.1, min(max_vel, 5.0))
        
        self._pid_pos_x.max_control = self._max_velocity
        self._pid_pos_y.max_control = self._max_velocity
        
        logger.info(f"Max velocity set to {self._max_velocity} m/s")
    
    def set_max_acceleration(self, max_accel: float):
        """Set maximum horizontal acceleration limit in m/s^2."""
        self._max_acceleration = max(0.5, min(max_accel, 10.0))
        logger.info(f"Max acceleration set to {self._max_acceleration} m/sВІ")
    
    def get_max_velocity(self) -> float:
        """Return configured maximum horizontal velocity."""
        return self._max_velocity
    
    def get_max_acceleration(self) -> float:
        """Return configured maximum horizontal acceleration."""
        return self._max_acceleration


    def set_direction_coefficients(self, roll: float = None, pitch: float = None, yaw: float = None):
        """Set sign coefficients for roll, pitch, and yaw channels."""
        if roll is not None:
            self._roll_direction = float(roll)
            logger.info(f"Roll direction set to {self._roll_direction}")
        if pitch is not None:
            self._pitch_direction = float(pitch)
            logger.info(f"Pitch direction set to {self._pitch_direction}")
        if yaw is not None:
            self._yaw_direction = float(yaw)
            logger.info(f"Yaw direction set to {self._yaw_direction}")

    def invert_roll(self):
        """Invert roll direction coefficient."""
        self._roll_direction *= -1
        logger.info(f"Roll direction inverted, now: {self._roll_direction}")

    def invert_pitch(self):
        """Invert pitch direction coefficient."""
        self._pitch_direction *= -1
        logger.info(f"Pitch direction inverted, now: {self._pitch_direction}")

    def invert_yaw(self):
        """Invert yaw direction coefficient."""
        self._yaw_direction *= -1
        logger.info(f"Yaw direction inverted, now: {self._yaw_direction}")

    def get_direction_coefficients(self) -> dict:
        """Return current direction coefficients as a dictionary."""
        return {
            "roll": self._roll_direction,
            "pitch": self._pitch_direction,
            "yaw": self._yaw_direction
        }


    def set_velocity_filter(self, alpha: float):
        """Set low-pass filter alpha for velocity estimation."""
        alpha = max(0.0, min(1.0, alpha))
        self._vel_filter_alpha = alpha
        self._vel_filter_initialized = False
        self._cascade.filter_initialized = False
        logger.info(f"Velocity filter alpha set to {alpha}")

    def get_velocity_filter_alpha(self) -> float:
        """Return current velocity filter alpha."""
        return self._vel_filter_alpha

    def reset_velocity_filter(self):
        """Reset velocity filter state variables."""
        self._vel_filter_initialized = False
        self._filtered_vx_world = 0.0
        self._filtered_vy_world = 0.0
        self._cascade.filter_initialized = False
        self._cascade.previous_body_velocity = None
        self._cascade.filtered_ax = self._cascade.filtered_ay = 0.0
        logger.info("Velocity filter reset")

    # ==============================================

    # ==============================================

    def go_to_xy(self, frame: str, x: float, y: float, max_speed: float = 0.5) -> bool:
        """Move to target XY coordinate in the selected reference frame."""
        '''self._pid_pos_x.reset()
        self._pid_pos_y.reset()
        self._pid_vel_pitch.reset()
        self._pid_vel_roll.reset()'''

        

        self._control_mode = "position"
        if self._yaw_mode == "velocity":
            self._yaw_mode = "position"
            self._target_yaw = self._get_yaw_cw()
        kin = self.get_sim_kinematics()
        if kin is None:
            logger.error("No kinematics data for go_to_xy")
            print("[control] go to xy failed")
            return False

        cx = sim_to_api_distance(kin["location"][0])
        cy = sim_to_api_distance(kin["location"][1])

        if frame == "odom":
            if self._odom0_xy != (0.0, 0.0):
                x0, y0 = self._odom0_xy
                self._target_position = (x0 + x, y0 + y)
            else:
                self._target_position = (x, y)

            print(f"\n[control] go to xy [odom] x:{round(self._target_position[0],2)} y:{round(self._target_position[1],2)}")
        elif frame == "base_link":
            #
            # [ cos_yaw   sin_yaw ] [x_body]
            # [-sin_yaw   cos_yaw ] [y_body]
            yaw = self._get_yaw_cw()
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)
            
            target_x_world = cx + x * cos_yaw + y * sin_yaw
            target_y_world = cy - x * sin_yaw + y * cos_yaw
            self._target_position = (target_x_world, target_y_world)

            print(f"\n[control] go to xy [local] x:{round(x,2)} y:{round(y,2)}| [world] x:{round(self._target_position[0],2)} y:{round(self._target_position[1],2)}")
        else:
            raise ValueError("frame must be 'odom' or 'base_link'")

        self._pid_pos_x.max_control = min(max_speed, self._max_velocity,
                                         self._cascade.pid_configs['pid_pos_x']['max_control'])
        self._pid_pos_y.max_control = min(max_speed, self._max_velocity,
                                         self._cascade.pid_configs['pid_pos_y']['max_control'])
        
        #self.unlock_motors()


        tx, ty = self._target_position
        dist = math.hypot(tx - cx, ty - cy)
        
        timeout = 5.0 + (3 * dist / (self._max_velocity ))
        start_time = time.monotonic()
        prev_dist = None

        while time.monotonic() - start_time < timeout:
            if not self._simulator_alive:
                logger.warning("Simulator died during go_to_xy")
                print("[control] go to xy failed")
                return False

            if self._is_abort:
                self._is_abort = False
                logger.info("go_to_xy: aborted")
                print("[control] go to xy aborted")
                return False

            tx, ty = self._target_position
            dist = math.hypot(tx - cx, ty - cy)
            velocity = math.hypot(self._filtered_vx_world, self._filtered_vy_world)
            if dist < 0.1: # and velocity < 0.1:
                logger.info(f"Reached target: {x}, {y}")
                print(f"[control] go to xy succeed x:{round(cx,2)} y:{round(cy,2)} velocity:{round(velocity,2)}")
                return True
            
            kin = self.get_sim_kinematics()
            if kin is not None:
                cx = sim_to_api_distance(kin["location"][0])
                cy = sim_to_api_distance(kin["location"][1])

            time.sleep(0.05)

        logger.warning(f"go_to_xy timeout: target={x}, {y}")
        print(f"[control] go to xy timeout x:{round(cx,2)} y:{round(cy,2)} velocity:{round(velocity,2)}")
        return False
    
    def gotoXYodom(self, x: float, y: float) -> bool:
        """Compatibility wrapper for go_to_xy in odom frame."""
        return self.go_to_xy("odom", x, y)
    
    def gotoXYdrone(self, x: float, y: float) -> bool:
        """Compatibility wrapper for go_to_xy in base_link frame."""
        return self.go_to_xy("base_link", x, y)
    
    def setYaw(self, yaw: float) -> bool:
        """Rotate to absolute yaw angle in blocking mode."""
        print(f"\n[control] set yaw: {yaw}")
        self._is_abort = False

        self._yaw_mode = "position"

        #self._pid_yaw_pos.reset()
        #self._pid_yaw_rate.reset()

        goal = self._wrap_pi(yaw)

        r, p, _ = self._rpy_vel_data

        timeout = 10.0
        start_time = time.monotonic()
        tol = 0.025
        self._target_yaw = goal

        while time.monotonic() - start_time < timeout:
            if not self._simulator_alive:
                return False

            if self._is_abort:
                self._is_abort = False
                logger.info("setYaw: aborted")
                print(f"[control] set yaw aborted")
                return False

            current = self._get_yaw_cw()
            error = self._wrap_pi(goal - current)

            if abs(error) < tol:
                r, p, _ = self._rpy_vel_data
                self._rpy_vel_data = (r, p, 1500)
                print(f"[control] set yaw succeed")
                return True

            time.sleep(0.05)

        r, p, _ = self._rpy_vel_data
        self._rpy_vel_data = (r, p, 1500)
        print(f"[control] set yaw failed")
        return False
    
    # =========================================================
    # =========================================================
    
    def _world_xy(self, kin=None) -> tuple[float, float]:
        """Return current world-frame XY coordinates."""
        if kin is None:
            kin = self.get_sim_kinematics()
        x_w = sim_to_api_distance(kin["location"][0])
        y_w = sim_to_api_distance(kin["location"][1])
        return x_w, y_w
    
    def _odom_xy(self, kin=None) -> tuple[float, float]:
        """Return current odometry-frame XY coordinates."""
        x_w, y_w = self._world_xy(kin)
        if self._odom0_xy == (0.0, 0.0):
            return x_w, y_w
        x0, y0 = self._odom0_xy
        return x_w - x0, y_w - y0
    
    def getHeightRange(self):
        """Return current altitude estimate."""
        with self._client_lock:
            return self._altitude
    
    def getHeightBarometer(self):
        """Return current barometric altitude estimate."""
        with self._client_lock:
            return self._altitude
    
    def getArm(self):
        """Return current arm state flag."""
        return self._armed_flag
    
    def setZeroOdomOpticflow(self) -> bool:
        """Reset odometry origin to current world position."""
        kin = self.get_sim_kinematics()
        if kin is None:
            raise RuntimeError("РќРµС‚ РєСЌС€Р° РєРёРЅРµРјР°С‚РёРєРё")
        self._odom0_xy = self._world_xy(kin)
        return True

    def get_laser_scan(
        self,
        angle_min: float = -np.pi,
        angle_max: float = np.pi,
        range_min: float = 0.1,
        range_max: float = 30.0,
        num_ranges: int = 360,
        is_clear: bool = True,
        range_error: float = 0.0,
    ) -> np.ndarray:
        """Return lidar scan data proxied from the low-level simulator client."""
        with self._client_lock:
            if not hasattr(self, "_client") or self._client is None:
                raise RuntimeError("Low-level simulator client is not connected")

            scan = self._client.get_laser_scan(
                angle_min=angle_min,
                angle_max=angle_max,
                range_min=range_min,
                range_max=range_max,
                num_ranges=num_ranges,
                is_clear=is_clear,
                range_error=range_error,
            )

        return np.asarray(scan, dtype=float)

    def get_lidar_point_cloud(
        self,
        angle_below_zero: float = np.deg2rad(15.0),
        angle_above_zero: float = np.deg2rad(15.0),
        range_min: float = 0.1,
        range_max: float = 100.0,
        channel_count: int = 16,
        points_per_channel: int = 512,
    ) -> PointCloud:
        """Acquire one organized sensor-local 3D lidar frame."""
        with self._client_lock:
            if not hasattr(self, "_client") or self._client is None:
                raise RuntimeError("Low-level simulator client is not connected")
            return self._client.get_lidar_point_cloud(
                angle_below_zero=angle_below_zero,
                angle_above_zero=angle_above_zero,
                range_min=range_min,
                range_max=range_max,
                channel_count=channel_count,
                points_per_channel=points_per_channel,
            )

    def getLidarScan(
        self,
        angle_min: float = -np.pi,
        angle_max: float = np.pi,
        range_min: float = 0.1,
        range_max: float = 30.0,
        num_ranges: int = 360,
        is_clear: bool = True,
        range_error: float = 0.0,
    ) -> np.ndarray:
        """Compatibility wrapper around get_laser_scan."""
        return self.get_laser_scan(
            angle_min=angle_min,
            angle_max=angle_max,
            range_min=range_min,
            range_max=range_max,
            num_ranges=num_ranges,
            is_clear=is_clear,
            range_error=range_error,
        )
    
    def getUltrasonic(self):
        """Return cached ultrasonic range value."""
        with self._client_lock:
            print(f"[control] ultrasinc: {round(self._sim_ultrasonic,3)}")
            return self._sim_ultrasonic
        
    def getUltrasonicById(self, sonic_id: int):
        """Query ultrasonic range for selected sensor identifier."""
        with self._client_lock:
            sonic_data = self._client.get_range_data(
                                            rangefinder_id=sonic_id,
                                            range_min=0.15,
                                            range_max=4,
                                            is_clear=True,
                                            range_error=0.0003
                                            ) 
            print(f"[control] ultrasinc: {round(sonic_data,3)}")
            return sonic_data

    def getRPY(self):
        """Return current roll, pitch, and yaw in radians."""
        kin = self.get_sim_kinematics()
        qx, qy, qz, qw = kin["orientation"]
        roll, pitch, yaw = quat2euler((qw, qx, qy, qz), axes='sxyz')
        return [roll, pitch, yaw]
    
    def getOdomOpticflow(self):
        """Return odometry XY and current altitude."""
        kin = self.get_sim_kinematics()
        x, y = self._world_xy(kin)
        last_x, last_y = self._odom0_xy
        odom_x = x - last_x
        odom_y = y - last_y
        return [odom_x, odom_y, self._altitude]
    
    @staticmethod
    def _wrap_pi(a: float) -> float:
        """Normalize angle to the [-pi, pi) interval."""
        return (a + math.pi) % (2 * math.pi) - math.pi
    
    def _get_yaw_cw(self) -> float:
        """Return current yaw in clockwise-positive convention."""
        kin = self.get_sim_kinematics()
        if kin is None:
            return 0.0
        qx, qy, qz, qw = kin["orientation"]
        _, _, yaw_ccw = quat2euler((qw, qx, qy, qz), axes='sxyz')
        return -yaw_ccw
    
    def _get_height(self) -> float:
        """Return current altitude as float."""
        return float(self._altitude)
    
    def _clamp_h(self, h: float, lo: float, hi: float) -> float:
        """Clamp altitude value between lower and upper bounds."""
        return max(lo, min(h, hi))
    
    def _sleep_until(self, t_deadline: float, period: float) -> bool:
        """Sleep for control period while respecting deadline."""
        now = time.monotonic()
        if now >= t_deadline:
            return False
        time.sleep(max(0.0, period - (time.monotonic() - now)))
        return True
    
    def takeoff(self) -> bool:
        """Take off to the drone preset's height above the launch ground."""
        if not self._armed_flag:
            logger.warning('takeoff requested while disarmed')
            return False
        if not self._althold_flag and not self.altholdOn():
            return False
        self.set_position_mode()
        return self.setHeight(self._takeoff_height)

    def boarding(self) -> bool:
        """Descend in steps, then use the calibrated throttle landing ramp."""
        if not self._armed_flag:
            return True
        if self._get_height() > 0.3 and not self.setHeight(0.2):
            return False
        self._descent_goal = None
        config = self._control_config
        start = previous = time.monotonic()
        grounded_since = None
        self._landing_throttle = float(config['throttle_landing_start_rc'])
        try:
            while time.monotonic() - start < config['landing_seconds']:
                if self._is_abort or not self._simulator_alive:
                    return False
                now = time.monotonic()
                dt, previous = now - previous, now
                sample = self._latest_sample
                if sample is None or now - sample.t > 0.5:
                    return False
                vz = sample.vz_world
                if vz is None:
                    return False
                rate = config['throttle_landing_rc_per_second']
                if vz < -config['throttle_landing_speed_limit']:
                    self._landing_throttle = min(config['throttle_landing_start_rc'],
                                                 self._landing_throttle + 4 * rate * dt)
                else:
                    self._landing_throttle = max(config['throttle_landing_end_rc'],
                                                 self._landing_throttle - rate * dt)
                grounded = (self._get_height() <= config['throttle_landing_ground_tolerance']
                            and abs(vz) <= config['throttle_landing_ground_speed']
                            and self._landing_throttle <= config['throttle_landing_end_rc'] + 50)
                if grounded:
                    grounded_since = grounded_since or now
                    if now - grounded_since >= config['throttle_landing_ground_hold_seconds']:
                        self.disarmDrone()
                        self.altholdOff()
                        return True
                else:
                    grounded_since = None
                time.sleep(1 / config['kinematics_hz'])
            return False
        finally:
            self._landing_throttle = None
            if self._armed_flag and self._simulator_alive:
                self.set_target_height(max(0.0, self._get_height()))

    def setHeight(self, target_height: float) -> bool:
        """Move to launch-relative height and wait for measured arrival."""
        if not self._armed_flag or not self._simulator_alive:
            return False
        if not self._althold_flag and not self.altholdOn():
            return False
        self._is_abort = False
        self.set_position_mode()
        start_height = self._get_height()
        self.set_target_height(target_height)
        timeout = max(15.0, 10.0 * abs(target_height - start_height) + 5.0)
        deadline = time.monotonic() + timeout
        stable_since = None
        while time.monotonic() < deadline:
            if self._is_abort or not self._simulator_alive:
                self._is_abort = False
                return False
            speed = abs(self._latest_sample.vz_world or 0.0) if self._latest_sample else float('inf')
            if abs(self._get_height() - target_height) <= 0.05 and speed <= 0.10:
                stable_since = stable_since or time.monotonic()
                if time.monotonic() - stable_since >= 0.6:
                    return True
            else:
                stable_since = None
            time.sleep(0.02)
        return False

    def set_camera_id(self, new_id: int):
        """Set active simulator camera identifier."""
        with self._client_lock:
            self.camera_id = new_id
    
    def sim_kinematics_callback(self):
        """Acquire one fresh world-frame velocity sample and advance control."""
        if not self._simulator_alive:
            return
        try:
            kin = self._kinematics_client.get_kinametics_data()
            sample = sample_from_kinematics(kin, time.monotonic())
            with self._control_lock:
                self._sim_kinematics = kin
                self._latest_sample = sample
                if not self._reference_initialized:
                    self._z_bias = sample.z
                    self._target_position = (sample.x, sample.y)
                    self._target_yaw = sample.yaw
                    self._target_height = 0.0
                    self._cascade.z_bias = sample.z
                    self._cascade.reset(sample)
                    self._reference_initialized = True
                self._altitude = sample.z - self._z_bias
            self._consecutive_errors = 0
            self._control_step()
        except Exception as exc:
            self._consecutive_errors += 1
            logger.warning('Kinematics update failed: %s', exc)
            self._check_simulator_death()

    def _sensor_callback(self):
        """Refresh camera/range independently from flight-control RPC."""
        if not self._simulator_alive:
            return
        try:
            self._sim_img = self._sensors_client.get_camera_capture(camera_id=self.camera_id)
            self._sim_ultrasonic = self._sensors_client.get_range_data(
                rangefinder_id=0, range_min=0.15, range_max=4,
                is_clear=True, range_error=0.0003) * 100
            self.image_processing_callback()
        except Exception as exc:
            logger.warning('Sensor update failed: %s', exc)

    def _control_step(self):
        if not self._simulator_alive or self._motors_locked:
            return
        with self._control_lock:
            sample = self._latest_sample
            if sample is None or sample.t == self._last_control_sample_t:
                return
            self._update_descent(sample)
            config = self._cascade.config
            config['max_xy_speed'] = self._max_velocity
            config['max_xy_acceleration'] = self._max_acceleration
            config['velocity_filter_alpha'] = self._vel_filter_alpha
            config['direction'] = self.get_direction_coefficients()
            self._cascade.z_bias = self._z_bias
            self._cascade.base_throttle_rc = self._base_throttle_hover
            velocity = self._target_velocity
            if self._control_mode == 'velocity' and self._velocity_frame == 'odom':
                vx, vy = velocity
                cs, sn = math.cos(sample.yaw), math.sin(sample.yaw)
                velocity = (vx * cs - vy * sn, vx * sn + vy * cs)
            target = {'height': self._target_height, 'height_relative': True,
                      'yaw': self._target_yaw, 'position': self._target_position,
                      'velocity': velocity, 'height_enabled': self._althold_flag and self._landing_throttle is None,
                      'yaw_enabled': self._yaw_mode == 'position'}
            result = self._cascade.step(sample, self._control_mode, target)
            self._last_control_sample_t = sample.t
            yaw_pwm = result['rc_yaw'] if self._yaw_mode == 'position' else self._rpy_vel_data[2]
            self._rpy_vel_data = (result['rc_roll'], result['rc_pitch'], yaw_pwm)
            if self._landing_throttle is not None:
                self._throttle_data = int(self._landing_throttle)
            elif self._althold_flag:
                self._throttle_data = result['rc_throttle']
            self._filtered_vx_world = self._cascade.filtered_vx
            self._filtered_vy_world = self._cascade.filtered_vy
            self._prev_x, self._prev_y = sample.x, sample.y
            self._control_telemetry = result

    def get_control_telemetry(self) -> dict:
        """Return the last cascade outputs, errors, speeds and accelerations."""
        with self._control_lock:
            return dict(self._control_telemetry)

    def _update_descent(self, sample):
        """Advance 5 cm steps using measured progress and vertical speed."""
        if self._descent_goal is None:
            return
        c = self._control_config
        actual = sample.z - self._z_bias
        goal = self._descent_goal
        if actual <= goal + c['height_descent_reach_tolerance']:
            self._target_height = goal
            self._descent_goal = None
            return
        now = sample.t
        first = self._descent_command is None
        elapsed = 0 if first else now - self._descent_step_started
        speed = abs(sample.vz_world or 0.0)
        progressed = (not first and actual <= self._descent_step_origin - c['height_descent_step'] * 0.6)
        reached = not first and abs(actual-self._descent_command) <= c['height_descent_reach_tolerance']
        ready = first or (elapsed >= c['height_descent_settle_seconds'] and
                          (reached or progressed) and speed <= c['height_descent_max_speed'])
        stalled = not first and elapsed >= c['height_descent_step_timeout_seconds'] and speed <= c['height_descent_max_speed']
        if ready or stalled:
            command = max(goal, actual - c['height_descent_step']) if first else min(
                self._descent_command, max(goal - c['height_descent_max_target_undershoot'],
                    actual - c['height_descent_max_command_gap'],
                    min(actual - c['height_descent_step'], self._descent_command - c['height_descent_step'])))
            self._target_height = command
            self._descent_command = command
            self._descent_step_started = now
            self._descent_step_origin = actual
            self._pid_height.reset()

    def get_sim_kinematics(self):
        """Return cached simulator kinematics dictionary."""
        return self._sim_kinematics
    
    def transmit_rc_to_sim(self):
        """Send current RC command packet to flight controller."""
        if self._rc_timer_suspended or not self._simulator_alive:
            return

        try:
            with self._msp_io_lock:
                if self._arming_rc_frame is not None:
                    raw_rc = self._arming_rc_frame
                else:
                    raw_rc = self._current_rc_frame()

                self._control.send_RAW_RC(self.clamp_rc_list(raw_rc))
                self._control.receive_msg()
        except Exception as exc:
            logger.warning("Could not send RC frame: %s", exc)

    def _current_rc_frame(self) -> list:
        """Build the normal seven-channel RC frame from current control state."""
        roll, pitch, yaw = self._rpy_vel_data
        return [
            roll,
            pitch,
            self._throttle_data,
            yaw,
            self._arm_data,
            self._fliyng_mode,
            self._nav_mode,
        ]
    
    def setVelXY(self, x, y):
        """Deprecated wrapper around set_velocity_xy."""
        self.set_velocity_xy(x, y, frame="odom")
    
    def setVelXYYaw(self, x, y, yaw):
        """Deprecated wrapper for XY velocity command with ignored yaw."""
        logger.warning("setVelXYYaw is deprecated, use set_velocity_xy + setYaw")
        self.set_velocity_xy(x, y, frame="base_link")
    
    def armDrone(self):
        """Arm with the raw-RC sequence verified by ``arm_drone_debug.py``."""
        if self._armed_flag:
            return True

        print("[info] Sending ARM RC sequence...")
        self._target_height_vel = 0.0
        self._rc_timer_suspended = True

        try:
            # Keep the same MSP traffic as arm_drone_debug.py.  The normal
            # RC timer is paused, rather than stopped, because LoopingTimer
            # instances cannot be started again after stop().
            self._send_arm_frame(self._ARM_RESET_FRAME, "reset/disarm")
            time.sleep(self._ARM_SETTLE_SECONDS)

            self._send_arm_frame(self._ARM_SWITCH_FRAME, "arm switch on")

            # INAV must immediately receive neutral roll/pitch/yaw with the
            # ARM switch high; this is the sequence used by input_driver.py.
            self._rpy_vel_data = (1500, 1500, 1500)
            self._throttle_data = 1000
            self._arm_data = 2000
            self._fliyng_mode = 1000
            self._nav_mode = 1000
            self._send_arm_frame(self._ARMED_NEUTRAL_FRAME, "armed neutral")

            # The debug script transmits this frame synchronously at 20 Hz.
            # Do the same here: the background RC timer may be stopped when
            # the simulator RPC becomes unavailable, but that must not cut
            # short the MSP arming sequence.
            self._hold_rc_frame(
                self._ARMED_NEUTRAL_FRAME,
                duration=self._ARM_SETTLE_SECONDS,
                frequency=self._ARM_FRAME_FREQUENCY,
            )
            self._armed_flag = True
            # Height control is allowed as soon as the ARM sequence has
            # completed, including when the caller intentionally keeps ANGLE.
            self.unlock_motors()
        except Exception:
            self._armed_flag = False
            self._arm_data = 1000
            raise
        finally:
            self._arming_rc_frame = None
            self._rc_timer_suspended = False

        print("[info] ARM RC sequence sent")
        return True

    def _send_rc_frame(self, raw_rc: Iterable):
        """Synchronously send one RC frame and return its MSP response."""
        with self._msp_io_lock:
            if not self._control.send_RAW_RC(self.clamp_rc_list(raw_rc)):
                raise ConnectionError("MSP did not accept the RC frame")
            response = self._control.receive_msg()
            if response is None:
                raise ConnectionError("MSP did not return a response to the RC frame")
            return response

    def _send_arm_frame(self, raw_rc: Iterable, label: str) -> None:
        """Send a key arming frame and print its MSP response metadata."""
        print(f"[rc] {label}: {list(raw_rc)}")
        response = self._send_rc_frame(raw_rc)
        print(
            "[msp] response: "
            f"code={response.get('code')}, "
            f"crc_error={response.get('crcError')}, "
            f"packet_error={response.get('packet_error')}"
        )

    def _hold_rc_frame(self, raw_rc: Iterable, *, duration: float, frequency: float) -> None:
        """Synchronously retain an RC frame for a fixed interval.

        This deliberately does not use ``transmit_rc_to_sim``: that callback
        is coupled to simulator-RPC liveness, while arming is an MSP-only
        operation and must complete even when RPC data is temporarily absent.
        """
        if duration < 0:
            raise ValueError("duration must not be negative")
        if frequency <= 0:
            raise ValueError("frequency must be positive")

        deadline = time.monotonic() + duration
        period = 1.0 / frequency
        while time.monotonic() < deadline:
            self._send_rc_frame(raw_rc)
            time.sleep(period)

    def disarmDrone(self):
        """Disarm drone and reset throttle outputs."""
        self._arming_rc_frame = None
        self._armed_flag = False
        self._arm_data = 1000
        self._throttle_data = 1000
        self.lock_motors()
        
    
    def initDrone(self):
        """Initialize RC channel defaults."""
        self._rpy_vel_data = (1500, 1500, 1500)
        self._throttle_data = 1000
        self._arm_data = 1000
        self._fliyng_mode = 1000
        self._nav_mode = 1000
    
    def posholdOn(self):
        """Enable POSHOLD-related channel configuration."""
        self._poshold_flag = True
        self._althold_flag = False
        self._nav_mode = 1500
        self._base_throttle_hover = getattr(self, "_calibrated_base_throttle", 1500)
        self.unlock_motors()
        

    def posholdOff(self):
        """Disable POSHOLD-related channel configuration."""
        self._poshold_flag = False
        self._nav_mode = 1000
        self._throttle_data = 1000
        self._base_throttle_hover = 1000
        self._pid_height.reset()
        self.lock_motors()

    def _read_msp_mode_ranges(self) -> None:
        """Request and decode the current MSP mode-range table."""
        with self._msp_io_lock:
            if not self._control.send_RAW_msg(MSPCodes["MSP_MODE_RANGES"], data=[]):
                raise ConnectionError("MSP_MODE_RANGES was not sent")
            response = self._control.receive_msg()

            if response is None:
                raise ConnectionError("MSP_MODE_RANGES did not return a response")
            if response.get("crcError") or response.get("packet_error"):
                raise RuntimeError("MSP_MODE_RANGES returned an invalid response")

            result = self._control.process_recv_data(response)
            if result is None or result < 0:
                raise RuntimeError(f"MSP_MODE_RANGES response was not decoded: {result}")

    def _write_msp_configuration(self, message_name: str, data: list) -> None:
        """Send one MSP configuration write and consume its ACK."""
        with self._msp_io_lock:
            if not self._control.send_RAW_msg(MSPCodes[message_name], data=data):
                raise ConnectionError(f"MSP command {message_name} was not sent")
            response = self._control.receive_msg()
            if response is None:
                raise ConnectionError(f"MSP command {message_name} did not return an ACK")
            if response.get("crcError") or response.get("packet_error"):
                raise RuntimeError(f"MSP command {message_name} returned an invalid ACK")
            result = self._control.process_recv_data(response)
            if result is None or result < 0:
                raise RuntimeError(f"MSP command {message_name} ACK was not decoded: {result}")

    def add_range_for_althold(
        self,
        mode_id: int = _ALTHOLD_MODE_ID,
        channel_index: int = _ALTHOLD_AUX_CHANNEL_INDEX,
        range_start: int = _ALTHOLD_RANGE_START,
        range_end: int = _ALTHOLD_RANGE_END,
    ) -> bool:
        """Configure NAV ALTHOLD on AUX3 for the current controller session.

        ``MSP_SET_MODE_RANGE`` updates the active INAV configuration
        immediately.  Do not follow it with ``MSP_EEPROM_WRITE``: the
        simulator's controller does not persist this write reliably and its
        MSP stream becomes unusable for the subsequent ARM sequence.  The
        range is therefore installed and verified again on every connection.
        """
        def find_mode_ranges():
            return [
                (index, mode_range)
                for index, mode_range in enumerate(self._control.MODE_RANGES)
                if mode_range["id"] == mode_id
                and mode_range["auxChannelIndex"] == channel_index
            ]

        try:
            self._read_msp_mode_ranges()
            if not self._control.MODE_RANGES:
                logger.warning("MSP_MODE_RANGES returned no configuration entries")
                return False

            existing = find_mode_ranges()
            if any(
                mode_range["range"]["start"] == range_start
                and mode_range["range"]["end"] == range_end
                for _, mode_range in existing
            ):
                logger.info("NAV ALTHOLD range is already configured on AUX3")
                return True

            empty_index = next(
                (
                    index
                    for index, mode_range in enumerate(self._control.MODE_RANGES)
                    if mode_range["id"] == 0
                    and mode_range["range"] == {"start": 900, "end": 900}
                ),
                None,
            )
            if empty_index is None:
                logger.warning("No free MSP mode-range entry is available for NAV ALTHOLD")
                return False

            payload = [
                empty_index,
                mode_id,
                channel_index,
                (range_start - 900) // 25,
                (range_end - 900) // 25,
            ]
            self._write_msp_configuration("MSP_SET_MODE_RANGE", payload)
            time.sleep(0.3)
            self._read_msp_mode_ranges()

            configured = any(
                mode_range["range"]["start"] == range_start
                and mode_range["range"]["end"] == range_end
                for _, mode_range in find_mode_ranges()
            )
            if configured:
                logger.info(
                    "NAV ALTHOLD range configured for this session: AUX3 = %s-%s",
                    range_start,
                    range_end,
                )
            else:
                logger.error("NAV ALTHOLD range was not present after MSP_SET_MODE_RANGE")
            return configured
        except Exception as exc:
            logger.error("Could not configure NAV ALTHOLD range: %s", exc)
            return False

    def altholdOn(self) -> bool:
        """Enable NAV ALTHOLD after arming and start the altitude controller."""
        if not self._armed_flag:
            logger.warning("NAV ALTHOLD was requested while the drone is disarmed")
            print("[warning] NAV ALTHOLD requires an armed drone")
            return False

        if not self._althold_range_configured:
            logger.warning("NAV ALTHOLD was requested but its MSP mode range is not configured")
            print("[warning] NAV ALTHOLD mode range is not configured")
            return False

        self._poshold_flag = False
        self._althold_flag = True
        self._nav_mode = self._ALTHOLD_AUX_VALUE
        self._base_throttle_hover = getattr(self, "_calibrated_base_throttle", 1500)
        self.unlock_motors()
        self._send_rc_frame(self._current_rc_frame())
        if not self._height_timer_started:
            if not hasattr(self, "_cascade"):
                self._height_timer.start()
            self._height_timer_started = True
        print("[info] NAV ALTHOLD enabled (AUX3 = 1300)")
        return True

    def altholdOff(self):
        """Disable NAV ALTHOLD channel value and lock motors."""
        self._althold_flag = False
        self._nav_mode = 1000
        self._throttle_data = 1000
        self._base_throttle_hover = 1000
        self._pid_height.reset()
        self.lock_motors()

    def clamp_rc(self, data):
        """Clamp single RC channel value to [1000, 2000]."""
        return max(min(data, 2000), 1000)

    def clamp_rc_list(self, data: Iterable):
        """Clamp each RC value in iterable to valid range."""
        return [self.clamp_rc(rc) for rc in data]

    def round_data(self, iterable):
        """Return iterator with rounded numeric values."""
        return map(lambda x: round(x, 3), iterable)

    def set_target_height(self, height):
        """Set height above the connection's launch ground; descend in 5 cm steps."""
        height = float(height)
        if not math.isfinite(height) or height < 0:
            raise ValueError('Height must be finite and nonnegative')
        with self._control_lock:
            self._descent_command = None
            self._descent_goal = height if height < self._get_height() else None
            if self._descent_goal is None:
                self._target_height = height
                self._pid_height.reset()
            elif self._latest_sample is not None:
                self._update_descent(self._latest_sample)

    def getImage(self):
        """Return latest simulator image frame."""
        with self._client_lock:
            return self._sim_img
    
    def getArucos(self):
        """Return latest detected ArUco marker data."""
        return self._aruco_data
    
    def getCameraPoseAruco(self):
        """Return latest ArUco-based camera pose estimates."""
        return self._camera_pose_aruco_data
    
    def getBlobs(self):
        """Return latest blob detection data."""
        return self._blob_data
    
    def getBlobsImage(self):
        """Return latest visualization image for blob detections."""
        return self._blob_img
    
    def getArucosImage(self):
        """Return latest visualization image for ArUco detections."""
        return self._aruco_img
    
    def image_processing_callback(self):
        """Run image preprocessing and update vision caches."""
        sim_img = self._sim_img.copy() if self._sim_img is not None else None
        camera_img = resolution_changes(sim_img, (320, 240))
        
        img_aruco = camera_img.copy() if self._sim_img is not None else None
        img_blob = camera_img.copy() if self._sim_img is not None else None
        
        if img_aruco is not None:
            self._aruco_data, self._camera_pose_aruco_data, aruco_img = process_aruco(img_aruco)
            if aruco_img is None:
                self._aruco_img = sim_img
            else:
                self._aruco_img = resolution_changes(aruco_img, (640, 480))
        
        if img_blob is not None:
            self._blob_data, blob_img = process_blob(img_blob)
            if blob_img is None:
                self._blob_img = sim_img
            else:
                self._blob_img = resolution_changes(blob_img, (640, 480))
    
    def setDiod(self,diod_id, r, g, b):
        """Set simulator LED color by diode identifier."""
        print(f"\n[control] set diod {diod_id} to ({r},{g},{b})")
        with self._client_lock:
            self._client.set_Diod(diod_id, float(r), float(g), float(b))

    
    def setShoot(self, time):
        """Trigger simulator action event."""
        with self._client_lock:
            return self._client.call_event_action()
    
    def set_simulator_death_callback(self, callback):
        """Set callback executed after simulator death detection."""
        self._on_death_callback = callback
    
    def is_simulator_alive(self):
        """Return simulator liveness flag."""
        return self._simulator_alive
    
    def _check_simulator_death(self):
        """Check error threshold and trigger death handling if needed."""
        if self._consecutive_errors >= self._error_threshold:
            self._simulator_alive = False
            logger.error(f"Simulator death detected! Consecutive errors: {self._consecutive_errors}")
            self._stop_all_timers()
            
            if self._on_death_callback is not None:
                logger.error("Calling simulator death callback...")
                try:
                    self._on_death_callback()
                except Exception as e:
                    logger.error(f"Error in death callback: {e}")
    
    def _stop_all_timers(self):
        """Stop every started timer without joining the caller's own thread."""
        current = threading.current_thread()
        for name in ('_rc_timer', '_sim_kinematics_timer', '_image_processing_timer',
                     '_yaw_timer', '_position_timer', '_velocity_timer', '_height_timer'):
            timer = getattr(self, name, None)
            if timer is not None:
                timer._stop_event.set()
                if timer._thread is not current and timer._thread.is_alive():
                    timer._thread.join(timeout=3)

    def abort(self):
        """Abort active blocking operation and freeze current XY target."""
        with self._client_lock:
            self._is_abort = True

            kin = self.get_sim_kinematics()
            if kin is not None:
                cx = sim_to_api_distance(kin["location"][0])
                cy = sim_to_api_distance(kin["location"][1])
                self._target_position = (cx, cy)
                self._control_mode = "position"

    def stop_go_to_xy(self):
        """Stop active go_to_xy command and hold current position."""
        self.abort()

    def stopGoToXY(self):
        """Compatibility wrapper around stop_go_to_xy."""
        self.stop_go_to_xy()


# Both names are supported; the historical class name remains unchanged.
HighLevelClient = HighLevelSimClient
