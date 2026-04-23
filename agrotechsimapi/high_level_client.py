"""High-level simulator client with cascaded PID control loops."""
from inavmspapi import MultirotorControl
from inavmspapi.transmitter import TCPTransmitter
from inavmspapi.msp_codes import MSPCodes
from agrotechsimapi.client import SimClient

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
    ):
        """Initialize client state, PID controllers, and runtime flags."""
        self.camera_id = 0

        preset_pid_setup = get_drone_pid_setup(drone_name)
        self._takeoff_height = get_drone_takeoff_height(drone_name)

        pid_pos_x_config = self._resolve_pid_config(preset_pid_setup["pid_pos_x"], pid_pos_x, "pid_pos_x")
        pid_pos_y_config = self._resolve_pid_config(preset_pid_setup["pid_pos_y"], pid_pos_y, "pid_pos_y")
        pid_vel_pitch_config = self._resolve_pid_config(
            preset_pid_setup["pid_vel_pitch"], pid_vel_pitch, "pid_vel_pitch"
        )
        pid_vel_roll_config = self._resolve_pid_config(
            preset_pid_setup["pid_vel_roll"], pid_vel_roll, "pid_vel_roll"
        )
        pid_yaw_config = self._resolve_pid_config(preset_pid_setup["pid_yaw"], pid_yaw, "pid_yaw")
        pid_height_config = self._resolve_pid_config(
            preset_pid_setup["pid_height"], pid_height, "pid_height"
        )

        self._pid_pos_x = PID(**pid_pos_x_config)
        self._pid_pos_y = PID(**pid_pos_y_config)

        self._pid_vel_pitch = PID(**pid_vel_pitch_config)
        self._pid_vel_roll = PID(**pid_vel_roll_config)


        #self._pid_yaw_rate = PID(kp=4.7, ki=0.0, kd=2.0, max_control=1.0, i_limit=None)

        self._pid_yaw = PID(**pid_yaw_config)
        self._pid_height = PID(**pid_height_config)
        self._base_throttle_hover = 1000
        self._max_throttle = 1675
        self._min_throttle = 1470

        self._roll_direction = -1.0
        self._pitch_direction = 1.0
        self._yaw_direction = 1.0
        # ====================================================================

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

        self._vel_filter_alpha = 0.82
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

        self._consecutive_errors = 0
        self._error_threshold = 2
        self._simulator_alive = True
        self._on_death_callback = None
        # =================================================

        self._is_abort = False
        # =====================================

        print("process started")
    
    def connect(self, ip, port):
        """Connect to simulator services, initialize clients, and start timers."""
        self.__HOST = ip
        self.__SIM_PORT = 8080
        self.__TCP_PORT = 5762
        self.__TCP_ADDRESS = (self.__HOST, self.__TCP_PORT)

        self.__tcp_transmitter = TCPTransmitter(self.__TCP_ADDRESS)
        self.__tcp_transmitter.connect()
        self._control = MultirotorControl(self.__tcp_transmitter)
        time.sleep(2)
        print("[info] Adding ALTHOLD range... ")
        self.add_range_for_althold()

        print("[info] Resetting MSP arming flags... ")
        try:
            self._control.send_RAW_RC([1000, 1000, 1000, 1000, 1000, 1000, 1000])
            time.sleep(0.5)
        except Exception as e:
            logger.warning(f"Could not reset MSP flags: {e}")

        with self._client_lock:
            self._client = SimClient(address=self.__HOST, port=self.__SIM_PORT)

        self._rc_timer = LoopingTimer(interval=1/50, callback=self.transmit_rc_to_sim, name="rc_timer")

        self._sim_kinematics_timer = LoopingTimer(interval=1/50, callback=self.sim_kinematics_callback, name="sim_kinematics_timer")
        self._image_processing_timer = LoopingTimer(interval=1/10, callback=self.image_processing_callback, name="image_processing_timer")

        self._yaw_timer = LoopingTimer(interval=1/25, callback=self.yaw_callback, name="yaw_timer")

        self._position_timer = LoopingTimer(interval=1/20, callback=self.position_callback, name="position_timer")
        self._velocity_timer = LoopingTimer(interval=1/50, callback=self.velocity_callback, name="velocity_timer")

        self._height_timer = LoopingTimer(interval=1/50, callback=self.height_callback, name="height_timer")

        self.sim_kinematics_callback()
        self.initDrone()

        time.sleep(0.5)
        self._z_bias = self._client.get_kinametics_data()["location"][2]
        print(f"[info] z_bias: {self._z_bias:.2f}")

        self._sim_kinematics_timer.start()
        self._rc_timer.start()
        self._image_processing_timer.start()
        self._yaw_timer.start()
        self._position_timer.start()
        self._velocity_timer.start()


        time.sleep(1)
        time.sleep(1)


    
    def disconnect(self):
        """Safely disarm, stop timers, and close transmitter connection."""
        print("[info] Disconect")

        print("[info] Disarming drone before disconnect... ")
        self.disarmDrone()
        self.posholdOff()

        time.sleep(0.5)

        self._sim_kinematics_timer.stop()
        self._rc_timer.stop()

        self._image_processing_timer.stop()

        self._yaw_timer.stop()

        self._position_timer.stop()
        self._velocity_timer.stop()

        self._height_timer.stop()

        if hasattr(self, '_tcp_transmitter') and self._tcp_transmitter:
            try:
                self._tcp_transmitter.disconnect()
            except Exception as e:
                logger.warning(f"Error disconnecting TCP: {e}")
    
    # =========================================================
    # =========================================================
    
    def yaw_callback(self):
        """Update yaw control loop and write yaw PWM output."""
        if not self._simulator_alive:
            return

        try:
            current_yaw = self._get_yaw_cw()
            error = self._target_yaw - current_yaw
            error = self._wrap_pi(error)
            
            self._pid_yaw.update_control(error)
            yaw_rate = self._pid_yaw.get_control()

            yaw_pwm = vel_to_rc_signal(yaw_rate * self._yaw_direction)
            if self._yaw_mode == "position":
                r, p, _ = self._rpy_vel_data
                self._rpy_vel_data = (r, p, yaw_pwm)

        except Exception as e:
            logger.warning(f"Error in yaw_callback: {e}")

    def position_callback(self):
        """Outer loop: position error to target body-frame velocity."""
        if not self._simulator_alive:# or self._motors_locked:
            return

        try:
            kin = self.get_sim_kinematics()
            if kin is None:
                return

            x_w = sim_to_api_distance(kin["location"][0])
            y_w = sim_to_api_distance(kin["location"][1])

            yaw = self._get_yaw_cw()
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)

            if self._control_mode == "position":
                tx, ty = self._target_position
                pos_error_x = tx - x_w
                pos_error_y = ty - y_w

                self._pid_pos_x.update_control(pos_error_x)
                self._pid_pos_y.update_control(pos_error_y)

                tvx_world = max(min(self._pid_pos_x.get_control(), self._max_velocity),-self._max_velocity)
                tvy_world = max(min(self._pid_pos_y.get_control(), self._max_velocity),-self._max_velocity)

                # [ cos_yaw   -sin_yaw ] [vx_world]
                # [ sin_yaw    cos_yaw ] [vy_world]
                tvx_body = tvx_world * cos_yaw - tvy_world * sin_yaw
                tvy_body = tvx_world * sin_yaw + tvy_world * cos_yaw

                self._target_velocity = (tvx_body, tvy_body)


        except Exception as e:
            logger.warning(f"Error in position_callback: {e}")

    def velocity_callback(self):
        """Inner loop: body-frame velocity error to roll and pitch PWM."""
        if not self._simulator_alive:# or self._motors_locked:

            return

        try:
            kin = self.get_sim_kinematics()
            if kin is None:

                return

            x_w = sim_to_api_distance(kin["location"][0])
            y_w = sim_to_api_distance(kin["location"][1])

            yaw = self._get_yaw_cw()
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)

            raw_vx_world = (x_w - self._prev_x) * 50
            raw_vy_world = (y_w - self._prev_y) * 50

            self._prev_x = x_w
            self._prev_y = y_w

            if not self._vel_filter_initialized:
                self._filtered_vx_world = raw_vx_world
                self._filtered_vy_world = raw_vy_world
                self._vel_filter_initialized = True
            else:
                alpha = self._vel_filter_alpha
                self._filtered_vx_world = alpha * raw_vx_world + (1 - alpha) * self._filtered_vx_world
                self._filtered_vy_world = alpha * raw_vy_world + (1 - alpha) * self._filtered_vy_world

            vx_body = self._filtered_vx_world * cos_yaw - self._filtered_vy_world * sin_yaw
            vy_body = self._filtered_vx_world * sin_yaw + self._filtered_vy_world * cos_yaw

            tvx_body, tvy_body = self._target_velocity

            dvx = tvx_body - self._prev_target_vx
            dvy = tvy_body - self._prev_target_vy
            dt = 0.02
            accel = math.hypot(dvx / dt, dvy / dt)

            if accel > self._max_acceleration:
                scale = self._max_acceleration / accel
                tvx_body = self._prev_target_vx + dvx * scale
                tvy_body = self._prev_target_vy + dvy * scale

            self._prev_target_vx = tvx_body
            self._prev_target_vy = tvy_body

            vel_error_x = tvx_body - vx_body
            vel_error_y = tvy_body - vy_body

            self._pid_vel_pitch.update_control(vel_error_x)
            self._pid_vel_roll.update_control(vel_error_y)

            feedforward_pitch = 0.0
            feedforward_roll = 0.0
            
            target_speed_x = abs(tvx_body)
            actual_speed_x = abs(vx_body)
            if target_speed_x > 0.05 and actual_speed_x < target_speed_x * 0.33:
                speed_ratio = actual_speed_x / target_speed_x if target_speed_x > 0 else 0
                feedforward_pitch = (1.0 - speed_ratio) * 1.33
            
            target_speed_y = abs(tvy_body)
            actual_speed_y = abs(vy_body)
            if target_speed_y > 0.05 and actual_speed_y < target_speed_y * 0.33:
                speed_ratio = actual_speed_y / target_speed_y if target_speed_y > 0 else 0
                feedforward_roll = (1.0 - speed_ratio) * 1.33

            pitch_control = self._pid_vel_pitch.get_control() * (1 + feedforward_pitch )
            roll_control = self._pid_vel_roll.get_control() *  (1 + feedforward_roll )
            
            pitch_pwm = int(vel_to_rc_signal(pitch_control * self._pitch_direction))
            roll_pwm = int(vel_to_rc_signal(roll_control * self._roll_direction))

            if self._control_mode == "position":
                _, _, y = self._rpy_vel_data
                self._rpy_vel_data = (roll_pwm, pitch_pwm, y)

        except Exception as e:
            logger.warning(f"Error in velocity_callback: {e}")

    def height_callback(self):
        """Height loop: altitude error to throttle command."""
        if not self._simulator_alive or self._motors_locked:
            return

        try:
            kin = self.get_sim_kinematics()
            if kin is not None:
                self._altitude = sim_to_api_distance(kin["location"][2])

            height_error = self._target_height - self._altitude + self._z_bias if self._target_height < 0.3 else self._target_height - self._altitude
            #print(f"height_error: {height_error}")

            self._pid_height.update_control(height_error)
            delta_throttle = int((self._pid_height.get_control() * 100))# // 2) * 2

            throttle_output = self._base_throttle_hover + delta_throttle

            throttle_output = max(self._min_throttle, min(throttle_output, self._max_throttle))

            self._throttle_data = throttle_output
            #print(f"height_error: {height_error} _throttle_data: {self._throttle_data}")

        except Exception as e:
            print(f"Error in height_callback: {e}")
            logger.warning(f"Error in height_callback: {e}")
        

    def set_velocity_xy(self, vx: float, vy: float, frame: str = "base_link"):
        """Set commanded XY velocity and switch to velocity mode."""
        print(f"\n[control] set velocity xy x:{round(vx,2)} y:{round(vy,2)}")
        self._control_mode = "velocity"

        pitch_pwm = int(vel_to_rc_signal(vx))
        roll_pwm = int(vel_to_rc_signal(vy))

        _, _, y = self._rpy_vel_data
        self._rpy_vel_data = (roll_pwm, pitch_pwm, y)

        '''if frame == "odom":
            yaw = self._get_yaw_cw()
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)
            # [ cos_yaw   -sin_yaw ] [vx_world]
            # [ sin_yaw    cos_yaw ] [vy_world]
            vx_body = vx * cos_yaw - vy * sin_yaw
            vy_body = vx * sin_yaw + vy * cos_yaw
        else:
            vx_body = vx
            vy_body = vy

        vx_body = max(-self._max_velocity * 2, min(self._max_velocity * 2, vx_body))
        vy_body = max(-self._max_velocity * 2, min(self._max_velocity * 2, vy_body))

        self._target_velocity = (vx_body, vy_body)'''


    def set_position_mode(self):
        """Switch horizontal controller to position mode."""
        self._control_mode = "position"

    def set_velocity_mode(self):
        """Switch horizontal controller to velocity mode."""
        self._control_mode = "velocity"

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
        self._motors_locked = False
    
    def set_max_velocity(self, max_vel: float):
        """Set maximum horizontal velocity limit in m/s."""
        self._max_velocity = max(0.1, min(max_vel, 5.0))
        
        self._pid_pos_x._max_control = self._max_velocity
        self._pid_pos_y._max_control = self._max_velocity
        
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
        logger.info(f"Velocity filter alpha set to {alpha}")

    def get_velocity_filter_alpha(self) -> float:
        """Return current velocity filter alpha."""
        return self._vel_filter_alpha

    def reset_velocity_filter(self):
        """Reset velocity filter state variables."""
        self._vel_filter_initialized = False
        self._filtered_vx_world = 0.0
        self._filtered_vy_world = 0.0
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
            self._target_yaw = self._wrap_pi(0)
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

        self._pid_pos_x._max_control = max_speed
        self._pid_pos_y._max_control = max_speed
        
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
            velocity = math.hypot(abs(self._prev_x), abs(self._prev_y))/50
            if dist < 0.15: # and velocity < 0.1:
                logger.info(f"Reached target: {x}, {y}")
                print(f"[control] go to xy succeed x:{round(cx,2)} y:{round(cy,2)} velocity:{round(velocity,2)}")
                return True
            
            if prev_dist is not None:
                if abs(dist - prev_dist) < 0.01 and dist > 0.5:
                    logger.warning(f"Drone stuck! dist={dist:.2f}, resetting PIDs...")
                    self._pid_pos_x.reset()
                    self._pid_pos_y.reset()
                    self._pid_vel_pitch.reset()
                    self._pid_vel_roll.reset()
                    prev_dist = None
                    time.sleep(0.1)
                    continue
            
            prev_dist = dist

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
        self._rpy_vel_data = (1500, 1500, r)

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
        """Perform blocking takeoff to nominal hover altitude."""
        print("\n[control] take off")

        PERIOD = 0.05
        MIN_H = 0.00
        TAKEOFF_H = self._takeoff_height
        Z_BIAS = 0.075
        MAX_H = 5.00
        REACH_COEF = 0.95
        TIMEOUT_COEF = 10.0

        if self._control_mode == "velocity":
            kin = self.get_sim_kinematics()
            if kin is not None:
                cx = sim_to_api_distance(kin["location"][0])
                cy = sim_to_api_distance(kin["location"][1])
                self._target_position = (cx, cy)
                self._control_mode = "position"
                logger.info("takeoff: switched to position mode, locked current position")

        h_now = self._get_height()
        tgt = self._clamp_h(TAKEOFF_H, MIN_H, MAX_H)

        self.set_target_height(tgt + Z_BIAS)

        climb = max(0.0, tgt - h_now)
        deadline = time.monotonic() + (TIMEOUT_COEF * climb if climb > 0.0 else TIMEOUT_COEF)

        while True:
            if self._is_abort:
                self._is_abort = False
                print("[control] take off aborted")
                logger.info("takeoff: aborted")
                return False
            h = self._get_height()
            if h >= REACH_COEF * tgt:
                print("[control] take off succeed")
                return True
            if time.monotonic() >= deadline:
                print("[control] take off failed")
                return False
            if not self._sleep_until(deadline, PERIOD):
                print("[control] take off failed")
                return False

    def boarding(self) -> bool:
        """Perform blocking landing sequence with stepwise descent."""
        PERIOD = 0.05
        MIN_H = 0.00
        MAX_H = 5.00
        STEP = 0.1
        TIMEOUT_MIN = 15.0
        TIMEOUT_COEF = 10.0

        self._is_abort = False

        print("\n[control] boarding")

        if self._control_mode == "velocity":
            kin = self.get_sim_kinematics()
            if kin is not None:
                cx = sim_to_api_distance(kin["location"][0])
                cy = sim_to_api_distance(kin["location"][1])
                self._target_position = (cx, cy)
                self._control_mode = "position"
                logger.info("boarding: switched to position mode, locked current position")

        curr_cmd = float(getattr(self, "_target_height", 0.0))
        curr_cmd = self._clamp_h(curr_cmd, MIN_H, MAX_H)

        total_drop = max(0.0, curr_cmd - MIN_H)
        deadline = time.monotonic() + TIMEOUT_MIN + TIMEOUT_COEF * total_drop

        while curr_cmd > MIN_H:
            if self._is_abort:
                self._is_abort = False
                logger.info("boarding: aborted")
                print("[control] boarding failed")
                return False
            curr_cmd = max(MIN_H, curr_cmd - STEP)
            self.set_target_height(curr_cmd)
            if time.monotonic() >= deadline:
                print("[control] boarding failed")
                return False
            if not self._sleep_until(deadline, PERIOD):
                print("[control] boarding failed")
                return False

        self._target_height = 0.0
        print("[control] boarding succeed")
        return True

    def setHeight(self, target_height: float) -> bool:
        """Move to target altitude in blocking mode."""
        print(f"\n[control] set height: {round(target_height,2)}")

        PERIOD = 0.05
        MIN_H = 0.00
        MAX_H = 5.00
        STEP = 0.05
        Z_BIAS = 0.075
        REACH_COEF = 0.95
        TIMEOUT_MIN = 15.0
        TIMEOUT_COEF = 10.0

        self._is_abort = False

        if self._control_mode == "velocity":
            kin = self.get_sim_kinematics()
            if kin is not None:
                cx = sim_to_api_distance(kin["location"][0])
                cy = sim_to_api_distance(kin["location"][1])
                self._target_position = (cx, cy)
                self._control_mode = "position"
                logger.info("setHeight: switched to position mode, locked current position")

        tgt = self._clamp_h(float(target_height), MIN_H, MAX_H)
        h0 = self._get_height()

        if tgt >= h0:
            self.set_target_height(tgt + Z_BIAS)
            deadline = time.monotonic() + TIMEOUT_MIN + TIMEOUT_COEF * max(0.0, tgt - h0)
            while True:
                if self._is_abort:
                    self._is_abort = False
                    logger.info("setHeight: aborted")
                    print("[control] set height aborted")
                    return False

                h = self._get_height()
                if h >= REACH_COEF * tgt:
                    print("[control] set height succeed")
                    return True
                if time.monotonic() >= deadline:
                    print("[control] set height failed")
                    return False
                if not self._sleep_until(deadline, PERIOD):
                    print("[control] set height failed")
                    return False
        else:
            curr_cmd = float(getattr(self, "_target_height", h0))
            curr_cmd = self._clamp_h(curr_cmd, MIN_H, MAX_H)
            deadline = time.monotonic() + TIMEOUT_MIN + TIMEOUT_COEF * max(0.0, curr_cmd - tgt)

            while curr_cmd > tgt:
                if self._is_abort:
                    self._is_abort = False
                    logger.info("setHeight: aborted")
                    print("[control] set height failed")
                    return False
                curr_cmd = max(tgt, curr_cmd - STEP)
                self.set_target_height(curr_cmd)
                if time.monotonic() >= deadline:
                    return False
                if not self._sleep_until(deadline, PERIOD):
                    print("[control] set height failed")
                    return False
        print("[control] set height succeed")
        return True
    
    def set_camera_id(self, new_id: int):
        """Set active simulator camera identifier."""
        with self._client_lock:
            self.camera_id = new_id
    
    def sim_kinematics_callback(self):
        """Refresh simulator kinematics, image, and range cache."""
        if not self._simulator_alive:
            return
        
        should_check_death = False
        
        try:
            with self._client_lock:
                if not hasattr(self, '_client') or self._client is None:
                    self._consecutive_errors += 1
                    should_check_death = True
                else:
                    try:
                        self._sim_kinematics = self._client.get_kinametics_data()
                        self._sim_img = self._client.get_camera_capture(camera_id=self.camera_id)
                        self._sim_ultrasonic = self._client.get_range_data(
                            rangefinder_id=0,
                            range_min=0.15,
                            range_max=4,
                            is_clear=True,
                            range_error=0.0003
                        ) * 100
                        
                        self._consecutive_errors = 0
                        self._simulator_alive = True
                        
                    except Exception as e:
                        self._consecutive_errors += 1
                        logger.warning(f"Error getting simulator data: {e}")
                        should_check_death = True
                        
        except Exception as e:
            self._consecutive_errors += 1
            logger.error(f"Critical error in sim_kinematics_callback: {e}")
            should_check_death = True
        
        if should_check_death:
            self._check_simulator_death()
    
    def get_sim_kinematics(self):
        """Return cached simulator kinematics dictionary."""
        return self._sim_kinematics
    
    def transmit_rc_to_sim(self):
        """Send current RC command packet to flight controller."""
        if not self._simulator_alive:
            return

        try:
            roll, pitch, yaw = self._rpy_vel_data
            raw_rc = [roll, pitch, self._throttle_data, yaw, self._arm_data, self._fliyng_mode, self._nav_mode]
            raw_rc = self.clamp_rc_list(raw_rc)
            msg = self._control.send_RAW_RC(raw_rc)
            data_handler = self._control.receive_msg()
        except Exception:
            pass
    
    def setVelXY(self, x, y):
        """Deprecated wrapper around set_velocity_xy."""
        self.set_velocity_xy(x, y, frame="odom")
    
    def setVelXYYaw(self, x, y, yaw):
        """Deprecated wrapper for XY velocity command with ignored yaw."""
        logger.warning("setVelXYYaw is deprecated, use set_velocity_xy + setYaw")
        self.set_velocity_xy(x, y, frame="base_link")
    
    def armDrone(self):
        """Arm drone and start altitude control timer."""
        self._throttle_data = 1000
        
        #self._pid_height_pos.reset()
        #self._pid_height_vel.reset()
        
        self._target_height_vel = 0.0
        
        self._armed_flag = True
        self._arm_data = 2000

        time.sleep(0.2)
        #self._height_pos_timer.start()
        #self._height_vel_timer.start()
        #self._height_pos_timer.start()
        #self._height_vel_timer.start()

        self._height_timer.start()

    def disarmDrone(self):
        """Disarm drone and reset throttle outputs."""
        self._armed_flag = False
        self._arm_data = 1000
        self._throttle_data = 1000
        
    
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
        self._nav_mode = 1500
        self._base_throttle_hover = 1500
        self.unlock_motors()
        

    def posholdOff(self):
        """Disable POSHOLD-related channel configuration."""
        self._poshold_flag = False
        self._nav_mode = 1000
        self._throttle_data = 1000
        self._base_throttle_hover = 1000
        self._pid_height.reset()
        self.lock_motors()

    def add_range_for_althold(self, mode_id=3, channel_index=2, range_start=1250, range_end=1350):
        """Add NAV ALTHOLD mode range entry in MSP configuration."""
        def encode_range_value(us_value):
            """Encode microseconds range value to MSP format."""
            return int((us_value - 900) / 25)

        def find_mode_ranges(msp, mode_id, channel_index):
            """Find mode range entries for mode and AUX channel."""
            results = []
            for i, mr in enumerate(msp.MODE_RANGES):
                if (mr['id'] == mode_id and
                    mr['auxChannelIndex'] == channel_index):
                    results.append((i, mr))
            return results

        def find_first_empty_range(msp):
            """Find first unused mode range entry index."""
            for i, mr in enumerate(msp.MODE_RANGES):
                if mr['id'] == 0 and mr['range']['start'] == 900 and mr['range']['end'] == 900:
                    return i
            return None

        try:
            self._control.send_RAW_msg(MSPCodes['MSP_MODE_RANGES'], data=[])
            data_handler = self._control.receive_msg()
            self._control.process_recv_data(data_handler)

            if not self._control.MODE_RANGES:
                logger.warning("MODE_RANGES РїСѓСЃС‚!")
                return False

            existing = find_mode_ranges(self._control, mode_id, channel_index)
            logger.info(f"РќР°Р№РґРµРЅРѕ СЃСѓС‰РµСЃС‚РІСѓСЋС‰РёС… РґРёР°РїР°Р·РѕРЅРѕРІ NAV ALTHOLD РЅР° CH{5 + channel_index}: {len(existing)}")
            for idx, mr in existing:
                logger.info(f"  [{idx}] {mr['range']['start']}-{mr['range']['end']}")

            for _, mr in existing:
                if mr['range']['start'] == range_start and mr['range']['end'] == range_end:
                    logger.info(f"Р”РёР°РїР°Р·РѕРЅ {range_start}-{range_end} СѓР¶Рµ СЃСѓС‰РµСЃС‚РІСѓРµС‚!")
                    return True

            empty_index = find_first_empty_range(self._control)
            if empty_index is None:
                logger.warning("РќРµС‚ СЃРІРѕР±РѕРґРЅС‹С… range entry! РњР°СЃСЃРёРІ РїРѕР»РЅРѕСЃС‚СЊСЋ Р·Р°РїРѕР»РЅРµРЅ.")
                logger.warning(f"Р’СЃРµРіРѕ entries: {len(self._control.MODE_RANGES)}")
                return False

            logger.info(f"РЎРІРѕР±РѕРґРЅС‹Р№ range entry РЅР°Р№РґРµРЅ: index={empty_index}")

            payload = [
                empty_index,
                mode_id,                             # modeId NAV ALTHOLD
                channel_index,                       # auxChannelIndex
                encode_range_value(range_start),     # start encoded
                encode_range_value(range_end),       # end encoded
            ]

            logger.info(f"\nР”РѕР±Р°РІР»СЏСЋ РґРёР°РїР°Р·РѕРЅ: {range_start}-{range_end}")
            logger.info(f"  rangeIndex     = {payload[0]}")
            logger.info(f"  modeId         = {payload[1]} (NAV ALTHOLD)")
            logger.info(f"  auxChannelIndex= {payload[2]} (CH{5 + payload[2]})")
            logger.info(f"  start          = {range_start} (encoded: {payload[3]})")
            logger.info(f"  end            = {range_end} (encoded: {payload[4]})")

            self._control.send_RAW_msg(MSPCodes['MSP_SET_MODE_RANGE'], data=payload)
            time.sleep(0.3)

            logger.info("РЎРѕС…СЂР°РЅСЏСЋ РІ EEPROM...")
            self._control.send_RAW_msg(MSPCodes['MSP_EEPROM_WRITE'], data=[])
            time.sleep(0.5)

            self._control.send_RAW_msg(MSPCodes['MSP_MODE_RANGES'], data=[])
            data_handler = self._control.receive_msg()
            self._control.process_recv_data(data_handler)

            updated = find_mode_ranges(self._control, mode_id, channel_index)
            logger.info(f"\nР’СЃРµ РґРёР°РїР°Р·РѕРЅС‹ NAV ALTHOLD РЅР° CH{5 + channel_index}:")
            for idx, mr in updated:
                logger.info(f"  [{idx}] {mr['range']['start']}-{mr['range']['end']}")

            for _, mr in updated:
                if mr['range']['start'] == range_start and mr['range']['end'] == range_end:
                    logger.info("РЈРЎРџР•РҐ! РќРѕРІС‹Р№ РґРёР°РїР°Р·РѕРЅ РґРѕР±Р°РІР»РµРЅ!")
                    return True

            logger.warning("Р’РќРРњРђРќРР•: РґРёР°РїР°Р·РѕРЅ РЅРµ РїРѕСЏРІРёР»СЃСЏ РїРѕСЃР»Рµ СЃРѕС…СЂР°РЅРµРЅРёСЏ!")
            return False

        except Exception as e:
            logger.error(f"РћС€РёР±РєР° РїСЂРё РґРѕР±Р°РІР»РµРЅРёРё РґРёР°РїР°Р·РѕРЅР° РґР»СЏ ALTHOLD: {e}")
            return False

    def altholdOn(self):
        """Enable NAV ALTHOLD channel value and unlock motors."""
        self._althold_flag = True
        self._nav_mode = 1300
        self._base_throttle_hover = 1500
        self.unlock_motors()

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
        """Set altitude target and reset height PID state."""
        self._target_height = height
        self._pid_height.reset()
    
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
        """Stop active timers except the current execution thread timer."""
        import threading
        current_thread = threading.current_thread()
        
        try:
            if hasattr(self, 'height_pos_timer') and self._height_pos_timer is not None:
                if self._height_pos_timer._thread != current_thread:
                    self._height_pos_timer.stop()
        except Exception:
            pass

        try:
            if hasattr(self, 'height_vel_timer') and self._height_vel_timer is not None:
                if self._height_vel_timer._thread != current_thread:
                    self._height_vel_timer.stop()
        except Exception:
            pass
            
        try:
            if hasattr(self, '_rc_timer') and self._rc_timer is not None:
                if self._rc_timer._thread != current_thread:
                    self._rc_timer.stop()
        except Exception:
            pass
            
        try:
            if hasattr(self, '_sim_kinematics_timer') and self._sim_kinematics_timer is not None:
                if self._sim_kinematics_timer._thread != current_thread:
                    self._sim_kinematics_timer.stop()
        except Exception:
            pass
            
        try:
            if hasattr(self, '_image_processing_timer') and self._image_processing_timer is not None:
                if self._image_processing_timer._thread != current_thread:
                    self._image_processing_timer.stop()
        except Exception:
            pass
            
        try:
            if hasattr(self, '_yaw_timer') and self._yaw_timer is not None:
                if self._yaw_timer._thread != current_thread:
                    self._yaw_timer.stop()
        except Exception:
            pass
            
        try:
            if hasattr(self, '_velocity_timer') and self._velocity_timer is not None:
                if self._velocity_timer._thread != current_thread:
                    self._velocity_timer.stop()
        except Exception:
            pass

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
