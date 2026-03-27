"""
HighLevelSimClient - клиент высокого уровня для управления дроном в симуляторе TechSim.

Реализует каскадное PID-управление с двумя режимами:
- position: полет в заданную точку (позиция → скорость → PWM)
- velocity: движение с заданной скоростью (скорость → PWM)

Архитектура:
- PID-регуляторы работают постоянно в фоновых коллбэках (50 Гц)
- PosHold включается только при взлете, посадке и удержании позиции
- Асинхронные операции для неблокирующего управления
"""
from inavmspapi import MultirotorControl
from inavmspapi.transmitter import TCPTransmitter
from inavmspapi.msp_codes import MSPCodes
from agrotechsimapi.client import SimClient

from agrotechsimapi.pid import PID, AdaptivePID
from typing import Iterable, Optional, Tuple, Literal

from agrotechsimapi.utils.utils import LoopingTimer, sim_to_api_distance, vel_to_rc_signal
from agrotechsimapi.utils.vision import process_aruco, process_blob, resolution_changes

from transforms3d.euler import quat2euler

import time
import math
import threading
import logging
import asyncio

# Настройка логирования
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def main():
    pass


if __name__ == "__main__":
    main()


class HighLevelSimClient:
    """
    Клиент высокого уровня для управления дроном в симуляторе TechSim.
    
    Реализует:
    - Каскадное PID-управление (позиция → скорость → PWM)
    - Два режима управления: position и velocity
    - Фоновые коллбэки для yaw и горизонтального движения (50 Гц)
    - Обнаружение смерти симулятора
    - Взлет, посадка, перемещение в точку
    """
    
    # Типы режимов управления
    ControlMode = Literal["position", "velocity"]
    
    def __init__(self):
        # ===== ПИД-РЕГУЛЯТОРЫ (горизонтальное движение) =====
        # Позиция → Скорость
        self._pid_pos_x = PID(kp=1.055, ki=0, kd=7.15, max_control=1.35, i_limit=0.0001)
        self._pid_pos_y = PID(kp=1.055, ki=0, kd=7.15, max_control=1.35, i_limit=0.0001)
        
        # Скорость → PWM
        self._pid_vel_x = PID(kp=7.5, ki=0.00001, kd=6.0, max_control=1.55, i_limit=0.001)
        self._pid_vel_y = PID(kp=7.5, ki=0.00001, kd=6.0, max_control=1.55, i_limit=0.001)
        
        # Yaw PID
        self._pid_yaw = PID(kp=3.5, ki=0.001, kd=0.4, max_control=1.0, i_limit=None)
        
        # ===== ПИД-регулятор высоты =====
        #self.__alt_pid = PID(3, 0.015, 5)
        self.__alt_pid = PID(0.1, 0.0, 3)
        # ===== ПЕРЕМЕННЫЕ СОСТОЯНИЯ =====
        self._control_mode = " position" #: self.ControlMode = "velocity"  # По умолчанию velocity
        self._target_position = (0.0, 0.0)  # Целевая позиция (x, y)
        self._target_velocity = (0.0, 0.0)  # Целевая скорость (vx, vy)
        self._target_yaw = 0.0  # Целевой угол поворота
        
        self._odom = (0.0, 0.0)
        self._altitude = 0.0
        self._target_height = 0.0
        
        self._armed_flag = False
        self._poshold_flag = False
        
        self._sim_img = None
        self._blob_img = None
        self._aruco_img = None
        self._aruco_data = []
        self._camera_pose_aruco_data = []
        self._blob_data = []
        
        self._odom0_xy = (0.0, 0.0)  # (x0, y0) в мировой СК на момент «сброса одометрии»
        
        self._sim_kinematics = None
        self._sim_ultrasonic = None
        
        # ===== Блокировки =====
        self._client_lock = threading.Lock()
        
        # ===== МЕХАНИЗМ ОТСЛЕЖИВАНИЯ СМЕРТИ СИМУЛЯТОРА =====
        self._consecutive_errors = 0
        self._error_threshold = 2
        self._simulator_alive = True
        self._on_death_callback = None
        # =================================================
        
        print("process started")
    
    def connect(self, ip, port):
        """Подключение к симулятору"""
        self.__HOST = ip
        self.__SIM_PORT = 8080
        self.__TCP_PORT = 5762
        self.__TCP_ADDRESS = (self.__HOST, self.__TCP_PORT)
        
        self.__tcp_transmitter = TCPTransmitter(self.__TCP_ADDRESS)
        self.__tcp_transmitter.connect()
        self._control = MultirotorControl(self.__tcp_transmitter)
        time.sleep(2)
        
        # Инициализируем клиент с блокировкой
        with self._client_lock:
            self._client = SimClient(address=self.__HOST, port=self.__SIM_PORT)
        
        # Создаем таймеры
        self._height_timer = LoopingTimer(interval=1/25, callback=self.calculate_height_rc, name="height_timer")
        self._rc_timer = LoopingTimer(interval=1/50, callback=self.transmit_rc_to_sim, name="rc_timer")
        self._sim_kinematics_timer = LoopingTimer(interval=1/50, callback=self.sim_kinematics_callback, name="sim_kinematics_timer")
        self._image_processing_timer = LoopingTimer(interval=1/10, callback=self.image_processing_callback, name="image_processing_timer")
        
        # Новые фоновые коллбэки для управления
        self._yaw_timer = LoopingTimer(interval=1/25, callback=self.yaw_callback, name="yaw_timer")
        self._velocity_timer = LoopingTimer(interval=1/50, callback=self.velocity_callback, name="velocity_timer")
        
        self.sim_kinematics_callback()
        self.initDrone()
        
        # Запускаем все таймеры
        self._sim_kinematics_timer.start()
        self._height_timer.start()
        self._rc_timer.start()
        self._image_processing_timer.start()
        self._yaw_timer.start()
        self._velocity_timer.start()
        
        time.sleep(1)
        self._armed_flag = False
        self._arm_data = 1000
        time.sleep(1)
        self._poshold_flag = False
        self._nav_mode = 1000
    
    def disconnect(self):
        """Отключение от симулятора"""
        print("===DISCONECT===")
        
        # Останавливаем все таймеры
        self._sim_kinematics_timer.stop()
        self._height_timer.stop()
        self._rc_timer.stop()
        self._image_processing_timer.stop()
        self._yaw_timer.stop()
        self._velocity_timer.stop()
        
        # Сбрасываем состояние
        self.disarmDrone()
        self.posholdOff()
    
    # =========================================================
    # НОВЫЕ МЕТОДЫ УПРАВЛЕНИЯ (Этап 1)
    # =========================================================
    
    def yaw_callback(self):
        """
        Фоновый коллбэк для управления yaw (50 Гц).
        Постоянно регулирует yaw дрона к target_yaw.
        """
        if not self._simulator_alive:
            return
        
        try:
            current_yaw = self._get_yaw_cw()
            error = self._target_yaw - current_yaw
            error = self._wrap_pi(error)  # Нормализуем ошибку
            
            # Обновляем PID
            self._pid_yaw.update_control(error)
            yaw_rate = self._pid_yaw.get_control()
            
            # Конвертируем в PWM
            yaw_pwm = vel_to_rc_signal(yaw_rate)
            
            # Обновляем только yaw, не трогая roll/pitch
            r, p, _ = self._rpy_vel_data
            self._rpy_vel_data = (r, p, yaw_pwm)
            
        except Exception as e:
            logger.warning(f"Error in yaw_callback: {e}")
    
    def velocity_callback(self):
        """
        Фоновый коллбэк для каскадного управления скоростью (50 Гц).
        
        Контур 1: Позиция → Скорость (работает только в режиме position)
        Контур 2: Скорость → PWM (работает всегда)
        """
        if not self._simulator_alive:
            return
        
        try:
            kin = self.get_sim_kinematics()
            if kin is None:
                return
            
            # Получаем текущую позицию и скорость
            x_w = sim_to_api_distance(kin["location"][0])
            y_w = sim_to_api_distance(kin["location"][1])
            
            # Вычисляем текущую скорость (простая разность позиций)
            if hasattr(self, '_prev_x') and hasattr(self, '_prev_y'):
                vx = (x_w - self._prev_x) * 50  # 50 Гц
                vy = (y_w - self._prev_y) * 50
            else:
                vx = vy = 0.0
            
            self._prev_x = x_w
            self._prev_y = y_w
            
            # ===== КОНТУР 1: Позиция → Скорость =====
            if self._control_mode == "position":
                # Вычисляем ошибку позиции
                tx, ty = self._target_position
                pos_error_x = tx - x_w
                pos_error_y = ty - y_w
                
                # PID позиции выдает целевую скорость
                self._pid_pos_x.update_control(pos_error_x)
                self._pid_pos_y.update_control(pos_error_y)
                
                self._target_velocity = (
                    self._pid_pos_x.get_control(),
                    self._pid_pos_y.get_control()
                )
            
            # ===== КОНТУР 2: Скорость → PWM =====
            # Работает ВСЕГДА (и в position, и в velocity)
            tvx, tvy = self._target_velocity
            
            # Ошибка скорости
            vel_error_x = tvx - vx
            vel_error_y = tvy - vy
            
            # PID скорости выдает PWM
            self._pid_vel_x.update_control(vel_error_x)
            self._pid_vel_y.update_control(vel_error_y)
            
            pitch_pwm = vel_to_rc_signal(self._pid_vel_x.get_control())
            roll_pwm = vel_to_rc_signal(-self._pid_vel_y.get_control())
            
            # Обновляем roll/pitch, не трогая yaw
            _, _, y = self._rpy_vel_data
            self._rpy_vel_data = (roll_pwm, pitch_pwm, y)
            
        except Exception as e:
            logger.warning(f"Error in velocity_callback: {e}")
    
    def set_velocity_xy(self, vx: float, vy: float):
        """
        Установить целевую скорость движения.
        
        Args:
            vx: Скорость по оси X (м/с)
            vy: Скорость по оси Y (м/с)
        """
        self._control_mode = "velocity"
        self._target_velocity = (vx, vy)
        #self.posholdOff()  # Выключаем PosHold
    
    def go_to_xy(self, frame: str, x: float, y: float, max_speed: float = 0.5) -> bool:
        """
        Полет в заданную точку.
        
        Args:
            frame: Система координат ("odom" или "base_link")
            x: Координата X (м)
            y: Координата Y (м)
            max_speed: Максимальная скорость (м/с)
            
        Returns:
            True если достиг цели, False если таймаут
        """
        # 1. Сброс PID
        self._pid_pos_x.reset()
        self._pid_pos_y.reset()
        self._pid_vel_x.reset()
        self._pid_vel_y.reset()
        
        # 2. Включаем режим позиции
        self._control_mode = "position"
        
        # 3. Вычисляем целевую позицию
        kin = self.get_sim_kinematics()
        if kin is None:
            logger.error("No kinematics data for go_to_xy")
            return False
        
        cx = sim_to_api_distance(kin["location"][0])
        cy = sim_to_api_distance(kin["location"][1])
        
        if frame == "odom":
            if self._odom0_xy != (0.0, 0.0):
                x0, y0 = self._odom0_xy
                self._target_position = (x0 + x, y0 + y)
            else:
                self._target_position = (x, y)
        elif frame == "base_link":
            _, _, yaw = self._get_yaw_cw()
            self._target_position = (
                cx + math.cos(yaw) * x - math.sin(yaw) * y,
                cy + math.sin(yaw) * x + math.cos(yaw) * y
            )
        else:
            raise ValueError("frame must be 'odom' or 'base_link'")
        
        # 4. Выключаем PosHold
        #self.posholdOff()
        
        # 5. Ограничиваем максимальную скорость
        self._pid_pos_x._max_control = max_speed
        self._pid_pos_y._max_control = max_speed
        
        # 6. Ожидаем достижения цели
        timeout = 60.0  # секунд
        start_time = time.monotonic()
        
        while time.monotonic() - start_time < timeout:
            if not self._simulator_alive:
                logger.warning("Simulator died during go_to_xy")
                self.posholdOn()
                return False
            
            # Проверяем расстояние до цели
            tx, ty = self._target_position
            dist = math.hypot(tx - cx, ty - cy)
            
            if dist < 0.1:  # 5 см допуск
                logger.info(f"Reached target: {x}, {y}")
                self.posholdOn()  # Включаем PosHold после достижения
                return True
            
            # Обновляем текущую позицию
            kin = self.get_sim_kinematics()
            if kin is not None:
                cx = sim_to_api_distance(kin["location"][0])
                cy = sim_to_api_distance(kin["location"][1])
            
            time.sleep(0.05)  # 20 Гц
        
        logger.warning(f"go_to_xy timeout: target={x}, {y}")
        #self.posholdOn()  # Включаем PosHold при таймауте
        return False
    
    def gotoXYodom(self, x: float, y: float) -> bool:
        """Полет в точку в системе координат одометрии"""
        return self.go_to_xy("odom", x, y)
    
    def gotoXYdrone(self, x: float, y: float) -> bool:
        """Полет в точку в системе координат дрона"""
        return self.go_to_xy("base_link", x, y)
    
    def setYaw(self, yaw: float) -> bool:
        """
        БЛОКИРУЮЩИЙ поворот до абсолютного угла.
        
        Args:
            yaw: Целевой угол (радианы, по часовой стрелке)
            
        Returns:
            True если достиг угла, False если таймаут
        """
        # Сброс PID
        self._pid_yaw.reset()
        
        # Нормализуем цель
        goal = self._wrap_pi(yaw)
        
        timeout = 10.0
        start_time = time.monotonic()
        tol = 0.025  # рад
        self._target_yaw = goal
        while time.monotonic() - start_time < timeout:
            if not self._simulator_alive:
                return False
            
            current = self._get_yaw_cw()
            error = self._wrap_pi(goal - current)
            
            if abs(error) < tol:
                # Достигли - центрируем yaw
                r, p, _ = self._rpy_vel_data
                self._rpy_vel_data = (r, p, 1500)
                return True
            
            time.sleep(0.05)
        
        # Таймаут
        r, p, _ = self._rpy_vel_data
        self._rpy_vel_data = (r, p, 1500)
        return False
    
    # =========================================================
    # СУЩЕСТВУЮЩИЕ МЕТОДЫ (с минимальными изменениями)
    # =========================================================
    
    def _world_xy(self, kin=None) -> tuple[float, float]:
        """Мировые координаты XY"""
        if kin is None:
            kin = self.get_sim_kinematics()
        x_w = sim_to_api_distance(kin["location"][0])
        y_w = sim_to_api_distance(kin["location"][1])
        return x_w, y_w
    
    def _odom_xy(self, kin=None) -> tuple[float, float]:
        """Одометрия (XY)"""
        x_w, y_w = self._world_xy(kin)
        if self._odom0_xy == (0.0, 0.0):
            return x_w, y_w
        x0, y0 = self._odom0_xy
        return x_w - x0, y_w - y0
    
    def getHeightRange(self):
        with self._client_lock:
            return self._altitude
    
    def getHeightBarometer(self):
        with self._client_lock:
            return self._altitude
    
    def getArm(self):
        return self._armed_flag
    
    def setZeroOdomOpticflow(self) -> bool:
        """Сброс одометрии"""
        kin = self.get_sim_kinematics()
        if kin is None:
            raise RuntimeError("Нет кэша кинематики")
        self._odom0_xy = self._world_xy(kin)
        return True
    
    def getUltrasonic(self):
        with self._client_lock:
            return self._sim_ultrasonic
    
    def calculate_height_rc(self):
        """Расчет throttle для удержания высоты"""
        if not self._simulator_alive:
            return
        
        try:
            self._altitude = sim_to_api_distance(self.get_sim_kinematics()["location"][2])
            error = self._target_height - self._altitude
            self.__alt_pid.update_control(error)
            alt_rc_output = self.__alt_pid.get_control()
            throttle_output = self.clamp_rc(1290 + alt_rc_output * 100)
            
            if self._armed_flag: #and self._poshold_flag:
                self._throttle_data = throttle_output
        except Exception:
            pass
    
    def getRPY(self):
        kin = self.get_sim_kinematics()
        qx, qy, qz, qw = kin["orientation"]
        roll, pitch, yaw = quat2euler((qw, qx, qy, qz), axes='sxyz')
        return [roll, pitch, yaw]
    
    def getOdomOpticflow(self):
        kin = self.get_sim_kinematics()
        x, y = self._world_xy(kin)
        last_x, last_y = self._odom0_xy
        odom_x = x - last_x
        odom_y = y - last_y
        return [odom_x, odom_y, self._altitude]
    
    @staticmethod
    def _wrap_pi(a: float) -> float:
        """Нормализация угла в [-pi, pi)"""
        return (a + math.pi) % (2 * math.pi) - math.pi
    
    def _get_yaw_cw(self) -> float:
        """Текущий курс в радианах (по часовой стрелке)"""
        kin = self.get_sim_kinematics()
        if kin is None:
            return 0.0
        qx, qy, qz, qw = kin["orientation"]
        _, _, yaw_ccw = quat2euler((qw, qx, qy, qz), axes='sxyz')
        return -yaw_ccw
    
    def _get_height(self) -> float:
        return float(self._altitude)
    
    def _clamp_h(self, h: float, lo: float, hi: float) -> float:
        return max(lo, min(h, hi))
    
    def _sleep_until(self, t_deadline: float, period: float) -> bool:
        now = time.monotonic()
        if now >= t_deadline:
            return False
        time.sleep(max(0.0, period - (time.monotonic() - now)))
        return True
    
    def takeoff(self) -> bool:
        """Взлет дрона"""
        PERIOD = 0.05
        MIN_H = 0.00
        TAKEOFF_H = 0.5
        MAX_H = 5.00
        REACH_COEF = 0.97
        TIMEOUT_COEF = 10.0
        
        h_now = self._get_height()
        tgt = self._clamp_h(TAKEOFF_H, MIN_H, MAX_H)
        
        self.set_target_height(tgt)
        
        climb = max(0.0, tgt - h_now)
        deadline = time.monotonic() + (TIMEOUT_COEF * climb if climb > 0.0 else TIMEOUT_COEF)
        
        while True:
            h = self._get_height()
            if h >= REACH_COEF * tgt:
                return True
            if time.monotonic() >= deadline:
                return False
            if not self._sleep_until(deadline, PERIOD):
                return False
    
    def boarding(self) -> bool:
        """Посадка дрона"""
        PERIOD = 0.05
        MIN_H = 0.00
        MAX_H = 5.00
        STEP = 0.1
        TIMEOUT_MIN = 15.0
        TIMEOUT_COEF = 10.0
        
        curr_cmd = float(getattr(self, "_target_height", 0.0))
        curr_cmd = self._clamp_h(curr_cmd, MIN_H, MAX_H)
        
        total_drop = max(0.0, curr_cmd - MIN_H)
        deadline = time.monotonic() + TIMEOUT_MIN + TIMEOUT_COEF * total_drop
        
        while curr_cmd > MIN_H:
            curr_cmd = max(MIN_H, curr_cmd - STEP)
            self.set_target_height(curr_cmd)
            if time.monotonic() >= deadline:
                return False
            if not self._sleep_until(deadline, PERIOD):
                return False
        
        self._target_height = 0.0
        self.__alt_pid.reset()
        return True
    
    def setHeight(self, target_height: float) -> bool:
        """Установка высоты"""
        PERIOD = 0.05
        MIN_H = 0.00
        MAX_H = 5.00
        STEP = 0.005
        REACH_COEF = 0.97
        TIMEOUT_MIN = 15.0
        TIMEOUT_COEF = 10.0
        
        tgt = self._clamp_h(float(target_height), MIN_H, MAX_H)
        h0 = self._get_height()
        
        if tgt >= h0:
            self.set_target_height(tgt)
            deadline = time.monotonic() + TIMEOUT_MIN + TIMEOUT_COEF * max(0.0, tgt - h0)
            while True:
                h = self._get_height()
                if h >= REACH_COEF * tgt:
                    return True
                if time.monotonic() >= deadline:
                    return False
                if not self._sleep_until(deadline, PERIOD):
                    return False
        else:
            curr_cmd = float(getattr(self, "_target_height", h0))
            curr_cmd = self._clamp_h(curr_cmd, MIN_H, MAX_H)
            deadline = time.monotonic() + TIMEOUT_MIN + TIMEOUT_COEF * max(0.0, curr_cmd - tgt)
            
            while curr_cmd > tgt:
                curr_cmd = max(tgt, curr_cmd - STEP)
                self.set_target_height(curr_cmd)
                if time.monotonic() >= deadline:
                    return False
                if not self._sleep_until(deadline, PERIOD):
                    return False
        
        return True
    
    def sim_kinematics_callback(self):
        """Обновление данных кинематики"""
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
                        self._sim_img = self._client.get_camera_capture(camera_id=1)
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
        return self._sim_kinematics
    
    def transmit_rc_to_sim(self):
        """Отправка RC команд"""
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
        """Устаревший метод, использовать set_velocity_xy"""
        self.set_velocity_xy(x, y)
    
    def setVelXYYaw(self, x, y, yaw):
        rpy = self._rpy_vel_data
        rc_x = vel_to_rc_signal(x)
        rc_y = vel_to_rc_signal(y)
        self._rpy_vel_data = (rc_y, rc_x, rpy[2])
    
    def armDrone(self):
        self._armed_flag = True
        self._arm_data = 2000
        self._throttle_data = 1000
        self.__alt_pid.reset()
    
    def disarmDrone(self):
        self._armed_flag = False
        self._arm_data = 1000
        self._throttle_data = 1000
    
    def initDrone(self):
        self._rpy_vel_data = (1500, 1500, 1500)
        self._throttle_data = 1000
        self._arm_data = 1000
        self._fliyng_mode = 1000
        self._nav_mode = 1000
    
    def posholdOn(self):
        self._poshold_flag = True
        self._nav_mode = 1500
    
    def posholdOff(self):
        self._poshold_flag = False
        self._nav_mode = 1000
        self._throttle_data = 1000
        self.__alt_pid.reset()
    
    def clamp_rc(self, data):
        return max(min(data, 2000), 1000)
    
    def clamp_rc_list(self, data: Iterable):
        return [self.clamp_rc(rc) for rc in data]
    
    def round_data(self, iterable):
        return map(lambda x: round(x, 3), iterable)
    
    def set_target_height(self, height):
        self._target_height = height
        self.__alt_pid.reset()
    
    def getImage(self):
        with self._client_lock:
            return self._sim_img
    
    def getArucos(self):
        return self._aruco_data
    
    def getCameraPoseAruco(self):
        return self._camera_pose_aruco_data
    
    def getBlobs(self):
        return self._blob_data
    
    def getBlobsImage(self):
        return self._blob_img
    
    def getArucosImage(self):
        return self._aruco_img
    
    def image_processing_callback(self):
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
    
    def setDiod(self, r, g, b):
        with self._client_lock:
            self._client.set_Diod(0, float(r), float(g), float(b))
            self._client.set_Diod(1, float(r), float(g), float(b))
            self._client.set_Diod(2, float(r), float(g), float(b))
            self._client.set_Diod(3, float(r), float(g), float(b))
    
    def setShoot(self, time):
        with self._client_lock:
            return self._client.call_event_action()
    
    def set_simulator_death_callback(self, callback):
        self._on_death_callback = callback
    
    def is_simulator_alive(self):
        return self._simulator_alive
    
    def _check_simulator_death(self):
        """Проверка на смерть симулятора"""
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
        """Остановка всех таймеров"""
        import threading
        current_thread = threading.current_thread()
        
        try:
            if hasattr(self, '_height_timer') and self._height_timer is not None:
                if self._height_timer._thread != current_thread:
                    self._height_timer.stop()
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
