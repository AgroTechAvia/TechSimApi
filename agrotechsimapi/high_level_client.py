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

import numpy as np

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

    # ===== НАСТРОЙКИ МАКСИМАЛЬНОЙ СКОРОСТИ И УСКОРЕНИЯ =====
    _max_velocity = 0.25  # Максимальная скорость (м/с)
    _max_acceleration = 0.75 # Максимальное ускорение (м/с²)
    # =======================================================

    _z_bias = 0

    def __init__(self):
        # ===== ПИД-РЕГУЛЯТОРЫ (горизонтальное движение) =====
        # Позиция → Скорость (с экспоненциальной зависимостью для плавности)

        self.camera_id = 0

        pos_pid_processing = lambda x:  ((2/(1+(2.7**(-x * 4.0)))) - 1) * x
        self._pid_pos_x = PID(kp=1.55, ki=0.0, kd=1.75, max_control=self._max_velocity,
                              i_limit=0.1)#, processing_func = pos_pid_processing)
        self._pid_pos_y = PID(kp=1.55, ki=0.0, kd=1.75, max_control=self._max_velocity,
                              i_limit=0.05)#, processing_func = pos_pid_processing)

        # Скорость → PWM (PD-регулятор, МАКСИМАЛЬНАЯ СТАБИЛЬНОСТЬ)
        # kp=2.0: меньше начальный толчок
        # ki=0.0: УБРАН — интеграл раскачивает систему!
        # kd=8.0: МАКСИМАЛЬНОЕ демпфирование для подавления осцилляций
        # max_control=1.0: ОГРАНИЧЕНО — дрон не сможет разогнаться слишком сильно
        #                   PWM диапазон: 1400-1600 (вместо 1300-1700)
        self._pid_vel_pitch = PID(kp=3.25, ki=0.0, kd=3.25,
                              max_control=1.5, i_limit=0.005)
        self._pid_vel_roll = PID(kp=3.25, ki=0.0, kd=3.25,
                              max_control=1.5, i_limit=0.005)

        # Yaw PID (линейный, т.к. точность важна)
        self._pid_yaw = PID(kp=3.5, ki=0.001, kd=0.4, max_control=1.0, i_limit=None)

        # ===== ПИД-регулятор высоты (ОДИН контур) =====
        # Ошибка высоты → throttle_delta
        # kp=2.5: ошибка 1м → delta throttle 250
        # ki=0.017: медленное накопление для точного висения
        # kd=7.0: демпфирование
        heigh_pid_processing = lambda x: ((2/(1+(2.7**(-x * 3)))) - 1) * 1.55 
        self._pid_height = PID(kp=5.0, ki=0.125, kd=0.8,
                               i_limit=12, processing_func=heigh_pid_processing)

        # Базовый throttle для висения (подобран экспериментально для TechSim)
        self._base_throttle_hover = 1500  # 1500 для висения с PosHold
        self._max_throttle = 1675  # Максимум для взлета/маневров
        self._min_throttle = 1475  # Минимум для безопасности

        # ===== КОЭФФИЦИЕНТЫ НАПРАВЛЕНИЯ (для подстройки под симулятор) =====
        # +1: PWM растет при движении вперед/вправо/по часовой
        # -1: PWM падает при движении вперед/вправо/по часовой
        # Подберите экспериментально для вашего симулятора
        self._roll_direction = -1.0    # Направление roll: +1 или -1
        self._pitch_direction = 1.0   # Направление pitch: +1 или -1
        self._yaw_direction = 1.0     # Направление yaw: +1 или -1
        # ====================================================================

        # ===== ПЕРЕМЕННЫЕ СОСТОЯНИЯ =====
        self._control_mode = "position"  #: "position" или "velocity"
        self._target_position = (0.0, 0.0)  # Целевая позиция (x, y)
        self._target_velocity = (0.0, 0.0)  # Целевая скорость (vx, vy)
        self._target_yaw = 0.0  # Целевой угол поворота

        # Флаг блокировки update_motors (для setYaw и аварий)
        self._motors_locked = True

        # Переменные для вычисления скорости
        self._prev_x = 0.0
        self._prev_y = 0.0
        self._prev_z = 0.0
        self._prev_target_vx = 0.0
        self._prev_target_vy = 0.0
        self._prev_target_vz = 0.0

        # ===== LOW-PASS ФИЛЬТР ДЛЯ СКОРОСТИ =====
        # Коэффициент фильтрации: 0.0 (полное сглаживание) → 1.0 (без фильтрации)
        # Рекомендуемые значения: 0.2-0.4 для баланса между шумом и отзывчивостью
        self._vel_filter_alpha = 0.55  # Коэффициент экспоненциального сглаживания
        self._filtered_vx_world = 0.0  # Отфильтрованная скорость по X
        self._filtered_vy_world = 0.0  # Отфильтрованная скорость по Y
        self._vel_filter_initialized = False
        # =========================================

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

        # ===== ФЛАГ АВАРИЙНОЙ ОСТАНОВКИ =====
        self._is_abort = False  # Флаг для прерывания blocking операций
        # =====================================

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

        # Сброс флагов арминга в MSP (важно для повторного подключения!)
        print("=== Resetting MSP arming flags... ===")
        try:
            # Отправляем команду дизарминга через MSP
            self._control.send_RAW_RC([1000, 1000, 1000, 1000, 1000, 1000, 1000])
            time.sleep(0.5)
        except Exception as e:
            logger.warning(f"Could not reset MSP flags: {e}")

        # Инициализируем клиент с блокировкой
        with self._client_lock:
            self._client = SimClient(address=self.__HOST, port=self.__SIM_PORT)

        # ПЕРЕСОЗДАЕМ таймеры при каждом подключении!
        # Это необходимо, потому что threading.Thread нельзя запустить повторно
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
        print(f"z_bias {self._z_bias}")

        # Запускаем все таймеры
        self._sim_kinematics_timer.start()
        self._rc_timer.start()
        self._image_processing_timer.start()
        self._yaw_timer.start()
        self._position_timer.start()
        self._velocity_timer.start()
        

        # Даем время на инициализацию и отправку RC команд
        time.sleep(1)
        # initDrone() уже установил _arm_data = 1000, таймеры отправляют это значение
        time.sleep(1)
    
    def disconnect(self):
        """Отключение от симулятора"""
        print("===DISCONECT===")

        # 1. СНАЧАЛА дизармим дрона (пока таймеры еще работают!)
        print("=== Disarming drone before disconnect... ===")
        self.disarmDrone()
        self.posholdOff()

        # Даем время на отправку RC команд с disarm
        time.sleep(0.5)

        # 2. ТОЛЬКО ПОТОМ останавливаем таймеры
        self._sim_kinematics_timer.stop()
        self._rc_timer.stop()

        self._image_processing_timer.stop()

        self._yaw_timer.stop()

        self._position_timer.stop()
        self._velocity_timer.stop()

        self._height_timer.stop()

        # 3. Отключаем TCP соединения
        if hasattr(self, '_tcp_transmitter') and self._tcp_transmitter:
            try:
                self._tcp_transmitter.disconnect()
            except Exception as e:
                logger.warning(f"Error disconnecting TCP: {e}")
    
    # =========================================================
    # НОВЫЕ МЕТОДЫ УПРАВЛЕНИЯ (Этап 1)
    # =========================================================
    
    def yaw_callback(self):
        """
        Фоновый коллбэк для управления yaw (25 Гц).
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

            # Конвертируем в PWM с учетом направления
            yaw_pwm = vel_to_rc_signal(yaw_rate * self._yaw_direction)

            # Обновляем только yaw, не трогая roll/pitch
            r, p, _ = self._rpy_vel_data
            self._rpy_vel_data = (r, p, yaw_pwm)

        except Exception as e:
            logger.warning(f"Error in yaw_callback: {e}")

    def position_callback(self):
        """
        ВНЕШНИЙ КОНТУР: Позиция → Скорость (20 Гц).

        Работает ТОЛЬКО в режиме position.
        Работает ТОЛЬКО в МИРОВОЙ СК (без учета yaw).
        Преобразует ошибку позиции в целевую скорость (в мировой СК).
        """
        if not self._simulator_alive:# or self._motors_locked:
            return

        try:
            kin = self.get_sim_kinematics()
            if kin is None:
                return

            # Получаем текущую позицию в мировой СК
            x_w = sim_to_api_distance(kin["location"][0])
            y_w = sim_to_api_distance(kin["location"][1])

            # ===== КОНТУР 1: Позиция → Скорость (в МИРОВОЙ СК) =====
            if self._control_mode == "position":
                # Вычисляем ошибку позиции в МИРОВОЙ СК
                tx, ty = self._target_position
                pos_error_x = tx - x_w  # Ошибка по X мира
                pos_error_y = ty - y_w  # Ошибка по Y мира

                # PID позиции выдает целевую скорость в МИРОВОЙ СК
                self._pid_pos_x.update_control(pos_error_x)
                self._pid_pos_y.update_control(pos_error_y)

                # Ограничиваем максимальную скорость
                tvx_world = max(min(self._pid_pos_x.get_control(), self._max_velocity),-self._max_velocity)
                tvy_world = max(min(self._pid_pos_y.get_control(), self._max_velocity),-self._max_velocity)

                if abs(tvx_world) <= 0.075:
                    tvx_world = -(1 - (1.75**tvx_world))
                if abs(tvy_world) <= 0.075:
                    tvy_world = -(1 - (1.75**tvy_world))

                # Нормализуем вектор скорости если он превышает максимум
                speed = math.hypot(tvx_world, tvy_world)
                '''if speed > self._max_velocity:
                    tvx_world = (tvx_world / speed) * self._max_velocity
                    tvy_world = (tvy_world / speed) * self._max_velocity'''

                self._target_velocity = (tvx_world, tvy_world)
                

        except Exception as e:
            logger.warning(f"Error in position_callback: {e}")

    def velocity_callback(self):
        """
        ВНУТРЕННИЙ КОНТУР: Скорость → PWM (50 Гц).

        Работает ВСЕГДА (и в position, и в velocity режиме).
        Преобразует скорости из мировой СК в СК дрона (с учетом yaw).
        Выдает PWM сигналы для roll/pitch.
        
        Конвенция yaw:
        - yaw=0: нос дрона → мировая X+
        - yaw=90°: нос дрона → мировая Y-
        - yaw=180°: нос дрона → мировая X-
        - yaw=-90°: нос дрона → мировая Y+
        
        """
        
        if not self._simulator_alive:# or self._motors_locked:
            
            return

        try:
            kin = self.get_sim_kinematics()
            if kin is None:
                
                return

            # Получаем текущую позицию в мировой СК
            
            x_w = sim_to_api_distance(kin["location"][0])
            y_w = sim_to_api_distance(kin["location"][1])

            # Получаем текущий yaw дрона (CW, радианы)
            yaw = self._get_yaw_cw()
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)

            # Вычисляем текущую скорость в мировой СК (конечные разности)
            raw_vx_world = (x_w - self._prev_x) * 50  # 50 Гц
            raw_vy_world = (y_w - self._prev_y) * 50

            self._prev_x = x_w
            self._prev_y = y_w

            # ===== LOW-PASS ФИЛЬТРАЦИЯ СКОРОСТИ =====
            # Экспоненциальное сглаживание: filtered = alpha * raw + (1 - alpha) * prev_filtered
            if not self._vel_filter_initialized:
                # Инициализируем фильтр первым измерением
                self._filtered_vx_world = raw_vx_world
                self._filtered_vy_world = raw_vy_world
                self._vel_filter_initialized = True
            else:
                alpha = self._vel_filter_alpha
                self._filtered_vx_world = alpha * raw_vx_world + (1 - alpha) * self._filtered_vx_world
                self._filtered_vy_world = alpha * raw_vy_world + (1 - alpha) * self._filtered_vy_world

            # Используем отфильтрованную скорость
            vx_world = self._filtered_vx_world
            vy_world = self._filtered_vy_world
            
            # DEBUG: выводим для диагностики
            
            # =========================================

            # ===== КОНТУР 2: Скорость → PWM =====
            # Целевая скорость в мировой СК (из position_callback или set_velocity_xy)
            tvx_world, tvy_world = self._target_velocity
            
            # Ограничение ускорения (в мировой СК)
            #print(f"tvx_world {tvx_world:.2} tvy_world {tvy_world:.2}")
            
            dvx = tvx_world - self._prev_target_vx
            dvy = tvy_world - self._prev_target_vy
            dt = 0.02  # 50 Гц
            accel = math.hypot(dvx / dt, dvy / dt)

            if accel > self._max_acceleration:
                scale = self._max_acceleration / accel
                tvx_world = self._prev_target_vx + dvx * scale
                tvy_world = self._prev_target_vy + dvy * scale

            self._prev_target_vx = tvx_world
            self._prev_target_vy = tvy_world

            # === ПРЕОБРАЗОВАНИЕ из мировой СК в СК ДРОНА ===
            # Матрица поворота (мир → дрон):
            # [ cos_yaw   -sin_yaw ] [vx_world]
            # [ sin_yaw    cos_yaw ] [vy_world]
            #
            # При yaw=0: vx_body = vx_world, vy_body = vy_world
            # При yaw=90°: vx_body = -vy_world, vy_body = vx_world

            # Целевая скорость в СК дрона
            tvx_body = tvx_world * cos_yaw - tvy_world * sin_yaw
            tvy_body = tvx_world * sin_yaw + tvy_world * cos_yaw

            # Текущая скорость в СК дрона
            vx_body = vx_world * cos_yaw - vy_world * sin_yaw
            vy_body = vx_world * sin_yaw + vy_world * cos_yaw

            # Ошибка скорости в СК ДРОНА (target - current)
            vel_error_x = tvx_body - vx_body
            vel_error_y = tvy_body - vy_body

            # DEBUG: выводим ошибки в СК дрона
            #print(f"DEBUG body: err=({vel_error_x:.3f}, {vel_error_y:.3f}) target_body=({tvx_body:.3f}, {tvy_body:.3f}) actual_body=({vx_body:.3f}, {vy_body:.3f})")

            # PID скорости выдает PWM
            # Отрицательная ошибка → положительный PWM (движемся к цели)
            self._pid_vel_pitch.update_control(vel_error_x)
            self._pid_vel_roll.update_control(vel_error_y)

            # Применяем коэффициенты направления и конвертируем в PWM
            pitch_pwm = int(vel_to_rc_signal(self._pid_vel_pitch.get_control() * self._pitch_direction))
            roll_pwm = int(vel_to_rc_signal(self._pid_vel_roll.get_control() * self._roll_direction))
            #print(f"PWM: roll={roll_pwm} pitch={pitch_pwm} pid_out=({self._pid_vel_pitch.get_control():.3f}, {self._pid_vel_roll.get_control():.3f})")
            # Обновляем roll/pitch, не трогая yaw
            _, _, y = self._rpy_vel_data
            self._rpy_vel_data = (roll_pwm, pitch_pwm, y)
            #print(f"DEBUG: target=({tvx_world:.3f}, {tvy_world:.3f}) actual=({vx_world:.3f}, {vy_world:.3f}) yaw={math.degrees(yaw):.1f}°")
        except Exception as e:
            logger.warning(f"Error in velocity_callback: {e}")

    def height_callback(self):
        """
        КОНТУР ВЫСОТЫ: Ошибка по высоте → Throttle (50 Гц).
        """
        if not self._simulator_alive or self._motors_locked:
            return

        try:
            # ОБНОВЛЯЕМ _altitude из кинематики!
            kin = self.get_sim_kinematics()
            if kin is not None:
                self._altitude = sim_to_api_distance(kin["location"][2])

            # Ошибка по высоте
            height_error = self._target_height - self._altitude + self._z_bias if self._target_height < 0.3 else self._target_height - self._altitude
            #print(f"height_error: {height_error}")

            # PID высоты выдает delta throttle
            self._pid_height.update_control(height_error)
            delta_throttle = int((self._pid_height.get_control() * 100))# // 2) * 2

            # Базовый throttle + коррекция от PID
            throttle_output = self._base_throttle_hover + delta_throttle

            # Ограничиваем throttle безопасными пределами
            throttle_output = max(self._min_throttle, min(throttle_output, self._max_throttle))

            self._throttle_data = throttle_output
            #print(f"height_error: {height_error} _throttle_data: {self._throttle_data}")

        except Exception as e:
            print(f"Error in height_callback: {e}")
            logger.warning(f"Error in height_callback: {e}")
        

    def set_velocity_xy(self, vx: float, vy: float, frame: str = "odom"):
        """
        Установить целевую скорость движения.

        Переключает в режим velocity и задает скорость.

        Args:
            vx: Скорость по оси X (м/с)
            vy: Скорость по оси Y (м/с)
            frame: Система координат скорости:
                   - "odom" (по умолчанию): скорость в мировой СК
                   - "base_link": скорость в СК дрона (будет преобразована в мировую)
        """
        self._control_mode = "velocity"

        # Если скорость задана в СК дрона, преобразуем в мировую СК
        if frame == "base_link":
            yaw = self._get_yaw_cw()
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)
            # Матрица поворота (дрон → мир):
            # [ cos_yaw   sin_yaw ] [vx_body]
            # [-sin_yaw   cos_yaw ] [vy_body]
            vx_world = vx * cos_yaw + vy * sin_yaw
            vy_world = -vx * sin_yaw + vy * cos_yaw
        else:
            vx_world = vx
            vy_world = vy

        # Ограничиваем скорость максимумом
        '''speed = math.hypot(vx_world, vy_world)
        
        if speed > self._max_velocity:
            vx_world = (vx_world / speed) * self._max_velocity
            vy_world = (vy_world / speed) * self._max_velocity'''

        vx_world = max(-self._max_velocity * 2, min(self._max_velocity * 2, vx_world))
        vy_world = max(-self._max_velocity * 2, min(self._max_velocity * 2, vy_world))

        self._target_velocity = (vx_world, vy_world)

    def set_position_mode(self):
        """Включить режим позиции (внешний контур активен)"""
        self._control_mode = "position"

    def set_velocity_mode(self):
        """Включить режим скорости (внешний контур отключен)"""
        self._control_mode = "velocity"

    def lock_motors(self):
        """Заблокировать обновление моторов (для setYaw и аварий)"""
        self._motors_locked = True

    def unlock_motors(self):
        """Разблокировать обновление моторов"""
        self._motors_locked = False
    
    def set_max_velocity(self, max_vel: float):
        """
        Установить максимальную скорость.
        
        Args:
            max_vel: Максимальная скорость (м/с)
        """
        self._max_velocity = max(0.1, min(max_vel, 5.0))  # Ограничение 0.1-5.0 м/с
        
        # Обновляем max_control в PID позиции
        self._pid_pos_x._max_control = self._max_velocity
        self._pid_pos_y._max_control = self._max_velocity
        
        logger.info(f"Max velocity set to {self._max_velocity} m/s")
    
    def set_max_acceleration(self, max_accel: float):
        """
        Установить максимальное ускорение.
        
        Args:
            max_accel: Максимальное ускорение (м/с²)
        """
        self._max_acceleration = max(0.5, min(max_accel, 10.0))  # Ограничение 0.5-10.0 м/с²
        logger.info(f"Max acceleration set to {self._max_acceleration} m/s²")
    
    def get_max_velocity(self) -> float:
        """Получить текущую максимальную скорость"""
        return self._max_velocity
    
    def get_max_acceleration(self) -> float:
        """Получить текущее максимальное ускорение"""
        return self._max_acceleration

    # ===== МЕТОДЫ ДЛЯ НАСТРОЙКИ НАПРАВЛЕНИЯ =====

    def set_direction_coefficients(self, roll: float = None, pitch: float = None, yaw: float = None):
        """
        Установить коэффициенты направления для roll, pitch и yaw.

        Args:
            roll: +1 или -1 (направление roll)
            pitch: +1 или -1 (направление pitch)
            yaw: +1 или -1 (направление yaw)

        Пример:
            # Если дрон летит назад вместо вперед:
            client.set_direction_coefficients(pitch=-1)

            # Если дрон летит влево вместо вправо:
            client.set_direction_coefficients(roll=-1)

            # Если дрон вращается против часовой вместо по часовой:
            client.set_direction_coefficients(yaw=-1)
        """
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
        """Инвертировать направление roll"""
        self._roll_direction *= -1
        logger.info(f"Roll direction inverted, now: {self._roll_direction}")

    def invert_pitch(self):
        """Инвертировать направление pitch"""
        self._pitch_direction *= -1
        logger.info(f"Pitch direction inverted, now: {self._pitch_direction}")

    def invert_yaw(self):
        """Инвертировать направление yaw"""
        self._yaw_direction *= -1
        logger.info(f"Yaw direction inverted, now: {self._yaw_direction}")

    def get_direction_coefficients(self) -> dict:
        """
        Получить текущие коэффициенты направления.

        Returns:
            Словарь с коэффициентами roll, pitch, yaw
        """
        return {
            "roll": self._roll_direction,
            "pitch": self._pitch_direction,
            "yaw": self._yaw_direction
        }

    # ===== МЕТОДЫ ДЛЯ НАСТРОЙКИ LOW-PASS ФИЛЬТРА =====

    def set_velocity_filter(self, alpha: float):
        """
        Установить коэффициент фильтрации скорости.

        Args:
            alpha: Коэффициент сглаживания (0.0 - 1.0)
                   - 0.0: Максимальное сглаживание (очень плавно, но медленно)
                   - 0.1-0.2: Сильное сглаживание (для очень шумных данных)
                   - 0.3-0.4: Умеренное сглаживание (рекомендуется)
                   - 0.5-0.7: Слабое сглаживание (для точных данных)
                   - 1.0: Без фильтрации (сырые данные)

        Пример:
            client.set_velocity_filter(0.3)  # Рекомендуемое значение
            client.set_velocity_filter(0.15)  # Если дрон очень нестабилен
            client.set_velocity_filter(0.6)   # Если данные очень точные
        """
        alpha = max(0.0, min(1.0, alpha))  # Ограничиваем 0-1
        self._vel_filter_alpha = alpha
        # Сбрасываем фильтр при изменении коэффициента
        self._vel_filter_initialized = False
        logger.info(f"Velocity filter alpha set to {alpha}")

    def get_velocity_filter_alpha(self) -> float:
        """Получить текущий коэффициент фильтрации скорости"""
        return self._vel_filter_alpha

    def reset_velocity_filter(self):
        """Сбросить состояние фильтра"""
        self._vel_filter_initialized = False
        self._filtered_vx_world = 0.0
        self._filtered_vy_world = 0.0
        logger.info("Velocity filter reset")

    # ==============================================

    # ==============================================

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
        # 1. Сброс PID (ОБЯЗАТЕЛЬНО для предотвращения скачков)
        '''self._pid_pos_x.reset()
        self._pid_pos_y.reset()
        self._pid_vel_pitch.reset()
        self._pid_vel_roll.reset()'''

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
            # Преобразуем из СК дрона в мировую СК
            # Согласно конвенции yaw:
            # При yaw=0: X дрона = X мира, Y дрона = Y мира
            # При yaw=90°: X дрона = -Y мира, Y дрона = X мира
            #
            # Матрица поворота (дрон → мир):
            # [ cos_yaw   sin_yaw ] [x_body]
            # [-sin_yaw   cos_yaw ] [y_body]
            yaw = self._get_yaw_cw()
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)
            
            target_x_world = cx + x * cos_yaw + y * sin_yaw
            target_y_world = cy - x * sin_yaw + y * cos_yaw
            self._target_position = (target_x_world, target_y_world)
        else:
            raise ValueError("frame must be 'odom' or 'base_link'")

        # 4. Ограничиваем максимальную скорость
        self._pid_pos_x._max_control = max_speed
        self._pid_pos_y._max_control = max_speed
        
        # 5. Разблокируем моторы (если были заблокированы)
        #self.unlock_motors()

        # 6. Ожидаем достижения цели

        tx, ty = self._target_position
        dist = math.hypot(tx - cx, ty - cy)
        
        timeout = 5.0 + (3 * dist / (self._max_velocity )) # секунд
        start_time = time.monotonic()
        prev_dist = None

        while time.monotonic() - start_time < timeout:
            if not self._simulator_alive:
                logger.warning("Simulator died during go_to_xy")
                return False

            if self._is_abort:
                self._is_abort = False
                logger.info("go_to_xy: aborted")
                return False

            # Проверяем расстояние до цели
            tx, ty = self._target_position
            dist = math.hypot(tx - cx, ty - cy)

            if dist < 0.1:  # 5 см допуск
                logger.info(f"Reached target: {x}, {y}")
                return True
            
            # Проверка на "залипание" - если расстояние не уменьшается
            if prev_dist is not None:
                if abs(dist - prev_dist) < 0.01 and dist > 0.5:  # Дрон не движется к цели
                    logger.warning(f"Drone stuck! dist={dist:.2f}, resetting PIDs...")
                    # Сброс PID и попытка снова
                    self._pid_pos_x.reset()
                    self._pid_pos_y.reset()
                    self._pid_vel_pitch.reset()
                    self._pid_vel_roll.reset()
                    prev_dist = None  # Сброс для новой проверки
                    time.sleep(0.1)
                    continue
            
            prev_dist = dist

            # Обновляем текущую позицию
            kin = self.get_sim_kinematics()
            if kin is not None:
                cx = sim_to_api_distance(kin["location"][0])
                cy = sim_to_api_distance(kin["location"][1])

            time.sleep(0.05)  # 20 Гц

        logger.warning(f"go_to_xy timeout: target={x}, {y}")
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
        
        Во время выполнения setYaw:
        - position_callback и velocity_callback блокируются
        - roll/pitch центрируются на 1500
        - После завершения PID позиции и скорости сбрасываются

        Args:
            yaw: Целевой угол (радианы, по часовой стрелке)

        Returns:
            True если достиг угла, False если таймаут
        """
        # Сброс PID yaw

        self._is_abort = False
        
        self._pid_yaw.reset()

        # Нормализуем цель
        goal = self._wrap_pi(yaw)

        # ===== БЛОКИРОВКА motor callbacks =====
        #self.lock_motors()

        # Центрируем roll/pitch чтобы дрон не летел
        r, p, _ = self._rpy_vel_data
        self._rpy_vel_data = (1500, 1500, r)  # Сохраняем текущий yaw PWM

        timeout = 10.0
        start_time = time.monotonic()
        tol = 0.025  # рад
        self._target_yaw = goal

        while time.monotonic() - start_time < timeout:
            if not self._simulator_alive:
                return False

            if self._is_abort:
                self._is_abort = False
                logger.info("setYaw: aborted")
                return False

            current = self._get_yaw_cw()
            error = self._wrap_pi(goal - current)

            if abs(error) < tol:
                # Достигли - центрируем yaw
                r, p, _ = self._rpy_vel_data
                self._rpy_vel_data = (r, p, 1500)

                # ===== РАЗБЛОКИРОВКА после успешного поворота =====
                #self.unlock_motors()

                # Сброс PID позиции и скорости для предотвращения скачка
                '''self._pid_pos_x.reset()
                self._pid_pos_y.reset()
                self._pid_vel_pitch.reset()
                self._pid_vel_roll.reset()'''

                return True

            time.sleep(0.05)

        # Таймаут
        r, p, _ = self._rpy_vel_data
        self._rpy_vel_data = (r, p, 1500)

        # ===== РАЗБЛОКИРОВКА при таймауте =====
        #self.unlock_motors()

        # Сброс PID позиции и скорости
        '''self._pid_pos_x.reset()
        self._pid_pos_y.reset()
        self._pid_vel_pitch.reset()
        self._pid_vel_roll.reset()'''

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
        
    def getUltrasonicById(self, sonic_id: int):
        with self._client_lock:
            return self._client.get_range_data(
                                            rangefinder_id=sonic_id,
                                            range_min=0.15,
                                            range_max=4,
                                            is_clear=True,
                                            range_error=0.0003
                                            ) 

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

        print("TAKE OFF")
        PERIOD = 0.05
        MIN_H = 0.00
        TAKEOFF_H = 1.05
        MAX_H = 5.00
        REACH_COEF = 0.93
        TIMEOUT_COEF = 10.0

        # Если дрон в режиме velocity, переключаемся в position и фиксируем текущую позицию
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

        self.set_target_height(tgt)

        climb = max(0.0, tgt - h_now)
        deadline = time.monotonic() + (TIMEOUT_COEF * climb if climb > 0.0 else TIMEOUT_COEF)

        while True:
            if self._is_abort:
                self._is_abort = False
                logger.info("takeoff: aborted")
                return False
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

        self._is_abort = False

        print("BOARDING")

        # Если дрон в режиме velocity, фиксируем текущую позицию и переключаемся в position
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
                return False
            curr_cmd = max(MIN_H, curr_cmd - STEP)
            self.set_target_height(curr_cmd)
            if time.monotonic() >= deadline:
                return False
            if not self._sleep_until(deadline, PERIOD):
                return False

        self._target_height = 0.0
        return True

    def setHeight(self, target_height: float) -> bool:
        """Установка высоты"""
        PERIOD = 0.05
        MIN_H = 0.00
        MAX_H = 5.00
        STEP = 0.005
        REACH_COEF = 0.93
        TIMEOUT_MIN = 15.0
        TIMEOUT_COEF = 10.0

        self._is_abort = False

        # Если дрон в режиме velocity, фиксируем текущую позицию и переключаемся в position
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
            self.set_target_height(tgt)
            deadline = time.monotonic() + TIMEOUT_MIN + TIMEOUT_COEF * max(0.0, tgt - h0)
            while True:
                if self._is_abort:
                    self._is_abort = False
                    logger.info("setHeight: aborted")
                    return False

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
                if self._is_abort:
                    self._is_abort = False
                    logger.info("setHeight: aborted")
                    return False
                curr_cmd = max(tgt, curr_cmd - STEP)
                self.set_target_height(curr_cmd)
                if time.monotonic() >= deadline:
                    return False
                if not self._sleep_until(deadline, PERIOD):
                    return False

        return True
    
    def set_camera_id(self, new_id: int):
        with self._client_lock:
            self.camera_id = new_id
    
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
        self.set_velocity_xy(x, y, frame="odom")
    
    def setVelXYYaw(self, x, y, yaw):
        """
        Устаревший метод - используйте set_velocity_xy и setYaw отдельно.
        
        Этот метод теперь просто делегирует set_velocity_xy,
        а yaw игнорируется (для обратной совместимости).
        
        Args:
            x: Скорость по X в мировой СК (м/с)
            y: Скорость по Y в мировой СК (м/с)
            yaw: Игнорируется (оставлен для обратной совместимости)
        """
        logger.warning("setVelXYYaw is deprecated, use set_velocity_xy + setYaw")
        # Преобразуем в мировую СК (предполагаем, что входные данные в СК дрона)
        self.set_velocity_xy(x, y, frame="base_link")
    
    def armDrone(self):
        """
        Арминг дрона.
        
        ВАЖНО: Сначала гарантируем throttle = 1000, потом армим!
        """
        # 1. СНАЧАЛА устанавливаем throttle в 1000 (минимум)
        self._throttle_data = 1000
        
        # 2. Сбрасываем PID высоты чтобы они не влияли на throttle
        #self._pid_height_pos.reset()
        #self._pid_height_vel.reset()
        
        # 3. Сбрасываем целевую вертикальную скорость
        self._target_height_vel = 0.0
        
        # 4. ТОЛЬКО ПОТОМ армим
        self._armed_flag = True
        self._arm_data = 2000

        # 5. Даем время на отправку RC команды с правильными значениями
        time.sleep(0.2)
        #self._height_pos_timer.start()
        #self._height_vel_timer.start()
        #self._height_pos_timer.start()
        #self._height_vel_timer.start()

        self._height_timer.start()

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
        self._base_throttle_hover = 1500
        self.unlock_motors()
        

    def posholdOff(self):
        self._poshold_flag = False
        self._nav_mode = 1000
        self._throttle_data = 1000
        self._base_throttle_hover = 1500
        self._pid_height.reset()
        self.lock_motors()

    def clamp_rc(self, data):
        return max(min(data, 2000), 1000)

    def clamp_rc_list(self, data: Iterable):
        return [self.clamp_rc(rc) for rc in data]

    def round_data(self, iterable):
        return map(lambda x: round(x, 3), iterable)

    def set_target_height(self, height):
        """
        Установка целевой высоты.
        """
        self._target_height = height
        # Сбрасываем PID высоты
        self._pid_height.reset()
    
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
        with self._client_lock:
            self._is_abort = True

            kin = self.get_sim_kinematics()
            if kin is not None:
                cx = sim_to_api_distance(kin["location"][0])
                cy = sim_to_api_distance(kin["location"][1])
                self._target_position = (cx, cy)
                self._control_mode = "position"
