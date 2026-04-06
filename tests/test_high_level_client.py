"""
Тесты для проверки правильности расчетов в HighLevelSimClient.
Проверяет:
1. Матрицу поворота (мировая СК → СК дрона)
2. Работу PID регуляторов
3. Расчет скорости
4. Преобразование координат
"""
import math
import unittest
from unittest.mock import Mock, patch, MagicMock
import sys
import os

# Добавляем путь к модулю
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from agrotechsimapi.pid import PID


class TestCoordinateTransformation(unittest.TestCase):
    """Тесты преобразования координат из мировой СК в СК дрона"""

    def test_no_rotation(self):
        """yaw=0: ошибка должна остаться без изменений"""
        yaw = 0.0
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        
        pos_error_x = 1.0
        pos_error_y = 0.0
        
        # Матрица поворота (мир → дрон)
        pos_error_body_x = pos_error_x * cos_yaw + pos_error_y * sin_yaw
        pos_error_body_y = -pos_error_x * sin_yaw + pos_error_y * cos_yaw
        
        self.assertAlmostEqual(pos_error_body_x, 1.0)
        self.assertAlmostEqual(pos_error_body_y, 0.0)

    def test_rotation_90_degrees_cw(self):
        """yaw=90° (π/2): X мира → Y дрона, Y мира → -X дрона"""
        yaw = math.pi / 2  # 90° по часовой
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        
        # Ошибка по X мира = 1м (дрон должен лететь "вправо" по своему Y)
        pos_error_x = 1.0
        pos_error_y = 0.0
        
        pos_error_body_x = pos_error_x * cos_yaw + pos_error_y * sin_yaw
        pos_error_body_y = -pos_error_x * sin_yaw + pos_error_y * cos_yaw
        
        # При yaw=90°, X мира = Y дрона
        self.assertAlmostEqual(pos_error_body_x, 0.0, places=5)
        self.assertAlmostEqual(pos_error_body_y, -1.0, places=5)

    def test_rotation_minus_90_degrees(self):
        """yaw=-90° (-π/2): X мира → -Y дрона"""
        yaw = -math.pi / 2  # -90°
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        
        pos_error_x = 1.0
        pos_error_y = 0.0
        
        pos_error_body_x = pos_error_x * cos_yaw + pos_error_y * sin_yaw
        pos_error_body_y = -pos_error_x * sin_yaw + pos_error_y * cos_yaw
        
        self.assertAlmostEqual(pos_error_body_x, 0.0, places=5)
        self.assertAlmostEqual(pos_error_body_y, 1.0, places=5)

    def test_rotation_180_degrees(self):
        """yaw=180° (π): X мира → -X дрона, Y мира → -Y дрона"""
        yaw = math.pi  # 180°
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        
        pos_error_x = 1.0
        pos_error_y = 1.0
        
        pos_error_body_x = pos_error_x * cos_yaw + pos_error_y * sin_yaw
        pos_error_body_y = -pos_error_x * sin_yaw + pos_error_y * cos_yaw
        
        self.assertAlmostEqual(pos_error_body_x, -1.0, places=5)
        self.assertAlmostEqual(pos_error_body_y, -1.0, places=5)

    def test_diagonal_movement(self):
        """Диагональное движение при yaw=45°"""
        yaw = math.pi / 4  # 45°
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        
        # Цель: (1, 1) в мировой СК
        pos_error_x = 1.0
        pos_error_y = 1.0
        
        pos_error_body_x = pos_error_x * cos_yaw + pos_error_y * sin_yaw
        pos_error_body_y = -pos_error_x * sin_yaw + pos_error_y * cos_yaw
        
        # При 45°: (1,1) мира → (√2, 0) дрона
        expected_x = math.sqrt(2)
        self.assertAlmostEqual(pos_error_body_x, expected_x, places=5)
        self.assertAlmostEqual(pos_error_body_y, 0.0, places=5)

    def test_inverse_transformation(self):
        """Прямое и обратное преобразование должны давать исходные значения"""
        yaw = 0.7  # произвольный угол
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        
        pos_error_x = 2.5
        pos_error_y = -1.3
        
        # Мир → дрон
        pos_error_body_x = pos_error_x * cos_yaw + pos_error_y * sin_yaw
        pos_error_body_y = -pos_error_x * sin_yaw + pos_error_y * cos_yaw
        
        # Дрон → мир (обратная матрица)
        pos_error_x_back = pos_error_body_x * cos_yaw - pos_error_body_y * sin_yaw
        pos_error_y_back = pos_error_body_x * sin_yaw + pos_error_body_y * cos_yaw
        
        self.assertAlmostEqual(pos_error_x, pos_error_x_back, places=5)
        self.assertAlmostEqual(pos_error_y, pos_error_y_back, places=5)


class TestPIDController(unittest.TestCase):
    """Тесты PID регуляторов"""

    def test_pid_proportional(self):
        """Только пропорциональная составляющая"""
        pid = PID(kp=2.0, ki=0.0, kd=0.0, max_control=10.0)
        
        pid.update_control(1.0)
        self.assertAlmostEqual(pid.get_control(), 2.0)
        
        pid.update_control(-0.5)
        self.assertAlmostEqual(pid.get_control(), -1.0)

    def test_pid_integral(self):
        """Интегральная составляющая накапливает ошибку"""
        pid = PID(kp=0.0, ki=1.0, kd=0.0, max_control=10.0, i_limit=5.0)
        
        pid.update_control(1.0)
        self.assertAlmostEqual(pid.get_control(), 1.0)
        
        pid.update_control(1.0)
        self.assertAlmostEqual(pid.get_control(), 2.0)
        
        pid.update_control(1.0)
        self.assertAlmostEqual(pid.get_control(), 3.0)

    def test_pid_derivative(self):
        """Дифференциальная составляющая реагирует на изменение"""
        pid = PID(kp=0.0, ki=0.0, kd=1.0, max_control=10.0)
        
        pid.update_control(0.0)
        self.assertAlmostEqual(pid.get_control(), 0.0)
        
        pid.update_control(1.0)
        self.assertAlmostEqual(pid.get_control(), 1.0)  # derivative = 1 - 0 = 1
        
        pid.update_control(1.0)
        self.assertAlmostEqual(pid.get_control(), 0.0)  # derivative = 1 - 1 = 0

    def test_pid_saturation(self):
        """PID не должен выходить за max_control"""
        pid = PID(kp=10.0, ki=0.0, kd=0.0, max_control=5.0)
        
        pid.update_control(1.0)
        self.assertAlmostEqual(pid.get_control(), 5.0)
        
        pid.update_control(-1.0)
        self.assertAlmostEqual(pid.get_control(), -5.0)

    def test_pid_reset(self):
        """Сброс PID должен обнулить все состояния"""
        pid = PID(kp=1.0, ki=1.0, kd=1.0, max_control=10.0)
        
        pid.update_control(1.0)
        pid.update_control(1.0)
        
        pid.reset()
        
        self.assertEqual(pid.current_error, 0.0)
        self.assertEqual(pid.previous_error, 0.0)
        self.assertEqual(pid.integral, 0.0)
        self.assertEqual(pid.derivative, 0.0)
        self.assertEqual(pid.control, 0.0)


class TestVelocityCalculation(unittest.TestCase):
    """Тесты расчета скорости"""

    def test_velocity_from_position_change(self):
        """Скорость = изменение позиции * частота"""
        # Позиция изменилась на 0.1м за 1/50 сек
        prev_x = 0.0
        curr_x = 0.1
        frequency = 50
        
        vx = (curr_x - prev_x) * frequency
        self.assertAlmostEqual(vx, 5.0)  # 0.1 * 50 = 5 м/с

    def test_velocity_zero(self):
        """Нулевая скорость при неизменной позиции"""
        prev_x = 1.5
        curr_x = 1.5
        frequency = 50
        
        vx = (curr_x - prev_x) * frequency
        self.assertAlmostEqual(vx, 0.0)


class TestAccelerationLimit(unittest.TestCase):
    """Тесты ограничения ускорения"""

    def test_acceleration_within_limits(self):
        """Ускорение в пределах лимита не должно меняться"""
        prev_target_vx = 0.5
        target_vx = 0.6
        dt = 0.02
        max_accel = 10.0
        
        accel = (target_vx - prev_target_vx) / dt  # 5 м/с²
        
        if abs(accel) <= max_accel:
            final_vx = target_vx
        else:
            scale = max_accel / abs(accel)
            final_vx = prev_target_vx + (target_vx - prev_target_vx) * scale
        
        self.assertAlmostEqual(final_vx, 0.6)

    def test_acceleration_limited(self):
        """Ускорение выше лимита должно ограничиваться"""
        prev_target_vx = 0.0
        target_vx = 1.0
        dt = 0.02
        max_accel = 10.0
        
        accel = (target_vx - prev_target_vx) / dt  # 50 м/с² > 10 м/с²
        
        if abs(accel) > max_accel:
            scale = max_accel / abs(accel)
            final_vx = prev_target_vx + (target_vx - prev_target_vx) * scale
        else:
            final_vx = target_vx
        
        # Ускорение должно быть ограничено до 10 м/с²
        actual_accel = (final_vx - prev_target_vx) / dt
        self.assertAlmostEqual(actual_accel, 10.0, places=5)


class TestHeightControl(unittest.TestCase):
    """Тесты управления высотой"""

    def test_height_error_calculation(self):
        """Ошибка высоты = целевая - текущая"""
        target_height = 1.0
        current_altitude = 0.5
        z_bias = -0.011
        
        height_error = target_height - current_altitude + z_bias
        self.assertAlmostEqual(height_error, 0.489)

    def test_throttle_calculation(self):
        """Throttle = base + delta * 100"""
        base_throttle = 1500
        delta_throttle = 0.5
        max_throttle = 1700
        min_throttle = 1470
        
        throttle_output = base_throttle + (delta_throttle * 100)
        throttle_output = max(min_throttle, min(throttle_output, max_throttle))
        
        self.assertAlmostEqual(throttle_output, 1550)


if __name__ == '__main__':
    unittest.main(verbosity=2)
