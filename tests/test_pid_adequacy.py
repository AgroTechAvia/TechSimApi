"""
Тесты для проверки адекватности текущих коэффициентов PID и processing_func.
Моделирует реальное поведение регуляторов в системе управления.
"""
import math
import unittest
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from agrotechsimapi.pid import PID


class TestCurrentPIDCoefficients(unittest.TestCase):
    """Тесты текущих коэффициентов PID регуляторов"""

    def setUp(self):
        """Настройка текущих коэффициентов из high_level_client.py"""
        # Position PIDs
        self.max_velocity = 0.33
        self.pid_pos_x = PID(kp=0.33, ki=0.0, kd=7.15, 
                             max_control=self.max_velocity, i_limit=0.1**3)
        self.pid_pos_y = PID(kp=0.33, ki=0.0, kd=7.15, 
                             max_control=self.max_velocity, i_limit=0.1**3)
        
        # Velocity PIDs с processing_func
        self.vel_pid_processing = lambda x: ((2/(1+(2.7**(-x * 4)))) - 1)
        self.pid_vel_x = PID(kp=5, ki=0.0, kd=7.0, 
                             i_limit=0.1**3, processing_func=self.vel_pid_processing)
        self.pid_vel_y = PID(kp=5, ki=0.0, kd=7.0, 
                             i_limit=0.1**3, processing_func=self.vel_pid_processing)

    def test_position_pid_response_1m_error(self):
        """Реакция PID позиции на ошибку 1м"""
        error = 1.0
        self.pid_pos_x.update_control(error)
        output = self.pid_pos_x.get_control()
        
        print(f"\n[Position PID] Ошибка: {error}м → Выход: {output:.3f} м/с")
        print(f"  Ограничение: max_control={self.max_velocity} м/с")
        
        # Выход должен быть ограничен max_velocity
        self.assertLessEqual(abs(output), self.max_velocity + 0.001)

    def test_position_pid_response_small_error(self):
        """Реакция PID позиции на малую ошибку 0.1м"""
        error = 0.1
        self.pid_pos_x.update_control(error)
        output = self.pid_pos_x.get_control()
        
        print(f"\n[Position PID] Ошибка: {error}м → Выход: {output:.3f} м/с")
        
        # При kd=7.15 и ошибке 0.1, derivative = 0.1
        # output = kp*error + kd*derivative = 0.33*0.1 + 7.15*0.1 = 0.033 + 0.715 = 0.748
        # Но это больше max_control=0.33, так что будет ограничено
        self.assertLessEqual(abs(output), self.max_velocity + 0.001)

    def test_position_pid_derivative_dominance(self):
        """Проверка доминирования дифференциальной составляющей"""
        # Первый вызов
        self.pid_pos_x.update_control(0.5)
        output1 = self.pid_pos_x.get_control()
        
        # Второй вызов (та же ошибка, derivative = 0)
        self.pid_pos_x.update_control(0.5)
        output2 = self.pid_pos_x.get_control()
        
        print(f"\n[Position PID] Derivative dominance test:")
        print(f"  Первый вызов (error=0.5): {output1:.3f} м/с")
        print(f"  Второй вызов (error=0.5, deriv=0): {output2:.3f} м/с")
        
        # При kd=7.15, первый вызов должен быть сильно больше второго
        # output1 = 0.33*0.5 + 7.15*0.5 = 0.165 + 3.575 = 3.74 → ограничено до 0.33
        # output2 = 0.33*0.5 + 7.15*0 = 0.165
        self.assertAlmostEqual(output2, 0.165, places=3)

    def test_velocity_pid_processing_func(self):
        """Проверка processing_func для PID скорости"""
        # processing_func = lambda x: ((2/(1+(2.7**(-x * 4)))) - 1)
        # Это сигмоида, которая ограничивает выход в диапазоне [-1, 1]
        
        test_values = [-2.0, -1.0, -0.5, 0.0, 0.5, 1.0, 2.0]
        
        print(f"\n[Velocity PID] processing_func (сигмоида):")
        for val in test_values:
            result = self.vel_pid_processing(val)
            print(f"  Вход: {val:5.2f} → Выход: {result:5.3f}")
            
            # Выход должен быть в диапазоне [-1, 1]
            self.assertGreaterEqual(result, -1.0)
            self.assertLessEqual(result, 1.0)

    def test_velocity_pid_full_response(self):
        """Полный тест PID скорости с processing_func"""
        # Симуляция: целевая скорость 0.33 м/с, текущая 0.0
        target_vx = 0.33
        current_vx = 0.0
        vel_error = target_vx - current_vx  # 0.33
        
        self.pid_vel_x.update_control(vel_error)
        output = self.pid_vel_x.get_control()
        
        print(f"\n[Velocity PID] Полный тест:")
        print(f"  Целевая скорость: {target_vx} м/с")
        print(f"  Текущая скорость: {current_vx} м/с")
        print(f"  Ошибка скорости: {vel_error} м/с")
        print(f"  Выход PID: {output:.3f}")
        
        # processing_func ограничивает выход в [-1, 1]
        # kp=5, kd=7.0
        # output = processing_func(kp*error + kd*derivative)
        # При первом вызове derivative = error = 0.33
        # raw_output = 5*0.33 + 7.0*0.33 = 1.65 + 2.31 = 3.96
        # processing_func(3.96) ≈ 1.0 (сигмоида насыщается)
        self.assertGreaterEqual(output, -1.0)
        self.assertLessEqual(output, 1.0)

    def test_cascade_system_response(self):
        """Тест каскадной системы: позиция → скорость → PWM"""
        print(f"\n[Cascade System] Полный тест каскада:")
        
        # Шаг 1: Ошибка позиции 1м
        pos_error = 1.0
        self.pid_pos_x.update_control(pos_error)
        target_velocity = self.pid_pos_x.get_control()
        
        print(f"  1. Ошибка позиции: {pos_error}м")
        print(f"     → Целевая скорость: {target_velocity:.3f} м/с")
        
        # Шаг 2: PID скорости реагирует на разницу
        current_velocity = 0.0
        vel_error = target_velocity - current_velocity
        
        self.pid_vel_x.update_control(vel_error)
        pwm_output = self.pid_vel_x.get_control()
        
        print(f"  2. Ошибка скорости: {vel_error:.3f} м/с")
        print(f"     → Выход PWM (после processing_func): {pwm_output:.3f}")
        
        # Шаг 3: Преобразование в RC сигнал
        # vel_to_rc_signal обычно: 1500 + output * 500
        rc_signal = 1500 + pwm_output * 500
        print(f"  3. RC сигнал: {rc_signal:.0f}")
        
        # Проверки
        self.assertLessEqual(abs(target_velocity), self.max_velocity + 0.001)
        self.assertGreaterEqual(pwm_output, -1.0)
        self.assertLessEqual(pwm_output, 1.0)
        self.assertGreaterEqual(rc_signal, 1000)
        self.assertLessEqual(rc_signal, 2000)

    def test_position_pid_stability_over_time(self):
        """Стабильность PID позиции при постоянной ошибке"""
        print(f"\n[Position PID] Стабильность при постоянной ошибке 0.5м:")
        
        errors = []
        outputs = []
        
        for i in range(10):
            self.pid_pos_x.update_control(0.5)
            output = self.pid_pos_x.get_control()
            errors.append(0.5)
            outputs.append(output)
            
            if i < 3 or i == 9:
                print(f"  Шаг {i+1}: output={output:.3f} м/с")
        
        # После первого шага derivative = 0, output = kp * error = 0.33 * 0.5 = 0.165
        self.assertAlmostEqual(outputs[-1], 0.165, places=3)

    def test_velocity_pid_step_response(self):
        """Реакция PID скорости на ступенчатое изменение"""
        print(f"\n[Velocity PID] Ступенчатая реакция:")
        
        # Ступенчатое изменение ошибки скорости
        step_errors = [0.0, 0.33, 0.33, 0.33, 0.0, 0.0, 0.0]
        outputs = []
        
        for i, error in enumerate(step_errors):
            self.pid_vel_x.update_control(error)
            output = self.pid_vel_x.get_control()
            outputs.append(output)
            print(f"  Шаг {i+1}: error={error:.2f} → output={output:.3f}")
        
        # При processing_func (сигмоида), выход должен быть плавным
        # и ограниченным в [-1, 1]
        for output in outputs:
            self.assertGreaterEqual(output, -1.0)
            self.assertLessEqual(output, 1.0)


class TestProcessingFunctionAnalysis(unittest.TestCase):
    """Анализ processing_func для PID скорости"""

    def test_sigmoid_properties(self):
        """Проверка свойств сигмоиды"""
        processing_func = lambda x: ((2/(1+(2.7**(-x * 4)))) - 1)
        
        print(f"\n[Sigmoid Analysis] processing_func:")
        
        # 1. Нелинейность
        test_points = [-2, -1, -0.5, 0, 0.5, 1, 2]
        print("  Нелинейность:")
        for x in test_points:
            y = processing_func(x)
            print(f"    x={x:5.2f} → y={y:5.3f}")
        
        # 2. Монотонность
        outputs = [processing_func(x) for x in test_points]
        for i in range(len(outputs)-1):
            self.assertLess(outputs[i], outputs[i+1])
        
        # 3. Насыщение
        self.assertAlmostEqual(processing_func(10), 1.0, places=2)
        self.assertAlmostEqual(processing_func(-10), -1.0, places=2)
        
        # 4. Прохождение через ноль
        self.assertAlmostEqual(processing_func(0), 0.0, places=2)

    def test_sigmoid_derivative(self):
        """Производная сигмоиды (чувствительность)"""
        processing_func = lambda x: ((2/(1+(2.7**(-x * 4)))) - 1)
        
        print(f"\n[Sigmoid Derivative] Чувствительность:")
        
        dx = 0.01
        test_points = [-1.0, -0.5, 0.0, 0.5, 1.0]
        
        for x in test_points:
            dy = (processing_func(x + dx) - processing_func(x - dx)) / (2 * dx)
            print(f"  x={x:5.2f} → dy/dx={dy:5.3f}")
        
        # Максимальная чувствительность в нуле
        # dy/dx при x=0 для сигмоиды с коэффициентом 4: ≈ 4.0
        # Но наша сигмоида: 2/(1+e^(-4x)) - 1
        # dy/dx = 2 * 4 * e^(-4x) / (1+e^(-4x))^2
        # при x=0: dy/dx = 8 / 4 = 2.0
        derivative_at_zero = (processing_func(dx) - processing_func(-dx)) / (2 * dx)
        self.assertAlmostEqual(derivative_at_zero, 2.0, places=1)


class TestSystemAdequacy(unittest.TestCase):
    """Оценка адекватности системы управления"""

    def test_position_pid_adequacy(self):
        """Оценка адекватности PID позиции"""
        pid_pos = PID(kp=0.33, ki=0.0, kd=7.15, max_control=0.33, i_limit=0.001)
        
        print(f"\n[Position PID Adequacy] Оценка:")
        print(f"  kp={pid_pos.kp}, ki={pid_pos.ki}, kd={pid_pos.kd}")
        print(f"  max_control={pid_pos.max_control}")
        
        # Тест на ошибку 0.5м
        pid_pos.update_control(0.5)
        output = pid_pos.get_control()
        
        print(f"  Ошибка 0.5м → Выход: {output:.3f} м/с")
        print(f"  Время достижения 0.5м: {0.5 / output:.1f} сек (при постоянной скорости)")
        
        # Оценка: kd доминирует, что вызывает резкие скачки
        # При ошибке 0.5м: derivative = 0.5, output = 0.33*0.5 + 7.15*0.5 = 3.74 → 0.33
        # Это означает, что система всегда работает на максимальной скорости
        self.assertLessEqual(output, 0.33)

    def test_velocity_pid_adequacy(self):
        """Оценка адекватности PID скорости"""
        vel_processing = lambda x: ((2/(1+(2.7**(-x * 4)))) - 1)
        pid_vel = PID(kp=5, ki=0.0, kd=7.0, i_limit=0.001, 
                      processing_func=vel_processing)
        
        print(f"\n[Velocity PID Adequacy] Оценка:")
        print(f"  kp={pid_vel.kp}, ki={pid_vel.ki}, kd={pid_vel.kd}")
        print(f"  processing_func: сигмоида с коэффициентом 4")
        
        # Тест на ошибку скорости 0.33 м/с
        pid_vel.update_control(0.33)
        output = pid_vel.get_control()
        
        print(f"  Ошибка скорости 0.33 м/с → Выход: {output:.3f}")
        print(f"  RC сигнал: {1500 + output * 500:.0f}")
        
        # Сигмоида ограничивает выход, что хорошо для стабильности
        self.assertGreaterEqual(output, -1.0)
        self.assertLessEqual(output, 1.0)

    def test_cascade_system_adequacy(self):
        """Оценка адекватности каскадной системы"""
        print(f"\n[Cascade System Adequacy] Общая оценка:")
        
        # Параметры системы
        max_velocity = 0.33  # м/с
        max_acceleration = 0.5  # м/с²
        
        print(f"  Максимальная скорость: {max_velocity} м/с")
        print(f"  Максимальное ускорение: {max_acceleration} м/с²")
        print(f"  Время разгона до макс: {max_velocity / max_acceleration:.2f} сек")
        print(f"  Путь разгона: {0.5 * max_acceleration * (max_velocity / max_acceleration)**2:.2f} м")
        
        # Оценка: при max_velocity=0.33 м/с и max_acceleration=0.5 м/с²
        # Время разгона: 0.66 сек
        # Путь разгона: 0.11 м
        # Это означает, что дрон будет разгоняться медленно и плавно
        # Но для точного позиционирования это может быть слишком медленно
        
        self.assertGreater(max_velocity, 0.0)
        self.assertGreater(max_acceleration, 0.0)


if __name__ == '__main__':
    # Запуск с подробным выводом
    unittest.main(verbosity=2)
