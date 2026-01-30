import numpy as np

class PID:
    def __init__(self, kp, ki, kd, max_control=float('inf'), i_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_control = max_control

        # лимит интегратора (в «единицах ошибки * тик»); None = без лимита
        self.i_limit = i_limit

        self.current_error = 0.0
        self.previous_error = 0.0
        self.integral = 0.0
        self.derivative = 0.0
        self.control = 0.0
        

    def update_control(self, current_error, reset_prev=False):
        if reset_prev:
            self.previous_error = 0.0
            self.integral = 0.0

        self.previous_error = self.current_error
        self.current_error = current_error

        # накапливаем интеграл и жёстко ограничиваем его по i_limit
        self.integral += self.current_error
        if self.i_limit is not None:
            if self.integral > self.i_limit:
                self.integral = self.i_limit
            elif self.integral < -self.i_limit:
                self.integral = -self.i_limit

        # дифференциал по тикам
        self.derivative = self.current_error - self.previous_error

        # PID-выход
        u = (
            self.kp * self.current_error +
            self.ki * self.integral +
            self.kd * self.derivative
        )

        # сатурация выхода
        if u > self.max_control:
            u = self.max_control
        elif u < -self.max_control:
            u = -self.max_control

        self.control = u

    def get_control(self):
        return self.control

    def reset(self):
        self.current_error = 0.0
        self.previous_error = 0.0
        self.integral = 0.0
        self.derivative = 0.0
        self.control = 0.0


class AdaptivePID:
    def __init__(self, error_bounds: list, kp_values: list, ki_values: list, kd_values: list, max_control: float = 1.0, max_i_anti_winup: float = np.inf):
        self.error_bounds = error_bounds
        self.kp_values = kp_values
        self.ki_values = ki_values
        self.kd_values = kd_values
        self.max_control = max_control
        self.max_i_anti_winup = max_i_anti_winup

        self.current_error = 0.0
        self.previous_error = 0.0
        self.integral = 0.0
        self.derivative = 0.0
        self.control = 0.0

    def update(self, current_error: float, dt: float):
        self.previous_error = self.current_error
        self.current_error = current_error

        # Найти текущую зону адаптации
        for i in range(len(self.error_bounds)):
            if abs(current_error) < self.error_bounds[i]:
                zone = i
                break
        else:
            zone = len(self.error_bounds)

        # Вычислить производную
        self.derivative = (self.current_error - self.previous_error) / dt

        self.integral += current_error * dt
        # Накапливать интеграл
        '''if zone <= 1:  # только вблизи цели
            self.integral += current_error * dt
        else:
            self.integral *= 0.9  # мягкое забывание'''
    

        # anti-windup
        self.integral = max(min(self.integral, self.max_i_anti_winup), -self.max_i_anti_winup)
        # Вычислить выходное значение PID-регулятора для текущей зоны
        u = (
            self.kp_values[zone] * self.current_error +
            self.ki_values[zone] * self.integral +
            self.kd_values[zone] * self.derivative
        )

        # Сатурация выхода
        if u > self.max_control:
            u = self.max_control
        elif u < -self.max_control:
            u = -self.max_control

        self.control = u

    def get_control(self):
        return self.control

    def reset(self):
        self.current_error = 0.0
        self.previous_error = 0.0
        self.integral = 0.0
        self.derivative = 0.0
        self.control = 0.0