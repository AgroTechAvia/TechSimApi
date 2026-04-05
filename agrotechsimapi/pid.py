import numpy as np

class PID:
    """
    PID-регулятор с опциональной экспоненциальной зависимостью.
    
    Args:
        kp, ki, kd: Коэффициенты пропорциональной, интегральной и дифференциальной составляющих
        max_control: Максимальное значение выхода
        i_limit: Лимит интеграла (None = без лимита)
        is_exp: Если True, используется экспоненциальная зависимость (нелинейная)
        exp_factor: Показатель степени для экспоненциальной зависимости (по умолчанию 2.0)
    """
    def __init__(self, kp, ki, kd, max_control=float('inf'), i_limit=None, 
                 is_exp=False, exp_factor=1.0, processing_func = lambda x:x*1):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_control = max_control

        # лимит интегратора (в «единицах ошибки * тик»); None = без лимита
        self.i_limit = i_limit
        
        # Параметры для экспоненциальной зависимости
        self.is_exp = is_exp
        self.exp_factor = exp_factor
        self._processing_func = processing_func

        self.current_error = 0.0
        self.previous_error = 0.0
        self.integral = 0.0
        self.derivative = 0.0
        self.control = 0.0


    def _apply_nonlinearity(self, value):
        """
        Применяет нелинейное преобразование к значению.
        
        При is_exp=True использует экспоненциальную зависимость:
        sign(value) * |value|^exp_factor
        
        Это дает:
        - Меньшую реакцию на малые ошибки (плавнее старт)
        - Большую реакцию на большие ошибки (быстрее коррекция)
        """
        if self.is_exp:
            return np.sign(value) * (self.exp_factor ** abs(value)) #* np.exp(abs(value * self.exp_factor))  #(abs(value) ** self.exp_factor)
        return value


    def update_control(self, current_error, reset_prev=False):
        if reset_prev:
            self.previous_error = 0.0
            self.integral = 0.0

        self.previous_error = self.current_error
        self.current_error = current_error

        # Применяем нелинейность к ошибке
        #error_nl = self._apply_nonlinearity(self.current_error)
        #prev_error_nl = self._apply_nonlinearity(self.previous_error)
        error_nl = self._processing_func(self.current_error)
        prev_error_nl = self._processing_func(self.previous_error)
        # накапливаем интеграл и жёстко ограничиваем его по i_limit
        self.integral += error_nl
        if self.i_limit is not None:
            if self.integral > self.i_limit:
                self.integral = self.i_limit
            elif self.integral < -self.i_limit:
                self.integral = -self.i_limit

        # дифференциал по тикам
        self.derivative = error_nl - prev_error_nl

        # PID-выход
        u = (
            self.kp * error_nl +
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