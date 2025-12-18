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