
MIN_NUM = float('-inf')
MAX_NUM = float('inf')


class PID(object):
    def __init__(self, kp, ki, kd, mn=MIN_NUM, mx=MAX_NUM):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min = mn
        self.max = mx

        self.int_val = self.last_int_val = self.last_error = 0.

    def reset(self):
        self.int_val = 0.0
        self.last_int_val = 0.0

    def step(self, error, sample_time, steer_goal, max_steer):
        self.min = -max_steer - steer_goal
        self.max = max_steer - steer_goal
        self.last_int_val = self.int_val

        integral = self.int_val + error * sample_time;
        kintegral = self.ki * integral
        if (kintegral > self.max):
            integral = self.max / self.ki
        if (kintegral < self.min):
            integral = self.min / self.ki
        derivative = (error - self.last_error) / sample_time;

        y = self.kp * error + self.ki * self.int_val + self.kd * derivative;
        val = max(self.min, min(y, self.max))

        if val > self.max:
            val = self.max
        elif val < self.min:
            val = self.min
        self.int_val = integral
        self.last_error = error

        return val, derivative
