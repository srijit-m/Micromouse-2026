class PID():
    def __init__(self, kp, kd, dt):
        self.kp = kp
        self.kd = kd
        self.dt = dt
        self.prev_error = 0
    def update(self, error):
        # Derivative term
        derivative = (error - self.prev_error) / self.dt

        # PD output
        output = self.kp * error + self.kd * derivative

        # Save error for next loop
        self.prev_error = error

        return output
    