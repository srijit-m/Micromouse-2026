class PID():

    def __init__(self, kp, kd, ki = 0.0):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.prev_error = 0
        self.integral = 0

    def update(self, error, dt):
        # Derivative term
        derivative = (error - self.prev_error) / dt

        #Integral Term
        self.integral = error * dt

        # PID output
        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        # Save error for next loop
        self.prev_error = error

        return output

    def reset(self):
        self.prev_error = 0
        self.integral = 0
