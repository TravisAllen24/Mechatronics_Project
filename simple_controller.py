class PIDController:
    """A standard PID controller with 'Derivative on Measurement', no NumPy."""

    def __init__(self, Kp, Ki, Kd, setpoint):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint

        self._integral_sum = 0.0
        self._last_time_ms = -1

        # Integral windup prevention limits
        self.integral_min = -1.0
        self.integral_max = 1.0

    def calculate(self, process_variable, velocity, current_time_ms):
        """
        Calculates the control signal using time in milliseconds.
        """
        if self._last_time_ms < 0:
            self._last_time_ms = current_time_ms
            return 0.0

        dt_s = (current_time_ms - self._last_time_ms) / 1000.0
        if dt_s == 0:
            return 0.0

        error = self.setpoint - process_variable

        # Proportional term
        p_term = self.Kp * error

        # Integral term (with anti-windup)
        self._integral_sum += error * dt_s
        # Clamp the integral sum
        if self._integral_sum > self.integral_max:
            self._integral_sum = self.integral_max
        elif self._integral_sum < self.integral_min:
            self._integral_sum = self.integral_min

        i_term = self.Ki * self._integral_sum

        # Derivative on Measurement
        d_term = -self.Kd * velocity

        output = p_term + i_term + d_term

        self._last_time_ms = current_time_ms
        return output

    def reset(self):
        """Resets the controller's integral sum and timing."""
        self._integral_sum = 0.0
        self._last_time_ms = -1
