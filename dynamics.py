class BuoyStateEstimator:
    """
    Handles state estimation using sensor fusion without NumPy.
    This class is designed to be lightweight for microcontrollers.
    It takes actual sensor readings as input.
    """

    def __init__(self, alpha=0.98):
        # State variables
        self.depth = 0.0
        self.fused_velocity = 0.0

        # Sensor fusion parameter (alpha is for the velocity fusion)
        self.alpha = alpha

        # Internal state for calculations
        self._last_pressure_depth = 0.0
        self._last_pressure_time = -1
        self._last_imu_time = -1

    def update_with_imu(self, measured_accel_z, current_time_ms):
        """High-frequency update using IMU's vertical acceleration."""
        if self._last_imu_time < 0:
            self._last_imu_time = current_time_ms
            return

        # time.ticks_diff should be used for robust timing on microcontrollers
        dt_s = (current_time_ms - self._last_imu_time) / 1000.0
        if dt_s <= 0:
            return

        # Integrate acceleration to get velocity (this will drift)
        self.fused_velocity += measured_accel_z * dt_s

        # Integrate velocity to get depth (for a high-frequency estimate)
        self.depth += self.fused_velocity * dt_s

        self._last_imu_time = current_time_ms

    def update_with_pressure(self, measured_depth, current_time_ms):
        """Low-frequency update to correct drift using the pressure sensor."""
        if self._last_pressure_time < 0:
            self._last_pressure_time = current_time_ms
            self._last_pressure_depth = measured_depth
            self.depth = measured_depth  # Initialize depth on first reading
            return

        dt_s = (current_time_ms - self._last_pressure_time) / 1000.0
        if dt_s <= 0:
            return

        # Calculate low-frequency velocity from pressure
        velocity_from_pressure = (measured_depth - self._last_pressure_depth) / dt_s

        # --- SENSOR FUSION (Complementary Filter for Velocity) ---
        self.fused_velocity = (
            self.alpha * self.fused_velocity
            + (1.0 - self.alpha) * velocity_from_pressure
        )

        # --- FUSION FOR DEPTH ---
        # Gently nudge our high-frequency depth estimate toward the reliable pressure reading
        self.depth = 0.95 * self.depth + 0.05 * measured_depth

        # Update for next iteration
        self._last_pressure_depth = measured_depth
        self._last_pressure_time = current_time_ms

    def get_state(self):
        return self.depth, self.fused_velocity
