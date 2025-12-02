import time
import machine
from simple_controller import PIDController
from dynamics import BuoyStateEstimator

# --- Constants & Tuning Parameters (Adjust these!) ---
# Hardware Pins
INFLATE_PUMP_PIN = 14
DEFLATE_PUMP_PIN = 15
# I2C Pins for sensors
I2C_SDA_PIN = 4
I2C_SCL_PIN = 5

# Control Settings
TARGET_DEPTH = 2.0  # meters

# PID Gains (Tune these based on simulation and testing)
KP = 3.0
KI = 0.4
KD = 5.0

# Pump Control
PUMP_TIME_SCALING_FACTOR = 0.5
MAX_PUMP_TIME_MS = 1000  # Max time pump can be on in one cycle (in ms)
DEADBAND = 0.05

# Loop Rates
CONTROL_LOOP_RATE_HZ = 50  # Run the main control loop at 50Hz
PRESSURE_LOOP_RATE_HZ = 1  # Read pressure sensor at 1Hz

# Physics Constants
P_ATMOSPHERE = 101325.0  # Pascals (Pa). Calibrate this at the surface!
RHO_WATER = 1000.0  # kg/m^3 (1000 for fresh, ~1024 for salt)
G = 9.81  # m/s^2

# --- Hardware Abstraction ---


# TODO: Replace these with your actual sensor classes and libraries
# Example: from ms5837 import MS5837
# Example: from mpu6050 import MPU6050
class PressureSensor:
    def __init__(self, i2c):
        # self.sensor = MS5837(i2c)
        print("Mock Pressure Sensor initialized.")

    def read_depth(self):
        # In reality:
        # pressure_pascals = self.sensor.pressure()
        # depth = (pressure_pascals - P_ATMOSPHERE) / (RHO_WATER * G)
        # return depth
        return 0.5  # Mock data for testing without sensor


class IMU:
    def __init__(self, i2c):
        # self.sensor = MPU6050(i2c)
        print("Mock IMU initialized.")

    def get_vertical_acceleration(self):
        # In reality:
        # 1. Read accel [ax, ay, az] and gyro [gx, gy, gz]
        # 2. Use a filter (e.g., Complementary) to get pitch/roll angles
        # 3. Rotate accel vector by inverse of pitch/roll to get world-frame accel
        # 4. Subtract gravity from the world-frame Z component
        # return world_accel_z - G
        return 0.1  # Mock data for testing


class PumpController:
    def __init__(self, inflate_pin, deflate_pin):
        self.inflate_pump = machine.Pin(inflate_pin, machine.Pin.OUT)
        self.deflate_pump = machine.Pin(deflate_pin, machine.Pin.OUT)
        self.stop()

    def inflate(self, duration_ms):
        self.inflate_pump.on()
        time.sleep_ms(duration_ms)
        self.inflate_pump.off()

    def deflate(self, duration_ms):
        self.deflate_pump.on()
        time.sleep_ms(duration_ms)
        self.deflate_pump.off()

    def stop(self):
        self.inflate_pump.off()
        self.deflate_pump.off()


# --- Main Program ---

# 1. Initialize hardware
# i2c = machine.I2C(0, sda=machine.Pin(I2C_SDA_PIN), scl=machine.Pin(I2C_SCL_PIN))
# pressure_sensor = PressureSensor(i2c)
# imu = IMU(i2c)
pump = PumpController(INFLATE_PUMP_PIN, DEFLATE_PUMP_PIN)

# 2. Initialize Controller and Estimator
estimator = BuoyStateEstimator(alpha=0.98)
pid = PIDController(Kp=KP, Ki=KI, Kd=KD, setpoint=TARGET_DEPTH)

# 3. Setup loop timing
control_period_ms = 1000 // CONTROL_LOOP_RATE_HZ
pressure_period_ms = 1000 // PRESSURE_LOOP_RATE_HZ
last_control_time = time.ticks_ms()
last_pressure_time = time.ticks_ms()

print("Initialization complete. Starting control loop.")

while True:
    current_time = time.ticks_ms()

    # --- High-Frequency IMU & Control Loop ---
    if time.ticks_diff(current_time, last_control_time) >= control_period_ms:
        last_control_time = current_time

        # a. Read IMU
        # vertical_accel = imu.get_vertical_acceleration()
        vertical_accel = 0.1  # Using mock data

        # b. Update state estimator with IMU data
        estimator.update_with_imu(vertical_accel, current_time)

        # c. Get latest state
        current_depth, current_velocity = estimator.get_state()

        # d. Calculate PID output
        control_signal = pid.calculate(current_depth, current_velocity, current_time)

        # e. Command the pumps
        pump_on_time_ms = 0
        if control_signal > DEADBAND:
            action = "deflate"
            pump_on_time_ms = int(
                min(
                    abs(control_signal) * PUMP_TIME_SCALING_FACTOR * 1000,
                    MAX_PUMP_TIME_MS,
                )
            )
            pump.deflate(pump_on_time_ms)
        elif control_signal < -DEADBAND:
            action = "inflate"
            pump_on_time_ms = int(
                min(
                    abs(control_signal) * PUMP_TIME_SCALING_FACTOR * 1000,
                    MAX_PUMP_TIME_MS,
                )
            )
            pump.inflate(pump_on_time_ms)
        else:
            action = "none"

        # Optional: Print status
        print(
            f"D:{current_depth:.2f} V:{current_velocity:.2f} Sig:{control_signal:.2f} Act:{action} {pump_on_time_ms}ms"
        )

    # --- Low-Frequency Pressure Sensor Loop ---
    if time.ticks_diff(current_time, last_pressure_time) >= pressure_period_ms:
        last_pressure_time = current_time

        # a. Read Pressure Sensor and convert to depth
        # depth_from_pressure = pressure_sensor.read_depth()
        depth_from_pressure = 0.51  # Using mock data

        # b. Update state estimator to correct drift
        estimator.update_with_pressure(depth_from_pressure, current_time)

    # A small delay to prevent the while loop from running at 100% CPU
    # if the timing logic has a hiccup. A pass statement would also work.
    time.sleep_ms(1)
