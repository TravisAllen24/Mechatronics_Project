# pc_simulation.py - For testing the controller on a computer.

import time
import random
import matplotlib.pyplot as plt
from dynamics import BuoyStateEstimator
from simple_controller import PIDController
from simulator import BuoySimulator


if __name__ == "__main__":
    # --- Constants ---
    TARGET_DEPTH = 2.0
    START_DEPTH = 0.5
    PRESSURE_UPDATE_RATE = 1.0
    IMU_UPDATE_RATE = 50.0

    # 1. Initialize components
    buoy_sim = BuoySimulator(mass=10.0, base_volume=0.01, start_depth=START_DEPTH)
    estimator = BuoyStateEstimator(alpha=0.98)
    pid = PIDController(Kp=3.0, Ki=0.4, Kd=5.0, setpoint=TARGET_DEPTH)

    PUMP_TIME_SCALING_FACTOR = 0.5
    MAX_PUMP_TIME = 1.0
    DEADBAND = 0.05

    SIM_DURATION = 120
    sim_time_s = 0.0
    dt_control_s = 1.0 / IMU_UPDATE_RATE
    last_pressure_update_s = -1

    history = {
        "time": [],
        "true_depth": [],
        "est_depth": [],
        "true_vel": [],
        "est_vel": [],
        "signal": [],
    }

    while sim_time_s < SIM_DURATION:
        sim_time_s += dt_control_s
        sim_time_ms = sim_time_s * 1000

        # --- Simulate Sensor Readings  ---
        # IMU
        true_accel_z = buoy_sim.acceleration
        imu_noise = random.gauss(0, 0.05)  # mu=0, sigma=0.05
        measured_accel_z = true_accel_z + imu_noise
        estimator.update_with_imu(measured_accel_z, sim_time_ms)

        # Pressure
        if sim_time_s - last_pressure_update_s >= (1.0 / PRESSURE_UPDATE_RATE):
            last_pressure_update_s = sim_time_s
            true_depth = buoy_sim.depth
            pressure_noise = random.gauss(0, 0.02)  # mu=0, sigma=0.02
            measured_depth = true_depth + pressure_noise
            estimator.update_with_pressure(measured_depth, sim_time_ms)

        # --- Control Loop ---
        current_depth, current_velocity = estimator.get_state()
        control_signal = pid.calculate(current_depth, current_velocity, sim_time_ms)

        pump_action = "none"
        pump_on_time = 0.0
        if control_signal > DEADBAND:
            pump_action = "deflate"
            pump_on_time = min(
                abs(control_signal) * PUMP_TIME_SCALING_FACTOR, MAX_PUMP_TIME
            )
        elif control_signal < -DEADBAND:
            pump_action = "inflate"
            pump_on_time = min(
                abs(control_signal) * PUMP_TIME_SCALING_FACTOR, MAX_PUMP_TIME
            )

        buoy_sim.update(pump_action, pump_on_time, dt_control_s)

        history["time"].append(sim_time_s)
        history["true_depth"].append(buoy_sim.depth)
        history["est_depth"].append(current_depth)
        history["true_vel"].append(buoy_sim.velocity)
        history["est_vel"].append(current_velocity)
        history["signal"].append(control_signal)

    # Plotting code remains the same...
    fig, axs = plt.subplots(3, 1, figsize=(12, 12), sharex=True)
    axs[0].plot(history["time"], history["true_depth"], "k-", label="True Depth")
    axs[0].plot(history["time"], history["est_depth"], "c--", label="Estimated Depth")
    axs[0].axhline(TARGET_DEPTH, color="r", linestyle="--", label="Target Depth")
    axs[0].set_ylabel("Depth (m)")
    axs[0].invert_yaxis()
    axs[0].legend()
    axs[0].grid(True)
    axs[1].plot(history["time"], history["true_vel"], "k-", label="True Velocity")
    axs[1].plot(history["time"], history["est_vel"], "m--", label="Estimated Velocity")
    axs[1].set_ylabel("Velocity (m/s)")
    axs[1].legend()
    axs[1].grid(True)
    axs[2].plot(history["time"], history["signal"], label="PID Control Signal")
    axs[2].axhline(DEADBAND, color="k", linestyle=":")
    axs[2].axhline(-DEADBAND, color="k", linestyle=":")
    axs[2].set_xlabel("Time (s)")
    axs[2].set_ylabel("Control Signal")
    axs[2].legend()
    axs[2].grid(True)
    plt.suptitle("Buoy Control (NumPy-Free Classes)")
    plt.show()
