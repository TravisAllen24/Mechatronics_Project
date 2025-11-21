"""
Simple simulation with proportional depth control
"""

import numpy as np
import matplotlib.pyplot as plt
from dynamics import RobotDynamics
from simple_controller import (
    ProportionalDepthController,
    ContinuousProportionalController,
    SimpleDepthController,
    PumpState,
)
from constants import *


def run_simulation(controller_type="proportional"):
    """
    Run simulation with specified controller

    Args:
        controller_type: 'proportional', 'continuous', or 'bangbang'
    """

    # Initialize robot
    robot = RobotDynamics()

    # Initialize controller
    if controller_type == "proportional":
        controller = ProportionalDepthController(
            kp=KP_PROPORTIONAL, control_cycle_time=CONTROL_CYCLE_TIME
        )
        print("Using Proportional Controller (cycle-based)")
    elif controller_type == "continuous":
        controller = ContinuousProportionalController(kp=KP_PROPORTIONAL)
        print("Using Continuous Proportional Controller")
    else:
        controller = SimpleDepthController()
        print("Using Bang-Bang Controller")

    # Start near surface
    robot.depth = 0.5

    # Time array
    time = np.arange(0, SIMULATION_TIME, DT)

    # Data storage
    depth_history = []
    velocity_history = []
    pump_state_history = []
    pump_flow_history = []
    balloon_volume_history = []
    control_effort_history = []

    print(f"\nStarting simulation...")
    print(f"Target: {TARGET_DEPTH}m ± {DEPTH_TOLERANCE}m")
    print(f"=" * 60)

    last_print_time = 0.0

    # Simulation loop
    for i, t in enumerate(time):
        current_depth = robot.depth

        # Print status every 2 seconds
        if t - last_print_time >= 2.0:
            status = controller.get_status()
            if controller_type == "proportional":
                print(
                    f"t={t:.1f}s | Depth={current_depth:.2f}m | Vel={robot.velocity*100:.1f}cm/s | "
                    f"Pump={controller.pump_state} | Duty={status['duty_cycle']*100:.0f}%"
                )
            else:
                print(
                    f"t={t:.1f}s | Depth={current_depth:.2f}m | Vel={robot.velocity*100:.1f}cm/s | "
                    f"Pump={controller.pump_state}"
                )
            last_print_time = t

        # Get control command
        pump_state, pump_flow_rate = controller.compute_control(
            current_depth, TARGET_DEPTH, DT
        )

        # Update robot
        robot.update(pump_flow_rate, DT)

        # Calculate control effort (for plotting)
        error = current_depth - TARGET_DEPTH
        if controller_type == "proportional":
            control_effort = (
                KP_PROPORTIONAL * abs(error) if abs(error) > DEPTH_TOLERANCE else 0
            )
        else:
            control_effort = PUMP_ON_TIME if pump_state != PumpState.OFF else 0

        # Store data
        depth_history.append(robot.depth)
        velocity_history.append(robot.velocity)
        pump_state_history.append(pump_state)
        pump_flow_history.append(pump_flow_rate)
        balloon_volume_history.append(robot.balloon_volume)
        control_effort_history.append(control_effort)

    print(f"=" * 60)
    print(f"Simulation complete!")

    # Plot
    plot_results(
        time,
        depth_history,
        velocity_history,
        pump_state_history,
        balloon_volume_history,
        control_effort_history,
        controller_type,
    )

    # Print summary
    print_summary(depth_history, pump_state_history, controller_type)

    return time, depth_history


def plot_results(
    time, depth, velocity, pump_state, balloon_vol, control_effort, controller_type
):
    """Plot results with control effort"""

    fig, axes = plt.subplots(3, 2, figsize=(14, 12))

    # Depth
    axes[0, 0].plot(time, depth, "b-", linewidth=2, label="Actual Depth")
    axes[0, 0].axhline(
        y=TARGET_DEPTH, color="r", linestyle="--", linewidth=2, label="Target"
    )
    axes[0, 0].fill_between(
        time,
        TARGET_DEPTH - DEPTH_TOLERANCE,
        TARGET_DEPTH + DEPTH_TOLERANCE,
        alpha=0.3,
        color="green",
        label="Tolerance",
    )
    axes[0, 0].set_xlabel("Time (s)", fontsize=12)
    axes[0, 0].set_ylabel("Depth (m)", fontsize=12)
    axes[0, 0].set_title(
        f"Depth Control ({controller_type.upper()})", fontsize=14, fontweight="bold"
    )
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].invert_yaxis()

    # Velocity
    axes[0, 1].plot(time, np.array(velocity) * 100, "g-", linewidth=2)
    axes[0, 1].axhline(y=0, color="k", linestyle="--", alpha=0.3)
    axes[0, 1].set_xlabel("Time (s)", fontsize=12)
    axes[0, 1].set_ylabel("Velocity (cm/s)", fontsize=12)
    axes[0, 1].set_title("Vertical Velocity", fontsize=14, fontweight="bold")
    axes[0, 1].grid(True, alpha=0.3)

    # Pump state
    pump_array = np.array(pump_state)
    axes[1, 0].step(time, pump_array, "c-", linewidth=2, where="post")

    inflate_mask = pump_array == PumpState.INFLATE
    deflate_mask = pump_array == PumpState.DEFLATE
    axes[1, 0].fill_between(
        time,
        -1.5,
        1.5,
        where=inflate_mask,
        alpha=0.3,
        color="blue",
        step="post",
        label="Inflate",
    )
    axes[1, 0].fill_between(
        time,
        -1.5,
        1.5,
        where=deflate_mask,
        alpha=0.3,
        color="red",
        step="post",
        label="Deflate",
    )

    axes[1, 0].set_xlabel("Time (s)", fontsize=12)
    axes[1, 0].set_ylabel("Pump State", fontsize=12)
    axes[1, 0].set_title("Pump Control", fontsize=14, fontweight="bold")
    axes[1, 0].set_yticks([-1, 0, 1])
    axes[1, 0].set_yticklabels(["Deflate", "Off", "Inflate"])
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)

    # Control effort (proportional output)
    axes[1, 1].plot(time, control_effort, "orange", linewidth=2)
    axes[1, 1].set_xlabel("Time (s)", fontsize=12)
    axes[1, 1].set_ylabel("Control Effort (s)", fontsize=12)
    axes[1, 1].set_title("Desired Pump Time", fontsize=14, fontweight="bold")
    axes[1, 1].grid(True, alpha=0.3)

    # Balloon volume
    axes[2, 0].plot(time, np.array(balloon_vol) * 1e6, "m-", linewidth=2)
    axes[2, 0].set_xlabel("Time (s)", fontsize=12)
    axes[2, 0].set_ylabel("Balloon Volume (cm³)", fontsize=12)
    axes[2, 0].set_title("Balloon Volume", fontsize=14, fontweight="bold")
    axes[2, 0].grid(True, alpha=0.3)

    # Error vs time
    error = np.array(depth) - TARGET_DEPTH
    axes[2, 1].plot(time, error * 100, "r-", linewidth=2)
    axes[2, 1].axhline(y=0, color="k", linestyle="--", alpha=0.3)
    axes[2, 1].axhline(y=DEPTH_TOLERANCE * 100, color="g", linestyle=":", alpha=0.5)
    axes[2, 1].axhline(y=-DEPTH_TOLERANCE * 100, color="g", linestyle=":", alpha=0.5)
    axes[2, 1].fill_between(
        time, -DEPTH_TOLERANCE * 100, DEPTH_TOLERANCE * 100, alpha=0.2, color="green"
    )
    axes[2, 1].set_xlabel("Time (s)", fontsize=12)
    axes[2, 1].set_ylabel("Error (cm)", fontsize=12)
    axes[2, 1].set_title("Tracking Error", fontsize=14, fontweight="bold")
    axes[2, 1].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(f"{controller_type}_control_results.png", dpi=300, bbox_inches="tight")
    plt.show()


def print_summary(depth, pump_state, controller_type):
    """Print performance summary"""

    depth = np.array(depth)
    pump_state = np.array(pump_state)

    error = depth - TARGET_DEPTH
    in_tolerance = np.abs(error) <= DEPTH_TOLERANCE

    if np.any(in_tolerance):
        first_in = np.where(in_tolerance)[0][0]
        settling_time = first_in * DT
    else:
        settling_time = None

    time_in_tolerance = np.sum(in_tolerance) * DT
    percent_in_tolerance = (time_in_tolerance / SIMULATION_TIME) * 100

    time_inflate = np.sum(pump_state == PumpState.INFLATE) * DT
    time_deflate = np.sum(pump_state == PumpState.DEFLATE) * DT
    time_active = time_inflate + time_deflate

    print(f"\n" + "=" * 60)
    print(f"PERFORMANCE SUMMARY - {controller_type.upper()}")
    print("=" * 60)

    print(f"\nDepth Control:")
    print(f"  Target: {TARGET_DEPTH:.2f} m")
    print(f"  Tolerance: ±{DEPTH_TOLERANCE*100:.0f} cm")
    print(f"  Final depth: {depth[-1]:.3f} m")
    print(f"  Final error: {error[-1]*100:.1f} cm")

    if settling_time:
        print(f"  Settling time: {settling_time:.1f} s")
    else:
        print(f"  Never reached tolerance")

    print(
        f"  Time in tolerance: {time_in_tolerance:.1f} s ({percent_in_tolerance:.1f}%)"
    )
    print(f"  Mean absolute error: {np.mean(np.abs(error))*100:.1f} cm")
    print(f"  RMS error: {np.sqrt(np.mean(error**2))*100:.1f} cm")
    print(f"  Max error: {np.max(np.abs(error))*100:.1f} cm")

    print(f"\nPump Usage:")
    print(
        f"  Inflate time: {time_inflate:.1f} s ({time_inflate/SIMULATION_TIME*100:.1f}%)"
    )
    print(
        f"  Deflate time: {time_deflate:.1f} s ({time_deflate/SIMULATION_TIME*100:.1f}%)"
    )
    print(
        f"  Total active: {time_active:.1f} s ({time_active/SIMULATION_TIME*100:.1f}%)"
    )
    print(f"  Duty cycle: {time_active/SIMULATION_TIME*100:.1f}%")

    print("\n" + "=" * 60)


if __name__ == "__main__":
    print("\n" + "=" * 60)
    print("PROPORTIONAL DEPTH CONTROL")
    print("=" * 60)

    # Run proportional controller
    print("\n1. PROPORTIONAL CONTROLLER (CYCLE-BASED)")
    print("   Control: Pump time ∝ Error")
    print("=" * 60)
    time_p, depth_p = run_simulation(controller_type="proportional")

    # Run continuous proportional controller
    print("\n" + "=" * 60)
    print("2. CONTINUOUS PROPORTIONAL CONTROLLER")
    print("=" * 60)
    time_c, depth_c = run_simulation(controller_type="continuous")

    # Run bang-bang for comparison
    print("\n" + "=" * 60)
    print("3. BANG-BANG CONTROLLER (FOR COMPARISON)")
    print("=" * 60)
    time_b, depth_b = run_simulation(controller_type="bangbang")

    # Comparison plot
    plt.figure(figsize=(12, 6))
    plt.plot(time_p, depth_p, "b-", linewidth=2, label="Proportional", alpha=0.8)
    plt.plot(time_c, depth_c, "g-", linewidth=2, label="Continuous P", alpha=0.8)
    plt.plot(time_b, depth_b, "r-", linewidth=2, label="Bang-Bang", alpha=0.8)
    plt.axhline(y=TARGET_DEPTH, color="k", linestyle="--", linewidth=2, label="Target")
    plt.fill_between(
        time_p,
        TARGET_DEPTH - DEPTH_TOLERANCE,
        TARGET_DEPTH + DEPTH_TOLERANCE,
        alpha=0.2,
        color="green",
        label="Tolerance",
    )
    plt.xlabel("Time (s)", fontsize=12)
    plt.ylabel("Depth (m)", fontsize=12)
    plt.title("Controller Comparison", fontsize=14, fontweight="bold")
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.gca().invert_yaxis()
    plt.tight_layout()
    plt.savefig("controller_comparison.png", dpi=300, bbox_inches="tight")
    plt.show()

    print("\nAll simulations complete!")
