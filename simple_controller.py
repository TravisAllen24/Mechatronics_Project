"""
Simple proportional depth controller
Pump intensity proportional to depth error
"""

import numpy as np
from constants import *


class PumpState:
    """Pump states"""

    OFF = 0
    INFLATE = 1
    DEFLATE = -1


class ProportionalDepthController:
    """
    Simple proportional controller:
    - Error = current_depth - target_depth
    - Control effort proportional to error
    - Map control effort to pump on-time within a control cycle

    The larger the error, the longer the pump runs in each cycle
    """

    def __init__(self, kp=1.0, control_cycle_time=3.0):
        """
        Args:
            kp: Proportional gain (seconds of pumping per meter of error)
            control_cycle_time: Duration of one control cycle (s)
        """
        self.kp = kp
        self.control_cycle_time = control_cycle_time

        # Controller state
        self.pump_state = PumpState.OFF
        self.cycle_timer = 0.0  # Tracks position in current cycle
        self.desired_pump_time = 0.0  # How long pump should run this cycle
        self.pump_direction = PumpState.OFF  # Which direction to pump

    def reset(self):
        """Reset controller"""
        self.pump_state = PumpState.OFF
        self.cycle_timer = 0.0
        self.desired_pump_time = 0.0
        self.pump_direction = PumpState.OFF

    def compute_control(self, current_depth, target_depth, dt):
        """
        Proportional control logic:
        1. Calculate error
        2. Calculate desired pump time proportional to error
        3. Run pump for that duration within the control cycle

        Args:
            current_depth: current depth (m)
            target_depth: desired depth (m)
            dt: time step (s)

        Returns:
            pump_state: OFF, INFLATE, or DEFLATE
            pump_flow_rate: actual flow rate (m³/s)
        """
        # Update cycle timer
        self.cycle_timer += dt

        # Check if we're starting a new control cycle
        if self.cycle_timer >= self.control_cycle_time:
            # New cycle - recalculate control
            self.cycle_timer = 0.0

            # Calculate error
            error = current_depth - target_depth

            # Apply deadband
            if abs(error) < DEPTH_TOLERANCE:
                error = 0.0

            # Proportional control: pump time = kp * |error|
            self.desired_pump_time = self.kp * abs(error)

            # Limit pump time to cycle duration
            self.desired_pump_time = min(
                self.desired_pump_time, self.control_cycle_time
            )

            # Determine direction
            if error > 0:
                # Too deep, need to rise
                self.pump_direction = PumpState.INFLATE
            elif error < 0:
                # Too shallow, need to sink
                self.pump_direction = PumpState.DEFLATE
            else:
                # Within tolerance
                self.pump_direction = PumpState.OFF
                self.desired_pump_time = 0.0

            print(
                f"  New cycle | Depth: {current_depth:.2f}m | Error: {error*100:.1f}cm | Pump time: {self.desired_pump_time:.2f}s"
            )

        # Execute control within cycle
        if self.cycle_timer < self.desired_pump_time:
            # Pump should be on
            self.pump_state = self.pump_direction
        else:
            # Pump should be off (waiting for next cycle)
            self.pump_state = PumpState.OFF

        # Convert state to flow rate
        if self.pump_state == PumpState.INFLATE:
            pump_flow_rate = PUMP_NOMINAL_FLOW_RATE
        elif self.pump_state == PumpState.DEFLATE:
            pump_flow_rate = -PUMP_NOMINAL_FLOW_RATE
        else:
            pump_flow_rate = 0.0

        return self.pump_state, pump_flow_rate

    def get_status(self):
        """Get controller status for debugging"""
        return {
            "pump_state": self.pump_state,
            "cycle_timer": self.cycle_timer,
            "desired_pump_time": self.desired_pump_time,
            "duty_cycle": (
                self.desired_pump_time / self.control_cycle_time
                if self.control_cycle_time > 0
                else 0
            ),
        }


class ContinuousProportionalController:
    """
    Continuous proportional controller (alternative approach):
    - Continuously adjusts pump state based on error
    - No fixed cycles, more responsive but may chatter
    """

    def __init__(self, kp=0.5):
        """
        Args:
            kp: Proportional gain (controls aggressiveness)
        """
        self.kp = kp
        self.pump_state = PumpState.OFF
        self.min_on_time = 0.3  # Minimum time pump must stay in one state
        self.time_in_state = 0.0

    def reset(self):
        """Reset controller"""
        self.pump_state = PumpState.OFF
        self.time_in_state = 0.0

    def compute_control(self, current_depth, target_depth, dt):
        """
        Continuous proportional control:
        - Large error -> pump stays on more
        - Small error -> pump toggles more frequently
        - Uses error magnitude to determine switching threshold

        Args:
            current_depth: current depth (m)
            target_depth: desired depth (m)
            dt: time step (s)

        Returns:
            pump_state: OFF, INFLATE, or DEFLATE
            pump_flow_rate: actual flow rate (m³/s)
        """
        self.time_in_state += dt

        # Calculate error
        error = current_depth - target_depth

        # Apply deadband
        if abs(error) < DEPTH_TOLERANCE:
            self.pump_state = PumpState.OFF
            self.time_in_state = 0.0
        else:
            # Determine desired state
            if error > 0:
                desired_state = PumpState.INFLATE
            else:
                desired_state = PumpState.DEFLATE

            # Calculate "on-time" based on error magnitude
            # Larger error = longer on-time before checking again
            required_on_time = self.min_on_time + self.kp * error

            # Switch state if:
            # 1. We're off and should be on
            # 2. We've been in current state long enough and should switch
            if self.pump_state == PumpState.OFF:
                self.pump_state = desired_state
                self.time_in_state = 0.0
                print(
                    f"  Starting pump | Depth: {current_depth:.2f}m | Error: {error*100:.1f}cm"
                )
            elif self.time_in_state >= required_on_time:
                # Check if we should stay on or turn off
                if abs(error) > DEPTH_TOLERANCE * 0.5:  # Hysteresis
                    # Keep running
                    self.pump_state = desired_state
                else:
                    # Turn off
                    self.pump_state = PumpState.OFF
                    self.time_in_state = 0.0
                    print(f"  Stopping pump | Depth: {current_depth:.2f}m")

        # Convert state to flow rate
        if self.pump_state == PumpState.INFLATE:
            pump_flow_rate = PUMP_NOMINAL_FLOW_RATE
        elif self.pump_state == PumpState.DEFLATE:
            pump_flow_rate = -PUMP_NOMINAL_FLOW_RATE
        else:
            pump_flow_rate = 0.0

        return self.pump_state, pump_flow_rate

    def get_status(self):
        """Get controller status"""
        return {"pump_state": self.pump_state, "time_in_state": self.time_in_state}


class SimpleDepthController:
    """
    Original bang-bang controller
    """

    def __init__(self):
        self.pump_state = PumpState.OFF
        self.time_in_current_state = 0.0
        self.cooldown_timer = 0.0

    def reset(self):
        """Reset controller"""
        self.pump_state = PumpState.OFF
        self.time_in_current_state = 0.0
        self.cooldown_timer = 0.0

    def compute_control(self, current_depth, target_depth, dt):
        """Bang-bang control (original simple version)"""
        # Update timers
        self.time_in_current_state += dt
        if self.cooldown_timer > 0:
            self.cooldown_timer -= dt

        # Calculate error
        error = current_depth - target_depth

        # Decision logic
        if self.pump_state == PumpState.OFF:
            if self.cooldown_timer <= 0:
                if error > DEPTH_TOLERANCE:
                    self.pump_state = PumpState.INFLATE
                    self.time_in_current_state = 0.0
                    print(f"  Depth: {current_depth:.2f}m | Too deep, INFLATING")

                elif error < -DEPTH_TOLERANCE:
                    self.pump_state = PumpState.DEFLATE
                    self.time_in_current_state = 0.0
                    print(f"  Depth: {current_depth:.2f}m | Too shallow, DEFLATING")

        else:
            if self.time_in_current_state >= PUMP_ON_TIME:
                print(f"  Depth: {current_depth:.2f}m | Pump timer done, turning OFF")
                self.pump_state = PumpState.OFF
                self.time_in_current_state = 0.0
                self.cooldown_timer = PUMP_OFF_TIME

        # Convert state to flow rate
        if self.pump_state == PumpState.INFLATE:
            pump_flow_rate = PUMP_NOMINAL_FLOW_RATE
        elif self.pump_state == PumpState.DEFLATE:
            pump_flow_rate = -PUMP_NOMINAL_FLOW_RATE
        else:
            pump_flow_rate = 0.0

        return self.pump_state, pump_flow_rate

    def get_status(self):
        """Get controller status"""
        return {
            "pump_state": self.pump_state,
            "time_in_state": self.time_in_current_state,
            "cooldown": self.cooldown_timer,
        }
