"""
Dynamics model for the underwater robot with buoyancy control
Simplified for basic on-off pump control
"""

from constants import *


class RobotDynamics:
    """
    Models the dynamics of an underwater robot with balloon-based buoyancy control

    State variables:
        - depth: current depth below water surface (m)
        - velocity: vertical velocity, positive = downward (m/s)
        - balloon_volume: current volume of balloon (m³)
        - internal_pressure: air pressure inside robot (Pa)
    """

    def __init__(self):
        # State variables
        self.depth = 0.0  # m (depth below water surface, positive downward)
        self.velocity = 0.0  # m/s (vertical velocity, positive downward)
        self.balloon_volume = BALLOON_MIN_VOLUME  # m³ (start with deflated balloon)
        self.internal_pressure = ATMOSPHERIC_PRESSURE  # Pa

        # Total air mass in the system (constant - closed system)
        self.total_air_mass = self.calculate_air_mass(
            ATMOSPHERIC_PRESSURE, INTERNAL_AIR_VOLUME + BALLOON_MIN_VOLUME
        )

    def calculate_air_mass(self, pressure, volume):
        """
        Calculate air mass using ideal gas law: PV = nRT = (m/M)RT
        Rearranging: m = PVM/(RT)

        Args:
            pressure: air pressure (Pa)
            volume: air volume (m³)

        Returns:
            mass: air mass (kg)
        """
        return (pressure * volume * M_AIR) / (R_UNIVERSAL * WATER_TEMPERATURE)

    def calculate_pressure_from_mass(self, mass, volume):
        """
        Calculate pressure from air mass and volume using ideal gas law
        P = mRT/(VM)

        Args:
            mass: air mass (kg)
            volume: air volume (m³)

        Returns:
            pressure: air pressure (Pa)
        """
        if volume <= 1e-6:  # Prevent division by zero
            return ATMOSPHERIC_PRESSURE * 100  # Very high pressure

        return (mass * R_UNIVERSAL * WATER_TEMPERATURE) / (M_AIR * volume)

    def get_water_pressure(self):
        """
        Calculate water pressure at current depth
        P = P_atm + rhogh

        Returns:
            pressure: water pressure (Pa)
        """
        return ATMOSPHERIC_PRESSURE + WATER_DENSITY * GRAVITY * self.depth

    def update_balloon_volume(self):
        """
        Update balloon volume based on pressure balance

        The balloon expands when internal pressure exceeds external water pressure.
        Volume change is proportional to pressure difference (balloon compliance).

        dV = C * dP
        where C is balloon compliance (m³/Pa)
        """
        water_pressure = self.get_water_pressure()
        pressure_diff = self.internal_pressure - water_pressure

        # Calculate balloon volume from pressure difference
        volume_change = BALLOON_COMPLIANCE * pressure_diff

        # Apply physical limits
        self.balloon_volume = BALLOON_MIN_VOLUME + volume_change

    def calculate_buoyancy_force(self):
        """
        Calculate net buoyancy force on robot

        Buoyancy force: F_b = rho_water * g * V_displaced (upward)
        Weight: F_w = m_total * g (downward)
        Net force: F_net = F_b - F_w (positive = upward)

        Returns:
            force: net buoyancy force (N), positive = upward
        """
        # Total volume displaced by robot
        total_volume = ROBOT_VOLUME + self.balloon_volume

        # Buoyancy force (Archimedes principle)
        buoyant_force = WATER_DENSITY * GRAVITY * total_volume  # N, upward

        # Weight of robot + air
        weight = (ROBOT_MASS + self.total_air_mass) * GRAVITY  # N, downward

        # Net force (positive = upward)
        return buoyant_force - weight

    def calculate_drag_force(self):
        """
        Calculate drag force opposing motion

        F_drag = 0.5 * rho * C_d * A * v²
        Direction opposes velocity

        Returns:
            force: drag force (N), opposes velocity
        """
        drag_magnitude = (
            0.5
            * WATER_DENSITY
            * DRAG_COEFFICIENT
            * CROSS_SECTIONAL_AREA
            * self.velocity
            * abs(self.velocity)
        )

        # Drag opposes motion
        return -drag_magnitude

    def update(self, pump_flow_rate, dt):
        """
        Update robot state for one time step

        Args:
            pump_flow_rate: volumetric flow rate of pump (m³/s)
                          Positive = inflating balloon
                          Negative = deflating balloon
                          Zero = pump off
            dt: time step (s)
        """
        # Clip pump flow rate to physical limits
        pump_flow_rate = pump_flow_rate

        # Calculate forces
        buoyancy_force = self.calculate_buoyancy_force()  # N (positive = up)
        drag_force = self.calculate_drag_force()  # N (opposes motion)
        net_force = buoyancy_force + drag_force  # N (positive = up)

        # Calculate acceleration (Newton's second law: F = ma)
        # Effective mass includes robot, air, and entrained water
        acceleration = net_force / EFFECTIVE_MASS  # m/s² (positive = up)

        # Note: Our convention is velocity positive = downward
        # But force positive = upward, so we need a sign flip
        acceleration_downward = -acceleration  # m/s² (positive = down)

        # Update velocity (kinematics)
        self.velocity += acceleration_downward * dt

        # Update depth (kinematics)
        self.depth += self.velocity * dt

        # Boundary condition: can't go above water surface
        if self.depth < 0:
            self.depth = 0
            # If trying to ascend through surface, stop
            if self.velocity < 0:
                self.velocity = 0

        # Update air system
        # Pump moves air between internal reservoir and balloon
        # This changes the distribution of fixed total air mass

        # Current total volume available to air
        total_air_volume = INTERNAL_AIR_VOLUME + self.balloon_volume

        # Calculate current internal pressure from air mass and volume
        self.internal_pressure = self.calculate_pressure_from_mass(
            self.total_air_mass, total_air_volume
        )

        # Update balloon volume based on pressure balance
        self.update_balloon_volume()

        # Pump action: changes effective volume distribution
        # When pump is inflating, it moves air toward balloon
        # This is modeled as changing the pressure by adjusting available volume

        # The pump changes how the air is distributed between reservoir and balloon
        # Simplified model: pump flow directly affects the target balloon inflation

        if pump_flow_rate != 0:
            # Pumping changes the pressure by changing volume distribution
            # We model this as: pump adds/removes from the "free" internal volume

            # Volume change from pumping
            volume_change = pump_flow_rate * dt

            # This effectively changes the pressure
            new_total_volume = total_air_volume - volume_change

            # Ensure volume stays positive
            if new_total_volume > BALLOON_MIN_VOLUME:
                self.internal_pressure = self.calculate_pressure_from_mass(
                    self.total_air_mass, new_total_volume
                )

                # Update balloon volume based on new pressure
                self.update_balloon_volume()

        # Ensure balloon stays within physical limits
        self.balloon_volume = self.balloon_volume

    def get_external_pressure(self):
        """
        Get pressure reading from external pressure sensor
        This is the water pressure at current depth

        Returns:
            pressure: water pressure (Pa)
        """
        return self.get_water_pressure()

    def get_internal_pressure(self):
        """
        Get pressure reading from internal pressure sensor

        Returns:
            pressure: internal air pressure (Pa)
        """
        return self.internal_pressure

    def get_depth_from_pressure(self, pressure):
        """
        Convert pressure reading to depth
        (What the pressure sensor does)

        Args:
            pressure: measured pressure (Pa)

        Returns:
            depth: calculated depth (m)
        """
        return (pressure - ATMOSPHERIC_PRESSURE) / (WATER_DENSITY * GRAVITY)

    def get_state(self):
        """
        Get current state of the robot

        Returns:
            dict with all state variables
        """
        return {
            "depth": self.depth,
            "velocity": self.velocity,
            "balloon_volume": self.balloon_volume,
            "internal_pressure": self.internal_pressure,
            "external_pressure": self.get_external_pressure(),
            "buoyancy_force": self.calculate_buoyancy_force(),
            "total_volume": ROBOT_VOLUME + self.balloon_volume,
        }

    def reset(self, initial_depth=0.0):
        """
        Reset robot to initial state

        Args:
            initial_depth: starting depth (m)
        """
        self.depth = initial_depth
        self.velocity = 0.0
        self.balloon_volume = BALLOON_MIN_VOLUME
        self.internal_pressure = ATMOSPHERIC_PRESSURE

        # Recalculate air mass for initial conditions
        self.total_air_mass = self.calculate_air_mass(
            ATMOSPHERIC_PRESSURE, INTERNAL_AIR_VOLUME + BALLOON_MIN_VOLUME
        )


# Verification function
if __name__ == "__main__":
    """Test the dynamics"""
    print("Testing Robot Dynamics")
    print("=" * 60)

    robot = RobotDynamics()

    print(f"\nInitial State:")
    state = robot.get_state()
    for key, value in state.items():
        if "pressure" in key:
            print(f"  {key}: {value/1000:.2f} kPa")
        elif "volume" in key:
            print(f"  {key}: {value*1e6:.2f} cm³")
        elif "force" in key:
            print(f"  {key}: {value:.3f} N")
        else:
            print(f"  {key}: {value:.4f}")

    print(f"\nRobot mass: {ROBOT_MASS:.2f} kg")
    print(f"Robot weight: {ROBOT_MASS * GRAVITY:.2f} N")
    print(f"Effective mass: {EFFECTIVE_MASS:.2f} kg")

    print(f"\nBuoyancy check:")
    min_vol = ROBOT_VOLUME + BALLOON_MIN_VOLUME
    max_vol = ROBOT_VOLUME + BALLOON_MAX_VOLUME
    min_buoy = WATER_DENSITY * GRAVITY * min_vol
    max_buoy = WATER_DENSITY * GRAVITY * max_vol
    weight = ROBOT_MASS * GRAVITY

    print(f"  Min buoyancy (deflated): {min_buoy:.2f} N")
    print(f"  Weight: {weight:.2f} N")
    print(f"  Max buoyancy (inflated): {max_buoy:.2f} N")

    if min_buoy < weight < max_buoy:
        print(f"Can achieve neutral buoyancy")
    else:
        print(f"Cannot achieve neutral buoyancy!")

    print(f"\n" + "=" * 60)
