from constants import *


class BuoySimulator:
    def __init__(self, mass, base_volume, start_depth=0.0):
        self.mass = mass
        self.base_volume = base_volume
        self.bladder_volume = 0.0
        self.depth = start_depth
        self.velocity = 0.0
        self.acceleration = 0.0
        self.neutral_bladder_volume = (self.mass / RHO_WATER) - self.base_volume
        self.bladder_volume = self.neutral_bladder_volume

    def update(self, pump_action: str, pump_duration: float, dt: float):
        if pump_action == "inflate":
            self.bladder_volume = min(
                MAX_BLADDER_VOLUME, self.bladder_volume + PUMP_RATE * pump_duration
            )
        elif pump_action == "deflate":
            self.bladder_volume = max(
                0, self.bladder_volume - PUMP_RATE * pump_duration
            )
        total_volume = self.base_volume + self.bladder_volume
        force_buoyant = total_volume * RHO_WATER * G
        force_gravity = self.mass * G
        force_drag = -0.5 * RHO_WATER * self.velocity * abs(self.velocity) * 1.0
        net_force = force_buoyant - force_gravity + force_drag
        self.acceleration = -net_force / self.mass
        self.velocity += self.acceleration * dt
        self.depth += self.velocity * dt
        if self.depth < 0:
            self.depth, self.velocity = 0, 0
