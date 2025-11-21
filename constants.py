"""
Minimal constants for simple depth control
"""

# import numpy as np
import math

# ============================================================================
# BASIC ROBOT PARAMETERS
# ============================================================================

ROBOT_MASS = 6  # kg
ROBOT_VOLUME = 6.97  # m³

BALLOON_MIN_VOLUME = 0.00001  # m³
BALLOON_MAX_VOLUME = 0.005  # m³

WATER_DENSITY = 1025  # kg/m³
GRAVITY = 9.81  # m/s²
ATMOSPHERIC_PRESSURE = 101325  # Pa
WATER_TEMPERATURE = 293.15  # K

R_UNIVERSAL = 8.314  # J/(mol·K)
M_AIR = 0.029  # kg/mol

TARGET_DEPTH = 5.0  # m
MAX_OPERATIONAL_DEPTH = 10.0  # m

# Simulation
DT = 0.01  # s
SIMULATION_TIME = 120.0  # s

# ============================================================================
# DERIVED PARAMETERS
# ============================================================================

BALLOON_VOLUME_RANGE = BALLOON_MAX_VOLUME - BALLOON_MIN_VOLUME
PRESSURE_MARGIN = 3000  # Pa
BALLOON_COMPLIANCE = BALLOON_VOLUME_RANGE / PRESSURE_MARGIN

INTERNAL_AIR_VOLUME = BALLOON_VOLUME_RANGE * 3
V_TOTAL_MIN = INTERNAL_AIR_VOLUME + BALLOON_MIN_VOLUME

INITIAL_PRESSURE = ATMOSPHERIC_PRESSURE
TOTAL_AIR_MASS = (INITIAL_PRESSURE * V_TOTAL_MIN * M_AIR) / (
    R_UNIVERSAL * WATER_TEMPERATURE
)

ROBOT_DIAMETER = (6 * ROBOT_VOLUME / math.pi) ** (1 / 3)
CROSS_SECTIONAL_AREA = math.pi * (ROBOT_DIAMETER / 2) ** 2
DRAG_COEFFICIENT = 0.47

ENTRAINED_WATER_MASS = (
    0.5 * WATER_DENSITY * (4 / 3) * math.pi * (ROBOT_DIAMETER / 2) ** 3
)
EFFECTIVE_MASS = ROBOT_MASS + TOTAL_AIR_MASS + ENTRAINED_WATER_MASS

# ============================================================================
# PUMP PARAMETERS
# ============================================================================

PUMP_NOMINAL_FLOW_RATE = 0.0002  # m³/s

# ============================================================================
# SIMPLE CONTROLLER PARAMETERS
# ============================================================================

DEPTH_TOLERANCE = 0.15  # m (15 cm deadband around target)
PUMP_ON_TIME = 0.5  # s (how long to run pump when activated)
PUMP_OFF_TIME = 0.0  # s (cooldown between pump activations)

print(f"Simple Controller Configuration:")
print(f"  Target depth: {TARGET_DEPTH} m")
print(f"  Depth tolerance: ±{DEPTH_TOLERANCE*100:.0f} cm")
print(f"  Pump on time: {PUMP_ON_TIME} s")
print(f"  Pump off time: {PUMP_OFF_TIME} s")
print(f"  Pump flow rate: {PUMP_NOMINAL_FLOW_RATE*1e6:.0f} cm³/s")

# ============================================================================
# PROPORTIONAL CONTROLLER PARAMETERS
# ============================================================================

# Proportional gain: how many seconds of pumping per meter of error
# Larger Kp = more aggressive (longer pump times for same error)
# Smaller Kp = gentler (shorter pump times for same error)
KP_PROPORTIONAL = 12.0  # s/m (2 seconds of pumping per meter of error)

# Control cycle time: how often to recalculate control
# Longer cycles = smoother but slower response
# Shorter cycles = faster but may oscillate
CONTROL_CYCLE_TIME = 3.0  # s

print(f"\nProportional Controller:")
print(f"  Proportional gain (Kp): {KP_PROPORTIONAL:.1f} s/m")
print(f"  Control cycle time: {CONTROL_CYCLE_TIME:.1f} s")
print(f"  Example: 0.5m error → {KP_PROPORTIONAL * 0.5:.1f}s pump time")
