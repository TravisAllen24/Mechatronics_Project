import numpy as np
import matplotlib.pyplot as plt

# Physical constants
rho = 1000.0       # kg/m^3 (water)
g   = 9.81         # m/s^2
P_atm = 101325.0   # Pa (1 atm)

# Robot parameters
M   = 7.5                      # kg
V   = M / rho                  # m^3, neutrally buoyant at baseline
z_max = 3.048                  # m (~10 ft max depth)

# Drag parameters: D = 1/2 * Cd * rho * Vd * v^2
Cd  = 1.1
Vd  = 0.05

# Time settings
dt           = 0.01             # s
segment_time = 150.0            # s per depth target
waypoints_ft = [10, 7, 4, 0]
n_segments   = len(waypoints_ft)
t_end        = segment_time * n_segments
t = np.arange(0.0, t_end, dt)

# Depth waypoints (ft -> m)
ft_to_m = 0.3048
waypoints_m  = [d * ft_to_m for d in waypoints_ft]

def z_ref_func(t_now):
    idx = int(t_now // segment_time)
    if idx >= len(waypoints_m):
        idx = len(waypoints_m) - 1
    return waypoints_m[idx]

# PID gains
Kp = 1e-4
Ki = 1e-6
Kd = 4e-5

# Actuator limits (volume and rate)
dV_max      = 3e-4        # abs max Delta V (m^3)
dVdot_max   = 1e-5        # max |d(Delta V)/dt| (m^3/s)

# Arrays
z          = np.zeros_like(t)
vz         = np.zeros_like(t)
dV_hist    = np.zeros_like(t)
z_ref_hist = np.zeros_like(t)
P_hpa      = np.zeros_like(t)
e_hist     = np.zeros_like(t)

# Initial conditions
z[0]   = 1.0
vz[0]  = 0.0
dV_now = 0.0              # current bladder volume offset

# PID state
integral_e = 0.0
prev_e     = 0.0

for i in range(1, len(t)):
    # Reference depth & error
    z_ref = z_ref_func(t[i-1])
    z_ref_hist[i-1] = z_ref

    e = z_ref - z[i-1]
    e_hist[i-1] = e

    de_dt = (e - prev_e) / dt

    # Provisional integral
    integral_candidate = integral_e + e * dt

    # --- PID output (commanded Delta V, before limits) ---
    dV_cmd_unsat = -(Kp * e + Ki * integral_candidate + Kd * de_dt)

    # Magnitude saturation
    dV_cmd = max(min(dV_cmd_unsat, dV_max), -dV_max)

    # --- Rate limiting on Delta V ---
    # allowable change this step:
    max_step = dVdot_max * dt
    delta_step = dV_cmd - dV_now
    delta_step = max(min(delta_step, max_step), -max_step)
    dV_next = dV_now + delta_step

    # Simple anti-windup: only integrate if we're not heavily saturating
    if abs(dV_cmd_unsat - dV_cmd) < 1e-6:
        integral_e = integral_candidate

    prev_e = e
    dV_now = dV_next
    dV_hist[i-1] = dV_now

    # --- Dynamics with drag ---
    v_now = vz[i-1]

    # Drag force (downward positive)
    F_drag = -0.5 * Cd * rho * Vd * abs(v_now) * v_now

    # Net acceleration: M z'' = Mg - rho g (V + dV) + F_drag
    az = (M * g - rho * g * (V + dV_now) + F_drag) / M

    vz[i] = vz[i-1] + az * dt
    z[i]  = z[i-1] + vz[i] * dt

    # Bounds on depth
    if z[i] < 0.0:
        z[i] = 0.0
        vz[i] = 0.0
    if z[i] > z_max:
        z[i] = z_max
        vz[i] = 0.0

# Last entries
dV_hist[-1]    = dV_hist[-2]
z_ref_hist[-1] = z_ref_hist[-2]
e_hist[-1]     = e_hist[-2]

# Pressure
P_hpa = (P_atm + rho * g * z) / 100.0

# --- Plotting ---
fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, 1, figsize=(10, 10), sharex=True)

ax1.plot(t, z, label='Actual depth')
ax1.plot(t, z_ref_hist, '--', label='Ref depth')
ax1.set_ylabel('Depth z (m)')
ax1.set_title('Buoyancy Robot Depth Control')
ax1.invert_yaxis()
ax1.legend()

ax2.plot(t, dV_hist)
ax2.set_ylabel(r'$\Delta V$ (m$^3$)')

ax3.plot(t, P_hpa)
ax3.set_ylabel('Pressure (hPa)')

ax4.plot(t, e_hist)
ax4.set_ylabel('Error (m)')
ax4.set_xlabel('Time (s)')

plt.tight_layout()
plt.show()
