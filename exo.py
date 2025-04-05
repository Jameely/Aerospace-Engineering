import numpy as np
import matplotlib.pyplot as plt

# Constants
G_MOON = 1.625  # Moon gravity (m/s^2)
THRUST_MAX = 430  # Max thrust (N)
THRUST_MIN = 0  # Min thrust (N)
MASS_EMPTY = 165  # kg
MASS_FUEL = 420  # kg
DT = 1.0  # Time step (s)
TARGET_VELOCITY = -2  # Vertical landing target (m/s)
TARGET_HS = 0  # Horizontal speed target (m/s)

# PID Controller class
class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

# Initial state
altitude = 30000
vs = 24.8
hs = 1700
fuel = MASS_FUEL
mass = MASS_EMPTY + fuel
angular_velocity = 0
angular_position = 0

# Data lists
altitudes, v_speeds, h_speeds, times, thrusts, angles, fuels = [], [], [], [], [], [], []

# PID controllers
pid_vs = PID(kp=10, ki=0.5, kd=1.0)
pid_hs = PID(kp=0.1, ki=0.01, kd=0.05)
pid_att = PID(kp=5, ki=0.2, kd=0.5)

# Simulation loop
for t in np.arange(0, 2000, DT):
    if altitude <= 0:
        break

    # Errors
    err_vs = TARGET_VELOCITY - vs
    err_hs = TARGET_HS - hs
    err_att = angular_position

    # PID outputs
    thrust_adj = pid_vs.compute(err_vs, DT)
    h_correction = pid_hs.compute(err_hs, DT)
    att_correction = pid_att.compute(err_att, DT)

    # Altitude-based thrust tuning
    if altitude < 5000:
        thrust_adj *= 1.5
    if altitude < 100:
        thrust_adj = THRUST_MAX

    # Thrust boundaries
    thrust = max(THRUST_MIN, min(THRUST_MAX, thrust_adj))

    # Fuel and mass update
    burn = (thrust / 10000) * DT
    if fuel > 0:
        fuel -= burn
        mass = MASS_EMPTY + fuel
    else:
        thrust = 0

    # Accelerations
    gravity = mass * G_MOON
    net_force = thrust - gravity
    acc_v = net_force / mass

    vs += acc_v * DT
    hs += h_correction * DT
    altitude += vs * DT

    angular_velocity += att_correction * DT
    angular_position += angular_velocity * DT

    # Data storage
    altitudes.append(altitude)
    v_speeds.append(vs)
    h_speeds.append(hs)
    times.append(t)
    thrusts.append(thrust)
    angles.append(angular_position)
    fuels.append(fuel)

# Plotting
plt.figure(figsize=(16, 12))

# Altitude over time
plt.subplot(3, 2, 1)
plt.plot(times, altitudes, color='royalblue', linewidth=2)
plt.title('Altitude over Time', fontsize=14)
plt.xlabel('Time (s)', fontsize=12)
plt.ylabel('Altitude (m)', fontsize=12)
plt.grid(True, linestyle='--', alpha=0.7)

# Vertical Speed over time
plt.subplot(3, 2, 2)
plt.plot(times, v_speeds, color='tomato', linewidth=2)
plt.title('Vertical Speed over Time', fontsize=14)
plt.xlabel('Time (s)', fontsize=12)
plt.ylabel('Vertical Speed (m/s)', fontsize=12)
plt.grid(True, linestyle='--', alpha=0.7)

# Horizontal Speed over time
plt.subplot(3, 2, 3)
plt.plot(times, h_speeds, color='seagreen', linewidth=2)
plt.title('Horizontal Speed over Time', fontsize=14)
plt.xlabel('Time (s)', fontsize=12)
plt.ylabel('Horizontal Speed (m/s)', fontsize=12)
plt.grid(True, linestyle='--', alpha=0.7)

# Thrust over time
plt.subplot(3, 2, 4)
plt.plot(times, thrusts, color='gold', linewidth=2)
plt.title('Thrust over Time', fontsize=14)
plt.xlabel('Time (s)', fontsize=12)
plt.ylabel('Thrust (N)', fontsize=12)
plt.grid(True, linestyle='--', alpha=0.7)

# Angular Position over time
plt.subplot(3, 2, 5)
plt.plot(times, angles, color='purple', linewidth=2)
plt.title('Angular Position over Time', fontsize=14)
plt.xlabel('Time (s)', fontsize=12)
plt.ylabel('Angle (deg)', fontsize=12)
plt.grid(True, linestyle='--', alpha=0.7)

# Fuel Remaining over time
plt.subplot(3, 2, 6)
plt.plot(times, fuels, color='darkorange', linewidth=2)
plt.title('Fuel Remaining over Time', fontsize=14)
plt.xlabel('Time (s)', fontsize=12)
plt.ylabel('Fuel (liters)', fontsize=12)
plt.grid(True, linestyle='--', alpha=0.7)

plt.tight_layout()
plt.show()
