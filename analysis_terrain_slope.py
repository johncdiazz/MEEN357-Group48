import numpy as np
import matplotlib.pyplot as plt

from subfunctions import F_net

Crr = 0.15

# terrain slopes (DEGREES)
slope_array_deg = np.linspace(-15, 35, 25)

# container for max velocities
v_max = np.zeros(len(slope_array_deg))

# Rover + planet definitions
wheel = {
    'radius': 0.30,
    'mass': 1.0
}

speed_reducer = {
    'type': 'reverted',
    'diam_pinion': 0.04,
    'diam_gear': 0.07,
    'mass': 1.5
}

motor = {
    'torque_stall': 170,
    'torque_noload': 0,
    'speed_noload': 3.80,
    'mass': 5.0
}

wheel_assembly = {
    'wheel': wheel,
    'speed_reducer': speed_reducer,
    'motor': motor
}

rover = {
    'wheel_assembly': wheel_assembly,
    'chassis': {'mass': 659},
    'science_payload': {'mass': 75},
    'power_subsys': {'mass': 90}
}

planet = {
    'g': 3.72
}

# -----------------------------
# Root-finding (bisection)
# -----------------------------
for i, slope in enumerate(slope_array_deg):

    omega_min = 0.0
    omega_max = motor['speed_noload']   # reset each slope

    def Fnet_at_omega(omega):
        return F_net(omega, slope, rover, planet, Crr)

    f_low = Fnet_at_omega(omega_min)
    f_high = Fnet_at_omega(omega_max)

    # no root → rover keeps accelerating downhill
    if f_low * f_high > 0:
        v_max[i] = np.nan
        continue

    for _ in range(50):
        omega_mid = 0.5 * (omega_min + omega_max)
        f_mid = Fnet_at_omega(omega_mid)

        if f_low * f_mid <= 0:
            omega_max = omega_mid
            f_high = f_mid
        else:
            omega_min = omega_mid
            f_low = f_mid

    omega_terminal = 0.5 * (omega_min + omega_max)

    # convert omega → rover velocity
    v_max[i] = wheel['radius'] * omega_terminal

# -----------------------------
# Plot
# -----------------------------
plt.figure()
plt.plot(slope_array_deg, v_max, 'o-')
plt.xlabel("Terrain Slope [deg]")
plt.ylabel("Maximum Rover Speed [m/s]")
plt.grid(True)
plt.show()
