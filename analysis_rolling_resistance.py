import numpy as np
import matplotlib.pyplot as plt

from subfunctions import F_net, get_gear_ratio

# ----------------------------------------------------
# Constants
# ----------------------------------------------------
planet = {"g": 3.72}
terrain_angle = 0.0          # flat ground (degrees)

# Rover definition (must be local to this script)
rover = {
    "wheel_assembly": {
        "wheel": {
            "radius": 0.30,
            "mass": 1.0
        },
        "motor": {
            "torque_stall": 170,
            "torque_noload": 0,
            "speed_noload": 3.80,
            "mass": 5.0
        },
        "speed_reducer": {
            "type": "reverted",
            "diam_pinion": 0.04,
            "diam_gear": 0.07,
            "mass": 1.5
        }
    },
    "chassis": {"mass": 659},
    "science_payload": {"mass": 75},
    "power_subsys": {"mass": 90}
}

# ----------------------------------------------------
# Rolling resistance values to test
# ----------------------------------------------------
Crr_array = np.linspace(0.01, 0.5, 25)
v_max = np.zeros(len(Crr_array))

# Gear ratio and wheel radius
Ng = get_gear_ratio(rover["wheel_assembly"]["speed_reducer"])
r = rover["wheel_assembly"]["wheel"]["radius"]

# ----------------------------------------------------
# Loop over rolling resistance values
# ----------------------------------------------------
for i, Crr in enumerate(Crr_array):

    # RESET bounds for each Crr
    omega_low = 1e-3
    omega_high = rover["wheel_assembly"]["motor"]["speed_noload"]

    def F(omega):
        return F_net(omega, terrain_angle, rover, planet, Crr)

    F_low = F(omega_low)
    F_high = F(omega_high)

    # If no terminal speed exists
    if np.isnan(F_low) or np.isnan(F_high) or F_low * F_high > 0:
        v_max[i] = np.nan
        continue

    # Bisection method
    for _ in range(60):
        omega_mid = 0.5 * (omega_low + omega_high)
        F_mid = F(omega_mid)

        if F_low * F_mid < 0:
            omega_high = omega_mid
            F_high = F_mid
        else:
            omega_low = omega_mid
            F_low = F_mid

    omega_star = 0.5 * (omega_low + omega_high)

    # Convert motor speed → rover speed
    v_max[i] = r * omega_star / Ng

# ----------------------------------------------------
# Plot results
# ----------------------------------------------------
plt.figure()
plt.plot(Crr_array, v_max, 'o-', linewidth=2)
plt.xlabel("Rolling resistance coefficient, $C_{rr}$ [-]")
plt.ylabel("Maximum rover speed [m/s]")
plt.grid(True)
plt.tight_layout()
plt.show()
