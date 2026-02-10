import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 (required for 3D)

from subfunctions import F_net, get_gear_ratio

# ----------------------------------------------------
# Planet and rover definitions (must be local)
# ----------------------------------------------------
planet = {"g": 3.72}

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
# Step 1–2: Independent variable arrays
# ----------------------------------------------------
Crr_array = np.linspace(0.01, 0.5, 25)
slope_array_deg = np.linspace(-15, 35, 25)

# ----------------------------------------------------
# Step 3: Meshgrid
# ----------------------------------------------------
CRR, SLOPE = np.meshgrid(Crr_array, slope_array_deg)

# ----------------------------------------------------
# Step 4: Output matrix
# ----------------------------------------------------
VMAX = np.zeros(np.shape(CRR), dtype=float)

# Gear ratio and wheel radius (constants)
Ng = get_gear_ratio(rover["wheel_assembly"]["speed_reducer"])
r = rover["wheel_assembly"]["wheel"]["radius"]
omega_nl = rover["wheel_assembly"]["motor"]["speed_noload"]

# ----------------------------------------------------
# Step 5: Double loop with root finding
# ----------------------------------------------------
N = CRR.shape[0]

for i in range(N):
    for j in range(N):

        Crr_sample = float(CRR[i, j])
        slope_sample = float(SLOPE[i, j])

        # Root-finding bounds
        omega_low = 1e-4
        omega_high = omega_nl

        def F(omega):
            return F_net(omega, slope_sample, rover, planet, Crr_sample)

        F_low = F(omega_low)
        F_high = F(omega_high)

        # No terminal speed → NaN
        if np.isnan(F_low) or np.isnan(F_high) or F_low * F_high > 0:
            VMAX[i, j] = np.nan
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
        VMAX[i, j] = r * omega_star / Ng

# ----------------------------------------------------
# Step 6: Surface plot
# ----------------------------------------------------
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.plot_surface(CRR, SLOPE, VMAX, cmap='viridis')

ax.set_xlabel("Rolling resistance coefficient, $C_{rr}$ [-]")
ax.set_ylabel("Terrain slope [deg]")
ax.set_zlabel("Maximum rover speed [m/s]")
ax.set_title("Maximum Rover Speed vs. Terrain Slope and Rolling Resistance")

plt.tight_layout()
plt.show()
