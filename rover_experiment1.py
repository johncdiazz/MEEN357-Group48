import numpy as np
import matplotlib.pyplot as plt

from define_experiment import experiment1
from subfunctions import simulate_rover


# -----------------------------
# Define planet dictionary
# -----------------------------
planet = {
    'g': 3.72
}


# -----------------------------
# Define rover dictionary
# -----------------------------
rover = {
    'wheel_assembly': {
        'wheel': {
            'radius': 0.30,
            'mass': 1.0
        },
        'speed_reducer': {
            'type': 'reverted',
            'diam_pinion': 0.04,
            'diam_gear': 0.07,
            'mass': 1.5
        },
        'motor': {
            'torque_stall': 170.0,
            'torque_noload': 0.0,
            'speed_noload': 3.80,
            'mass': 5.0,
            'effcy_tau': np.array([0, 10, 20, 40, 75, 165]),
            'effcy': np.array([0, 0.55, 0.75, 0.71, 0.50, 0.05])
        }
    },
    'chassis': {
        'mass': 659.0
    },
    'science_payload': {
        'mass': 75.0
    },
    'power_subsys': {
        'mass': 90.0
    }
}


# -----------------------------
# Load experiment and end_event
# -----------------------------
experiment, end_event = experiment1()

# Required Task 8 settings
end_event['max_distance'] = 1000
end_event['max_time'] = 10000
end_event['min_velocity'] = 0.01


# -----------------------------
# Simulate rover
# -----------------------------
rover = simulate_rover(rover, planet, experiment, end_event)


# -----------------------------
# Extract telemetry
# -----------------------------
time = rover['telemetry']['Time']
position = rover['telemetry']['position']
velocity = rover['telemetry']['velocity']
power = rover['telemetry']['power']


# -----------------------------
# Plot results
# -----------------------------
plt.figure(figsize=(8, 10))

plt.subplot(3, 1, 1)
plt.plot(time, position)
plt.xlabel('Time [s]')
plt.ylabel('Position [m]')
plt.title('Position vs. Time')
plt.grid(True)

plt.subplot(3, 1, 2)
plt.plot(time, velocity)
plt.xlabel('Time [s]')
plt.ylabel('Velocity [m/s]')
plt.title('Velocity vs. Time')
plt.grid(True)

plt.subplot(3, 1, 3)
plt.plot(time, power)
plt.xlabel('Time [s]')
plt.ylabel('Power [W]')
plt.title('Power vs. Time')
plt.grid(True)

plt.tight_layout()
plt.show()


# -----------------------------
# Print telemetry values
# -----------------------------
print('completion_time =', rover['telemetry']['completion_time'])
print('distance_traveled =', rover['telemetry']['distance_traveled'])
print('max_velocity =', rover['telemetry']['max_velocity'])
print('average_velocity =', rover['telemetry']['average_velocity'])
print('battery_energy =', rover['telemetry']['battery_energy'])
print('energy_per_distance =', rover['telemetry']['energy_per_distance'])