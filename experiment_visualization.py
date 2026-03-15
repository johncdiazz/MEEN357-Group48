import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

def experiment1():
    experiment = {
        'time_range': np.array([0, 20000]),
        'initial_conditions': np.array([0.325, 0]),
        'alpha_dist': np.array([0, 100, 200, 300, 400, 500, 600,
                                700, 800, 900, 1000]),
        'alpha_deg': np.array([2.032, 11.509, 2.478, 7.182,
                               5.511, 10.981, 5.601, -0.184,
                               0.714, 4.151, 4.042]),
        'Crr': 0.1
    }

    end_event = {
        'max_distance': 50,
        'max_time': 5000,
        'min_velocity': 0.01
    }

    return experiment, end_event


# Get experiment data
experiment, end_event = experiment1()

# Extract terrain data
alpha_dist = experiment['alpha_dist']
alpha_deg = experiment['alpha_deg']

# Create cubic interpolation function
alpha_fun = interp1d(alpha_dist, alpha_deg, kind='cubic', fill_value='extrapolate')

# Create 100 evenly spaced points
dist_eval = np.linspace(np.min(alpha_dist), np.max(alpha_dist), 100)

# Evaluate terrain angle
alpha_eval = alpha_fun(dist_eval)

# Plot
plt.figure()
plt.plot(alpha_dist, alpha_deg, '*', label='Given data')
plt.plot(dist_eval, alpha_eval, '-', label='Cubic spline')
plt.xlabel('Position [m]')
plt.ylabel('Terrain Angle [deg]')
plt.title('Terrain Angle vs. Position')
plt.legend()
plt.grid(True)
plt.show()