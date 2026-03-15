import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

# Example rover definition
# Replace this with your actual rover import if you already have one
def define_rover():
    rover = {
        'wheel_assembly': {
            'motor': {
                'effcy_tau': np.array([0, 10, 20, 40, 75, 165]),
                'effcy': np.array([0, 0.60, 0.75, 0.73, 0.55, 0.05])
            }
        }
    }
    return rover

# Get rover dictionary
rover = define_rover()

# Extract efficiency data
effcy_tau = rover['wheel_assembly']['motor']['effcy_tau']
effcy = rover['wheel_assembly']['motor']['effcy']

# Create cubic interpolation function
effcy_fun = interp1d(effcy_tau, effcy, kind='cubic')

# Create 100 evenly spaced torque points
tau_plot = np.linspace(np.min(effcy_tau), np.max(effcy_tau), 100)

# Evaluate efficiency at those points
effcy_plot = effcy_fun(tau_plot)

# Plot
plt.figure()
plt.plot(effcy_tau, effcy, '*', label='Given data')
plt.plot(tau_plot, effcy_plot, '-', label='Cubic interpolation')
plt.xlabel('Motor Torque [N·m]')
plt.ylabel('Efficiency')
plt.title('Motor Efficiency vs. Torque')
plt.grid(True)
plt.legend()
plt.show()