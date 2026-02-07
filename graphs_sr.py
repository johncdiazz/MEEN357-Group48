import numpy as np 
import matplotlib.pyplot as plt 
from get_gear_ratio import get_gear_ratio
from tau_dcmotor import tau_dcmotor
# create dictionary and necassary values for speed reducer
d1 = 0.04
d2 = 0.07
speed_reducer = { 'diam_pinion' : d1, 'diam_gear': d2, 'type': 'reverted'}

omega = np.linspace(0.0,3.8) # for input to reducer
motor = { 'torque_stall' :170, 'torque_noload': 0, 'speed_noload': 3.8} # for input 
tau_in = tau_dcmotor(omega, motor)

# call get gear ratio for Ng
Ng = get_gear_ratio(speed_reducer)

# find torques and speeds ####### tau in and omega in are output from dc motor
## tau_out = tau_in * Ng
## omega_out = omega_in / Ng
tau_out = omega/Ng
omega_out = tau_in * Ng

# plot
plt.figure(figsize=(7, 7))
plt.subplot(3,1,1)
plt.xlabel("Speed Reducer Torque [Nm]")
plt.ylabel("Speed Reducer Speed [rad/s]")
plt.plot(tau_out, omega_out)


# finding power
power = tau_out * omega_out 
# plotting power vs. torque
plt.subplot(3,1,2)
plt.xlabel("Motor Shaft Torque [Nm]")
plt.ylabel("Motor Power [W]")
plt.plot(tau_out, power)

#plotting power vs. shaft speed
plt.subplot(3,1,3)
plt.xlabel("Speed Reducer Torque [Nm]")
plt.ylabel("Speed Reducer Speed [rad/s]")
plt.plot(omega_out,power)
plt.tight_layout()
plt.show()