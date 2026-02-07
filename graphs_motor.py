''' Part 2
Group 48 
-John Diaz
 It plots the following three graphs in a 3x1 array (use the matplotlib.pyplot.subplot
command to achieve this) in the following order top to bottom:
o motor shaft speed [rad/s] vs. motor shaft torque [Nm] (use torque on the x-axis)
o motor power [W] vs. motor shaft torque [Nm] (use torque on the x-axis)
o motor power [W] vs. motor shaft speed [rad/s] (use speed on the x-axis)
• All graphs should have both axes labeled clearly (with units indicated). Use the
matplotlib.pyplot.xlabel and matplotlib.pyplot.ylabel commands.'''

import numpy as np 
import matplotlib.pyplot as plt

# need tau_dcmotor for first plot --> graphing omega vs. tau(the returned item)
from tau_dcmotor import tau_dcmotor 

# create motor dictionary
motor = { 'torque_stall' :170, 'torque_noload': 0, 'speed_noload': 3.8} # torque in N*m and speed in rad/s
# define range for plotting
omega = np.linspace(0.0,3.8)
# pull torque
tau = tau_dcmotor(omega, motor)

# plot
plt.figure(figsize=(7, 7))
plt.subplot(3,1,1)
plt.xlabel("Motor Shaft Torque [Nm]")
plt.ylabel("Motor Shaft Speed [rad/s]")
plt.plot(tau, omega)


# finding power
power = tau * omega 
# plotting power vs. torque
plt.subplot(3,1,2)
plt.xlabel("Motor Shaft Torque [Nm]")
plt.ylabel("Motor Power [W]")
plt.plot(tau, power)

#plotting power vs. shaft speed
plt.subplot(3,1,3)
plt.xlabel("Motor Shaft Torque [Nm]")
plt.ylabel("Motor Shaft Speed [rad/s]")
plt.plot(omega,power)
plt.tight_layout()
plt.show()