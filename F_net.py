import numpy as np

def F_net(omega, terrain_angle, rover, planet, Crr):
    """
    Computes the net force acting on the rover in the direction of motion.

    Parameters
    ----------
    omega : scalar or numpy array
        Motor shaft speed [rad/s]
    terrain_angle : scalar or numpy array
        Terrain inclination angle [deg]
    rover : dict
        Dictionary containing rover parameters
    planet : dict
        Dictionary containing planet gravity parameter
    Crr : scalar
        Rolling resistance coefficient [-]

    Returns
    -------
    Fnet : scalar or numpy array
        Net force acting on the rover [N]
    """

    # --- Input validation ---
    omega_array = np.array(omega, copy=False)
    angle_array = np.array(terrain_angle, copy=False)

    if omega_array.shape != angle_array.shape:
        raise Exception("omega and terrain_angle must be the same size.")

    if not isinstance(rover, dict):
        raise Exception("rover must be a dictionary.")

    if not isinstance(planet, dict):
        raise Exception("planet must be a dictionary.")

    if not np.isscalar(Crr) or Crr <= 0:
        raise Exception("Crr must be a positive scalar.")

    if np.any(angle_array < -75) or np.any(angle_array > 75):
        raise Exception("terrain_angle must be between -75 and +75 degrees.")

    # --- Force components ---
    Fd = F_drive(omega_array, rover)
    Fg = F_gravity(angle_array, rover, planet)
    Fr = F_rolling(omega_array, angle_array, rover, planet, Crr)

    # --- Net force ---
    Fnet = Fd + Fg + Fr

    # Return scalar if inputs were scalar
    if np.isscalar(omega) and np.isscalar(terrain_angle):
        return float(Fnet)

    return Fnet
