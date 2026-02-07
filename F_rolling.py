import numpy as np
from math import erf

def F_rolling(omega, terrain_angle, rover, planet, Crr):
    """
    Computes the rolling resistance force acting on the rover.

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
    Frr : scalar or numpy array
        Rolling resistance force [N]
    """

    # --- Input validation ---
    if not isinstance(rover, dict):
        raise Exception("rover must be a dictionary.")

    if not isinstance(planet, dict):
        raise Exception("planet must be a dictionary.")

    if not np.isscalar(Crr) or Crr <= 0:
        raise Exception("Crr must be a positive scalar.")

    # Convert inputs to numpy arrays for vectorized operations
    omega_array = np.array(omega, copy=False)
    angle_array = np.array(terrain_angle, copy=False)

    # Validate matching sizes
    if omega_array.shape != angle_array.shape:
        raise Exception("omega and terrain_angle must be the same size.")

    # Validate angle bounds
    if np.any(angle_array < -75) or np.any(angle_array > 75):
        raise Exception("terrain_angle must be between -75 and +75 degrees.")

    # --- Physics ---
    m = get_mass(rover)
    g = planet['g']

    # Normal force (total for rover)
    angle_rad = np.deg2rad(angle_array)
    Fn_total = m * g * np.cos(angle_rad)

    # Simple rolling resistance (total, all wheels)
    Frr_simple = Crr * Fn_total

    # Rover translational velocity (needed for erf term)
    wheel_assembly = rover['wheel_assembly']
    wheel = wheel_assembly['wheel']
    speed_reducer = wheel_assembly['speed_reducer']

    Ng = get_gear_ratio(speed_reducer)
    r = wheel['radius']

    # Rover velocity v = r * (omega_out) = r * (omega / Ng)
    v_rover = r * (omega_array / Ng)

    # Apply velocity-dependent correction
    Frr = erf(40 * v_rover) * Frr_simple

    # Rolling resistance always opposes motion (assumed positive direction)
    Frr = -np.abs(Frr)

    # Return scalar if inputs were scalar
    if np.isscalar(omega) and np.isscalar(terrain_angle):
        return float(Frr)

    return Frr
