import numpy as np

def F_gravity(terrain_angle, rover, planet):
    """
    Computes the component of gravitational force acting on the rover
    in the direction of translational motion.

    Parameters
    ----------
    terrain_angle : scalar or numpy array
        Terrain inclination angle [deg]
    rover : dict
        Dictionary containing rover parameters
    planet : dict
        Dictionary containing planet gravity parameter

    Returns
    -------
    Fgt : scalar or numpy array
        Gravitational force component [N]
    """

    # --- Input validation ---
    if not isinstance(rover, dict):
        raise Exception("rover must be a dictionary.")

    if not isinstance(planet, dict):
        raise Exception("planet must be a dictionary.")

    if 'g' not in planet:
        raise Exception("planet dictionary must contain gravity field 'g'.")

    # Convert terrain_angle to numpy array for vectorized operations
    angle_array = np.array(terrain_angle, copy=False)

    # Validate angle bounds
    if np.any(angle_array < -75) or np.any(angle_array > 75):
        raise Exception("terrain_angle must be between -75 and +75 degrees.")

    # --- Physics ---
    m = get_mass(rover)
    g = planet['g']

    # Convert degrees to radians
    angle_rad = np.deg2rad(angle_array)

    # Positive slope (uphill) → negative force
    Fgt = -m * g * np.sin(angle_rad)

    # Return scalar if input was scalar
    if np.isscalar(terrain_angle):
        return float(Fgt)

    return Fgt
