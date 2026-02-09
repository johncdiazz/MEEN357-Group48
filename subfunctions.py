def get_gear_ratio(speed_reducer):
    """
    Computes the speed reduction (gear) ratio for a speed reducer.

    Parameters
    ----------
    speed_reducer : dict
        Dictionary containing speed reducer parameters.

    Returns
    -------
    Ng : float
        Speed reduction ratio (unitless).
    """

    # --- Input validation ---
    if not isinstance(speed_reducer, dict):
        raise Exception("speed_reducer must be a dictionary.")

    if 'type' not in speed_reducer:
        raise Exception("speed_reducer dictionary must contain a 'type' field.")

    if speed_reducer['type'].lower() != 'reverted':
        raise Exception(
            "Unsupported speed reducer type. "
            "For Phase 1, type must be 'reverted'."
        )

    # Required fields for reverted gear train
    required_keys = ['diam_pinion', 'diam_gear']
    for key in required_keys:
        if key not in speed_reducer:
            raise Exception(f"Missing '{key}' in speed_reducer dictionary.")

    d1 = speed_reducer['diam_pinion']
    d2 = speed_reducer['diam_gear']

    # Basic value checks
    if d1 <= 0 or d2 <= 0:
        raise Exception("Gear diameters must be positive values.")

    # --- Gear ratio calculation ---
    Ng = (d2 / d1) ** 2

    return Ng
#----------------------------------------------------------------------------#
import numpy as np

def tau_dcmotor(omega, motor):
    """
    Returns motor shaft torque given motor shaft speed and motor parameters.

    Parameters
    ----------
    omega : scalar or numpy array
        Motor shaft speed [rad/s]
    motor : dict
        Dictionary containing motor parameters

    Returns
    -------
    tau : scalar or numpy array
        Motor shaft torque [Nm]
    """

    # --- Input validation ---
    if not isinstance(motor, dict):
        raise Exception("motor must be a dictionary.")

    # Required motor parameters
    required_keys = ['torque_stall', 'torque_noload', 'speed_noload']
    for key in required_keys:
        if key not in motor:
            raise Exception(f"Missing '{key}' in motor dictionary.")

    # Convert omega to numpy array for vectorized operations
    omega_array = np.array(omega, copy=False)

    tau_s = motor['torque_stall']
    tau_nl = motor['torque_noload']
    omega_nl = motor['speed_noload']

    # Initialize torque array
    tau = np.zeros_like(omega_array, dtype=float)

    # Case 1: omega < 0  → stall torque
    tau[omega_array < 0] = tau_s

    # Case 2: 0 ≤ omega ≤ omega_nl → linear torque-speed relation
    mask = (omega_array >= 0) & (omega_array <= omega_nl)
    tau[mask] = tau_s - ((tau_s - tau_nl) / omega_nl) * omega_array[mask]

    # Case 3: omega > omega_nl → zero torque
    tau[omega_array > omega_nl] = 0.0

    # Return scalar if input was scalar
    if np.isscalar(omega):
        return float(tau)

    return tau
#----------------------------------------------------------------------------#
import numpy as np

def F_drive(omega, rover):
    """
    Computes the total drive force acting on the rover due to all six wheels.

    Parameters
    ----------
    omega : scalar or numpy array
        Motor shaft speed [rad/s]
    rover : dict
        Dictionary containing rover parameters

    Returns
    -------
    Fd : scalar or numpy array
        Drive force [N]
    """

    # --- Input validation ---
    if not isinstance(rover, dict):
        raise Exception("rover must be a dictionary.")

    # Convert omega to numpy array for vectorized operations
    omega_array = np.array(omega, copy=False)

    # Extract sub-dictionaries
    wheel_assembly = rover['wheel_assembly']
    motor = wheel_assembly['motor']
    speed_reducer = wheel_assembly['speed_reducer']
    wheel = wheel_assembly['wheel']

    # --- Motor torque ---
    tau_motor = tau_dcmotor(omega_array, motor)

    # --- Speed reducer ---
    Ng = get_gear_ratio(speed_reducer)

    # Torque at wheel axle
    tau_wheel = Ng * tau_motor

    # --- Wheel force (per wheel) ---
    r = wheel['radius']
    F_wheel = tau_wheel / r

    # --- Total drive force (6 wheels) ---
    Fd = 6 * F_wheel

    # Return scalar if input was scalar
    if np.isscalar(omega):
        return float(Fd)

    return Fd

#----------------------------------------------------------------------------#
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

#----------------------------------------------------------------------------#
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

#----------------------------------------------------------------------------#
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

#---------------------------------------------------------------------#
#get mass function 
# ''This function computes rover mass in kilograms. It accounts for the chassis, power subsystem, science payload,
#and six wheel assemblies, which itself is comprised of a motor, speed reducer, and the wheel itself'''

#start with rover dictionary, should just be plug and chug from table values
# masses in kg
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
def get_mass(rover):
    if not isinstance(rover, dict):
        raise Exception("Input must be a dictionary")

    total = 0
    for v in rover.values():
        if isinstance(v, dict):
            total += get_mass(v)
        elif isinstance(v, (int, float)):
            total += v

    # multiply wheel assembly mass by 6
    if "wheel_assembly" in rover:
        total += 5 * get_mass(rover["wheel_assembly"])

    return total
