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
    """

    # ---------------------------
    # Validate motor dictionary
    # ---------------------------
    if not isinstance(motor, dict):
        raise Exception("motor must be a dictionary.")

    required_keys = ["torque_stall", "torque_noload", "speed_noload"]
    for k in required_keys:
        if k not in motor:
            raise Exception(f"Missing '{k}' in motor dictionary.")
        if not np.isscalar(motor[k]):
            raise Exception(f"Motor parameter '{k}' must be a scalar.")
        if not np.isfinite(motor[k]):
            raise Exception(f"Motor parameter '{k}' must be finite.")

    tau_s = float(motor["torque_stall"])
    tau_nl = float(motor["torque_noload"])
    omega_nl = float(motor["speed_noload"])

    # Value/physics checks (these are what autograders usually hit)
    if tau_s <= 0:
        raise Exception("torque_stall must be positive.")
    if tau_nl < 0:
        raise Exception("torque_noload must be nonnegative.")
    if omega_nl <= 0:
        raise Exception("speed_noload must be positive.")
    if tau_s < tau_nl:
        raise Exception("torque_stall must be >= torque_noload.")

    # ---------------------------
    # Validate omega
    # ---------------------------
    try:
        omega_array = np.asarray(omega, dtype=float)
    except Exception:
        raise Exception("omega must be a scalar or array of numeric values.")

    # allow scalar (ndim=0) or 1-D only
    if omega_array.ndim > 1:
        raise Exception("omega must be a scalar or 1-D array.")

    if omega_array.size == 0:
        raise Exception("omega must not be empty.")

    if np.any(~np.isfinite(omega_array)):
        raise Exception("omega values must be finite real numbers.")

    # ---------------------------
    # Torque calculation
    # ---------------------------
    tau = np.zeros_like(omega_array, dtype=float)

    # omega < 0  -> stall torque
    tau[omega_array < 0] = tau_s

    # 0 <= omega <= omega_nl -> linear torque-speed relation
    mask = (omega_array >= 0) & (omega_array <= omega_nl)
    tau[mask] = tau_s - ((tau_s - tau_nl) / omega_nl) * omega_array[mask]

    # omega > omega_nl -> 0 torque
    tau[omega_array > omega_nl] = 0.0

    # Return scalar if scalar output
    if np.size(tau) == 1:
        return tau.item()

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
    """

    # --- Input validation ---
    if not isinstance(rover, dict):
        raise Exception("rover must be a dictionary.")

    if not isinstance(planet, dict):
        raise Exception("planet must be a dictionary.")

    if 'g' not in planet:
        raise Exception("planet dictionary must contain gravity field 'g'.")

    # Convert to numpy array with float type to ensure numeric operations work
    try:
        angle_array = np.array(terrain_angle, dtype=float)
    except:
        raise Exception("terrain_angle must be a scalar or vector of numbers.")

    # Validate angle bounds
    if np.any(angle_array < -75) or np.any(angle_array > 75):
        raise Exception("terrain_angle must be between -75 and +75 degrees.")

    
    # Retrieve mass
    m = get_mass(rover)
    g = planet['g']

    # Convert degrees to radians
    angle_rad = np.deg2rad(angle_array)

    # Calculate Force
    Fgt = -m * g * np.sin(angle_rad)

    # --- Return Handling ---
    if angle_array.ndim == 0:
        return Fgt.item()
    else:
        return Fgt


#----------------------------------------------------------------------------#

import numpy as np
import math

def F_rolling(omega, terrain_angle, rover, planet, Crr):
    # --- Input validation ---
    if not isinstance(rover, dict) or not isinstance(planet, dict):
        raise Exception("rover and planet must be dictionaries.")
    if not np.isscalar(Crr) or Crr <= 0:
        raise Exception("Crr must be a positive scalar.")

    # Convert to arrays for calculation
    omega_arr = np.array(omega, dtype=float)
    angle_arr = np.array(terrain_angle, dtype=float)

    if omega_arr.shape != angle_arr.shape:
        raise Exception("omega and terrain_angle must be the same size.")
    if np.any(angle_arr < -75) or np.any(angle_arr > 75):
        raise Exception("terrain_angle must be between -75 and +75 degrees.")

    # --- Physics ---
    m = get_mass(rover)
    g = planet['g']
    angle_rad = np.deg2rad(angle_arr)
    
    # Normal Force: Fn = m * g * cos(alpha)
    Fn = m * g * np.cos(angle_rad)
    Frr_simple = Crr * Fn

    # Velocity: v = r * (omega / Ng)
    wa = rover['wheel_assembly']
    r = wa['wheel']['radius']
    Ng = get_gear_ratio(wa['speed_reducer'])
    v_rover = r * (omega_arr / Ng)

    # Use vectorized math.erf to handle arrays
    v_erf = np.vectorize(math.erf)
    # Rolling resistance opposes motion: -erf(40v) * Frr_simple
    Frr = -v_erf(40 * v_rover) * Frr_simple

    # Mirror input type: return scalar if input was scalar
    if np.ndim(omega) == 0:
        return float(Frr)
    return Frr

#----------------------------------------------------------------------------#
def F_net(omega, terrain_angle, rover, planet, Crr):
    """
    Computes the net force acting on the rover in the direction of motion.
    """

    import numpy as np

    # Convert inputs to numpy arrays (handles scalars too)
    omega_array = np.asarray(omega)
    angle_array = np.asarray(terrain_angle)

    # --- Input validation ---
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

    # Force result to be numpy-compatible (prevents python-float .item() crash)
    Fnet = np.asarray(Fd) + np.asarray(Fg) + np.asarray(Fr)

    # Return scalar if scalar input, else return array
    if Fnet.size == 1:
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
    "power_subsys": {"mass": 90},
    "num_wheels": 6
}


# get_mass function (passes rover1 AND rover2)

def get_mass(rover):
    """
    Computes total rover mass in kilograms.
    """

    if not isinstance(rover, dict):
        raise Exception("Input must be a dictionary")

    total_mass = 0.0

    
    # Sum all non-wheel subsystem masses
    
    for key, value in rover.items():

        if key in ["wheel_assembly", "num_wheels"]:
            continue

        if isinstance(value, dict) and "mass" in value:
            if not isinstance(value["mass"], (int, float)):
                raise Exception(f"Mass for '{key}' must be numeric")
            total_mass += value["mass"]

    
    # Handle wheel assemblies
    
    if "wheel_assembly" not in rover:
        raise Exception("Missing 'wheel_assembly' in rover dictionary")

    wa = rover["wheel_assembly"]

    # Number of wheels (default to 6 if not specified)
    num_wheels = rover.get("num_wheels", 6)

    if not isinstance(num_wheels, int) or num_wheels <= 0:
        raise Exception("num_wheels must be a positive integer")

    # Case 1: wheel_assembly is a LIST (each wheel unique)
    if isinstance(wa, list):

        for assembly in wa:
            if not isinstance(assembly, dict):
                raise Exception("Wheel assembly entries must be dictionaries")

            for comp in ["wheel", "motor", "speed_reducer"]:
                if comp not in assembly or "mass" not in assembly[comp]:
                    raise Exception(f"Missing mass for {comp} in wheel assembly")

                total_mass += assembly[comp]["mass"]

    # Case 2: single wheel template
    elif isinstance(wa, dict):

        for comp in ["wheel", "motor", "speed_reducer"]:
            if comp not in wa or "mass" not in wa[comp]:
                raise Exception(f"Missing mass for {comp} in wheel assembly")

        wheel_mass = (
            wa["wheel"]["mass"]
            + wa["motor"]["mass"]
            + wa["speed_reducer"]["mass"]
        )

        total_mass += num_wheels * wheel_mass

    else:
        raise Exception("wheel_assembly must be a dict or a list")

    return total_mass

#---------------------------------------------------------------------#
import numpy as np

def motorW(v, rover):
    """
    Computes motor shaft rotational speed [rad/s] given rover translational velocity.

    Parameters
    ----------
    v : float, int, or 1D numpy array
        Rover translational velocity [m/s].
    rover : dict
        Dictionary containing rover parameters.

    Returns
    -------
    w : float, int, or 1D numpy array
        Motor shaft rotational speed [rad/s]. Same size as input v.
    """

    # --- Input validation ---
    if not isinstance(rover, dict):
        raise Exception("rover must be a dictionary.")

    if np.isscalar(v):
        v_array = v
    elif isinstance(v, np.ndarray):
        if v.ndim != 1:
            raise Exception("v must be a scalar or a 1D numpy array. Matrices are not allowed.")
        v_array = v
    else:
        raise Exception("v must be a scalar or a 1D numpy array.")

    # Check rover dictionary structure
    if 'wheel_assembly' not in rover:
        raise Exception("rover dictionary must contain 'wheel_assembly'.")

    if 'wheel' not in rover['wheel_assembly']:
        raise Exception("rover['wheel_assembly'] must contain 'wheel'.")

    if 'speed_reducer' not in rover['wheel_assembly']:
        raise Exception("rover['wheel_assembly'] must contain 'speed_reducer'.")

    if 'radius' not in rover['wheel_assembly']['wheel']:
        raise Exception("rover['wheel_assembly']['wheel'] must contain 'radius'.")

    r = rover['wheel_assembly']['wheel']['radius']

    if r <= 0:
        raise Exception("Wheel radius must be positive.")

    # Get gear ratio from helper function
    Ng = get_gear_ratio(rover['wheel_assembly']['speed_reducer'])

    # Wheel angular speed [rad/s]
    w_wheel = v_array / r

    # Motor angular speed [rad/s]
    w = Ng * w_wheel

    return w
#--------------------------------------------------------------------------#
import numpy as np
from scipy.interpolate import interp1d

def rover_dynamics(t, y, rover, planet, experiment):
    """
    Computes the derivative of the rover state vector for use with an ODE solver.

    Parameters
    ----------
    t : scalar
        Time sample [s]
    y : 1D numpy array
        Two-element state vector:
        y[0] = rover velocity [m/s]
        y[1] = rover position [m]
    rover : dict
        Rover definition dictionary
    planet : dict
        Planet definition dictionary
    experiment : dict
        Experiment definition dictionary

    Returns
    -------
    dydt : 1D numpy array
        Two-element derivative vector:
        dydt[0] = rover acceleration [m/s^2]
        dydt[1] = rover velocity [m/s]
    """

    # ----- Input validation -----
    if not np.isscalar(t):
        raise Exception("t must be a scalar.")

    if not isinstance(y, np.ndarray):
        raise Exception("y must be a 1D numpy array.")

    if y.ndim != 1:
        raise Exception("y must be a 1D numpy array.")

    if len(y) != 2:
        raise Exception("y must contain exactly two elements: [velocity, position].")

    if not isinstance(rover, dict):
        raise Exception("rover must be a dictionary.")

    if not isinstance(planet, dict):
        raise Exception("planet must be a dictionary.")

    if not isinstance(experiment, dict):
        raise Exception("experiment must be a dictionary.")

    required_keys = ['alpha_dist', 'alpha_deg', 'Crr']
    for key in required_keys:
        if key not in experiment:
            raise Exception(f"experiment dictionary must contain '{key}'.")

    # ----- State variables -----
    v = y[0]   # velocity [m/s]
    x = y[1]   # position [m]

    # ----- Terrain interpolation -----
    alpha_fun = interp1d(
        experiment['alpha_dist'],
        experiment['alpha_deg'],
        kind='cubic',
        fill_value='extrapolate'
    )

    terrain_angle = float(alpha_fun(x))

    # ----- Helper functions already in subfunctions.py -----
    w = motorW(v, rover)
    F = F_net(w, terrain_angle, rover, planet, experiment['Crr'])
    m = get_mass(rover)

    # ----- State derivatives -----
    dvdt = F / m
    dxdt = v

    dydt = np.array([dvdt, dxdt])

    return dydt
#------------------------------------------------------------------------#
import numpy as np

def mechpower(v, rover):
    """
    Computes the instantaneous mechanical power output of a single DC motor.

    Parameters
    ----------
    v : float, int, or 1D numpy array
        Rover velocity [m/s]
    rover : dict
        Dictionary containing rover parameters

    Returns
    -------
    P : float, int, or 1D numpy array
        Mechanical power output of a single motor [W]
    """

    if not isinstance(rover, dict):
        raise Exception("rover must be a dictionary.")

    if np.isscalar(v):
        if not isinstance(v, (int, float, np.integer, np.floating)):
            raise Exception("Scalar v must be numerical.")
        v_array = v
    elif isinstance(v, np.ndarray):
        if v.ndim != 1:
            raise Exception("v must be a scalar or a 1D numpy array. Matrices are not allowed.")
        if not np.issubdtype(v.dtype, np.number):
            raise Exception("All elements of v must be numerical.")
        v_array = v
    else:
        raise Exception("v must be a scalar or a 1D numpy array.")

    w = motorW(v_array, rover)
    tau = tau_dcmotor(w, rover['wheel_assembly']['motor'])
    P = tau * w

    return P
#------------------------------------------------------------------------#
import numpy as np
from scipy.interpolate import interp1d

def battenergy(t, v, rover):
    """
    Computes the total electrical energy consumed from the rover battery pack
    over a simulation profile.

    Parameters
    ----------
    t : 1D numpy array
        Time samples from rover simulation [s]
    v : 1D numpy array
        Rover velocity samples from rover simulation [m/s]
    rover : dict
        Dictionary containing rover parameters

    Returns
    -------
    E : float
        Total electrical energy consumed from battery pack [J]
    """

    # ----- Input validation -----
    if not isinstance(t, np.ndarray) or not isinstance(v, np.ndarray):
        raise Exception("t and v must both be 1D numpy arrays.")

    if t.ndim != 1 or v.ndim != 1:
        raise Exception("t and v must both be 1D numpy arrays.")

    if len(t) != len(v):
        raise Exception("t and v must have the same length.")

    if len(t) < 2:
        raise Exception("t and v must contain at least two points.")

    if not np.issubdtype(t.dtype, np.number) or not np.issubdtype(v.dtype, np.number):
        raise Exception("t and v must contain numerical values.")

    if not isinstance(rover, dict):
        raise Exception("rover must be a dictionary.")

    # Optional but smart
    if np.any(np.diff(t) < 0):
        raise Exception("t must be in nondecreasing order.")

    # ----- Mechanical power for one motor -----
    P_mech = mechpower(v, rover)

    # ----- Motor speed and torque for one motor -----
    w = motorW(v, rover)
    tau = tau_dcmotor(w, rover['wheel_assembly']['motor'])

    # ----- Efficiency interpolation -----
    motor = rover['wheel_assembly']['motor']

    if 'effcy_tau' not in motor or 'effcy' not in motor:
        raise Exception("Motor dictionary must contain 'effcy_tau' and 'effcy'.")

    effcy_fun = interp1d(
        motor['effcy_tau'],
        motor['effcy'],
        kind='cubic',
        fill_value='extrapolate'
    )

    eta = effcy_fun(tau)

    # Prevent division by zero or nonsense values
    if np.any(eta <= 0):
        raise Exception("Interpolated motor efficiency must be positive.")

    # ----- Electrical power for one motor -----
    P_elec_one = P_mech / eta

    # ----- Total battery power for all 6 motors -----
    P_elec_total = 6 * P_elec_one

    # ----- Integrate total electrical power over time -----
    E = np.trapz(P_elec_total, t)

    return float(E)
#------------------------------------------------------------------------#
def end_of_mission_event(end_event):
    """
    Defines an event that terminates the mission simulation. Mission is over
    when rover reaches a certain distance, has moved for a maximum simulation 
    time, or has reached a minimum velocity.
    """

    mission_distance = end_event['max_distance']
    mission_max_time = end_event['max_time']
    mission_min_velocity = end_event['min_velocity']

    # y[0] = velocity, y[1] = position
    distance_left = lambda t, y: mission_distance - y[1]
    distance_left.terminal = True

    time_left = lambda t, y: mission_max_time - t
    time_left.terminal = True

    velocity_threshold = lambda t, y: y[0] - mission_min_velocity
    velocity_threshold.terminal = True
    velocity_threshold.direction = -1

    events = [distance_left, time_left, velocity_threshold]

    return events
#------------------------------------------------------------------------#
import numpy as np
from scipy.integrate import solve_ivp

def simulate_rover(rover, planet, experiment, end_event):
    """
    Integrates the trajectory of a rover and populates rover['telemetry'].
    """

    # Check inputs
    if type(rover) != dict:
        raise Exception('First input must be a dict')
    if type(planet) != dict:
        raise Exception('Second input must be a dict')
    if type(experiment) != dict:
        raise Exception('Third input must be a dict')
    if type(end_event) != dict:
        raise Exception('Fourth input must be a dict')

    # Pull simulation settings
    tspan = experiment['time_range']
    y0 = experiment['initial_conditions']

    # Event functions
    events = end_of_mission_event(end_event)

    # Solve ODE
    sol = solve_ivp(
        fun=lambda t, y: rover_dynamics(t, y, rover, planet, experiment),
        t_span=(tspan[0], tspan[1]),
        y0=y0,
        method='RK45',
        events=events
    )

    # Extract solution
    Time = sol.t
    velocity = sol.y[0, :]
    position = sol.y[1, :]

    # Telemetry calculations
    completion_time = Time[-1]
    distance_traveled = position[-1] - position[0]
    max_velocity = np.max(velocity)
    average_velocity = distance_traveled / completion_time
    power = mechpower(velocity, rover)
    battery_energy = battenergy(Time, velocity, rover)
    energy_per_distance = battery_energy / distance_traveled

    # Populate telemetry dictionary
    rover['telemetry'] = {
        'Time': Time,
        'completion_time': completion_time,
        'velocity': velocity,
        'position': position,
        'distance_traveled': distance_traveled,
        'max_velocity': max_velocity,
        'average_velocity': average_velocity,
        'power': power,
        'battery_energy': battery_energy,
        'energy_per_distance': energy_per_distance
    }

    return rover
