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
