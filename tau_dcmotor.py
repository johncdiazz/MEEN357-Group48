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
