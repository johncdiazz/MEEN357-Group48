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
