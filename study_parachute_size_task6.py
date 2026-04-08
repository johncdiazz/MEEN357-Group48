import numpy as np
np.NaN = np.nan

import matplotlib.pyplot as plt
from scipy.interpolate import PchipInterpolator as pchip

from define_edl_system import define_edl_system_1
from define_planet import define_planet
from define_mission_events import define_mission_events
import subfunctions_EDL as sf


# -------------------------------------------------
# Task 6: Mach efficiency factor model
# -------------------------------------------------
def mach_efficiency_factor(M):
    """
    Returns MEF(M) using shape-preserving interpolation of the
    experimental data from the assignment.
    """

    M_data = np.array([
        0.25, 0.5, 0.65, 0.7, 0.8, 0.9, 0.95, 1.0,
        1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.8, 1.9,
        2.0, 2.2, 2.5, 2.6
    ])

    MEF_data = np.array([
        1.0, 1.0, 1.0, 0.97, 0.91, 0.72, 0.66, 0.75,
        0.90, 0.96, 0.990, 0.999, 0.992, 0.98, 0.91, 0.85,
        0.82, 0.75, 0.64, 0.62
    ])

    fit = pchip(M_data, MEF_data)

    # clip to the tabulated Mach range
    M = np.clip(M, M_data[0], M_data[-1])

    return fit(M)


# -------------------------------------------------
# Original constant-Cd study
# -------------------------------------------------
def run_study_constant_cd():
    diameters = np.arange(14.0, 19.0 + 0.001, 0.5)

    sim_time = []
    rover_speed_term = []
    landing_success = []

    for D in diameters:
        edl_system = define_edl_system_1()
        mars = define_planet()
        mission_events = define_mission_events()

        # Task 5 / Task 6 required initial conditions
        edl_system['altitude'] = 11000
        edl_system['velocity'] = -590
        edl_system['rocket']['on'] = False
        edl_system['parachute']['deployed'] = True
        edl_system['parachute']['ejected'] = False
        edl_system['heat_shield']['ejected'] = False
        edl_system['sky_crane']['on'] = False
        edl_system['speed_control']['on'] = False
        edl_system['position_control']['on'] = False
        edl_system['rover']['on_ground'] = False
        edl_system['parachute']['diameter'] = D

        t, Y, edl_system = sf.simulate_edl(edl_system, mars, mission_events, 2000, False)

        t_end = t[-1]
        rover_v_ground = Y[0, -1] + Y[5, -1]

        success = int(
            edl_system['rover']['on_ground']
            and abs(rover_v_ground) <= abs(edl_system['sky_crane']['danger_speed'])
            and Y[1, -1] >= edl_system['sky_crane']['danger_altitude']
        )

        sim_time.append(t_end)
        rover_speed_term.append(rover_v_ground)
        landing_success.append(success)

    return (
        diameters,
        np.array(sim_time),
        np.array(rover_speed_term),
        np.array(landing_success)
    )


# -------------------------------------------------
# Task 6 Mach-dependent drag study
# -------------------------------------------------
def F_drag_descent_mach(edl_system, planet, altitude, velocity):
    """
    Revised drag model:
    Cd_mod = MEF(M) * Cd
    applies to parachute drag only.
    """

    density, _, _ = sf.get_local_atm_properties(planet, altitude)
    rhov2 = 0.5 * density * velocity**2

    # body drag
    if not edl_system['heat_shield']['ejected']:
        ACd_body = np.pi * (edl_system['heat_shield']['diameter'] / 2.0)**2 * edl_system['heat_shield']['Cd']
    else:
        ACd_body = edl_system['sky_crane']['area'] * edl_system['sky_crane']['Cd']

    # parachute drag
    if edl_system['parachute']['deployed'] and not edl_system['parachute']['ejected']:
        M = sf.v2M_Mars(velocity, altitude)
        MEF = mach_efficiency_factor(M)
        Cd_mod = MEF * edl_system['parachute']['Cd']
        ACd_parachute = np.pi * (edl_system['parachute']['diameter'] / 2.0)**2 * Cd_mod
    else:
        ACd_parachute = 0.0

    F = rhov2 * (ACd_body + ACd_parachute)
    return F


def run_study_mach_model():
    diameters = np.arange(14.0, 19.0 + 0.001, 0.5)

    sim_time = []
    rover_speed_term = []
    landing_success = []

    # temporarily replace the drag model inside subfunctions_EDL
    original_drag_function = sf.F_drag_descent
    sf.F_drag_descent = F_drag_descent_mach

    try:
        for D in diameters:
            edl_system = define_edl_system_1()
            mars = define_planet()
            mission_events = define_mission_events()

            # required initial conditions
            edl_system['altitude'] = 11000
            edl_system['velocity'] = -590
            edl_system['rocket']['on'] = False
            edl_system['parachute']['deployed'] = True
            edl_system['parachute']['ejected'] = False
            edl_system['heat_shield']['ejected'] = False
            edl_system['sky_crane']['on'] = False
            edl_system['speed_control']['on'] = False
            edl_system['position_control']['on'] = False
            edl_system['rover']['on_ground'] = False
            edl_system['parachute']['diameter'] = D

            t, Y, edl_system = sf.simulate_edl(edl_system, mars, mission_events, 2000, False)

            t_end = t[-1]
            rover_v_ground = Y[0, -1] + Y[5, -1]

            success = int(
                edl_system['rover']['on_ground']
                and abs(rover_v_ground) <= abs(edl_system['sky_crane']['danger_speed'])
                and Y[1, -1] >= edl_system['sky_crane']['danger_altitude']
            )

            sim_time.append(t_end)
            rover_speed_term.append(rover_v_ground)
            landing_success.append(success)

    finally:
        # restore original function so old behavior remains intact
        sf.F_drag_descent = original_drag_function

    return (
        diameters,
        np.array(sim_time),
        np.array(rover_speed_term),
        np.array(landing_success)
    )


# -------------------------------------------------
# Main
# -------------------------------------------------
def main():
    # Run both studies
    d_const, t_const, v_const, s_const = run_study_constant_cd()
    d_mach, t_mach, v_mach, s_mach = run_study_mach_model()

    # -----------------------------
    # Plot 1: MEF vs Mach
    # -----------------------------
    M_data = np.array([
        0.25, 0.5, 0.65, 0.7, 0.8, 0.9, 0.95, 1.0,
        1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.8, 1.9,
        2.0, 2.2, 2.5, 2.6
    ])

    MEF_data = np.array([
        1.0, 1.0, 1.0, 0.97, 0.91, 0.72, 0.66, 0.75,
        0.90, 0.96, 0.990, 0.999, 0.992, 0.98, 0.91, 0.85,
        0.82, 0.75, 0.64, 0.62
    ])

    M_plot = np.linspace(0.25, 2.6, 400)
    MEF_plot = mach_efficiency_factor(M_plot)

    plt.figure(figsize=(7, 5))
    plt.plot(M_data, MEF_data, 'o', label='Given data')
    plt.plot(M_plot, MEF_plot, label='PCHIP model')
    plt.xlabel('Mach number, M')
    plt.ylabel('Mach efficiency factor, MEF')
    plt.title('Task 6: MEF Model for Parachute Drag')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()

    # -----------------------------
    # Plot 2: Task 5 vs Task 6 comparison
    # -----------------------------
    fig, axs = plt.subplots(3, 1, figsize=(8, 11), sharex=True)

    axs[0].plot(d_const, t_const, marker='o', label='Task 5: Constant Cd')
    axs[0].plot(d_mach, t_mach, marker='o', label='Task 6: Mach-dependent Cd')
    axs[0].set_ylabel('Termination time (s)')
    axs[0].set_title('Task 5 vs Task 6 Parachute Diameter Comparison')
    axs[0].grid(True)
    axs[0].legend()

    axs[1].plot(d_const, v_const, marker='o', label='Task 5: Constant Cd')
    axs[1].plot(d_mach, v_mach, marker='o', label='Task 6: Mach-dependent Cd')
    axs[1].set_ylabel('Rover speed at termination (m/s)')
    axs[1].grid(True)

    axs[2].plot(d_const, s_const, marker='o', label='Task 5: Constant Cd')
    axs[2].plot(d_mach, s_mach, marker='o', label='Task 6: Mach-dependent Cd')
    axs[2].set_xlabel('Parachute diameter (m)')
    axs[2].set_ylabel('Landing success (1 = success, 0 = failure)')
    axs[2].set_yticks([0, 1])
    axs[2].grid(True)

    plt.tight_layout()
    plt.show()

    # -----------------------------
    # Print comparison table
    # -----------------------------
    print('Diameter   Time Const   Speed Const   Success Const   Time Mach   Speed Mach   Success Mach')
    print('-------------------------------------------------------------------------------------------')
    for i in range(len(d_const)):
        print(f'{d_const[i]:6.1f}   {t_const[i]:10.4f}   {v_const[i]:11.4f}   {s_const[i]:13d}   '
              f'{t_mach[i]:9.4f}   {v_mach[i]:10.4f}   {s_mach[i]:12d}')

    # -----------------------------
    # Recommendation
    # -----------------------------
    success_idx = np.where(s_mach == 1)[0]

    if len(success_idx) > 0:
        best_idx = success_idx[np.argmin(t_mach[success_idx])]
        print('\nRecommended parachute diameter with revised drag model:')
        print(f'{d_mach[best_idx]:.1f} m')
    else:
        print('\nNo successful landing occurred with the revised drag model.')


if __name__ == "__main__":
    main()