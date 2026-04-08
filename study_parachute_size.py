import numpy as np
np.NaN = np.nan

import matplotlib.pyplot as plt
from scipy.interpolate import PchipInterpolator as pchip
from scipy.integrate import solve_ivp


# =========================
# Dictionary setup
# =========================

def define_rover_4():
    wheel = {'radius': 0.20, 'mass': 2}
    speed_reducer = {'type': 'reverted',
                     'diam_pinion': 0.04,
                     'diam_gear': 0.06,
                     'mass': 1.5}
    motor = {'torque_stall': 165,
             'torque_noload': 0,
             'speed_noload': 3.85,
             'mass': 5.0}

    motor['effcy_tau'] = np.array([0, 10, 20, 40, 75, 170])
    motor['effcy'] = np.array([0, 0.60, 0.75, 0.73, 0.55, 0.05])

    chassis = {'mass': 674}
    science_payload = {'mass': 80}
    power_subsys = {'mass': 100}

    wheel_assembly = {'wheel': wheel,
                      'speed_reducer': speed_reducer,
                      'motor': motor}

    rover = {'wheel_assembly': wheel_assembly,
             'chassis': chassis,
             'science_payload': science_payload,
             'power_subsys': power_subsys,
             'on_ground': False}

    return rover


def define_edl_system_1():
    parachute = {'deployed': True,
                 'ejected': False,
                 'diameter': 16.25,
                 'Cd': 0.615,
                 'mass': 185.0}

    rocket = {'on': False,
              'structure_mass': 8.0,
              'initial_fuel_mass': 230.0,
              'fuel_mass': 230.0,
              'effective_exhaust_velocity': 4500.0,
              'max_thrust': 3100.0,
              'min_thrust': 40.0}

    speed_control = {'on': False,
                     'Kp': 2000,
                     'Kd': 20,
                     'Ki': 50,
                     'target_velocity': -3.0}

    position_control = {'on': False,
                        'Kp': 2000,
                        'Kd': 1000,
                        'Ki': 50,
                        'target_altitude': 7.6}

    sky_crane = {'on': False,
                 'danger_altitude': 4.5,
                 'danger_speed': -1.0,
                 'mass': 35.0,
                 'area': 16.0,
                 'Cd': 0.9,
                 'max_cable': 7.6,
                 'velocity': -0.1}

    heat_shield = {'ejected': False,
                   'mass': 225.0,
                   'diameter': 4.5,
                   'Cd': 0.35}

    rover = define_rover_4()

    edl_system = {'altitude': np.NaN,
                  'velocity': np.NaN,
                  'num_rockets': 8,
                  'volume': 150,
                  'parachute': parachute,
                  'heat_shield': heat_shield,
                  'rocket': rocket,
                  'speed_control': speed_control,
                  'position_control': position_control,
                  'sky_crane': sky_crane,
                  'rover': rover}

    return edl_system


def define_planet():
    high_altitude = {
        'temperature': lambda altitude: -23.4 - 0.00222 * altitude,
        'pressure': lambda altitude: 0.699 * np.exp(-0.00009 * altitude)
    }

    low_altitude = {
        'temperature': lambda altitude: -31 - 0.000998 * altitude,
        'pressure': lambda altitude: 0.699 * np.exp(-0.00009 * altitude)
    }

    density = lambda temperature, pressure: pressure / (0.1921 * (temperature + 273.15))

    mars = {'g': -3.72,
            'altitude_threshold': 7000,
            'low_altitude': low_altitude,
            'high_altitude': high_altitude,
            'density': density}

    return mars


def define_mission_events():
    mission_events = {'alt_heatshield_eject': 8000,
                      'alt_parachute_eject': 900,
                      'alt_rockets_on': 1800,
                      'alt_skycrane_on': 7.6}
    return mission_events


# =========================
# Physics helper functions
# =========================

def get_mass_rover(edl_system):
    m = 6 * (
        edl_system['rover']['wheel_assembly']['motor']['mass'] +
        edl_system['rover']['wheel_assembly']['speed_reducer']['mass'] +
        edl_system['rover']['wheel_assembly']['wheel']['mass']
    ) + edl_system['rover']['chassis']['mass'] + \
        edl_system['rover']['science_payload']['mass'] + \
        edl_system['rover']['power_subsys']['mass']
    return m


def get_mass_rockets(edl_system):
    return edl_system['num_rockets'] * (
        edl_system['rocket']['structure_mass'] + edl_system['rocket']['fuel_mass']
    )


def get_mass_edl(edl_system):
    m = int(not edl_system['parachute']['ejected']) * edl_system['parachute']['mass'] + \
        int(not edl_system['heat_shield']['ejected']) * edl_system['heat_shield']['mass'] + \
        get_mass_rockets(edl_system) + \
        edl_system['sky_crane']['mass'] + \
        get_mass_rover(edl_system)
    return m


def get_local_atm_properties(planet, altitude):
    if altitude > planet['altitude_threshold']:
        temperature = planet['high_altitude']['temperature'](altitude)
        pressure = planet['high_altitude']['pressure'](altitude)
    else:
        temperature = planet['low_altitude']['temperature'](altitude)
        pressure = planet['low_altitude']['pressure'](altitude)

    density = planet['density'](temperature, pressure)
    return density, temperature, pressure


def F_buoyancy_descent(edl_system, planet, altitude):
    density, _, _ = get_local_atm_properties(planet, altitude)
    F = np.sign(planet['g']) * planet['g'] * density * edl_system['volume']
    return F


def F_drag_descent(edl_system, planet, altitude, velocity):
    density, _, _ = get_local_atm_properties(planet, altitude)
    rhov2 = 0.5 * density * velocity**2

    if not edl_system['heat_shield']['ejected']:
        ACd_body = np.pi * (edl_system['heat_shield']['diameter'] / 2.0)**2 * edl_system['heat_shield']['Cd']
    else:
        ACd_body = edl_system['sky_crane']['area'] * edl_system['sky_crane']['Cd']

    if edl_system['parachute']['deployed'] and not edl_system['parachute']['ejected']:
        ACd_parachute = np.pi * (edl_system['parachute']['diameter'] / 2.0)**2 * edl_system['parachute']['Cd']
    else:
        ACd_parachute = 0.0

    F = rhov2 * (ACd_body + ACd_parachute)
    return F


def F_gravity_descent(edl_system, planet):
    return get_mass_edl(edl_system) * planet['g']


def v2M_Mars(v, a):
    SPD_data = np.array([
        [0, 244.4], [1000, 243.7], [2000, 243.2], [3000, 242.7], [4000, 242.2],
        [5000, 241.7], [6000, 241.2], [7000, 240.7], [8000, 239.6], [9000, 238.4],
        [10000, 237.3], [11000, 236.1], [12000, 235.0], [13000, 233.8], [14000, 232.6]
    ])
    v_sound_fit = pchip(SPD_data[:, 0], SPD_data[:, 1])
    v_sound = v_sound_fit(a)
    return abs(v) / v_sound

def mach_efficiency_factor(M):
    """
    Returns parachute Mach efficiency factor MEF(M) using shape-preserving
    interpolation of the given experimental data.
    """

    M_data = np.array([0.25, 0.5, 0.65, 0.7, 0.8, 0.9, 0.95, 1.0,
                       1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.8, 1.9,
                       2.0, 2.2, 2.5, 2.6])

    MEF_data = np.array([1.0, 1.0, 1.0, 0.97, 0.91, 0.72, 0.66, 0.75,
                         0.90, 0.96, 0.990, 0.999, 0.992, 0.98, 0.91, 0.85,
                         0.82, 0.75, 0.64, 0.62])

    mef_fit = pchip(M_data, MEF_data)

    # Clip Mach number to data range so we do not rely on uncontrolled extrapolation
    M = np.clip(M, M_data[0], M_data[-1])

    return mef_fit(M)

def F_drag_descent_Mach(edl_system, planet, altitude, velocity):
    """
    Drag model with Mach-dependent parachute drag coefficient.
    Uses Cd_mod = MEF(M) * Cd for the parachute only.
    """

    density, _, _ = get_local_atm_properties(planet, altitude)
    rhov2 = 0.5 * density * velocity**2

    # Body drag
    if not edl_system['heat_shield']['ejected']:
        ACd_body = np.pi * (edl_system['heat_shield']['diameter'] / 2.0)**2 * edl_system['heat_shield']['Cd']
    else:
        ACd_body = edl_system['sky_crane']['area'] * edl_system['sky_crane']['Cd']

    # Parachute drag
    if edl_system['parachute']['deployed'] and not edl_system['parachute']['ejected']:
        M = v2M_Mars(velocity, altitude)
        MEF = mach_efficiency_factor(M)
        Cd_mod = MEF * edl_system['parachute']['Cd']
        ACd_parachute = np.pi * (edl_system['parachute']['diameter'] / 2.0)**2 * Cd_mod
    else:
        ACd_parachute = 0.0

    F = rhov2 * (ACd_body + ACd_parachute)
    return F

# =========================
# Simulation functions
# =========================

def edl_events(edl_system, mission_events):
    event0 = lambda t, y: y[1] - mission_events['alt_heatshield_eject'] - int(edl_system["heat_shield"]["ejected"]) * 999999
    event0.terminal = True
    event0.direction = -1

    event1 = lambda t, y: y[1] - mission_events['alt_parachute_eject'] - int(edl_system["parachute"]["ejected"]) * 999999
    event1.terminal = True
    event1.direction = -1

    event2 = lambda t, y: y[1] - mission_events['alt_rockets_on'] - int(edl_system["rocket"]["on"]) * 999999
    event2.terminal = True
    event2.direction = -1

    event3 = lambda t, y: y[1] - mission_events['alt_skycrane_on'] - int(edl_system["sky_crane"]["on"]) * 999999
    event3.terminal = True
    event3.direction = -1

    event4 = lambda t, y: y[2]
    event4.terminal = True
    event4.direction = -1

    event5 = lambda t, y: y[1]
    event5.terminal = True
    event5.direction = -1

    event6 = lambda t, y: y[0] - 3 * edl_system['speed_control']['target_velocity'] + int(edl_system["speed_control"]["on"]) * 999999
    event6.terminal = True
    event6.direction = 1

    event7 = lambda t, y: y[1] - 1.2 * mission_events['alt_skycrane_on'] - int(edl_system["position_control"]["on"]) * 999999
    event7.terminal = True
    event7.direction = -1

    event8 = lambda t, y: y[1] + y[6]
    event8.terminal = True
    event8.direction = -1

    return [event0, event1, event2, event3, event4, event5, event6, event7, event8]


def edl_dynamics(t, y, edl_system, planet):
    vel_edl = y[0]
    altitude_edl = y[1]
    fuel_mass = y[2]
    ei_vel = y[3]
    ei_pos = y[4]
    vel_rov = y[5]
    pos_rov = y[6]

    edl_system['rocket']['fuel_mass'] = fuel_mass / edl_system['num_rockets']
    edl_mass = get_mass_edl(edl_system)

    if edl_system['parachute'].get('use_mach_model', False):
        F_drag = F_drag_descent_Mach(edl_system, planet, altitude_edl, vel_edl)
    else:
        F_drag = F_drag_descent(edl_system, planet, altitude_edl, vel_edl)

    F_ext = F_gravity_descent(edl_system, planet) + \
        F_buoyancy_descent(edl_system, planet, altitude_edl) + \
        F_drag

    if edl_system['rocket']['on'] and not edl_system['speed_control']['on'] and not edl_system['position_control']['on']:
        F_thrust = 0.9 * edl_system['rocket']['max_thrust'] * edl_system['num_rockets']
        dy1dt = (F_ext + F_thrust) / edl_mass
        dy2dt = vel_edl
        dmdt = -(F_thrust / edl_system['rocket']['effective_exhaust_velocity'])
        e_vel = 0
        e_pos = 0

    elif edl_system['rocket']['on'] and edl_system['speed_control']['on']:
        Kp = edl_system['speed_control']['Kp']
        Kd = edl_system['speed_control']['Kd']
        Ki = edl_system['speed_control']['Ki']

        e_vel = edl_system['speed_control']['target_velocity'] - vel_edl
        num = (Kp * e_vel + Kd * (F_ext / edl_mass) + Ki * ei_vel) - edl_mass * planet['g']
        den = (1 - Kd / edl_mass)
        F_thrust = num / den

        F_thrust = max(edl_system['num_rockets'] * edl_system['rocket']['min_thrust'], F_thrust)
        F_thrust = min(F_thrust, edl_system['num_rockets'] * edl_system['rocket']['max_thrust'])

        dy1dt = (F_ext + F_thrust) / edl_mass
        dy2dt = vel_edl
        dmdt = -(F_thrust / edl_system['rocket']['effective_exhaust_velocity'])
        e_pos = 0

    elif edl_system['rocket']['on'] and edl_system['position_control']['on']:
        Kp = edl_system['position_control']['Kp']
        Kd = edl_system['position_control']['Kd']
        Ki = edl_system['position_control']['Ki']

        e_pos = edl_system['position_control']['target_altitude'] - altitude_edl
        dedt_pos = -vel_edl

        F_thrust = edl_system['num_rockets'] * (Kp * e_pos + Kd * dedt_pos + Ki * ei_pos) - planet['g'] * edl_mass
        F_thrust = max(edl_system['rocket']['min_thrust'] * edl_system['num_rockets'], F_thrust)
        F_thrust = min(F_thrust, edl_system['num_rockets'] * edl_system['rocket']['max_thrust'])

        dy2dt = vel_edl
        dy1dt = (F_ext + F_thrust) / edl_mass
        dmdt = -(F_thrust / edl_system['rocket']['effective_exhaust_velocity'])
        e_vel = 0

    else:
        dy1dt = F_ext / edl_mass
        dy2dt = vel_edl
        dmdt = 0
        e_vel = 0
        e_pos = 0

    if edl_system['sky_crane']['on']:
        dy6dt = 0
        dy7dt = edl_system['sky_crane']['velocity']
    else:
        dy6dt = 0
        dy7dt = 0

    dydt = np.array([dy1dt, dy2dt, dmdt, e_vel, e_pos, dy6dt, dy7dt])
    return dydt


def update_edl_state(edl_system, TE, YE, Y, ITER_INFO):
    y0 = Y[:, -1]
    edl_system["rocket"]["fuel_mass"] = y0[2] / edl_system["num_rockets"]
    edl_system["altitude"] = y0[1]
    edl_system["velocity"] = y0[0]

    TERMINATE_SIM = False

    for i in range(9):
        if TE[i].size == 0:
            continue

        time = TE[i][0]
        altitude = YE[i][0, 1]
        speed = YE[i][0, 0]
        rover_rel_pos = YE[i][0, 6]
        rover_rel_vel = YE[i][0, 5]

        if i == 0:
            if not edl_system["heat_shield"]["ejected"]:
                edl_system["heat_shield"]["ejected"] = True
                y0 = Y[:, -1]

        elif i == 1:
            if not edl_system["parachute"]["ejected"]:
                edl_system["parachute"]["ejected"] = True
                y0 = Y[:, -1]

        elif i == 2:
            if not edl_system["rocket"]["on"]:
                edl_system["rocket"]["on"] = True
                y0 = Y[:, -1]

        elif i == 3:
            if not edl_system["sky_crane"]["on"] and edl_system["position_control"]["on"]:
                edl_system["sky_crane"]["on"] = True
            y0 = Y[:, -1]
            y0[5] = edl_system["sky_crane"]["velocity"]

        elif i == 4:
            if edl_system["rocket"]["on"]:
                edl_system["rocket"]["on"] = False
                y0 = []
                TERMINATE_SIM = True

        elif i == 5:
            y0 = []
            TERMINATE_SIM = True

        elif i == 6:
            if not edl_system["speed_control"]["on"] and not edl_system["position_control"]["on"]:
                edl_system["speed_control"]["on"] = True
            y0 = Y[:, -1]
            y0[3] = 0
            y0[4] = 0

        elif i == 7:
            if not edl_system["position_control"]["on"]:
                edl_system["speed_control"]["on"] = False
                edl_system["position_control"]["on"] = True
            y0 = Y[:, -1]
            y0[3] = 0
            y0[4] = 0

        elif i == 8:
            rover_touchdown_speed = speed + rover_rel_vel

            y0 = []
            TERMINATE_SIM = True
            edl_system["sky_crane"]["on"] = False
            edl_system["rover"]["on_ground"] = True

    return edl_system, y0, TERMINATE_SIM


def simulate_edl(edl_system, planet, mission_events, tmax, ITER_INFO):
    events = edl_events(edl_system, mission_events)
    tspan = (0, tmax)

    y0 = np.array([
        edl_system['velocity'],
        edl_system['altitude'],
        edl_system['rocket']['initial_fuel_mass'] * edl_system['num_rockets'],
        0,
        0,
        0,
        0
    ])

    T = np.array([])
    Y = np.array([[], [], [], [], [], [], []])
    TERMINATE_SIM = False

    while not TERMINATE_SIM:
        fun = lambda t, y: edl_dynamics(t, y, edl_system, planet)
        sol = solve_ivp(fun, tspan, y0, method='DOP853', events=events, max_step=0.1)

        t_part = sol.t
        Y_part = sol.y
        TE = sol.t_events
        YE = sol.y_events

        edl_system, y0, TERMINATE_SIM = update_edl_state(edl_system, TE, YE, Y_part, ITER_INFO)

        tspan = (t_part[-1], tmax)

        T = np.append(T, t_part)
        Y = np.hstack((Y, Y_part))

        if tspan[0] >= tspan[1]:
            TERMINATE_SIM = True

    return T, Y, edl_system


# =========================
# Task 5 study script
# =========================

def main():
    diameters = np.arange(14.0, 19.0 + 0.5, 0.5)

    sim_time = []
    rover_speed_term = []
    landing_success = []

    for D in diameters:
        edl_system = define_edl_system_1()
        mars = define_planet()
        mission_events = define_mission_events()

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

        t, Y, edl_system = simulate_edl(edl_system, mars, mission_events, 2000, False)

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

    sim_time = np.array(sim_time)
    rover_speed_term = np.array(rover_speed_term)
    landing_success = np.array(landing_success)

    print(' Diameter (m)   Time at Termination (s)   Rover Speed at Termination (m/s)   Success')
    print('-------------------------------------------------------------------------------------')
    for i in range(len(diameters)):
        print(f'    {diameters[i]:4.1f}               {sim_time[i]:10.4f}                    {rover_speed_term[i]:10.4f}          {landing_success[i]}')

    success_idx = np.where(landing_success == 1)[0]
    if len(success_idx) > 0:
        best_idx = success_idx[np.argmin(sim_time[success_idx])]
        print('\nRecommended parachute diameter:')
        print(f'{diameters[best_idx]:.1f} m')
    else:
        print('\nNo successful landing occurred in the tested diameter range.')

    fig, axs = plt.subplots(3, 1, figsize=(8, 10), sharex=True)

    axs[0].plot(diameters, sim_time, marker='o')
    axs[0].set_ylabel('Simulated time at termination (s)')
    axs[0].set_title('Parachute Diameter Study')
    axs[0].grid(True)

    axs[1].plot(diameters, rover_speed_term, marker='o')
    axs[1].set_ylabel('Rover speed at termination (m/s)')
    axs[1].grid(True)

    axs[2].plot(diameters, landing_success, marker='o')
    axs[2].set_xlabel('Parachute diameter (m)')
    axs[2].set_ylabel('Landing success (1=yes, 0=no)')
    axs[2].set_yticks([0, 1])
    axs[2].grid(True)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()