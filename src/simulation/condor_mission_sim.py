"""
Quadplane Condor Hybrid VTOL Full Mission Simulation & Energy Analysis
======================================================================
Simulates complete 5-stage flight profile:
  1. VTOL Climb to 15m (Hover power)
  2. Forward Transition to 18 m/s (Tractor acceleration + Wing lift blending)
  3. Fixed-Wing Waypoint Cruise across 400m survey legs (High efficiency, VTOL motors OFF)
  4. Back-Transition (Airspeed deceleration + Quad rotor spin-up)
  5. Precision VTOL Descent & Landing

Generates comprehensive flight performance plots and energy comparisons:
  - 3D Flight Path
  - True Airspeed & Altitude vs Flight Phases
  - Motor Thrusts (VTOL 4-Rotors vs Tractor Nose Motor)
  - Power & Energy Consumption vs Pure Multicopter
"""

from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

from src.controllers.condor_vtol_controller import VTOLHybridController
from src.uav_model.condor_dynamics import QuadplaneDynamics, QuadplaneParams


def run_vtol_mission_simulation():
    print("=" * 65)
    print(" Running Quadplane Condor Full Hybrid VTOL Mission Simulation")
    print("=" * 65)

    params = QuadplaneParams()
    drone = QuadplaneDynamics(params)
    controller = VTOLHybridController(params, target_cruise_speed=18.0, transition_altitude=15.0)

    # Mission Waypoints in NED frame [North, East, Down]
    waypoints = [
        np.array([0.0, 0.0, -15.0]),      # WP0: VTOL Takeoff & Hover at 15m
        np.array([200.0, 0.0, -35.0]),    # WP1: Forward Transition & Climb to 35m
        np.array([400.0, 200.0, -40.0]),  # WP2: Fixed-Wing High-speed Cruise leg 1
        np.array([200.0, 400.0, -40.0]),  # WP3: Fixed-Wing High-speed Cruise leg 2
        np.array([0.0, 200.0, -25.0]),    # WP4: Approach & Back-Transition initiation
        np.array([0.0, 0.0, 0.0]),        # WP5: Precision VTOL Land
    ]

    dt = 0.01  # 100 Hz simulation
    sim_time = 65.0  # seconds
    current_wp_idx = 0

    # Data loggers
    times = []
    positions = []
    velocities = []
    airspeeds = []
    phases = []
    vtol_thrusts = []
    tractor_thrusts = []
    powers_hybrid = []
    powers_mc_only = []

    # Reset
    drone.reset()
    state = drone.state.copy()

    for step in range(int(sim_time / dt)):
        t = step * dt
        pos = state[0:3]
        vel = state[3:6]
        airspeed = float(np.linalg.norm(vel))

        target_wp = waypoints[current_wp_idx]

        # Waypoint acceptance radius
        dist_to_wp = float(np.linalg.norm(target_wp - pos))
        if dist_to_wp < 15.0 and current_wp_idx < len(waypoints) - 1:
            current_wp_idx += 1
            target_wp = waypoints[current_wp_idx]

        # Controller compute
        control = controller.compute_hybrid(
            current_state=state,
            target_pos_ned=target_wp,
            target_airspeed=18.0,
            dt=dt,
        )

        # Power models
        # 1. VTOL 4-rotor power: P_hover = (T^1.5) / sqrt(2 * rho * A_rotors)
        # For Quadplane Condor: ~280 W at 23N hover
        p_vtol = (control["vtol_thrust"] / (params.mass * 9.81)) * 280.0
        # 2. Nose tractor motor power: P_fw = T_tractor * V_airspeed / eta_prop (eta ~ 0.7)
        p_tractor = (control["tractor_thrust"] * max(airspeed, 5.0)) / 0.70
        p_avionics = 25.0  # W
        total_p_hybrid = p_vtol + p_tractor + p_avionics

        # Baseline: Pure Multicopter flying at same speed/altitude
        p_mc_baseline = 280.0 + (0.5 * params.rho * (airspeed**3) * 0.15) + p_avionics

        # Physics step
        state = drone.step(
            dt=dt,
            vtol_thrust=control["vtol_thrust"],
            vtol_torque=control["vtol_torque"],
            tractor_thrust=control["tractor_thrust"],
            delta_e=control["delta_e"],
            delta_r=control["delta_r"],
        )

        # Log data
        times.append(t)
        positions.append(pos.copy())
        velocities.append(vel.copy())
        airspeeds.append(airspeed)
        phases.append(control["phase"].name)
        vtol_thrusts.append(control["vtol_thrust"])
        tractor_thrusts.append(control["tractor_thrust"])
        powers_hybrid.append(total_p_hybrid)
        powers_mc_only.append(p_mc_baseline)

    # Convert to arrays
    times = np.array(times)
    positions = np.array(positions)
    airspeeds = np.array(airspeeds)
    vtol_thrusts = np.array(vtol_thrusts)
    tractor_thrusts = np.array(tractor_thrusts)
    powers_hybrid = np.array(powers_hybrid)
    powers_mc_only = np.array(powers_mc_only)

    energy_hybrid_kJ = np.trapezoid(powers_hybrid, times) / 1000.0
    energy_mc_kJ = np.trapezoid(powers_mc_only, times) / 1000.0
    energy_savings = ((energy_mc_kJ - energy_hybrid_kJ) / energy_mc_kJ) * 100.0

    tot_dist = float(np.sum(np.linalg.norm(np.diff(positions[:, 0:2], axis=0), axis=1)))
    max_spd = float(np.max(airspeeds))

    print("\n--- Mission Energy Performance Analysis ---")
    print(f" Total Flight Distance    : {tot_dist:.1f} m")
    print(f" Max Airspeed Reached     : {max_spd:.1f} m/s ({max_spd * 3.6:.1f} km/h)")
    print(f" Hybrid Quadplane Energy  : {energy_hybrid_kJ:.1f} kJ")
    print(f" Pure Multicopter Energy  : {energy_mc_kJ:.1f} kJ")
    print(f" Energy Efficiency Gain   : {energy_savings:.1f}% Energy Saved in Fixed-Wing Cruise!")

    # Plotting
    out_dir = Path(__file__).resolve().parents[2] / "output"
    out_dir.mkdir(exist_ok=True)

    fig = plt.figure(figsize=(16, 12))

    # 1. 3D Trajectory
    ax1 = fig.add_subplot(2, 2, 1, projection="3d")
    ax1.plot(
        positions[:, 1], positions[:, 0], -positions[:, 2],
        "b-", linewidth=2, label="Quadplane Path"
    )
    wp_arr = np.array(waypoints)
    ax1.scatter(
        wp_arr[:, 1], wp_arr[:, 0], -wp_arr[:, 2],
        color="red", s=50, label="Mission Waypoints"
    )
    ax1.set_xlabel("East (m)")
    ax1.set_ylabel("North (m)")
    ax1.set_zlabel("Altitude (m)")
    ax1.set_title("3D Hybrid VTOL Mission Profile")
    ax1.legend()
    ax1.view_init(elev=25, azim=-45)

    # 2. Airspeed & Altitude vs Flight Phases
    ax2 = fig.add_subplot(2, 2, 2)
    color = "tab:blue"
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("True Airspeed (m/s)", color=color)
    ax2.plot(times, airspeeds, color=color, linewidth=2, label="Airspeed (m/s)")
    ax2.axhline(
        y=15.0, color="orange", linestyle="--", alpha=0.7, label="Transition Speed (15 m/s)"
    )
    ax2.tick_params(axis="y", labelcolor=color)
    ax2.grid(True)

    ax2_alt = ax2.twinx()
    color = "tab:green"
    ax2_alt.set_ylabel("Altitude (m)", color=color)
    ax2_alt.plot(
        times, -positions[:, 2], color=color, linewidth=2, linestyle="-.", label="Altitude (m)"
    )
    ax2_alt.tick_params(axis="y", labelcolor=color)
    ax2.set_title("Airspeed & Altitude Transition Profile")

    # 3. Motor Thrust Blending (VTOL vs Forward Tractor)
    ax3 = fig.add_subplot(2, 2, 3)
    ax3.plot(times, vtol_thrusts, "purple", linewidth=2, label="VTOL 4-Rotor Lift (N)")
    ax3.plot(times, tractor_thrusts, "red", linewidth=2, label="Nose Tractor Thrust (N)")
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Thrust Force (N)")
    ax3.set_title("Propulsion Blending: VTOL Lift vs Tractor Thrust")
    ax3.grid(True)
    ax3.legend()

    # 4. Power & Energy Consumption Comparison
    ax4 = fig.add_subplot(2, 2, 4)
    mc_lbl = f"Pure Multicopter ({energy_mc_kJ:.0f} kJ)"
    hyb_lbl = f"Hybrid Quadplane ({energy_hybrid_kJ:.0f} kJ)"
    sav_lbl = f"{energy_savings:.1f}% Energy Saved"
    ax4.plot(times, powers_mc_only, "gray", linestyle="--", label=mc_lbl)
    ax4.plot(times, powers_hybrid, "green", linewidth=2, label=hyb_lbl)
    ax4.fill_between(times, powers_hybrid, powers_mc_only, color="green", alpha=0.15, label=sav_lbl)
    ax4.set_xlabel("Time (s)")
    ax4.set_ylabel("Power Consumption (W)")
    ax4.set_title("Power & Energy Savings in Fixed-Wing Cruise")
    ax4.grid(True)
    ax4.legend()

    plt.tight_layout()
    plot_path = out_dir / "vtol_mission_performance.png"
    plt.savefig(plot_path, dpi=150)
    print(f"\n Saved mission performance plot: {plot_path}")


if __name__ == "__main__":
    run_vtol_mission_simulation()
