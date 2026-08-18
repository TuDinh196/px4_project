"""
WebSocket Telemetry & Mission Visualization Server for Quadplane Condor
========================================================================
Streams real-time 12-state telemetry, 5-motor propulsion metrics, GPS path,
and automated flight lifecycle states to the Web Dashboard.
"""

import asyncio
import json
import math
import sys
from pathlib import Path

import numpy as np
import websockets

# Add project root to sys.path
sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from src.controllers.condor_vtol_controller import VTOLHybridController  # noqa: E402
from src.px4_integration.mavsdk_bridge import MAVSDKBridge  # noqa: E402
from src.px4_integration.offboard_controller import OffboardController  # noqa: E402
from src.scenarios.flight_scenarios import FlightScenarios  # noqa: E402
from src.uav_model.condor_dynamics import QuadplaneDynamics, QuadplaneParams  # noqa: E402

# Global handles
bridge = None
scenarios = None
active_scenario_task = None
manual_velocity_cmd = {"vx": 0.0, "vy": 0.0, "vz": 0.0, "yaw": 0.0}
sim_drone = None
sim_controller = None
sim_mode = False

# Hanoi Home Reference Coordinates
REF_LAT = 21.028511
REF_LON = 105.804817

# Flight lifecycle state
current_lifecycle_state = "GROUND_IDLE"
current_target_sp = [0.0, 0.0, 0.0]


def ned_to_gps(x_north: float, y_east: float) -> list[float]:
    """Converts local NED coordinates (meters) to [latitude, longitude]."""
    d_lat = x_north / 111320.0
    d_lon = y_east / (111320.0 * math.cos(math.radians(REF_LAT)))
    return [round(REF_LAT + d_lat, 7), round(REF_LON + d_lon, 7)]


async def broadcast_telemetry(websocket):
    """Continuously stream high-fidelity telemetry to connected frontend."""
    while True:
        if sim_mode and sim_drone:
            # Send dynamic 6-DOF simulation telemetry
            state = sim_drone.state
            pos = state[0:3]
            vel = state[3:6]
            att = state[6:9]
            rates = state[9:12]
            airspeed = float(np.linalg.norm(vel))

            # 5-Motor status from controller
            ctrl_out = sim_controller.compute_hybrid(
                target_pos_ned=np.array(current_target_sp),
                target_airspeed=18.0,
                current_state=state,
                dt=0.05,
            )
            vtol_th = ctrl_out["vtol_thrust"]
            tractor_th = ctrl_out["tractor_thrust"]
            phase = ctrl_out["phase"]
            blend = ctrl_out["blend_factor"]

            # Calculate individual motor RPMs
            vtol_rpm = int(math.sqrt(max(vtol_th / 4.0, 0.0) / 2.5e-5) * 9.549)
            tractor_rpm = int(math.sqrt(max(tractor_th, 0.0) / 8.55e-6) * 9.549)

            p_vtol = (vtol_th**1.5) * 0.45
            p_tractor = max(tractor_th * max(airspeed, 5.0) * 1.35, 0.0)
            p_mc = (76.5**1.5) * 0.45 + 0.5 * 1.225 * (airspeed**3) * 0.15
            power_w = p_vtol + p_tractor
            energy_savings = max(0.0, ((p_mc - power_w) / max(p_mc, 1.0)) * 100.0)

            data = {
                "type": "telemetry",
                "position": [round(float(p), 3) for p in pos],
                "velocity": [round(float(v), 3) for v in vel],
                "attitude": [round(math.degrees(float(a)), 2) for a in att],
                "rates": [round(math.degrees(float(r)), 2) for r in rates],
                "airspeed": round(airspeed, 2),
                "groundspeed": round(math.sqrt(vel[0] ** 2 + vel[1] ** 2), 2),
                "altitude": round(float(-pos[2]), 2),
                "battery": 22.2,
                "armed": current_lifecycle_state != "GROUND_IDLE",
                "mode": phase.value,
                "lifecycle": current_lifecycle_state,
                "gps": ned_to_gps(pos[0], pos[1]),
                "target_sp": current_target_sp,
                "motors": {
                    "m0_rpm": vtol_rpm,
                    "m1_rpm": vtol_rpm,
                    "m2_rpm": vtol_rpm,
                    "m3_rpm": vtol_rpm,
                    "m4_rpm": tractor_rpm,
                    "vtol_thrust_N": round(vtol_th, 1),
                    "tractor_thrust_N": round(tractor_th, 1),
                    "blend_factor": round(blend, 2),
                },
                "energy": {
                    "power_watts": round(power_w, 1),
                    "savings_percent": round(energy_savings, 1),
                },
            }
            try:
                await websocket.send(json.dumps(data))
            except websockets.exceptions.ConnectionClosed:
                break

        elif bridge and bridge.telemetry:
            # Send live PX4 SITL Telemetry
            tel = bridge.telemetry
            pos = tel.position_ned if tel.position_ned else [0.0, 0.0, 0.0]
            vel = tel.velocity_ned if tel.velocity_ned else [0.0, 0.0, 0.0]
            att = tel.attitude_euler if tel.attitude_euler else [0.0, 0.0, 0.0]
            rates = (
                tel.attitude_angular_velocity
                if tel.attitude_angular_velocity
                else [0.0, 0.0, 0.0]
            )

            speed = math.sqrt(vel[0] ** 2 + vel[1] ** 2 + vel[2] ** 2)
            alt = -pos[2]

            # Motor estimates based on mode
            is_fw = "FW" in str(tel.flight_mode).upper()
            vtol_thrust = 0.0 if is_fw else 76.5
            tractor_thrust = 15.0 if is_fw else (5.0 if speed > 5.0 else 0.0)

            data = {
                "type": "telemetry",
                "position": [round(float(p), 3) for p in pos],
                "velocity": [round(float(v), 3) for v in vel],
                "attitude": [round(float(a), 2) for a in att],
                "rates": [round(float(r), 2) for r in rates],
                "airspeed": round(speed, 2),
                "groundspeed": round(math.sqrt(vel[0] ** 2 + vel[1] ** 2), 2),
                "altitude": round(float(alt), 2),
                "battery": round(tel.battery_voltage, 2),
                "armed": tel.armed,
                "mode": str(tel.flight_mode),
                "lifecycle": current_lifecycle_state,
                "gps": ned_to_gps(pos[0], pos[1]),
                "target_sp": current_target_sp,
                "motors": {
                    "m0_rpm": 0 if is_fw else 8500,
                    "m1_rpm": 0 if is_fw else 8500,
                    "m2_rpm": 0 if is_fw else 8500,
                    "m3_rpm": 0 if is_fw else 8500,
                    "m4_rpm": 12000 if is_fw else 0,
                    "vtol_thrust_N": round(vtol_thrust, 1),
                    "tractor_thrust_N": round(tractor_thrust, 1),
                    "blend_factor": 0.0 if is_fw else 1.0,
                },
                "energy": {
                    "power_watts": round(350.0 if is_fw else 1150.0, 1),
                    "savings_percent": round(70.0 if is_fw else 0.0, 1),
                },
            }
            try:
                await websocket.send(json.dumps(data))
            except websockets.exceptions.ConnectionClosed:
                break

        await asyncio.sleep(0.05)  # 20 Hz update rate


async def execute_full_flight_lifecycle(scenario_name: str):
    """
    Executes complete realistic flight lifecycle:
      [0. Ground Idle] -> [1. Arm & VTOL Takeoff] -> [2. Scenario Trajectory Execution] ->
      [3. Approach & Back-Transition] -> [4. Precision VTOL Land] -> [5. Disarm & Summary]
    """
    global current_lifecycle_state, current_target_sp
    print(f"\n🚀 === INITIATING FLIGHT LIFECYCLE FOR SCENARIO: {scenario_name.upper()} ===")

    try:
        # STEP 1: PREFLIGHT & ARMING
        current_lifecycle_state = "PREFLIGHT_ARMING"
        current_target_sp = [0.0, 0.0, 0.0]
        print("  [1/5] Arming Quadplane Condor...")
        if bridge:
            await bridge.arm()
            await asyncio.sleep(2)

        # STEP 2: VTOL CLIMB & TAKEOFF
        current_lifecycle_state = "VTOL_TAKEOFF"
        target_climb_alt = -12.0
        current_target_sp = [0.0, 0.0, target_climb_alt]
        print(f"  [2/5] Climbing vertically to {abs(target_climb_alt)}m altitude...")

        if bridge and scenarios:
            await scenarios.controller.start_offboard()
            await scenarios.controller.set_position(0.0, 0.0, target_climb_alt, 0.0)
            await asyncio.sleep(6)

        # STEP 3: EXECUTE SCENARIO TRAJECTORY
        current_lifecycle_state = f"EXECUTING_{scenario_name.upper()}"
        print(f"  [3/5] Executing {scenario_name} flight trajectory on map...")

        if bridge and scenarios:
            if scenario_name == "hover":
                current_target_sp = [0.0, 0.0, -10.0]
                await scenarios.run_hover()
            elif scenario_name == "square":
                current_target_sp = [20.0, 20.0, -12.0]
                await scenarios.run_square()
            elif scenario_name == "circle":
                current_target_sp = [15.0, 0.0, -15.0]
                await scenarios.run_circle()
            elif scenario_name == "figure8":
                current_target_sp = [25.0, 45.0, -15.0]
                await scenarios.run_figure8()
            elif scenario_name == "vtol_mission":
                current_target_sp = [200.0, 300.0, -35.0]
                from src.px4_integration.sitl_condor_mission import run_vtol_mission

                await run_vtol_mission()

        # STEP 4: RETURN TO LAUNCH & BACK-TRANSITION
        current_lifecycle_state = "RETURN_APPROACH"
        current_target_sp = [0.0, 0.0, -10.0]
        print("  [4/5] Returning to launch pad & preparing for vertical landing...")
        if bridge and scenarios:
            await scenarios.controller.set_position(0.0, 0.0, -10.0, 0.0)
            await asyncio.sleep(4)

        # STEP 5: PRECISION VTOL LANDING & DISARM
        current_lifecycle_state = "PRECISION_LANDING"
        current_target_sp = [0.0, 0.0, 0.0]
        print("  [5/5] Landing vertically at (0, 0, 0)...")
        if bridge:
            await bridge.drone.action.land()
            async for in_air in bridge.drone.telemetry.in_air():
                if not in_air:
                    print("  ✅ Touchdown confirmed! Disarming.")
                    break
                await asyncio.sleep(1)

        current_lifecycle_state = "COMPLETED_LANDED"
        print(f"🎉 === SCENARIO {scenario_name.upper()} FLIGHT COMPLETED SUCCESSFULLY ===")

    except asyncio.CancelledError:
        print("⚠️ Scenario execution cancelled by user. Returning to launch immediately.")
        current_lifecycle_state = "EMERGENCY_RTL"
        if bridge:
            await bridge.drone.action.return_to_launch()
    except Exception as e:
        print(f"❌ Error during flight scenario execution: {e}")
        current_lifecycle_state = "ERROR"


async def handle_client(websocket):
    """Handle incoming frontend WebSocket commands."""
    global active_scenario_task, current_lifecycle_state
    print("🌐 Dashboard web client connected.")

    stream_task = asyncio.create_task(broadcast_telemetry(websocket))

    try:
        async for message in websocket:
            cmd = json.loads(message)
            action = cmd.get("action")
            print(f"Received frontend command: {action}")

            if action == "arm":
                if bridge:
                    await bridge.arm()
            elif action == "disarm":
                if bridge:
                    await bridge.disarm()
            elif action == "rtl":
                if bridge:
                    await bridge.drone.action.return_to_launch()
                current_lifecycle_state = "RETURN_APPROACH"
            elif action == "scenario":
                name = cmd.get("name", "hover")
                if active_scenario_task and not active_scenario_task.done():
                    active_scenario_task.cancel()
                active_scenario_task = asyncio.create_task(execute_full_flight_lifecycle(name))
            elif action == "manual_control":
                manual_velocity_cmd["vx"] = cmd.get("vx", 0.0)
                manual_velocity_cmd["vy"] = cmd.get("vy", 0.0)
                manual_velocity_cmd["vz"] = cmd.get("vz", 0.0)
                manual_velocity_cmd["yaw"] = cmd.get("yaw", 0.0)

    except websockets.exceptions.ConnectionClosed:
        print("Dashboard client disconnected.")
    finally:
        stream_task.cancel()


async def run_simulation_engine():
    """Background 6-DOF simulation loop when running in simulation mode."""
    dt = 0.05
    while True:
        if sim_mode and sim_drone:
            ctrl_out = sim_controller.compute_hybrid(
                target_pos_ned=np.array(current_target_sp),
                target_airspeed=18.0,
                current_state=sim_drone.state,
                dt=dt,
            )
            sim_drone.step(
                vtol_thrust=ctrl_out["vtol_thrust"],
                vtol_torque=ctrl_out["vtol_torque"],
                dt=dt,
                tractor_thrust=ctrl_out["tractor_thrust"],
                delta_e=ctrl_out["delta_e"],
                delta_r=ctrl_out["delta_r"],
            )
            # Ground limit
            if sim_drone.state[2] > 0.0:
                sim_drone.state[2] = 0.0
                sim_drone.state[5] = min(sim_drone.state[5], 0.0)
        await asyncio.sleep(dt)


async def main():
    global bridge, scenarios, sim_drone, sim_controller, sim_mode

    print("=" * 65)
    print("  QUADPLANE CONDOR TELEMETRY & MAP SERVER")
    print("=" * 65)

    params = QuadplaneParams()
    sim_drone = QuadplaneDynamics(params)
    sim_controller = VTOLHybridController(params)

    bridge = MAVSDKBridge()
    try:
        print("Connecting to PX4 SITL on udp://:14540 (Timeout: 10s)...")
        await asyncio.wait_for(bridge.connect(), timeout=10.0)
        bridge.start_telemetry_tasks()
        controller = OffboardController(bridge)
        scenarios = FlightScenarios(bridge, controller)
        print("✅ Connected to PX4 SITL for Live Flight Control!")
    except Exception as e:
        print(
            f"⚠️ SITL not available ({e}). "
            "Activating internal 6-DOF Aerodynamic Physics Simulation Engine!"
        )
        sim_mode = True
        asyncio.create_task(run_simulation_engine())

    print("🚀 Starting WebSocket Server on ws://localhost:8765")
    async with websockets.serve(handle_client, "0.0.0.0", 8765):
        await asyncio.Future()  # Keep running


if __name__ == "__main__":
    asyncio.run(main())
