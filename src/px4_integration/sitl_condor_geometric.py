import asyncio
import sys
import time
from pathlib import Path

import numpy as np

# Add project root to path
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from src.controllers.geometric_controller import GeometricController  # noqa: E402
from src.px4_integration.mavsdk_bridge import MAVSDKBridge  # noqa: E402
from src.px4_integration.offboard_controller import OffboardController  # noqa: E402
from src.simulation.condor_closed_loop_sim import generate_giant_lemniscate_setpoints  # noqa: E402


async def run_geometric_lemniscate():
    bridge = MAVSDKBridge()
    await bridge.connect()

    bridge.start_telemetry_tasks()
    controller = OffboardController(bridge)

    # Wait for telemetry to settle
    await asyncio.sleep(2)

    # Initialize Geometric Controller with parameters tuned for Quadplane Condor (7.8kg)
    geom_ctrl = GeometricController(
        mass=7.8,
        gravity=9.81,
        Ixx=1.46,
        Iyy=1.06,
        Izz=2.50,
        kp=16.0,
        kv=7.0,
        kR=4.5,
        kw=0.6,
    )
    max_thrust_N = 7.8 * 9.81 * 2.0  # Total max hover thrust (~153 N)

    print("--- Arming")
    await bridge.arm()

    print("--- Setting initial attitude setpoint")
    await controller.set_attitude(0.0, 0.0, 0.0, 0.5)

    success = await controller.start_offboard()
    if not success:
        await bridge.stop_tasks()
        return

    print("--- Starting Geometric Lemniscate Trajectory")

    dt = 0.02  # 50 Hz
    start_time = time.time()
    initial_pos = None

    t_log = []
    pos_log = []
    sp_log = []
    v_log = []
    sp_v_log = []
    thrust_log = []
    att_log = []
    cmd_att_log = []

    try:
        while True:
            t = time.time() - start_time

            # Stop after 60 seconds for a quick aggressive metric evaluation
            if t > 60.0:
                break

            # Read telemetry from bridge
            state = np.zeros(12)
            if bridge.telemetry.position_ned and bridge.telemetry.velocity_ned:
                state[0] = bridge.telemetry.position_ned[0]
                state[1] = bridge.telemetry.position_ned[1]
                state[2] = bridge.telemetry.position_ned[2]
                state[3] = bridge.telemetry.velocity_ned[0]
                state[4] = bridge.telemetry.velocity_ned[1]
                state[5] = bridge.telemetry.velocity_ned[2]

            current_att = [0.0, 0.0, 0.0]
            if bridge.telemetry.attitude_euler:
                current_att = bridge.telemetry.attitude_euler.copy()

            if initial_pos is None and bridge.telemetry.position_ned:
                initial_pos = np.array(bridge.telemetry.position_ned)

            # Generate giant setpoint
            sp = generate_giant_lemniscate_setpoints(t)
            if initial_pos is not None:
                sp[0] += initial_pos[0]
                sp[1] += initial_pos[1]
                # Don't offset Z, let it climb to exactly -5m from local origin

            # Tính toán hướng nhìn (Yaw) theo Vector vận tốc mục tiêu để quay đầu theo hướng bay
            if len(sp) >= 6 and (abs(sp[3]) > 0.01 or abs(sp[4]) > 0.01):
                yaw_setpoint = float(np.arctan2(sp[4], sp[3]))
            else:
                yaw_setpoint = 0.0

            roll_deg, pitch_deg, yaw_deg, thrust_N = geom_ctrl.compute_attitude_thrust(
                position_setpoint=sp, yaw_setpoint=yaw_setpoint, current_state=state
            )

            # Normalize thrust for PX4 (0.0 to 1.0)
            norm_thrust = np.clip(thrust_N / max_thrust_N, 0.0, 1.0)

            if initial_pos is not None:
                t_log.append(t)
                pos_log.append(state[0:3].copy())
                sp_log.append(sp[0:3].copy())
                v_log.append(state[3:6].copy())
                sp_v_log.append(sp[3:6].copy())
                thrust_log.append(norm_thrust)
                att_log.append(current_att)
                cmd_att_log.append([roll_deg, pitch_deg, yaw_deg])

            await controller.set_attitude(roll_deg, pitch_deg, yaw_deg, norm_thrust)

            # Sleep to maintain loop rate
            elapsed = time.time() - start_time - t
            sleep_time = max(0.0, dt - elapsed)
            await asyncio.sleep(sleep_time)

    except asyncio.CancelledError:
        pass
    finally:
        print("--- Flight Complete. Calculating Metrics...")
        if len(t_log) > 0:
            pos_arr = np.array(pos_log)
            sp_arr = np.array(sp_log)
            v_arr = np.array(v_log)
            sp_v_arr = np.array(sp_v_log)
            att_arr = np.array(att_log)
            cmd_att_arr = np.array(cmd_att_log)
            thrust_arr = np.array(thrust_log)

            pos_errors = np.linalg.norm(pos_arr - sp_arr, axis=1)
            pos_rmse = np.sqrt(np.mean(pos_errors**2))

            v_errors = np.linalg.norm(v_arr - sp_v_arr, axis=1)
            v_rmse = np.sqrt(np.mean(v_errors**2))

            # Attitude error (roll, pitch) ignoring yaw
            att_errors = np.linalg.norm(att_arr[:, 0:2] - cmd_att_arr[:, 0:2], axis=1)
            att_rmse = np.sqrt(np.mean(att_errors**2))

            avg_thrust = np.mean(thrust_arr) * 100.0  # Percentage

            print("=========================================")
            print("      EXTENDED GAZEBO METRICS (60s)    ")
            print("=========================================")
            print(f" Flight Time      : {t_log[-1]:.2f} s")
            print(f" Position RMSE    : {pos_rmse:.4f} m")
            print(f" Max Pos Error    : {np.max(pos_errors):.4f} m")
            print(f" Velocity RMSE    : {v_rmse:.4f} m/s")
            print(f" Attitude RMSE    : {att_rmse:.4f} deg (Roll/Pitch tracking lag)")
            print(f" Avg Thrust Cmd   : {avg_thrust:.1f} %")
            print("=========================================")

            import csv

            log_dir = Path(__file__).resolve().parents[2] / "logs"
            log_dir.mkdir(exist_ok=True)
            csv_path = str(log_dir / "flight_telemetry.csv")
            with open(csv_path, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(
                    [
                        "time_s",
                        "pos_x",
                        "pos_y",
                        "pos_z",
                        "sp_x",
                        "sp_y",
                        "sp_z",
                        "vel_x",
                        "vel_y",
                        "vel_z",
                        "sp_vx",
                        "sp_vy",
                        "sp_vz",
                        "roll_deg",
                        "pitch_deg",
                        "yaw_deg",
                        "cmd_roll_deg",
                        "cmd_pitch_deg",
                        "cmd_yaw_deg",
                        "thrust_cmd_pct",
                    ]
                )
                for i in range(len(t_log)):
                    writer.writerow(
                        [
                            t_log[i],
                            pos_arr[i, 0],
                            pos_arr[i, 1],
                            pos_arr[i, 2],
                            sp_arr[i, 0],
                            sp_arr[i, 1],
                            sp_arr[i, 2],
                            v_arr[i, 0],
                            v_arr[i, 1],
                            v_arr[i, 2],
                            sp_v_arr[i, 0],
                            sp_v_arr[i, 1],
                            sp_v_arr[i, 2],
                            att_arr[i, 0],
                            att_arr[i, 1],
                            att_arr[i, 2],
                            cmd_att_arr[i, 0],
                            cmd_att_arr[i, 1],
                            cmd_att_arr[i, 2],
                            thrust_arr[i] * 100.0,
                        ]
                    )
            print(f"--- Saved detailed telemetry to {csv_path}")

        print("--- Returning to launch")
        await controller.stop_offboard()
        await bridge.drone.action.return_to_launch()
        await asyncio.sleep(5)
        await bridge.stop_tasks()


if __name__ == "__main__":
    asyncio.run(run_geometric_lemniscate())
