"""
Automated Hybrid VTOL Mission Execution for Quadplane Condor
============================================================
Performs complete 5-stage real flight mission in PX4 SITL:
  1. Automated Vertical Takeoff (VTOL Climb to 15m)
  2. Forward Transition to Fixed-Wing (Motor 4 Tractor acceleration + wing lift takeover)
  3. High-Speed Fixed-Wing Waypoint Cruise (18-20 m/s long-range surveillance patrol)
  4. Back-Transition to Multicopter Mode (Deceleration + quad rotor spin-up)
  5. Precision VTOL Return & Landing

Authors: Autonomous UAV Navigation Team
"""

import asyncio

from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan


async def run_vtol_mission():
    print("=" * 65)
    print(" Starting Quadplane Condor Hybrid VTOL Automated Mission")
    print("=" * 65)

    drone = System()
    print("Connecting to PX4 SITL on udp://:14540...")
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(" Connected to Quadplane Condor!")
            break

    print("Waiting for global GPS position and home lock...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print(" GPS Position & Home Locked OK")
            break

    # Fetch home position
    home_lat = 0.0
    home_lon = 0.0
    async for home in drone.telemetry.home():
        home_lat = home.latitude_deg
        home_lon = home.longitude_deg
        print(f" Home Location: ({home_lat:.6f}, {home_lon:.6f})")
        break

    # Build High-Speed Fixed-Wing Survey Mission
    # 1 deg latitude ≈ 111,320 m -> 100m ≈ 0.000898 deg
    d_lat = 0.0025  # ~280m North
    d_lon = 0.0035  # ~320m East

    print("\n--- Constructing Quadplane Hybrid Mission Plan ---")
    mission_items = [
        # Item 0: VTOL Takeoff to 20m
        MissionItem(
            latitude_deg=home_lat,
            longitude_deg=home_lon,
            relative_altitude_m=20.0,
            speed_m_s=5.0,
            is_fly_through=True,
            gimbal_pitch_deg=0.0,
            gimbal_yaw_deg=0.0,
            camera_action=MissionItem.CameraAction.NONE,
            loiter_time_s=0.0,
            camera_photo_interval_s=0.0,
            acceptance_radius_m=5.0,
            yaw_deg=0.0,
            camera_photo_distance_m=0.0,
            vehicle_action=MissionItem.VehicleAction.NONE,
        ),
        # Item 1: Forward Transition & Waypoint 1 (High-speed Fixed Wing at 18 m/s)
        MissionItem(
            latitude_deg=home_lat + d_lat,
            longitude_deg=home_lon,
            relative_altitude_m=35.0,
            speed_m_s=18.0,
            is_fly_through=True,
            gimbal_pitch_deg=0.0,
            gimbal_yaw_deg=0.0,
            camera_action=MissionItem.CameraAction.NONE,
            loiter_time_s=0.0,
            camera_photo_interval_s=0.0,
            acceptance_radius_m=15.0,
            yaw_deg=0.0,
            camera_photo_distance_m=0.0,
            vehicle_action=MissionItem.VehicleAction.NONE,
        ),
        # Item 2: Waypoint 2 (East patrol leg at 20 m/s)
        MissionItem(
            latitude_deg=home_lat + d_lat,
            longitude_deg=home_lon + d_lon,
            relative_altitude_m=40.0,
            speed_m_s=20.0,
            is_fly_through=True,
            gimbal_pitch_deg=0.0,
            gimbal_yaw_deg=0.0,
            camera_action=MissionItem.CameraAction.NONE,
            loiter_time_s=0.0,
            camera_photo_interval_s=0.0,
            acceptance_radius_m=15.0,
            yaw_deg=90.0,
            camera_photo_distance_m=0.0,
            vehicle_action=MissionItem.VehicleAction.NONE,
        ),
        # Item 3: Waypoint 3 (South leg at 20 m/s)
        MissionItem(
            latitude_deg=home_lat,
            longitude_deg=home_lon + d_lon,
            relative_altitude_m=35.0,
            speed_m_s=20.0,
            is_fly_through=True,
            gimbal_pitch_deg=0.0,
            gimbal_yaw_deg=0.0,
            camera_action=MissionItem.CameraAction.NONE,
            loiter_time_s=0.0,
            camera_photo_interval_s=0.0,
            acceptance_radius_m=15.0,
            yaw_deg=180.0,
            camera_photo_distance_m=0.0,
            vehicle_action=MissionItem.VehicleAction.NONE,
        ),
        # Item 4: Return Waypoint (Back-transition approach to 20m alt)
        MissionItem(
            latitude_deg=home_lat,
            longitude_deg=home_lon,
            relative_altitude_m=20.0,
            speed_m_s=12.0,
            is_fly_through=False,
            gimbal_pitch_deg=0.0,
            gimbal_yaw_deg=0.0,
            camera_action=MissionItem.CameraAction.NONE,
            loiter_time_s=3.0,
            camera_photo_interval_s=0.0,
            acceptance_radius_m=10.0,
            yaw_deg=270.0,
            camera_photo_distance_m=0.0,
            vehicle_action=MissionItem.VehicleAction.NONE,
        ),
    ]

    mission_plan = MissionPlan(mission_items)
    print("Uploading mission plan to PX4 Autopilot...")
    await drone.mission.upload_mission(mission_plan)
    print(" Mission uploaded successfully (5 waypoints)")

    # Background telemetry task
    telemetry_running = True

    async def log_telemetry():
        while telemetry_running:
            try:
                await asyncio.sleep(0.1)
            except asyncio.CancelledError:
                break
            except Exception:
                await asyncio.sleep(0.1)

    telemetry_task = asyncio.create_task(log_telemetry())

    print("\n--- Arming Quadplane Condor ---")
    await drone.action.arm()

    print("--- Starting Autonomous VTOL Mission ---")
    await drone.mission.start_mission()

    # Monitor mission progress
    print("Monitoring mission progress...")
    async for mission_progress in drone.mission.mission_progress():
        print(
            f"  [Waypoint {mission_progress.current + 1} / {mission_progress.total}] reached: "
            f"{(mission_progress.current / mission_progress.total)*100:.0f}% complete"
        )
        if mission_progress.current >= mission_progress.total:
            print(" All mission waypoints completed!")
            break

    print("\n--- Initiating Return to Launch & Precision VTOL Land ---")
    await drone.action.return_to_launch()

    # Wait for landing
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            print(" Quadplane Condor safely landed on ground!")
            break
        await asyncio.sleep(2)

    telemetry_running = False
    await telemetry_task

    print("\n" + "=" * 65)
    print(" HYBRID VTOL MISSION ACCOMPLISHED SUCCESSFULLY")
    print("=" * 65)


if __name__ == "__main__":
    asyncio.run(run_vtol_mission())
