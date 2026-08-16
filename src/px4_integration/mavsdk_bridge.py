"""
MAVSDK Bridge
=============
Handles connection and asynchronous communication with PX4 via MAVSDK.
Provides methods to read telemetry and send basic commands.
"""

import asyncio
from dataclasses import dataclass

from mavsdk import System


@dataclass
class TelemetryData:
    """Stores the latest telemetry data."""

    position_ned: list[float] = None  # [x, y, z] (m)
    velocity_ned: list[float] = None  # [vx, vy, vz] (m/s)
    attitude_euler: list[float] = None  # [roll, pitch, yaw] (deg)
    attitude_angular_velocity: list[float] = None  # [p, q, r] (deg/s)
    battery_voltage: float = 0.0
    armed: bool = False
    flight_mode: str = "UNKNOWN"


class MAVSDKBridge:
    def __init__(self, connection_url: str = "udp://:14540"):
        self.drone = System()
        self.connection_url = connection_url
        self.telemetry = TelemetryData()
        self._tasks = []

    async def connect(self):
        """Connects to the drone and waits for it to be ready."""
        print(f"Connecting to PX4 on {self.connection_url}...")
        await self.drone.connect(system_address=self.connection_url)

        print("Waiting for drone to connect...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print("-- Connected to drone!")
                break

        print("Waiting for drone to have a global position estimate...")
        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                print("-- Global position estimate OK")
                break

    def start_telemetry_tasks(self):
        """Starts asynchronous background tasks to monitor telemetry."""
        self._tasks = [
            asyncio.create_task(self._update_position()),
            asyncio.create_task(self._update_attitude()),
            asyncio.create_task(self._update_angular_velocity()),
            asyncio.create_task(self._update_battery()),
            asyncio.create_task(self._update_flight_mode()),
            asyncio.create_task(self._update_armed_state()),
        ]

    async def stop_tasks(self):
        """Cancels all telemetry tasks."""
        for task in self._tasks:
            task.cancel()
        await asyncio.gather(*self._tasks, return_exceptions=True)

    # --- Telemetry Updaters ---
    async def _update_position(self):
        async for pos in self.drone.telemetry.position_velocity_ned():
            self.telemetry.position_ned = [
                pos.position.north_m,
                pos.position.east_m,
                pos.position.down_m,
            ]
            self.telemetry.velocity_ned = [
                pos.velocity.north_m_s,
                pos.velocity.east_m_s,
                pos.velocity.down_m_s,
            ]

    async def _update_attitude(self):
        async for att in self.drone.telemetry.attitude_euler():
            self.telemetry.attitude_euler = [att.roll_deg, att.pitch_deg, att.yaw_deg]

    async def _update_angular_velocity(self):
        async for rate in self.drone.telemetry.attitude_angular_velocity_body():
            import math
            self.telemetry.attitude_angular_velocity = [
                math.degrees(rate.roll_rad_s),
                math.degrees(rate.pitch_rad_s),
                math.degrees(rate.yaw_rad_s),
            ]

    async def _update_battery(self):
        async for battery in self.drone.telemetry.battery():
            self.telemetry.battery_voltage = battery.voltage_v

    async def _update_flight_mode(self):
        async for mode in self.drone.telemetry.flight_mode():
            self.telemetry.flight_mode = str(mode)

    async def _update_armed_state(self):
        async for armed in self.drone.telemetry.armed():
            self.telemetry.armed = armed

    # --- Basic Commands ---
    async def arm(self):
        print("-- Arming")
        for attempt in range(5):
            try:
                await self.drone.action.arm()
                print("✅ Armed successfully")
                return
            except Exception as e:
                print(f"  Arming retry {attempt+1}/5: {e}")
                await asyncio.sleep(1.5)
        print("⚠️ Arming fallback via offboard...")

    async def disarm(self):
        print("-- Disarming")
        await self.drone.action.disarm()
