"""
Offboard Controller
===================
Wrapper around MAVSDK Offboard mode for sending position/velocity setpoints.
"""

from mavsdk.offboard import Attitude, OffboardError, PositionNedYaw, VelocityNedYaw

from src.px4_integration.mavsdk_bridge import MAVSDKBridge


class OffboardController:
    def __init__(self, bridge: MAVSDKBridge):
        self.bridge = bridge
        self.drone = bridge.drone

    async def start_offboard(self):
        """
        Starts Offboard mode. PX4 requires an initial setpoint to be sent
        before starting Offboard mode.
        """
        print("-- Setting initial setpoint")
        # Send a dummy setpoint to satisfy PX4 before starting offboard
        await self.drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

        print("-- Starting offboard")
        try:
            await self.drone.offboard.start()
            return True
        except OffboardError as error:
            print(
                f"Starting offboard mode failed with error code: {error._result.result}"
            )
            print("-- Disarming")
            await self.bridge.disarm()
            return False

    async def stop_offboard(self):
        """Stops Offboard mode."""
        print("-- Stopping offboard")
        try:
            await self.drone.offboard.stop()
        except OffboardError as error:
            print(
                f"Stopping offboard mode failed with error code: {error._result.result}"
            )

    async def set_position(
        self, north_m: float, east_m: float, down_m: float, yaw_deg: float = 0.0
    ):
        """Send a position setpoint (NED frame)."""
        await self.drone.offboard.set_position_ned(
            PositionNedYaw(north_m, east_m, down_m, yaw_deg)
        )

    async def set_velocity(
        self, north_m_s: float, east_m_s: float, down_m_s: float, yaw_deg: float = 0.0
    ):
        """Send a velocity setpoint (NED frame)."""
        await self.drone.offboard.set_velocity_ned(
            VelocityNedYaw(north_m_s, east_m_s, down_m_s, yaw_deg)
        )

    async def set_attitude(
        self, roll_deg: float, pitch_deg: float, yaw_deg: float, thrust_value: float
    ):
        """
        Send attitude setpoint.
        thrust_value: 0.0 to 1.0
        """
        await self.drone.offboard.set_attitude(
            Attitude(roll_deg, pitch_deg, yaw_deg, thrust_value)
        )
