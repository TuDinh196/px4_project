"""
Flight Scenarios
================
Executes pre-defined flight trajectories using MAVSDK offboard control.
"""

import asyncio
import math
from pathlib import Path

import yaml

from src.px4_integration.mavsdk_bridge import MAVSDKBridge
from src.px4_integration.offboard_controller import OffboardController


class FlightScenarios:
    def __init__(self, bridge: MAVSDKBridge, controller: OffboardController):
        self.bridge = bridge
        self.controller = controller

        # Load config
        config_path = Path(__file__).resolve().parent / "scenario_config.yaml"
        with open(config_path, "r") as f:
            self.config = yaml.safe_load(f)

    async def run_hover(self):
        """Takeoff, hover at target altitude, then land."""
        cfg = self.config["hover"]
        alt = cfg["target_altitude"]
        hover_time = cfg["hover_time"]

        print(f"--- Scenario: Hover at {alt}m for {hover_time}s")
        await self.controller.set_position(0, 0, alt, 0)
        await asyncio.sleep(hover_time)

    async def run_square(self):
        """Fly a square pattern."""
        cfg = self.config["square"]
        alt = cfg["altitude"]
        side = cfg["side_length"]
        speed = cfg["speed"]

        # Simple waypoint execution (assuming constant velocity isn't strictly enforced)
        waypoints = [
            (side, 0, alt, 0),
            (side, side, alt, 90),
            (0, side, alt, 180),
            (0, 0, alt, 270),
        ]

        print(f"--- Scenario: Square with side {side}m")
        # Go to start altitude
        await self.controller.set_position(0, 0, alt, 0)
        await asyncio.sleep(5)

        for wp in waypoints:
            print(f"    Going to waypoint: {wp}")
            await self.controller.set_position(wp[0], wp[1], wp[2], wp[3])
            # Wait for approx time to reach (dist/speed) + margin
            wait_time = (side / speed) + 2.0
            await asyncio.sleep(wait_time)

    async def run_circle(self):
        """Fly a circular pattern using velocity setpoints."""
        cfg = self.config["circle"]
        alt = cfg["altitude"]
        radius = cfg["radius"]
        omega = cfg["angular_speed"]
        duration = cfg["duration"]

        print(f"--- Scenario: Circle radius {radius}m")

        # Go to starting point of the circle (R, 0, alt)
        await self.controller.set_position(radius, 0, alt, 0)
        await asyncio.sleep(8)

        print("    Starting circular trajectory")
        dt = 0.1
        steps = int(duration / dt)

        for i in range(steps):
            t = i * dt
            # Position: x = R*cos(w*t), y = R*sin(w*t)
            # Velocity: vx = -R*w*sin(w*t), vy = R*w*cos(w*t)
            vx = -radius * omega * math.sin(omega * t)
            vy = radius * omega * math.cos(omega * t)

            # Yaw tangentially
            yaw_deg = math.degrees(math.atan2(vy, vx))

            await self.controller.set_velocity(vx, vy, 0, yaw_deg)
            await asyncio.sleep(dt)

        # Stop
        await self.controller.set_velocity(0, 0, 0, 0)

    async def run_figure8(self):
        """Fly a Figure-8 (Lissajous) trajectory using velocity setpoints."""
        cfg = self.config.get("figure8", {})
        X_amp = cfg.get("x_amplitude", 20.0)
        Y_amp = cfg.get("y_amplitude", 40.0)
        omega = cfg.get("angular_speed", math.pi / 20.0)
        alt = cfg.get("altitude", -10.0)
        duration = cfg.get("duration", 80.0)

        print(f"--- Scenario: Figure 8 ({X_amp}m x {Y_amp}m)")

        # Takeoff to target altitude first
        await self.controller.set_position(0, 0, alt, 0)
        await asyncio.sleep(8)

        print("    Starting Figure-8 trajectory")
        dt = 0.05  # 20Hz update
        steps = int(duration / dt)

        for i in range(steps):
            t = i * dt
            # Envelope filter for smooth start
            blend = 1.0 - math.exp(-t / 3.0)

            # Lissajous Figure-8: x = X*sin(2ωt), y = Y*sin(ωt)
            vx = 2 * X_amp * omega * math.cos(2 * omega * t) * blend
            vy = Y_amp * omega * math.cos(omega * t) * blend

            yaw_deg = (
                math.degrees(math.atan2(vy, vx))
                if (abs(vx) > 0.01 or abs(vy) > 0.01)
                else 0.0
            )

            await self.controller.set_velocity(vx, vy, 0, yaw_deg)
            await asyncio.sleep(dt)

        # Stop
        await self.controller.set_velocity(0, 0, 0, 0)

    async def run_manual(self):
        """Allow manual control via continuous velocity updates from websocket."""
        print("--- Scenario: Manual Control Started")

        # Need to start offboard with some valid setpoint
        await self.controller.set_velocity(0, 0, 0, 0)

        # We will loop and continuously send the global velocity target
        # Global velocity target is updated from server.py
        import src.dashboard.web_dashboard.server as server_module

        dt = 0.05  # 20Hz update
        try:
            while True:
                # Read global manual velocity vector
                v = server_module.manual_velocity_cmd
                await self.controller.set_velocity(v["vx"], v["vy"], v["vz"], v["yaw"])
                await asyncio.sleep(dt)
        except asyncio.CancelledError:
            print("Manual control ended.")
            # Stop vehicle
            await self.controller.set_velocity(0, 0, 0, 0)


async def main_runner(scenario_name: str):
    """Main entry point to execute a scenario."""
    bridge = MAVSDKBridge()
    await bridge.connect()

    # Start telemetry in background
    bridge.start_telemetry_tasks()

    controller = OffboardController(bridge)
    scenarios = FlightScenarios(bridge, controller)

    # Arm and start offboard
    await bridge.arm()
    success = await controller.start_offboard()

    if not success:
        await bridge.stop_tasks()
        return

    try:
        # Run selected scenario
        if scenario_name == "hover":
            await scenarios.run_hover()
        elif scenario_name == "square":
            await scenarios.run_square()
        elif scenario_name == "circle":
            await scenarios.run_circle()
        elif scenario_name == "figure8":
            await scenarios.run_figure8()
        elif scenario_name == "manual":
            await scenarios.run_manual()
        else:
            print(f"Unknown scenario: {scenario_name}")

    except asyncio.CancelledError:
        pass
    finally:
        print("--- Returning to home / Landing")
        await controller.stop_offboard()
        await bridge.drone.action.return_to_launch()

        # Wait a bit for landing to initiate
        await asyncio.sleep(5)
        await bridge.stop_tasks()


if __name__ == "__main__":
    import sys

    scenario = sys.argv[1] if len(sys.argv) > 1 else "hover"
    asyncio.run(main_runner(scenario))
