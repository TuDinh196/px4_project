import asyncio

from mavsdk import System


async def run():
    # Connect to the GCS port (14550) so we don't interfere with the AI training on 14540
    drone = System()
    await drone.connect(system_address="udp://:14550")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Connected to drone!")
            break

    print("\n--- LIVE SIMULATION PARAMETERS ---")

    # Get one snapshot of telemetry
    async for pos in drone.telemetry.position_velocity_ned():
        print(
            f"Position (NED): N={
                pos.position.north_m:.2f}m, E={
                pos.position.east_m:.2f}m, D={
                pos.position.down_m:.2f}m")
        print(
            f"Velocity (NED): N={
                pos.velocity.north_m_s:.2f}m/s, E={
                pos.velocity.east_m_s:.2f}m/s, D={
                pos.velocity.down_m_s:.2f}m/s")
        break

    async for att in drone.telemetry.attitude_euler():
        print(
            f"Attitude (Euler): Roll={
                att.roll_deg:.2f}°, Pitch={
                att.pitch_deg:.2f}°, Yaw={
                att.yaw_deg:.2f}°")
        break

    async for battery in drone.telemetry.battery():
        print(f"Battery Voltage: {battery.voltage_v:.2f} V")
        pct = battery.remaining_percent
        rem = pct * 100 if pct <= 1.0 else pct
        print(f"Battery Remaining: {rem:.1f}%")
        break

    async for mode in drone.telemetry.flight_mode():
        print(f"Flight Mode: {mode}")
        break

    async for armed in drone.telemetry.armed():
        print(f"Armed Status: {'ARMED' if armed else 'DISARMED'}")
        break

    print("----------------------------------\n")


if __name__ == "__main__":
    asyncio.run(run())
