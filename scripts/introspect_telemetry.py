import asyncio

from mavsdk import System


async def run():
    drone = System()
    await drone.connect(system_address="udp://:14550")
    print("Waiting for drone...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            break

    print([method for method in dir(drone.telemetry) if not method.startswith("_")])


if __name__ == "__main__":
    asyncio.run(run())
