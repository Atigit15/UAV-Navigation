#!/usr/bin/env python3
# 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

import asyncio

from mavsdk import System
from mavsdk.mission import (MissionItem, MissionPlan)


async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    print_mission_progress_task = asyncio.ensure_future(print_mission_progress(drone))
    print_battery = asyncio.ensure_future(check_battery(drone))


    running_tasks = [print_mission_progress_task]
    termination_task = asyncio.ensure_future(observe_is_in_air(drone, running_tasks))

    mission_items = []

    #A
    mission_items.append(MissionItem(26.934841,75.923682,25,10,True,float('nan'),float('nan'),MissionItem.CameraAction.NONE,float('nan'),float('nan'),float('nan'),float('nan'),float('nan')))
    #B
    mission_items.append(MissionItem(26.934841,75.923987,25,10,True,float('nan'),float('nan'),MissionItem.CameraAction.NONE,float('nan'),float('nan'),float('nan'),float('nan'),float('nan')))
    #C
    mission_items.append(MissionItem(26.934536,75.923987,25,10,True,float('nan'),float('nan'),MissionItem.CameraAction.NONE,float('nan'),float('nan'),float('nan'),float('nan'),float('nan')))
    #D
    mission_items.append(MissionItem(26.934536,75.923682,25,10,True,float('nan'),float('nan'),MissionItem.CameraAction.NONE,float('nan'),float('nan'),float('nan'),float('nan'),float('nan')))

    mission_plan = MissionPlan(mission_items)

    print("-- Uploading mission")
    await drone.mission.upload_mission(mission_plan)

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break

    print("-- Arming")
    await drone.action.arm()

    print("-- Starting mission")
    await drone.mission.start_mission()

    await termination_task


async def print_mission_progress(drone):
    async for mission_progress in drone.mission.mission_progress():
        print(f"Mission progress: "f"{mission_progress.current}/"f"{mission_progress.total}")

async def check_battery(drone):
    async for battery in drone.telemetry.battery():
        print(battery.remaining_percent)
        if battery.remaining_percent < 0.3:
            print("-- Battery insufficient, initiating RTL")
            await drone.mission.set_return_to_launch_after_mission(True)
            drone.mission.cancel()


async def observe_is_in_air(drone, running_tasks):
    """ Monitors whether the drone is flying or not and
    returns after landing """

    was_in_air = False

    async for is_in_air in drone.telemetry.in_air():
        if is_in_air:
            was_in_air = is_in_air

        if was_in_air and not is_in_air:
            for task in running_tasks:
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass
            await asyncio.get_event_loop().shutdown_asyncgens()

            return


if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(run())
