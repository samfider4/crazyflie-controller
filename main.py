import logging
import time
import keyboard

from crazyflie_client import CrazyflieClient
from control import Goal
from flight_service import FlightService


CRAZYFLIE_URI = 'radio://0/100/2M/E7E7E7E7E9'
# CRAZYFLIE_URI = 'radio://0/90/2M/E7E7E7E7E8'
MOCAP_HOSTNAME = '128.101.167.111'
MOCAP_SYSTEM_TYPE = 'vicon'
DRONE_OBJECT_NAME = '2026_Drone1'
# DRONE_OBJECT_NAME = '2026_Drone2'
GROUND_OBJECT_NAME = 'box'

TAKEOFF_HEIGHT = 1.5
TAKEOFF_HOLD_SECONDS = 5.0


logging.basicConfig(level=logging.ERROR)


def build_goal(start_pose, ground_pose, runtime: float) -> Goal:
    if runtime < TAKEOFF_HOLD_SECONDS or ground_pose is None:
        return Goal(x=start_pose.x, y=start_pose.y, z=TAKEOFF_HEIGHT)

    return Goal(x=ground_pose.x, y=ground_pose.y, z=ground_pose.z + TAKEOFF_HEIGHT)


def main() -> None:
    CrazyflieClient.init_drivers()

    flight_service = FlightService(
        crazyflie_uri=CRAZYFLIE_URI,
        mocap_hostname=MOCAP_HOSTNAME,
        mocap_system_type=MOCAP_SYSTEM_TYPE,
        drone_object_name=DRONE_OBJECT_NAME,
        ground_object_name=GROUND_OBJECT_NAME,
    )

    flight_service.start()

    start_pose = None
    while start_pose is None and not keyboard.is_pressed('esc'):
        start_pose = flight_service.get_latest_pose(DRONE_OBJECT_NAME)
        time.sleep(0.01)

    if start_pose is None:
        print('Could not get initial drone pose')
        flight_service.stop()
        return

    try:
        while not keyboard.is_pressed('esc'):
            frame = flight_service.get_latest_frame()
            runtime = flight_service.get_runtime()
            ground_pose = frame.get(GROUND_OBJECT_NAME)

            goal = build_goal(start_pose, ground_pose, runtime)
            flight_service.set_goal(goal)

            time.sleep(0.01)

    except KeyboardInterrupt:
        pass
    finally:
        print('\nKill button pressed, shutting down')
        flight_service.stop()


if __name__ == '__main__':
    main()