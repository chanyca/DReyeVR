import argparse
import csv
import gc
import os
import sys
import time
from pprint import pprint

import carla
import numpy as np
from DReyeVR_utils import DReyeVRSensor, find_ego_vehicle
from HapticSharedControl.haptic_algo import *
from HapticSharedControl.path_planning import *
from HapticSharedControl.simulation import *
from HapticSharedControl.wheel_control import *


def main():
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument(
        "--host",
        metavar="H",
        default="127.0.0.1",
        help="IP of the host server (default: 127.0.0.1)",
    )
    argparser.add_argument(
        "-p",
        "--port",
        metavar="P",
        default=2000,
        type=int,
        help="TCP port to listen to (default: 2000)",
    )

    args = argparser.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    sync_mode = True  # synchronous mode
    np.random.seed(int(time.time()))

    world = client.get_world()

    sensor = DReyeVRSensor(world)

    controller = WheelController()
    offset = 0.0

    def control_loop(data):
        sensor.update(data)
        measured_carla_data = sensor.data
        # pprint(sensor.data)  # more useful print here (contains all attributes)
        # 1. get vehicle current states
        # - position
        # - yaw
        # - speed
        # - wheel angle
        position_to_world = measured_carla_data["Location"][0:2]  # [x, y]
        vehicle_yaw = measured_carla_data["Rotation"][1]  # yaw

        velocity = measured_carla_data["Velocity"][0:2]  # [x, y]
        speed = np.linalg.norm(velocity)
        # get signal from Logitech Wheel SDK, if pressed the reverse button, then reverse the steering angle

        steering_wheel_angle = None
        steering_angles = [
            measured_carla_data["FL_Wheel_Angle"],
            measured_carla_data["FR_Wheel_Angle"],
        ]  # front wheel angles

        # 3. predict future position

        # 4. calculate DAS torque

        # 5. Control steering wheel angle

        time.sleep(1.0)

    # print(sensor.ego_vehicle.get_physics_control())
    # subscribe to DReyeVR sensor
    sensor.ego_sensor.listen(control_loop)
    try:
        while True:
            if sync_mode:
                world.tick()
            else:
                world.wait_for_tick()
    finally:
        if sync_mode:
            settings = world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            world.apply_settings(settings)
        controller.__del__()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print("\ndone.")
