import argparse
import csv
import gc
import os
import sys
import time
from pprint import pprint

import numpy as np
from DReyeVR_utils import DReyeVRSensor, find_ego_vehicle
from HapticSharedControl.bezier_path import *
from HapticSharedControl.haptic_algo import *
from logidrivepy import LogitechController

import carla

P_0 = [-147.066772, -1322.415039]  # [y, x]
P_f = [-687.066772, -2162.415039]
P_d = [-37.066772, -2902.415039]

yaw_0 = 0
yaw_d = 10
yaw_f = 90

vel = 2.0  # km/h

n_points = 100

path1, control_points1, plot_el_1 = calculate_bezier_trajectory(
    start_pos=P_0[::-1],
    end_pos=P_d[::-1],
    start_yaw=yaw_0,
    end_yaw=yaw_d,
    n_points=n_points,
    turning_radius=R,
    control_param=0.86,
    show_animation=False,
)
# backward so reverse the yaw angle (+180)
path2, control_points2, plot_el_2 = calculate_bezier_trajectory(
    start_pos=P_d[::-1],
    end_pos=P_f[::-1],
    start_yaw=180 + yaw_d,
    end_yaw=180 + yaw_f,
    n_points=n_points,
    turning_radius=R,
    control_param=0.86,
    show_animation=False,
)

# show 2 path in same plot
plt.figure()
plt.plot(path1.T[0], path1.T[1], label="Bezier Path 1")
plt.plot(control_points1.T[0], control_points1.T[1], "--o", label="Control Points 1")
plt.plot(plot_el_1["x_target"], plot_el_1["y_target"])
plt.plot(plot_el_1["tangent"][:, 0], plot_el_1["tangent"][:, 1], label="Tangent 1")
plt.plot(plot_el_1["normal"][:, 0], plot_el_1["normal"][:, 1], label="Normal 1")
plt.gca().add_artist(plot_el_1["circle"])
plot_arrow(
    plot_el_1["start_x"],
    plot_el_1["start_y"],
    np.pi - plot_el_1["start_yaw"],
    length=0.1 * plot_el_1["dist"],
    width=0.02 * plot_el_1["dist"],
)
plot_arrow(
    plot_el_1["end_x"],
    plot_el_1["end_y"],
    np.pi - plot_el_1["end_yaw"],
    length=0.1 * plot_el_1["dist"],
    width=0.02 * plot_el_1["dist"],
)

plt.plot(path2.T[0], path2.T[1], label="Bezier Path 2")
plt.plot(control_points2.T[0], control_points2.T[1], "--o", label="Control Points 2")
plt.plot(plot_el_2["x_target"], plot_el_2["y_target"])
plt.plot(plot_el_2["tangent"][:, 0], plot_el_2["tangent"][:, 1], label="Tangent 2")
plt.plot(plot_el_2["normal"][:, 0], plot_el_2["normal"][:, 1], label="Normal 2")
plt.gca().add_artist(plot_el_2["circle"])
plot_arrow(
    plot_el_2["start_x"],
    plot_el_2["start_y"],
    np.pi - plot_el_2["start_yaw"],
    length=0.1 * plot_el_2["dist"],
    width=0.02 * plot_el_2["dist"],
)
plot_arrow(
    plot_el_2["end_x"],
    plot_el_2["end_y"],
    np.pi - plot_el_2["end_yaw"],
    length=0.1 * plot_el_2["dist"],
    width=0.02 * plot_el_2["dist"],
)

plt.legend()
plt.grid(True)
plt.show()
plt.close()

time_step = 1.0
path = path2

i_points = P_d[::-1]
f_points = P_f[::-1]
i_yaw = 180 + yaw_d

# Simulation Parameters
# speed = int((f_points[0] - i_points[0]) // 100)
speed = -2
current_position = i_points
current_time = 0

# Haptic Shared Control Initialization
haptic_control = HapticSharedControl(
    Cs=0.5, Kc=0.5, T=1, tp=2, speed=speed, desired_trajectory=path
)

# Simulation Loop
trajectory = [current_position]
torques = []
steering_angles = [0]
offset_percentage = []


def main():
    global df

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
    argparser.add_argument(
        "-sh",
        "--selfhost",
        metavar="Sh",
        default="163.221.139.137",
        help="IP of the ROS node (this machine) (default: 192.168.86.123)",
    )

    args = argparser.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    sync_mode = True  # synchronous mode
    np.random.seed(int(time.time()))

    world = client.get_world()

    sensor = DReyeVRSensor(world)

    controller = LogitechController()
    controller.steering_initialize()
    max_range = 900
    controller.set_operating_range(index=0, range=max_range)
    print("\n---Logitech Spin Test---")
    ff_available = controller.has_force_feedback(index=0)
    print("Force feedback available:", ff_available)

    def get_data(data):
        # global df
        global prev_yaw
        sensor.update(data)
        data = sensor.data
        # pprint(sensor.data)  # more useful print here (contains all attributes)
        current_pos = data["Location"][:-1]
        current_yaw = data["Rotation"][1]
        delta_yaw = current_yaw - prev_yaw

        steering_curr = (delta_yaw + 0.016482) / (-0.000348)
        haptic_control.theta_curr = steering_curr

        print(f"Current Position: {current_pos}, Current Yaw: {current_yaw}")
        torque = haptic_control.calculate_torque(
            current_position=current_position,
            steering_wheel_angle_t=steering_angles[-1],
            t=current_time,
        )
        next_position = haptic_control.predicted_position

        steering_angles = np.degrees(haptic_control.theta_d) / 450
        offset = (float(steering_angles) - 23.004634) / 3.849622
        print(int(offset))
        if offset >= 100:
            offset = 100
        elif offset <= -100:
            offset = -100
        print(int(offset))
        controller.play_spring_force(
            index=0,
            offset_percentage=int(offset),
            saturation_percentage=50,
            coefficient_percentage=100,
        )
        controller.logi_update()
        prev_yaw = current_yaw
        time.sleep(1.0)
        # print(data) # this print is defined in LibCarla/source/carla/data/DReyeVREvent.h

    # print(sensor.ego_vehicle.get_physics_control())
    # subscribe to DReyeVR sensor
    sensor.ego_sensor.listen(get_data)
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
        controller.stop_spring_force(index=0)
        controller.steering_shutdown()
        del controller
        gc.collect()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print("\ndone.")
