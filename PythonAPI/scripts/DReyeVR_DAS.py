import argparse
import sys
import time
from pprint import pprint

import numpy as np
from DReyeVR_utils import DReyeVRSensor, find_ego_vehicle
from HapticSharedControl.haptic_algo import *
from HapticSharedControl.path_planning import *
from HapticSharedControl.utils import *
from HapticSharedControl.wheel_control import *

import carla

# cspell: ignore dreyevr dreyevrsensor libcarla harplab vergence numer linalg argparser Bezier polyfit arctan

with open("../data/paths/driving_path_left2right.txt", "r") as f:
    data_left2right = f.readlines()
    data_left2right = [line.strip().split(",") for line in data_left2right]
    data_left2right = [[float(val) for val in line] for line in data_left2right]
    data_left2right = np.array(data_left2right)

with open("../data/paths/driving_path_right2left.txt", "r") as f:
    data_right2left = f.readlines()
    data_right2left = [line.strip().split(",") for line in data_right2left]
    data_right2left = [[float(val) for val in line] for line in data_right2left]
    data_right2left = np.array(data_right2left)

with open("../data/paths/hitachi.txt", "r") as f:
    data_hitachi = f.readlines()
    data_hitachi = [line.strip().split(",") for line in data_hitachi]
    data_hitachi = [[float(val) for val in line] for line in data_hitachi]
    data_hitachi = np.array(data_hitachi)


predefined_path = {
    "0": {
        "P_0": [-1.47066772, -13.22415039],
        "P_d": [-0.37066772, -29.02415039],
        "P_f": [-6.87066772, -21.62415039],
        "yaw_0": 90 - (-90),
        "yaw_d": 90 - (-80),
        "yaw_f": 90 - 0,
        "forward paths": None,
        "backward paths": None,
    },
    "1": {
        "P_0": [-2.376371583333333, -13.749357381666668],
        "P_d": [-0.566469992644628, -27.297582133636364],
        "P_f": [-7.7486732, -21.271947543333333],
        "yaw_0": 90 - (88.97628786666667),
        "yaw_d": 90 - (62.168829599999995),
        "yaw_f": 90 - (-1.9554259149999922),
        "forward paths": process_exist_path(data_left2right[:40]),
        "backward paths": process_exist_path(data_left2right[40:]),
    },
    "2": {
        "P_0": [-2.284308814, -31.734065629999996],
        "P_d": [0.409550333177305, -18.459612079588652],
        "P_f": [-7.449310874, -21.459648512],
        "yaw_0": 90 - (-88.9748993),
        "yaw_d": 90 - (-38.76474382600003),
        "yaw_f": 90 - (1.296984835999993),
        "forward paths": process_exist_path(data_right2left[:46]),
        "backward paths": process_exist_path(data_right2left[46:]),
    },
    "3": {
        "P_0": [],
        "P_d": [1.035116, -18.325821],
        "P_f": [-7.685114, -21.214608],
        "yaw_0": None,
        "yaw_d": 90 - (50),
        "yaw_f": 90 - (0),
        "forward paths": [],
        "backward paths": process_exist_path(data_hitachi),
    },
}

# Load the vehicle configuration
__file_path__ = Path(__file__).resolve().parent
with open(f"{__file_path__}/HapticSharedControl/wheel_setting.json", "r") as f:
    vehicle_config = json.load(f)
vehicle = Vehicle(vehicle_config=vehicle_config)
R = vehicle.minimum_turning_radius

# using Bezier curve to generate the path for path 0
n_points = 60
_, _, param_fw = calculate_bezier_trajectory(
    start_pos=predefined_path["0"]["P_0"][::-1],
    end_pos=predefined_path["0"]["P_d"][::-1],
    start_yaw=predefined_path["0"]["yaw_0"],
    end_yaw=predefined_path["0"]["yaw_d"],
    n_points=n_points,
    turning_radius=R,
    show_animation=False,
)

_, _, param_bw = calculate_bezier_trajectory(
    start_pos=predefined_path["0"]["P_d"][::-1],
    end_pos=predefined_path["0"]["P_f"][::-1],
    start_yaw=predefined_path["0"]["yaw_d"] + 180,  # reverse the yaw angle
    end_yaw=predefined_path["0"]["yaw_f"] + 180,  # reverse the yaw angle
    n_points=n_points,
    turning_radius=R,
    show_animation=False,
)

predefined_path["0"]["forward paths"] = param_fw
predefined_path["0"]["backward paths"] = param_bw


# ==============================================================================
# -- main function -------------------------------------------------------------
def main():
    global vehicle

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
    vehicle_ego = find_ego_vehicle(world)    
    controller = WheelController()

    ready = True
    take_control = False

    delta_t = 1.0 / 60

    backward_btn_pressed_cnt = 0
    
    # TODO: Change the idx to the path you want to test
    idx = "3"
    
    param = None

    # To calibrate the steering wheel angle and steering angle
    # find the linear relationship between steering wheel angle and steering angle
    init_sa_swa = [[], []]
    coef = [[], []]  
    
    def control_loop(data: dict) -> None:
        """This function is called every time the sensor receives new data from the simulation.

        Args:
            data (dict): dictionary containing the data from the CARLA simulation
        """

        global vehicle
        global predefined_path
        nonlocal idx

        nonlocal param

        nonlocal init_sa_swa
        nonlocal coef
        
        nonlocal delta_t
        nonlocal ready
        nonlocal take_control
        nonlocal controller
        nonlocal backward_btn_pressed_cnt

        # update the sensor data
        sensor.update(data)
        measured_carla_data = sensor.data
        # pprint(measured_carla_data) # more useful print here (contains all attributes)

        # 0. Initialize the steering wheel angle and steering angle
        if not ready:
            desired_offset = len(init_sa_swa[0]) - 100

            controller.play_spring_force(
                offset_percentage=desired_offset,
                saturation_percentage=100,
                coefficient_percentage=100,
            )

            steering_wheel_angle = controller.get_angle() * 450.0  # in degrees
            steering_angles = [
                measured_carla_data["FL_Wheel_Angle"],
                measured_carla_data["FR_Wheel_Angle"],
            ]  # front wheel angles
            
            vehicle_steering_angle = vehicle.calc_turning_radius(steering_angles)["Delta"]
            init_sa_swa[0].append(vehicle_steering_angle)
            init_sa_swa[1].append(steering_wheel_angle)

            if len(init_sa_swa[0]) > 201:  # from -100 to 100

                x = np.array(init_sa_swa[0])
                y = np.array(init_sa_swa[1])

                coef[0] = np.polyfit(x, y, 1)
                coef[1] = np.polyfit(y, x, 1)

                print("Calibration completed")
                print(f"Steering Wheel Angle = {coef[0][0]} * Steering Angle + {coef[0][1]}")
                print(f"Steering Angle = {coef[1][0]} * Steering Wheel Angle + {coef[1][1]}")

                # ready to start the simulation
                ready = True
                print("Ready to start the simulation")
                sys.exit(0)
            time.sleep(0.5)
            return

        print("=====================================")

        try:
            # 1. get vehicle current states (Position, Yaw, Speed, Wheel Angle)

            position_to_world = measured_carla_data["Location"][0:2]  # [x, y]
            vehicle_yaw = measured_carla_data["Rotation"][1]  # yaw
            velocity = measured_carla_data["Velocity"][0:2]  # [Vx, Vy]
            
            speed = np.linalg.norm(velocity)
            
            # get signal from Logitech Wheel SDK, if pressed the reverse button, then reverse the steering angle
            buttons = controller.get_buttons_pressed()
            if any([btn_value for btn_value in list(buttons.values())[1:]]):
                backward_btn_pressed_cnt += 1
                print("Backward button pressed")
                
            backward = backward_btn_pressed_cnt % 2 == 1

            speed *= -1 if backward else 1
            
            # 2. get steering wheel angle
            steering_wheel_angle = controller.get_angle() * 450.0  # in degrees
            steering_angles = [
                measured_carla_data["FL_Wheel_Angle"],
                measured_carla_data["FR_Wheel_Angle"],
            ]  # front wheel angles

            # # 3. If vehicle enter the DCP zone notice the driver to pressed backward button, then when the button is pressed, plan the path and start the simulation
            # if dist(position_to_world, predefined_path[idx]["P_0"]) < 2 and not take_control:
            #     param = predefined_path[idx]["forward paths"]
            #     # NOTE: Uncomment to control the vehicle to the correct direction
            #     # if abs((90 - vehicle_yaw) - predefined_path["1"]["yaw_0"]) < 5:
            #     #     take_control = True
            #     # else:
            #     #     if (90 - vehicle_yaw) - predefined_path["1"]["yaw_0"] > 0:
            #     #         print("Please turn the vehicle to the right")
            #     #     else:
            #     #         print("Please turn the vehicle to the left")
            #     take_control = True

            # 4. If vehicle enter the DCP zone notice the driver to pressed backward button, then when the button is pressed, plan the path and start the simulation
            if dist(position_to_world, predefined_path[idx]["P_d"]) < 5 and not take_control:
                param = predefined_path[idx]["backward paths"]
                # NOTE: Uncomment to control the vehicle to the correct direction
                # if abs((90 - vehicle_yaw) - predefined_path["1"]["yaw_0"]) < 5:
                #     take_control = True
                # else:
                #     if (90 - vehicle_yaw) - predefined_path["1"]["yaw_0"] > 0:
                #         print("Please turn the vehicle to the right")
                #     else:
                #         print("Please turn the vehicle to the left")
                take_control = True

            # 5. if take control is allowed, then start the self driving
            if take_control:
                # backward velocity
                vehicle_ego.set_target_velocity(carla.Vector3D(x=-1.0, y =0.0, z=0.0))
                
                haptic_control = HapticSharedControl(
                    speed=speed,
                    desired_trajectory_params=param,
                    vehicle_config=vehicle_config,
                    simulation=False
                )
                
                torque, coef, desired_steering_angle_deg = haptic_control.calculate_torque(
                    current_position=position_to_world,
                    current_yaw_angle_deg=vehicle_yaw,
                    steering_angles_deg=steering_angles,
                    steering_wheel_angle_deg=steering_wheel_angle,
                )
                
                desired_steering_angle_deg = np.clip(desired_steering_angle_deg, -450, 450)
                
                try:
                    desired_offset = int(desired_steering_angle_deg * 100 / 450.0)
                except:
                    desired_offset_deg = np.degrees(
                        np.arctan(
                            param["end_y"]
                            - position_to_world[1] / param["end_x"]
                            - position_to_world[0]
                        )
                    )
                    desired_offset = int(desired_offset_deg * 100 / 450.0)
                
                # desired_offset = - 60
                print(f"--> Desired Offset: {desired_offset}")
                controller.play_spring_force(
                    offset_percentage=desired_offset,
                    saturation_percentage=100,
                    coefficient_percentage=100,
                )
                
            else:
                distance_to_SP = dist(position_to_world, predefined_path["1"]["P_0"])
                distance_to_DCP = dist(position_to_world, predefined_path["1"]["P_d"])
                print(
                    f"Distance to SP: {distance_to_SP:3f}m",
                    f" | Distance to DCP: {distance_to_DCP:3f}m",
                )

            # 6. If vehicle reach the final point, stop the program
            if dist(position_to_world, predefined_path["1"]["P_f"]) < 1:
                controller.stop_spring_force()
                print("Simulation Completed")
                
                sys.exit()

            time.sleep(delta_t)
        except Exception as e:
            print("Error: ", repr(e))
            # sys.exit()

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
        controller.exit()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print("\ndone.")
