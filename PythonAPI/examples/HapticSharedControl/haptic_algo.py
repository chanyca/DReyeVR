import matplotlib.pyplot as plt
import numpy as np
from utils import *


class HapticSharedControl:
    debug = True

    def __init__(
        self,
        Cs=0.5,
        Kc=0.5,
        T=1.0,
        tp=1.0,
        speed=2,
        desired_trajectory_params=[],
        vehicle_config={},
    ):
        self.Cs = Cs
        self.Kc = Kc
        self.T = T

        self.tp = tp  # preview time
        self.speed = speed  # vehicle speed
        self.vehicle_config = vehicle_config  # vehicle configuration
        self.vehicle = Vehicle(vehicle_config)

        self.desired_trajectory_params = desired_trajectory_params
        self.desired_trajectory = desired_trajectory_params["path"]

    def calculate_torque(
        self,
        current_position,
        steering_angles_deg,
        current_yaw_angle_deg,
        steering_wheel_angle_deg=None,
    ):
        """
        Calculate the torque for the haptic shared control system.
        Args:
            current_position : tuple or list
                The current position of the vehicle in the form (X, Y) in meters.
            steering_angles_deg : tuple or list
                The steering angles of the front left (FL) and front right (FR) wheels in degrees.
            current_yaw_angle_deg : float
                The current yaw angle of the vehicle in degrees.
            steering_wheel_angle_deg : float, optional
                The steering wheel angle in degrees. If not provided, it is assumed to be the same as the vehicle steering angle.
        Returns:
            tau_das : float
                The calculated torque in Newton-meters.
            coef : float
                The coefficient derived from the sigmoid function based on the error to the trajectory.
            desired_steering_angle_deg : float
                The desired steering wheel angle in degrees.
        """

        if self.debug:
            print(
                "[Measured] Current Position ~ r(t) [X(t), Y(t)] (m):",
                np.array(current_position).round(3),
            )
            print(
                "[Measured] Steering Angles [FL, FR] (degree):",
                np.array(steering_angles_deg).round(3),
            )
            print(
                "[Measured] Current Yaw Angle ~ Phi(t) (degree):",
                np.array(current_yaw_angle_deg).round(3),
            )

        # define inner and outer steering angles
        if steering_angles_deg[0] < 0:
            inner_steering_angle_deg = steering_angles_deg[0]
            outer_steering_angle_deg = steering_angles_deg[1]
        else:
            inner_steering_angle_deg = steering_angles_deg[1]
            outer_steering_angle_deg = steering_angles_deg[0]

        # calculate the average steering angle and turning radius
        self.turning_radius, self.vehicle_steering_angle_deg = self.vehicle.calc_turning_radius(
            inner_steering_angle_deg, outer_steering_angle_deg
        )
        self.vehicle_steering_angle_rad = np.radians(self.vehicle_steering_angle_deg)

        # NOTE: In simulation, use bicycle model to calculate the turning radius
        # suppose the steering wheel angle is the same as the vehicle steering angle
        if not steering_wheel_angle_deg:
            self.steering_wheel_angle_deg = self.vehicle_steering_angle_deg
        else:
            self.steering_wheel_angle_deg = steering_wheel_angle_deg

        if self.debug:
            print(
                "[Input] Steering Wheel Angle ~ Theta(t) (degree):",
                np.array(self.steering_wheel_angle_deg).round(3),
            )
            print("[Vehicle] Turning Radius (m):", np.array(self.turning_radius).round(3))
            print(
                "[Vehicle] Vehicle Steering Angle ~ Delta(t) (degree):",
                np.array(self.vehicle_steering_angle_deg).round(3),
            )

        # Calculate the error of current position to the desired trajectory
        self.e_t = self.distance_to_trajectory(current_position)

        # Calculate the previewed driver model
        self.theta_d_rad = self.preview_driver_model(current_position, current_yaw_angle_deg)

        # Calculate the torque

        tau_das = -(self.Cs * self.e_t) * (self.vehicle_steering_angle_rad - self.theta_d_rad)

        if self.debug:
            print("[Algo] Current Error to Trajectory ~ e(t):", np.array(self.e_t))

            print(
                "[Algo] Predicted Position ~ rtp(t) [Xtp(t), Ytp(t)] (m):",
                np.array(self.predicted_position).round(3),
            )
            print("[Algo] - Delta Phi (degree):", np.degrees(self.delta_phi_rad).round(3))
            print("[Algo] - Turning Angle (degree):", np.array(self.turning_angle_deg).round(3))
            print("[Algo] - Rotating Angle (degree):", np.array(self.rotating_angle_deg).round(3))
            print(
                "[Algo] - Center of Rotation with world (m):",
                np.array(self.center_of_rotation_with_world).round(3),
            )

            print("[Algo] Predicted Error to Trajectory ~ eps_tp(t):", self.epsilon_tp_t)
            print(
                "[Algo] Desired Steering Wheel Angle ~ Theta_d(t) (degree):",
                np.degrees(self.theta_d_rad).round(3),
            )

            print("[Output] Torque ~ Tau_das (N.m):", tau_das)

        coef = sigmoid(self.Cs * self.e_t)
        desired_steering_angle_deg = np.degrees(self.theta_d_rad)
        return tau_das, coef, desired_steering_angle_deg

    def preview_driver_model(self, current_position, current_yaw_angle_deg, method="simple"):
        """
        Predicts the desired steering angle based on the current position and yaw angle of the vehicle.
        Args:
            current_position (tuple): The current position of the vehicle as (x, y) coordinates.
            current_yaw_angle_deg (float): The current yaw angle of the vehicle in degrees.
            method (str, optional): The method to use for calculating the desired steering angle.
                                    Options are "simple" or "complex". Defaults to "simple".
        Returns:
            float: The desired steering angle in radians.
        """

        # Predict the position of the vehicle at the preview time
        self.predicted_position = self.predict_position(current_position, current_yaw_angle_deg)

        # Calculate the error between desired trajectory and predicted position with tp[s] ahead
        self.epsilon_tp_t = self.distance_to_trajectory(
            self.predicted_position
        ) * self.get_sign_of_error(self.predicted_position)

        # Calculate the desired steering angle
        if method == "simple":
            theta_d = self.Kc * self.epsilon_tp_t - self.vehicle_steering_angle_rad
        else:
            theta_d_curr = self.vehicle_steering_angle_rad
            theta_d_next = None
            delta_t = 1 / 100  # 100 Hz
            for _ in range(100):
                theta_d_next = (
                    self.Kc * self.epsilon_tp_t + ((self.T / delta_t) - 1) * theta_d_curr
                ) * (delta_t / self.T)
                theta_d_curr = theta_d_next
            theta_d = theta_d_next

        return theta_d

    def predict_position(self, current_position, current_yaw_angle_deg):
        """
        Predicts the future position of the vehicle based on the current position and yaw angle.
        Args:
            current_position (array-like): The current position of the vehicle in world coordinates [x, y].
            current_yaw_angle_deg (float): The current yaw angle of the vehicle in degrees.
        Returns:
            numpy.ndarray: The predicted position of the vehicle in world coordinates [x, y].
        """

        current_yaw_angle_rad = np.radians(current_yaw_angle_deg)

        center_of_mass = self.vehicle.center_of_mass_ratio * self.vehicle.wheelbase

        # vehicle coordinates, origin at the center of mass
        rotating_direction = -np.sin(self.vehicle_steering_angle_rad) / np.abs(
            np.sin(self.vehicle_steering_angle_rad)
        )  #  -1 if clockwise, 1 if counter-clockwise
        center_of_rotation_with_vehicle = np.array(
            [
                -self.turning_radius * np.cos(self.vehicle_steering_angle_rad),
                -center_of_mass,
            ]
        )

        # print("[Vehicle] Center of Rotation in Vehicle:", center_of_rotation_with_vehicle)

        # transform the vehicle coordinates to the global coordinates, rotate and shift
        self.center_of_rotation_with_world = rotation_matrix_cw(
            current_yaw_angle_rad - np.pi / 2
        ).dot(center_of_rotation_with_vehicle) + np.array(current_position)

        # changes of vehicle angle
        self.turning_angle_rad = (
            -1 * self.vehicle_steering_angle_rad + current_yaw_angle_rad
        )
        self.turning_angle_deg = np.degrees(self.turning_angle_rad)

        # rotating around the center of rotation
        self.rotating_angle_rad = self.turning_angle_rad - np.pi / 2
        self.rotating_angle_deg = np.degrees(self.rotating_angle_rad)

        # calculate the predicted position
        self.delta_phi_rad = rotating_direction * self.speed * self.tp / self.turning_radius
        # self.delta_phi_rad = - self.angular_speed * self.tp
        self.delta_phi_deg = np.degrees(self.delta_phi_rad)

        self.predict_yaw_angle_deg = (
            current_yaw_angle_deg + rotating_direction * self.delta_phi_deg
        )

        predicted_position_in_world = (
            self.center_of_rotation_with_world
            + self.turning_radius
            * np.array(
                [
                    np.cos(self.rotating_angle_rad + rotating_direction * self.delta_phi_rad),
                    np.sin(self.rotating_angle_rad + rotating_direction * self.delta_phi_rad),
                ]
            )
        )
        # print(self.dist(current_position, predicted_position_in_world))

        return predicted_position_in_world

    def distance_to_trajectory(self, position):
        """
        Calculate the distance from a given position to the desired trajectory.
        This method finds the closest point on the desired trajectory to the given position
        and calculates the distance between them. It also prints the index of the closest point
        for debugging purposes.
        Args:
            position (tuple): The (x, y) coordinates of the current position.
        Returns:
            float: The distance from the given position to the closest point on the desired trajectory.
        """

        # More robust distance calculation by checking multiple points
        min_dist = float("inf")
        closest_point, idx = find_closest_point(position, self.desired_trajectory)
        print(f"[Algo] Closest Point Index of  {position}:", idx)

        return min(dist(position, closest_point), min_dist)

    def get_sign_of_error(self, position):

        p1 = position
        _, pt_index = find_closest_point(p1, self.desired_trajectory)

        p2, p3 = self.desired_trajectory_params["tangent"][pt_index]
        angle_rad = getAngle(p1, p2, p3, degrees=False)

        return -np.sin(angle_rad) / abs(np.sin(angle_rad))


if __name__ == "__main__":
    import json
    from pathlib import Path

    from bezier_path import *

    # cspell: ignore bezier arctan xlabel figsize ylabel

    __file_path__ = Path(__file__).resolve().parent

    # Load the vehicle configuration
    with open(f"{__file_path__}/wheel_setting.json", "r") as f:
        vehicle_config = json.load(f)
    vehicle = Vehicle(vehicle_config=vehicle_config)
    R = vehicle.minimum_turning_radius
    print("Minimum Turning Radius:", R, "m")

    # define initial and final points
    P_0 = [-1.47066772, -13.22415039]  # [x, y] in carla -> [y, x] in matplotlib
    P_f = [-6.87066772, -21.62415039]
    P_d = [-0.37066772, -29.02415039]

    yaw_0 = 0
    yaw_d = 10
    yaw_f = 90

    # calculate the bezier path
    print("Calculating Bezier Path")

    n_points = 10
    path1, control_points1, params1 = calculate_bezier_trajectory(
        start_pos=P_0[::-1],
        end_pos=P_d[::-1],
        start_yaw=yaw_0,
        end_yaw=yaw_d,
        n_points=n_points,
        turning_radius=R,
        show_animation=False,
    )
    # backward so reverse the yaw angle (+180)
    path2, control_points2, params2 = calculate_bezier_trajectory(
        start_pos=P_d[::-1],
        end_pos=P_f[::-1],
        start_yaw=180 + yaw_d,
        end_yaw=180 + yaw_f,
        n_points=n_points,
        turning_radius=R,
        show_animation=False,
    )

    # calculate the control loop
    path = path2
    param = params2

    i_points = P_d[::-1]
    f_points = P_f[::-1]
    i_yaw = 180 - yaw_d

    # Simulation Parameters
    n_steps = 3
    speed = -2  # m/s
    init_steering_angle = -10  # degrees
    current_position = i_points

    # Haptic Shared Control Initialization
    haptic_control = HapticSharedControl(
        Cs=0.5,
        Kc=0.5,
        T=2,
        tp=1,
        speed=speed,
        desired_trajectory_params=param,
        vehicle_config=vehicle_config,
    )

    # Simulation Loop
    print("Starting Simulation")

    s_angleL = init_steering_angle
    s_angleR = init_steering_angle

    trajectory = [current_position]
    torques = []
    steering_angles_deg = [init_steering_angle]
    vehicle_angles_deg = [i_yaw]

    offset_percentage = []
    cows = []
    turning_angles_deg = [init_steering_angle + i_yaw]  # if change sign of swa, change sign here

    # Calculate the torque at each step
    for i in range(n_steps):
        print("--------------------")
        print("Step:", i)

        torque, coef, desired_steering_angle_deg = haptic_control.calculate_torque(
            current_position=current_position,
            steering_angles_deg=[s_angleL, s_angleR],
            current_yaw_angle_deg=i_yaw,
        )

        next_position = haptic_control.predicted_position

        torques.append(torque)
        trajectory.append(next_position)

        steering_angles_deg.append(desired_steering_angle_deg)
        turning_angles_deg.append(haptic_control.turning_angle_deg)
        vehicle_angles_deg.append(
            np.degrees(
                np.arctan(
                    (next_position[1] - current_position[1])
                    / (next_position[0] - current_position[0])
                )
                + np.pi
            )
        )
        cows.append(haptic_control.center_of_rotation_with_world)

        # s_angleL = np.clip(desired_steering_angle_deg, -47.95, 69.99)
        # s_angleR = np.clip(desired_steering_angle_deg, -69.99, 47.95)
        s_angleL = desired_steering_angle_deg
        s_angleR = desired_steering_angle_deg

        i_yaw = haptic_control.predict_yaw_angle_deg
        current_position = next_position

    # Extract trajectory points
    trajectory = np.array(trajectory)
    x_points = trajectory[:, 0]
    y_points = trajectory[:, 1]

    # Plot the trajectory
    plt.figure()

    plt.plot(path[:, 0], path[:, 1], "--", label="Bezier Path")
    for i, point in enumerate(zip(path[:, 0], path[:, 1])):
        plt.plot(point[0], point[1], "o", color="blue")
    # plt.plot(path[0, 0], path[0, 1], "o", label="Start", color="blue")
    plt.plot(path[-1, 0], path[-1, 1], "o", label="End", color="green")

    # Plot the vehicle trajectory
    plt.plot(x_points, y_points, label="Predicted Path", color="red")
    for i, point in enumerate(zip(x_points, y_points)):
        plt.plot(point[0], point[1], "o", color="red")
        plot_arrow(point[0], point[1], np.radians(turning_angles_deg[i]), 0.3, width=0.2)
        # if i >= 1:
        #     plt.plot(cows[i - 1][0], cows[i - 1][1], "o", color="black")
        # plot_arrow(point[0], point[1], np.radians(vehicle_angles_deg[i]), 0.3, width=0.2)
    print(turning_angles_deg)
    print(vehicle_angles_deg)

    plt.xlabel("Y")
    plt.ylabel("X")
    plt.title("Vehicle Path and Bezier Trajectory")

    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.axis("square")
    plt.show()

    # # plt.figure(figsize=(10, 6))
    # plt.plot(range(n_steps), torques, label="Torque")
    # plt.xlabel("Time (s)")
    # plt.ylabel("Torque (Nm)")
    # plt.title("Torque Feedback")
    # plt.grid(True)
    # plt.show()

    # # plt.figure(figsize=(10, 6))
    # plt.plot(range(n_steps), steering_angles_deg[:-1], label="Steering Angle")
    # plt.xlabel("Time (s)")
    # plt.ylabel("Steering angle (rad)")
    # plt.title("Steering angle")
    # plt.grid(True)
    # plt.show()
