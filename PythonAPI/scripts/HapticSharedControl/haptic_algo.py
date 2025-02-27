import time
from pprint import pprint

import matplotlib.pyplot as plt
import numpy as np

from .utils import *


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
        log: bool = False,
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

        self.log = log
        self.log_data = {
            "Time (t)": [],
            "[Measured] Current Position (m)": {
                "X": [],
                "Y": [],
            },
            "[Measured] Steering Angles (degree)": {
                "FL": [],
                "FR": [],
            },
            "[Measured] Current Yaw Angle ~ Phi(t) (degree)": [],
            "[Measured] Steering Wheel Angle ~ Theta(t) (degree)": [],
            "[Computed] Turning Radius ~ R(delta) (m)": [],
            "[Computed] Vehicle Steering Angle ~ Delta(t) (degree)": [],
            "[Computed] Current Error to Trajectory ~ e(t)": [],
            "[Computed] Predicted Position (m)": {
                "X": [],
                "Y": [],
            },
            "[Computed] Change of Yaw Angle ~ Delta_phi(t) (degree)": [],
            "[Computed] Center of Rotation to WORLD (m)": [],
            "[Computed] Predicted Yaw Angle ~ Phi_tp(t) (degree)": [],
            "[Computed] Predicted Error to Trajectory ~ eps_tp(t)": [],
            "[Computed] Desired Steering Wheel Angle ~ Theta_d(t) (degree)": [],
            "[Computed] Torque applied ~ Tau_das (N.m)": [],
        }

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

        self.r = current_position
        self.deltas = steering_angles_deg
        self.phi = current_yaw_angle_deg

        # calculate the average steering angle and turning radius
        self.turning_radius, self.vehicle_turning_angle_deg = self.vehicle.calc_turning_radius(
            steering_angles_deg
        )
        self.vehicle_turning_angle_rad = np.radians(self.vehicle_turning_angle_deg)

        # NOTE: In simulation, use bicycle model to calculate the turning radius
        # suppose the steering wheel angle is the same as the vehicle steering angle
        self.vehicle_steering_angle_deg = (
            self.vehicle.inner_wheel_steering_angle_deg
            + self.vehicle.outer_wheel_steering_angle_deg
        ) / 2
        self.vehicle_steering_angle_rad = np.radians(self.vehicle_steering_angle_deg)

        if not steering_wheel_angle_deg:
            self.steering_wheel_angle_deg = self.translate_sa_to_swa(
                self.vehicle_steering_angle_deg
            )
        else:
            self.steering_wheel_angle_deg = steering_wheel_angle_deg

        # Calculate the error of current position to the desired trajectory
        self.e_t = self.distance_to_trajectory(current_position)

        # Calculate the previewed driver model
        self.theta_d_rad = self.preview_driver_model(current_position, current_yaw_angle_deg)

        # Calculate the torque

        self.tau_das = -(self.Cs * self.e_t) * (self.vehicle_steering_angle_rad - self.theta_d_rad)

        coef = sigmoid(self.Cs * self.e_t)
        desired_steering_angle_deg = np.degrees(self.theta_d_rad)

        self.logging()

        return self.tau_das, coef, desired_steering_angle_deg

    def preview_driver_model(self, current_position, current_yaw_angle_deg, method="complex"):
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
        self.rtp = self.predict_position(current_position, current_yaw_angle_deg)

        # Calculate the error between desired trajectory and predicted position with tp[s] ahead
        self.epsilon_tp_t = self.distance_to_trajectory(self.rtp) * self.get_sign_of_error(
            self.rtp
        )

        # Calculate the desired steering angle
        if method == "simple":
            theta_d = self.Kc * self.epsilon_tp_t + self.vehicle_steering_angle_rad
        else:
            theta_d_curr = self.vehicle_steering_angle_rad
            theta_d_next = None
            update_frequency = 100  # 100 Hz
            delta_t = 1 / update_frequency  # 0.01 s
            for _ in range(update_frequency):
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
        self.rotating_direction = np.sin(self.vehicle_steering_angle_rad) / np.abs(
            np.sin(self.vehicle_steering_angle_rad)
        )  #  -1 if clockwise, 1 if counter-clockwise

        center_of_rotation_to_vehicle = np.array(
            [
                self.vehicle.wheelbase / np.tan(self.vehicle_steering_angle_rad),
                -center_of_mass,
            ]
        )

        # transform the vehicle coordinates to the global coordinates, rotate and shift
        self.center_of_rotation_to_world = rotation_matrix_cw(
            current_yaw_angle_rad - np.pi / 2
        ).dot(center_of_rotation_to_vehicle) + np.array(current_position)

        # calculate the predicted position
        self.delta_phi_rad = self.rotating_direction * self.speed * self.tp / self.turning_radius
        # self.delta_phi_rad = - self.angular_speed * self.tp
        self.delta_phi_deg = np.degrees(self.delta_phi_rad)

        self.predict_yaw_angle_rad = current_yaw_angle_rad - self.delta_phi_rad
        self.predict_yaw_angle_deg = np.degrees(self.predict_yaw_angle_rad)

        predicted_position_in_world = (
            self.center_of_rotation_to_world
            + self.turning_radius
            * np.array(
                [
                    np.cos(
                        self.predict_yaw_angle_rad
                        - self.vehicle_turning_angle_rad
                        + self.rotating_direction * np.pi / 2
                    ),
                    np.sin(
                        self.predict_yaw_angle_rad
                        - self.vehicle_turning_angle_rad
                        + self.rotating_direction * np.pi / 2
                    ),
                ]
            )
        )

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

        return -(self.speed / np.abs(self.speed)) * np.sin(angle_rad) / abs(np.sin(angle_rad))

    def translate_sa_to_swa(self, sa_deg):
        """
        Translate the steering angle to the steering wheel angle.
        Args:
            sa_deg (float): The steering angle in degrees.
        Returns:
            float: The steering wheel angle in degrees.
        """
        return sa_deg

    def translate_swa_to_sa(self, swa_deg):
        """
        Translate the steering wheel angle to the steering angle.
        Args:
            swa_deg (float): The steering wheel angle in degrees.
        Returns:
            float: The steering angle in degrees.
        """
        return swa_deg

    def logging(self):
        """
        Log the data for debugging purposes.
        """

        self.log_data["Time (t)"].append(time.time())
        self.log_data["[Measured] Current Position (m)"]["X"].append(self.r[0])
        self.log_data["[Measured] Current Position (m)"]["Y"].append(self.r[1])

        self.log_data["[Measured] Steering Angles (degree)"]["FL"].append(self.deltas[0])
        self.log_data["[Measured] Steering Angles (degree)"]["FR"].append(self.deltas[1])
        self.log_data["[Measured] Current Yaw Angle ~ Phi(t) (degree)"].append(self.phi)
        self.log_data["[Measured] Steering Wheel Angle ~ Theta(t) (degree)"].append(
            self.steering_wheel_angle_deg
        )
        self.log_data["[Computed] Turning Radius ~ R(delta) (m)"].append(self.turning_radius)
        self.log_data["[Computed] Vehicle Steering Angle ~ Delta(t) (degree)"].append(
            self.vehicle_turning_angle_deg
        )
        self.log_data["[Computed] Current Error to Trajectory ~ e(t)"].append(self.e_t)
        self.log_data["[Computed] Predicted Position (m)"]["X"].append(self.rtp[0])
        self.log_data["[Computed] Predicted Position (m)"]["Y"].append(self.rtp[1])
        self.log_data["[Computed] Change of Yaw Angle ~ Delta_phi(t) (degree)"].append(
            self.delta_phi_deg
        )
        self.log_data["[Computed] Center of Rotation to WORLD (m)"].append(
            self.center_of_rotation_to_world
        )
        self.log_data["[Computed] Predicted Yaw Angle ~ Phi_tp(t) (degree)"].append(
            self.predict_yaw_angle_deg
        )
        self.log_data["[Computed] Predicted Error to Trajectory ~ eps_tp(t)"].append(
            self.epsilon_tp_t
        )
        self.log_data["[Computed] Desired Steering Wheel Angle ~ Theta_d(t) (degree)"].append(
            self.theta_d_rad
        )
        self.log_data["[Computed] Torque applied ~ Tau_das (N.m)"].append(self.tau_das)

        if self.debug:
            print("Logging data...")
            keys = list(self.log_data.keys())
            values = list(self.log_data.values())

            for i in range(len(keys)):
                if type(values[i]) == dict:
                    print(f"{keys[i]}:")
                    for sub_key, sub_value in values[i].items():
                        print(f"    {sub_key}: {sub_value[-1]}")
                else:
                    print(f"{keys[i]}: {(values[i][-1])}")

        if self.log:
            self.save_log()

    def save_log(self):
        """
        Save the log data to a CSV file.
        """
        with open("haptic_shared_control_log.csv", "a") as f:
            for key, value in self.log_data.items():
                f.write("%s,%s\n" % (key, value))
        pass


if __name__ == "__main__":
    pass
