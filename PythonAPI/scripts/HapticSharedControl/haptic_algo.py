import csv
import datetime
import os
import time
from pprint import pprint

import matplotlib.pyplot as plt
import numpy as np

from .utils import *

__current_time__ = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")


def get_sign_of_error(given_position, desired_trajectory=None, tangent_vectors=None, velocity=None):
    """Get sign of epsilon error.
    This method calculates the sign of the error between the current position and the desired trajectory.
    
    Args:
        position (numpy.array): x, y coordinate of the current position.
        desired_trajectory (numpy.ndarray): x, y coordinates of the desired trajectory. [[x1, y1], [x2, y2], ...]
        tangent_vectors (numpy.ndarray): tangent vectors of the desired trajectory. [[[x1, y1], [x2, y2]], [[x2, y2], [x3, y3]], ...]
        velocity (float): The speed of the vehicle.

    Returns:
        _type_: _description_
    """
    # cspell: ignore ndarray
    p1 = given_position
    _, pt_index = find_closest_point(p1, desired_trajectory)

    p2, p3 = tangent_vectors[pt_index]
    angle_rad = getAngle(p1, p2, p3, degrees=False)

    return sign(velocity) * (np.sin(angle_rad) / abs(np.sin(angle_rad)))


def distance_to_trajectory(given_position, desired_trajectory=None):
    """
    Calculate the distance from a given position to the desired trajectory.
    This method finds the closest point on the desired trajectory to the given position
    and calculates the distance between them. 
    It also prints the index of the closest point for debugging purposes.
    Args:
        position (tuple): The (x, y) coordinates of the current position.
        desired_trajectory (numpy.ndarray): The desired trajectory represented as a 2D array of shape (n_points, 2).
            Each row represents a point in the trajectory with (x, y) coordinates.
    Returns:
        tuple: A tuple containing:
            - distance (float): The distance from the given position to the desired trajectory.
            - closest_point (numpy.ndarray): The closest point on the desired trajectory to the given position.
            - index (int): The index of the closest point in the desired trajectory.
    """

    # More robust distance calculation by checking multiple points
    min_dist = float("inf")
    closest_point, idx = find_closest_point(given_position, desired_trajectory)
    return min(dist(given_position, closest_point), min_dist), closest_point, idx

def predict_position(
    current_position: list,
    current_yaw_angle_deg: float,
    vehicle_steering_angle_deg: float = None,   
    center_of_rotation_to_vehicle: list = None,
    turning_radius: float = None,
    velocity: float = None,
    tp: float = None,    
) -> list:
    """
    Predict the position of the vehicle at a given time in the future based on its current state.
    The prediction is based on the vehicle's current position, yaw angle, steering angle, and turning radius.
    The function also considers the vehicle's velocity and the time preview (tp) for the prediction.
    
    Args:
        current_position (list): The current position of the vehicle as [x, y].
        current_yaw_angle_deg (float): The current yaw angle of the vehicle in degrees.
        vehicle_steering_angle_deg (float, optional): The current steering angle of the vehicle in degrees.
            If not provided, it will be calculated based on the vehicle steering angle.            
        center_of_rotation_to_vehicle (list, optional): The center of rotation to the vehicle as [x, y].
            If not provided, it will be calculated based on the vehicle configuration.
        turning_radius (float, optional): The turning radius of the vehicle. If not provided, it will be calculated based on the vehicle configuration.
        velocity (float, optional): The speed of the vehicle. If not provided, it will be set to 0.
        tp (float, optional): The time preview for the prediction. If not provided, it will be set to 0.
    
    Returns:
        dict: A dictionary containing the following:
        - predicted_position (list): The predicted position of the vehicle as [x, y].
        - rotating_direction (int): The direction of rotation (-1 for clockwise, 1 for counter-clockwise).
        - delta_phi_rad (float): The change of yaw angle in radians.
        - predict_yaw_angle_rad (float): The predicted yaw angle in radians.
        - center_of_rotation_to_world (list): The center of rotation in world coordinates as [x, y].
    """    
    
    current_yaw_angle_rad = np.radians(current_yaw_angle_deg)
    vehicle_steering_angle_rad = np.radians(vehicle_steering_angle_deg)
    
    # *0 First calculate the center of rotation to the vehicle
    # convert carla coordinates to popular [x, y]
    current_yaw_angle_rad = np.pi / 2 - current_yaw_angle_rad
    current_position = np.array(current_position)[::-1]
    center_of_rotation_to_vehicle = np.array(center_of_rotation_to_vehicle)[::-1]
    # [
    #     a2 / np.tan(delta_v),
    #     -a2,
    # ]
    
    # define rotation direction for the vehicle
    rotating_direction = sign(np.sin(vehicle_steering_angle_rad))
    #  1 if clockwise, -1 if counter-clockwise for positive forward velocity
    
    # *1 transform the vehicle coordinates to the global coordinates
    # -- rotate current_yaw_angle_rad - np.pi / 2
    # -- shift with vector current_position
    # convert carla coordinates to popular [x, y]
    center_of_rotation_to_world = rotation_matrix_cw(
        (current_yaw_angle_rad) - np.pi / 2
    ).dot(center_of_rotation_to_vehicle) + np.array(current_position)
    
    # *2 calculate the predicted position
    # -- calculate the delta_phi_rad
    # -- calculate the predicted yaw angle
    # -- calculate the predicted position

    delta_phi_rad = rotating_direction * velocity * tp / turning_radius

    predict_yaw_angle_rad = current_yaw_angle_rad - delta_phi_rad

    predicted_position_in_world = (
        center_of_rotation_to_world
        + turning_radius
        * np.array(
            [
                np.cos(
                    predict_yaw_angle_rad
                    - vehicle_steering_angle_rad
                    + rotating_direction * np.pi / 2
                ),
                np.sin(
                    predict_yaw_angle_rad
                    - vehicle_steering_angle_rad
                    + rotating_direction * np.pi / 2
                ),
            ]
        )
    )
    
    # *3 convert back to carla coordinates
    predict_yaw_angle_rad = np.pi / 2 - predict_yaw_angle_rad 
    predicted_position_in_world = predicted_position_in_world[::-1]
    center_of_rotation_to_world = center_of_rotation_to_world[::-1]

    return {
            "predicted_position": predicted_position_in_world,
            "rotating_direction": rotating_direction,
            "delta_phi_rad": delta_phi_rad,
            "predict_yaw_angle_rad": predict_yaw_angle_rad,
            "center_of_rotation_to_world": center_of_rotation_to_world,
            }

# === HapticSharedControl Class ===
# This class implements the Haptic Shared Control algorithm for vehicle steering.
# It calculates the torque to be applied to the steering wheel based on the current position,
# steering angles, and yaw angle of the vehicle.
# The algorithm uses a preview driver model to predict the desired steering angle and
# calculates the torque based on the error between the current and desired steering angles.
# It also logs the data for debugging purposes.
# The class is designed to be used in a simulation environment, but can also be adapted for real-world applications.

log_dict = {
    "Time (t)": [],
    "[Measured] Current Position ~ r(t) (m)": {
        "X": [],
        "Y": [],
    },
    "[Measured] Steering Angles ~ delta(t) (deg)": {
        "FL": [],
        "FR": [],
    },
    "[Measured] Current Yaw Angle ~ Phi(t) (deg)": [],
    "[Measured] Steering Wheel Angle ~ Theta(t) (deg)": [],
    "[Computed] Rotation direction ~ (CW/CCW)" : [],
    "[Computed] Turning Radius ~ R(delta) (m)": [],
    "[Computed] Vehicle Steering Angle ~ Delta(t) (deg)": [],
    "[Computed] Center of Rotation to WORLD ~ r_c(t) (m)": {
        "X": [],
        "Y": [],
    },
    "[Computed] Current Error to Trajectory ~ e(t)": [],
    "[Computed] Predicted Position ~ r_tp(t) (m)": {
        "X": [],
        "Y": [],
    },
    "[Computed] Change of Yaw Angle ~ Delta_phi(t) (deg)": [],
    "[Computed] Predicted Yaw Angle ~ Phi_tp(t) (deg)": [],
    "[Computed] Predicted Error to Trajectory ~ eps_tp(t)": [],
    "[Computed] Desired Steering Wheel Angle ~ Theta_d(t) (deg)": [],
    "[Computed] Torque applied ~ Tau_das (N.m)": [],
}

class HapticSharedControl:
    debug = True

    def __init__(
        self,
        Cs=0.5,
        Kc=0.5,
        T=1.0,
        tp=1.0,
        speed=1.0,
        desired_trajectory_params=[],
        vehicle_config={},
        log: bool = True,
        simulation: bool = True,
    ):
        self.Cs = Cs
        self.Kc = Kc
        self.T = T

        self.tp = tp  # preview time
        self.speed = speed  # vehicle speed
        self.vehicle_config = vehicle_config  # vehicle configuration
        self.vehicle = Vehicle(vehicle_config)

        self.desired_trajectory_params = desired_trajectory_params

        self.log = log
        self.simulation = simulation

        self.log_data = log_dict.copy()

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
            current_position (tuple): The current position of the vehicle (x, y).
            steering_angles_deg (float): The current steering angle of the vehicle in degrees.
            current_yaw_angle_deg (float): The current yaw angle of the vehicle in degrees.
            steering_wheel_angle_deg (float, optional): The current steering wheel angle in degrees. 
                If not provided, it will be calculated based on the vehicle steering angle.
        Returns:
            tuple: A tuple containing:
                - tau_das (float): The calculated torque.
                - coef (float): The coefficient based on the sigmoid function.
                - desired_steering_wheel_angle_deg (float): The desired steering wheel angle in degrees.
        """

        self.r = current_position  # measured position from carla
        self.phi = current_yaw_angle_deg  # measured yaw angle from carla
        self.deltas = steering_angles_deg  # measured steering angles from carla
        
        # *1 calculate the average steering angle and turning radius
        (
            self.turning_radius,
            self.vehicle_steering_angle_deg,
            self.center_of_rotation_to_vehicle,
        ) = self.vehicle.calc_turning_radius(steering_angles=self.deltas).values()
        
        self.vehicle_steering_angle_rad = np.radians(self.vehicle_steering_angle_deg)

        # Check if the steering wheel angle is provided, otherwise use the vehicle steering angle
        if steering_wheel_angle_deg is None:
            self.steering_wheel_angle_deg = self.translate_sa_to_swa(
                self.vehicle_steering_angle_deg
            )
        else:
            self.steering_wheel_angle_deg = steering_wheel_angle_deg

        self.steering_wheel_angle_rad = np.radians(self.steering_wheel_angle_deg)

        # *2 Calculate the error of current position to the desired trajectory
        self.e_t, self.closest_pt_rt, self.closest_pt_rt_idx = distance_to_trajectory(
            given_position=current_position,
            desired_trajectory=self.desired_trajectory_params["path"]
        )

        # *3 Calculate the previewed driver model
        self.theta_d_rad = self.preview_driver_model(current_position=self.r, 
                                                     current_yaw_angle_deg=self.phi, 
                                                     current_vehicle_steering_angle_deg=self.vehicle_steering_angle_deg)
        
        self.theta_d_deg = np.degrees(self.theta_d_rad)

        # *4 Calculate the torque
        self.tau_das = -(self.Cs * self.e_t) * (self.steering_wheel_angle_rad - self.theta_d_rad)
        # Calculate the coefficient based on the sigmoid function
        coef = sigmoid(self.Cs * self.e_t)
        desired_steering_wheel_angle_deg = self.theta_d_deg

        # Log the data for debugging
        self.logging()

        return self.tau_das, coef, desired_steering_wheel_angle_deg

    def preview_driver_model(self, current_position, current_yaw_angle_deg, current_vehicle_steering_angle_deg, method="simple"):
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

        # *3.1 Predict the position of the vehicle at the preview time
        (
            self.rtp, 
            self.rotating_direction, 
            self.delta_phi_rad, 
            self.predict_yaw_angle_rad, 
            self.center_of_rotation_to_world
        ) = predict_position(
            current_position=current_position,
            current_yaw_angle_deg=current_yaw_angle_deg,
            vehicle_steering_angle_deg=current_vehicle_steering_angle_deg,
            center_of_rotation_to_vehicle=self.center_of_rotation_to_vehicle,
            turning_radius=self.turning_radius,
            velocity=self.speed,
            tp=self.tp,
        ).values()
        
        self.delta_phi_deg = np.degrees(self.delta_phi_rad)
        self.predict_yaw_angle_deg = np.degrees(self.predict_yaw_angle_rad)
        
        
        # *3.2 Calculate the error between desired trajectory and predicted position with tp[s] ahead
        # NOTE: check the sign of the error fn
        epsilon_tp_t, self.closest_pt_rtp, self.closest_pt_rtp_idx = distance_to_trajectory(
            given_position=self.rtp,
            desired_trajectory=self.desired_trajectory_params["path"]
        )
        
        self.epsilon_tp_t = get_sign_of_error(given_position=self.rtp,
                                              desired_trajectory=self.desired_trajectory_params["path"],
                                              tangent_vectors=self.desired_trajectory_params["tangent"],
                                              velocity=self.speed,
                                              ) * epsilon_tp_t

        # *3.3 Calculate the desired steering angle
        if method == "simple":
            theta_d_rad = self.Kc * self.epsilon_tp_t + self.steering_wheel_angle_rad
        else:
            # ?... development in progress
            theta_d_rad_curr = self.steering_wheel_angle_rad
            theta_d_rad_next = None
            update_frequency = 60  # 100 Hz
            d_t = 1 / update_frequency  # 0.01 s
            for _ in range(update_frequency):
                theta_d_rad_next = (
                    self.Kc * self.epsilon_tp_t + ((self.T / d_t) - 1) * theta_d_rad_curr
                ) * (d_t / self.T)
                theta_d_rad_curr = theta_d_rad_next
            theta_d_rad = theta_d_rad_next

        return theta_d_rad

    def translate_sa_to_swa(self, sa_deg):
        """
        Translate the steering angle to the steering wheel angle.
        Args:
            sa_deg (float): The steering angle in degrees.
        Returns:
            float: The steering wheel angle in degrees.
        """
        if not self.simulation:
            return linear_fn(slope=28.809413448185634, 
                             intercept=0.00036213148264344503)(sa_deg)
        return linear_fn(1, 0)(sa_deg)

    def translate_swa_to_sa(self, swa_deg):
        """
        Translate the steering wheel angle to the steering angle.
        Args:
            swa_deg (float): The steering wheel angle in degrees.
        Returns:
            float: The steering angle in degrees.
        """
        if not self.simulation:
            return linear_fn(slope=0.03466993588625678, 
                             intercept=1.8142561990902186e-05)(swa_deg)
        return linear_fn(1, 0)(swa_deg)

    def logging(self):
        """
        Log the data for debugging purposes.
        """
        print("----------------------")
        self.log_data["Time (t)"].append(time.time())
        self.log_data["[Measured] Current Position ~ r(t) (m)"]["X"].append(self.r[0])
        self.log_data["[Measured] Current Position ~ r(t) (m)"]["Y"].append(self.r[1])

        self.log_data["[Measured] Steering Angles ~ delta(t) (deg)"]["FL"].append(self.deltas[0])
        self.log_data["[Measured] Steering Angles ~ delta(t) (deg)"]["FR"].append(self.deltas[1])
        self.log_data["[Measured] Current Yaw Angle ~ Phi(t) (deg)"].append(self.phi)
        self.log_data["[Measured] Steering Wheel Angle ~ Theta(t) (deg)"].append(
            self.steering_wheel_angle_deg
        )
        self.log_data["[Computed] Rotation direction ~ (CW/CCW)"].append(self.rotating_direction)
        self.log_data["[Computed] Turning Radius ~ R(delta) (m)"].append(self.turning_radius)
        self.log_data["[Computed] Vehicle Steering Angle ~ Delta(t) (deg)"].append(
            self.vehicle_steering_angle_deg
        )
        self.log_data["[Computed] Current Error to Trajectory ~ e(t)"].append(self.e_t)
        self.log_data["[Computed] Predicted Position ~ r_tp(t) (m)"]["X"].append(self.rtp[0])
        self.log_data["[Computed] Predicted Position ~ r_tp(t) (m)"]["Y"].append(self.rtp[1])

        self.log_data["[Computed] Change of Yaw Angle ~ Delta_phi(t) (deg)"].append(
            self.delta_phi_deg
        )

        self.log_data["[Computed] Center of Rotation to WORLD ~ r_c(t) (m)"]["X"].append(
            self.center_of_rotation_to_world[0]
        )
        self.log_data["[Computed] Center of Rotation to WORLD ~ r_c(t) (m)"]["Y"].append(
            self.center_of_rotation_to_world[1]
        )

        self.log_data["[Computed] Predicted Yaw Angle ~ Phi_tp(t) (deg)"].append(
            self.predict_yaw_angle_deg
        )
        self.log_data["[Computed] Predicted Error to Trajectory ~ eps_tp(t)"].append(
            self.epsilon_tp_t
        )

        self.log_data["[Computed] Desired Steering Wheel Angle ~ Theta_d(t) (deg)"].append(
            self.theta_d_deg
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
        
        The method creates a CSV file if it doesn't exist, writing headers only once.
        It then appends the latest data point to the file each time it's called.
        """
        file_path = f"./logs/haptic_shared_control_log_{__current_time__}.csv"
        file_exists = os.path.isfile(file_path)
        
        # Prepare the data structure
        # For nested dictionaries, flatten the structure for CSV format
        headers = []
        row_data = {}
        
        for key, value in self.log_data.items():
            if isinstance(value, dict):
                for sub_key, sub_value in value.items():
                    column_name = f"{key}_{sub_key}"
                    headers.append(column_name)
                    # Get the last value if available
                    if sub_value and len(sub_value) > 0:
                        row_data[column_name] = sub_value[-1]
                    else:
                        row_data[column_name] = None
            else:
                headers.append(key)
                # Get the last value if available
                if value and len(value) > 0:
                    row_data[key] = value[-1]
                else:
                    row_data[key] = None
        
        # Write to file
        with open(file_path, 'a', newline='') as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=headers)
            
            # Write header only if the file is newly created
            if not file_exists:
                writer.writeheader()
            
            # Write the latest data point
            writer.writerow(row_data)
            
        # Optional: Log that data was successfully saved
        if self.debug:
            print(f"Log data saved to {file_path}")


if __name__ == "__main__":
    pass
