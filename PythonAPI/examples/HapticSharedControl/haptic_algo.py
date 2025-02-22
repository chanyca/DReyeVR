import matplotlib.pyplot as plt
import numpy as np
from HapticSharedControl.utils import *


class HapticSharedControl:
    def __init__(self, Cs=0.5, Kc=0.5, T=1.0, tp=1.0, speed=2, desired_trajectory=[], vehicle_config={}):
        self.Cs = Cs
        self.Kc = Kc
        self.T = T

        self.tp = tp  # preview time
        self.speed = speed  # vehicle speed
        self.vehicle_config = vehicle_config # vehicle configuration

        self.desired_trajectory = desired_trajectory

    def calculate_torque(self, current_position, steering_wheel_angles, current_yaw_angle, t):
        # calculate the average steering angle
        self.avg_steering_angle = self.calculate_avg_steering_angle(steering_wheel_angles)
        # calculate turning radius
        self.R = self.calculate_turning_radius(steering_wheel_angles)

        # Calculate the previewed driver model
        self.theta_d = self.preview_driver_model(current_position, current_yaw_angle, t)

        self.e_t = self.distance_to_trajectory(current_position, t)

        coef = self.sigmoid(self.Cs * self.e_t)
        
        desired_steering_angle = self.theta_d
        
        tau_das = - (self.Cs * self.e_t) * (self.avg_steering_angle - self.theta_d)

        return tau_das, coef, desired_steering_angle

    def preview_driver_model(self, current_position, current_yaw_angle, t, method="simple"):
        # Predict the position of the vehicle at the preview time
        self.predicted_position = self.predict_position(current_position, self.avg_steering_angle, current_yaw_angle)
        # Calculate the error between desired trajectory and predicted position with tp[s] ahead
        # todo: consider the sign of the error epsilon_tp_t
        self.epsilon_tp_t = self.distance_to_trajectory(self.predicted_position, t + self.tp) 

        # Calculate the desired steering angle
        if method == 'simple':
            theta_d = self.Kc * self.epsilon_tp_t - self.avg_steering_angle
        else:
            theta_d_curr = self.avg_steering_angle
            theta_d_next = None
            for delta_t in np.linspace(0, self.tp, 10):
                theta_d_next = (self.Kc * self.epsilon_tp_t + ((self.T / delta_t) - 1)  * theta_d_curr) * (delta_t / self.T)
                theta_d_curr = theta_d_next
            theta_d = theta_d_next
        return theta_d

    def predict_position(self, current_position, steering_wheel_angle_t, current_yaw_angle):
        x, y = current_position
        rotating_angle = steering_wheel_angle_t + current_yaw_angle
        delta_phi = self.speed * self.tp / self.R
        
        # center of rotation
        x_c = x + self.R * np.sin(rotating_angle)
        y_c = y + self.R * np.cos(rotating_angle)

        # predicted position
        x_tp = x_c + self.R * np.cos(rotating_angle + delta_phi)
        y_tp = y_c + self.R * np.sin(rotating_angle + delta_phi)

        predicted_position = np.array([x_tp, y_tp])
        return predicted_position

    def dist(self, p1, p2):
        # cspell: ignore linalg
        return np.linalg.norm(p1 - p2)
    
    def distance_to_trajectory(self, position, current_time):
        # More robust distance calculation by checking multiple points
        min_dist = float("inf")
        closest_point = None

        for i in range(
            -self.lookaround_window // 2, self.lookaround_window // 2
        ):  # Check points along the trajectory around current time
            t = current_time + i  # Check in a window of 10 second.

            point_on_trajectory = self.get_desired_trajectory(t)

            dist = self.dist(position, point_on_trajectory)

            if dist < min_dist:
                min_dist = dist
                closest_point = point_on_trajectory

        if closest_point is None:
            closest_point = self.get_desired_trajectory(
                current_time
            )  # If no better point is found

        return self.dist(position, closest_point)

    def calculate_turning_radius(self, steering_wheel_angles, method="simple"):
        if method == "simple":
            return simple_vehicle_model(steering_wheel_angles, self.vehicle_config)["Turning Radius"]
        else:
            return calculate_steering_mechanism(steering_wheel_angles, self.vehicle_config)["Turning Radius"]
        
    def calculate_avg_steering_angle(self, steering_wheel_angles, method="simple"):
        if method == "simple":
            return simple_vehicle_model(steering_wheel_angles, self.vehicle_config)["Steering Angle"]
        else:
            return calculate_steering_mechanism(steering_wheel_angles, self.vehicle_config)["Steering Angle"]
        
    def sigmoid(self, x):
        return 1 / (1 + np.exp(-x))
    
    def get_desired_trajectory(self, t):
        if int(t) >= len(self.desired_trajectory):
            return self.desired_trajectory[-1]
        return self.desired_trajectory[int(t)]
