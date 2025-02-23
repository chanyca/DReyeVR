import matplotlib.pyplot as plt
import numpy as np
from utils import *


class HapticSharedControl:
    def __init__(self, Cs=0.5, Kc=0.5, T=1.0, tp=1.0, speed=2, desired_trajectory_params=[], vehicle_config={}):
        self.Cs = Cs
        self.Kc = Kc
        self.T = T

        self.tp = tp  # preview time
        self.speed = speed  # vehicle speed
        self.vehicle_config = vehicle_config # vehicle configuration

        self.desired_trajectory_params = desired_trajectory_params
        self.desired_trajectory = self.desired_trajectory_params["path"]

    def calculate_torque(self, current_position, steering_wheel_angles, current_yaw_angle, t):
        # calculate the average steering angle
        self.avg_steering_angle = self.calculate_avg_steering_angle(steering_wheel_angles)
        # calculate turning radius
        self.R = self.calculate_turning_radius(steering_wheel_angles)

        # Calculate the previewed driver model
        self.theta_d = self.preview_driver_model(current_position, current_yaw_angle, t)

        self.e_t = self.distance_to_trajectory(current_position)

        coef = self.sigmoid(self.Cs * self.e_t)
        
        desired_steering_angle = self.theta_d
        
        tau_das = - (self.Cs * self.e_t) * (self.avg_steering_angle - self.theta_d)

        return tau_das, coef, desired_steering_angle

    def preview_driver_model(self, current_position, current_yaw_angle, t, method="simple"):
        # Predict the position of the vehicle at the preview time
        self.predicted_position = self.predict_position(current_position, self.avg_steering_angle, current_yaw_angle)
        
        # Calculate the error between desired trajectory and predicted position with tp[s] ahead
        self.epsilon_tp_t = self.distance_to_trajectory(self.predicted_position) * self.get_sign_of_error(self.predicted_position)

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
    
    def distance_to_trajectory(self, position):
        # More robust distance calculation by checking multiple points
        min_dist = float("inf")
        closest_point = find_closest_point(position, self.desired_trajectory) 
        return min(self.dist(position, closest_point), min_dist)

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
    
    def get_sign_of_error(self, position):
        p1 = position
        _, pt_index = find_closest_point(p1, self.desired_trajectory) 
        p2, p3 = self.desired_trajectory_params["tangent"][pt_index]
        angle = getAngle(p1, p2, p3)
        
        return np.sin(angle) / abs(np.sin(angle))


if __name__ == "__main__":
    import json
    from pathlib import Path
    
    __file_path__ = Path(__file__).resolve().parent

    with open(f"{__file_path__}/wheel_setting.json", "r") as f:
        Wheel_setting = json.load(f)
        
    L = abs(Wheel_setting['wheels'][0]['position']['y'] - Wheel_setting['wheels'][2]['position']['y'])
    T = abs(Wheel_setting['wheels'][0]['position']['x'] - Wheel_setting['wheels'][1]['position']['x'])
    theta1 = np.radians(69.99)
    theta2 = np.radians(47.95)

    R = (L/np.sin(theta2) + np.sqrt((L/np.tan(theta1) + T)**2 + L**2))/2

    # define initial and final points
    P_0 = [-147.066772, -1322.415039] # [y, x]
    P_f = [-687.066772, -2162.415039]
    P_d = [-37.066772 , -2902.415039]

    yaw_0 = 0
    yaw_d = 10
    yaw_f = 90
    
    time_step = 1.0
    
    # calculate the bezier path
    from bezier_path import *
    n_points = 100
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

    vehicle_config = Wheel_setting

    i_points = P_d[::-1]
    f_points = P_f[::-1]
    i_yaw = 180 + yaw_d

    # Simulation Parameters
    # speed = int((f_points[0] - i_points[0]) // 100)
    speed = -200
    current_position = i_points
    current_time = 0

    # Haptic Shared Control Initialization
    haptic_control = HapticSharedControl(
        Cs=0.5, Kc=0.5, T=2, tp=2, speed=speed, desired_trajectory_params=param, vehicle_config=vehicle_config
    )

    # Simulation Loop
    trajectory = [current_position]
    torques = []
    steering_angles = [0]
    offset_percentage = []
    s_angleL = 0
    s_angleR = 0

    for _ in range(len(path)):
        # print(current_time, current_position, desired_trajectory(current_time), haptic_control.theta_curr)
        # print("\tT:", current_time, "\tP:", current_position, "\tD:", desired_trajectory(current_time), "\tSA:", haptic_control.theta_curr)
        torque, coef, desired_steering_angle  = haptic_control.calculate_torque(
            current_position=current_position,
            steering_wheel_angles=[s_angleL, s_angleR],
            current_yaw_angle=i_yaw,
            t=current_time,
        )

        next_position = haptic_control.predicted_position
        
        trajectory.append(next_position)
        torques.append(torque)
        steering_angles.append(desired_steering_angle)

        current_position = next_position
        s_angleL, s_angleR = desired_steering_angle, desired_steering_angle
        
        current_time += time_step

    # Extract trajectory points
    trajectory = np.array(trajectory)
    x_points = trajectory[:, 0]
    y_points = trajectory[:, 1]

    # Plot the trajectory

    # plt.figure(figsize=(10, 6))
    plt.plot(x_points, y_points, label="Predicted Path", color="red")
    plt.plot(path[:, 0], path[:, 1], "--", label="Bezier Path")
    plt.xlabel("Y")
    plt.ylabel("X")
    plt.title("Vehicle Path and Bezier Trajectory")
    plt.legend()
    plt.grid(True)
    plt.show()

    # plt.figure(figsize=(10, 6))
    plt.plot(np.arange(0, current_time, time_step), torques, label="Torque")
    plt.xlabel("Time (s)")
    plt.ylabel("Torque (Nm)")
    plt.title("Torque Feedback")
    plt.grid(True)
    plt.show()

    # plt.figure(figsize=(10, 6))
    plt.plot(np.arange(0, current_time, time_step), steering_angles[:-1], label="Steering Angle")
    plt.xlabel("Time (s)")
    plt.ylabel("Steering angle (rad)")
    plt.title("Steering angle")
    plt.grid(True)
    plt.show()