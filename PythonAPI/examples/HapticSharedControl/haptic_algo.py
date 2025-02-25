import matplotlib.pyplot as plt
import numpy as np
from utils import *


class HapticSharedControl:
    debug = True
    def __init__(self, Cs=0.5, Kc=0.5, T=1.0, tp=1.0, speed=2, desired_trajectory_params=[], vehicle_config={}):
        self.Cs = Cs
        self.Kc = Kc
        self.T = T

        self.tp = tp  # preview time
        self.speed = speed  # vehicle speed
        self.vehicle_config = vehicle_config # vehicle configuration

        self.desired_trajectory_params = desired_trajectory_params
        self.desired_trajectory = self.desired_trajectory_params["path"]

    def calculate_torque(self, current_position, steering_angles, current_yaw_angle, steering_wheel_angle=None):
        """
        Calculate the torque for the haptic shared control system.
        Parameters:
            current_position (tuple): The current position of the vehicle (x, y).
            steering_wheel_angles (list): List of steering wheel angles.
            current_yaw_angle (float): The current yaw angle of the vehicle. (in degrees)
        Returns:
            tuple: A tuple containing:
            - tau_das (float): The calculated torque.
            - coef (float): The coefficient based on the error to trajectory.
            - desired_steering_angle (float): The desired steering angle in degrees.
        """
        if self.debug:
            print("[Measured] Current Position [X(t), Y(t)] (m):", current_position)
            print("[Measured] Steering Angles [FL, FR] (degree):", steering_angles)
            print("[Measured] Current Yaw Angle - Phi(t):", current_yaw_angle)
 
        # calculate the average steering angle and turning radius
        self.R, self.vehicle_steering_angle = self.vehicle_model(steering_angles) # in degrees
        
        #NOTE: In simulation, suppose the steering wheel angle is the same as the vehicle steering angle
        if not steering_wheel_angle:
            self.steering_wheel_angle = self.vehicle_steering_angle
        else:
            self.steering_wheel_angle = steering_wheel_angle
            
        if self.debug:
            print("[Input] Steering Wheel Angle - Theta(t) (degree):", self.steering_wheel_angle)
            print("[Vehicle] Turning Radius: (m)", self.R)
            print("[Vehicle] Vehicle Steering Angle - Delta(t) (degree):", self.vehicle_steering_angle)
            
        # Calculate the previewed driver model
        self.theta_d = self.preview_driver_model(current_position, current_yaw_angle)

        self.e_t = self.distance_to_trajectory(current_position)

        coef = sigmoid(self.Cs * self.e_t)
        
        desired_steering_angle = np.degrees(self.theta_d)
        
        tau_das = - (self.Cs * self.e_t) * (self.vehicle_steering_angle - self.theta_d)
        
        if self.debug:
            print("[Algo] Current Error to Trajectory - e(t):", self.e_t)

            print("[Algo] Predicted Position - r_tp(t):", self.predicted_position)
            print("[Algo] - Delta Phi (degree):", np.degrees(self.delta_phi))
            print("[Algo] - Rotating Angle (degree):", np.degrees(self.rotating_angle))
            
            print("[Algo] Predicted Error to Trajectory - eps_tp(t):", self.epsilon_tp_t)
            print("[Algo] Desired Steering Wheel Angle - Theta_d(t) (degree):",  np.degrees(self.theta_d))
            
            print("[Output] Torque - Tau_das (N.m):", tau_das)
            
        return tau_das, coef, desired_steering_angle

    def preview_driver_model(self, current_position, current_yaw_angle, method="simple"):
        """
        Predicts the desired steering angle based on the current position, yaw angle, and a specified method.
        Parameters:
            current_position (tuple): The current position of the vehicle as (x, y) coordinates.
            current_yaw_angle (float): The current yaw angle of the vehicle in radians.
            method (str, optional): The method to use for calculating the desired steering angle. 
                                Options are "simple" or "complex". Default is "simple".
        Returns:
            float: The desired steering angle in radians.
        """
        
        # Predict the position of the vehicle at the preview time
        self.predicted_position = self.predict_position(current_position, self.vehicle_steering_angle, current_yaw_angle)
        
        # Calculate the error between desired trajectory and predicted position with tp[s] ahead
        self.epsilon_tp_t = self.distance_to_trajectory(self.predicted_position)  * self.get_sign_of_error(self.predicted_position)

        # Calculate the desired steering angle
        if method == 'simple':
            theta_d = self.Kc * self.epsilon_tp_t - self.vehicle_steering_angle
        else:
            theta_d_curr = self.vehicle_steering_angle
            theta_d_next = None
            for delta_t in np.linspace(0, self.tp, 10):
                theta_d_next = (self.Kc * self.epsilon_tp_t + ((self.T / delta_t) - 1)  * theta_d_curr) * (delta_t / self.T)
                theta_d_curr = theta_d_next
            theta_d = theta_d_next
        
        return theta_d

    def predict_position(self, current_position, steering_angle_t, current_yaw_angle):
        """
        Predict the future position of the vehicle based on the current position, 
        steering angle, and yaw angle.
        Parameters:
            current_position (tuple): The current (x, y) position of the vehicle.
            steering_angle_t (float): The steering angle at time t in degrees.
            current_yaw_angle (float): The current yaw angle of the vehicle in degrees.
        Returns:
            np.notarray: The predicted (x, y) position of the vehicle.
        """
        
        steering_angle_t = np.radians(steering_angle_t)
        current_yaw_angle = np.radians(current_yaw_angle)
        
        x, y = current_position
        self.rotating_angle = steering_angle_t + current_yaw_angle
        self.delta_phi = self.speed * self.tp / self.R
        
        # self.delta_phi = self.delta_phi % (2 * np.pi)
        
        # center of rotation
        x_c = x + self.R * np.sin(self.rotating_angle)
        y_c = y + self.R * np.cos(self.rotating_angle)

        # predicted position
        x_tp = x_c + self.R * np.cos(self.rotating_angle - self.delta_phi)
        y_tp = y_c + self.R * np.sin(self.rotating_angle - self.delta_phi)

        predicted_position = np.array([x_tp, y_tp])
        return predicted_position

    def dist(self, p1, p2):
        # cspell: ignore linalg
        return np.linalg.norm(p1 - p2)
    
    def distance_to_trajectory(self, position):
        """
        Calculate the minimum distance from a given position to the desired trajectory.
        This method finds the closest point on the desired trajectory to the given position
        and calculates the distance between them. It ensures a more robust distance calculation
        by checking multiple points along the trajectory.
        Parameters:
            position (tuple or list): The current position as a tuple or list of coordinates (e.g., (x, y)).
        Returns:
            float: The minimum distance from the given position to the desired trajectory.
        """
        
        # More robust distance calculation by checking multiple points
        min_dist = float("inf")
        closest_point, _ = find_closest_point(position, self.desired_trajectory) 
        return min(self.dist(position, closest_point), min_dist)

    def vehicle_model(self, steering_angles, method="simple"):
        if method == "simple":
            radius = simple_vehicle_model(*steering_angles, self.vehicle_config)["Turning Radius"]
            steering_angle = simple_vehicle_model(*steering_angles, self.vehicle_config)["Steering Angle"]
        else:
            radius = calculate_steering_mechanism(*steering_angles, self.vehicle_config)["Turning Radius"]
            steering_angle = calculate_steering_mechanism(*steering_angles, self.vehicle_config)["Steering Angle"]
        return radius, steering_angle

    def get_sign_of_error(self, position):
        """
        Calculate the sign of the error based on the position relative to the desired trajectory.
        Args:
            position (tuple or list): The current position as a tuple or list of coordinates (x, y).
        Returns:
            float: The sign of the error, which is -1 or 1 depending on the direction of the error.
        """
        
        p1 = position
        _, pt_index = find_closest_point(p1, self.desired_trajectory) 
        p2, p3 = self.desired_trajectory_params["tangent"][pt_index]
        angle = getAngle(p1, p2, p3)
        
        return - np.sin(angle) / abs(np.sin(angle))


if __name__ == "__main__":
    import json
    from pathlib import Path

    from bezier_path import *

    # cspell: ignore bezier arctan xlabel figsize ylabel
    
    __file_path__ = Path(__file__).resolve().parent

    # Load the vehicle configuration
    with open(f"{__file_path__}/wheel_setting.json", "r") as f:
        Wheel_setting = json.load(f)
        
    L = abs(Wheel_setting['wheels'][0]['position']['y'] - Wheel_setting['wheels'][2]['position']['y']) / 100 # cm to m
    T = abs(Wheel_setting['wheels'][0]['position']['x'] - Wheel_setting['wheels'][1]['position']['x']) / 100 # cm to m
    theta1 = np.radians(69.99)
    theta2 = np.radians(47.95)

    R = (L/np.sin(theta2) + np.sqrt((L/np.tan(theta1) + T)**2 + L**2))/2
    print("Minimum Turning Radius:", R)
    
    # define initial and final points
    P_0 = [-1.47066772, -13.22415039]  # [x, y] in carla -> [y, x] in matplotlib
    P_f = [-6.87066772, -21.62415039] 
    P_d = [-0.37066772 , -29.02415039]

    yaw_0 = 0
    yaw_d = 10
    yaw_f = 90
    
    time_step = 1.0
    
    # calculate the bezier path
    print("Calculating Bezier Path")
    
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
    i_yaw = yaw_d 

    # Simulation Parameters
    speed = 2 # m/s
    current_position = i_points

    # Haptic Shared Control Initialization
    haptic_control = HapticSharedControl(
        Cs=0.5, Kc=0.5, T=2, tp=2, speed=speed, desired_trajectory_params=param, vehicle_config=vehicle_config
    )

    # Simulation Loop
    print("Starting Simulation")
    n_steps = 5
    
    s_angleL = 0
    s_angleR = 0
    
    trajectory = [current_position]
    torques = []
    steering_angles = [0]
    offset_percentage = []
    rotation_angles = [steering_angles[0] + s_angleL]

    for i in range(n_steps):
        print("--------------------")
        print("Step:", i)
        
        torque, coef, desired_steering_angle  = haptic_control.calculate_torque(
            current_position=current_position,
            steering_angles=[s_angleL, s_angleR],
            current_yaw_angle=i_yaw,
        )
        
        next_position = haptic_control.predicted_position
        
        trajectory.append(next_position)
        torques.append(torque)
        steering_angles.append(desired_steering_angle)
        rotation_angles.append(haptic_control.rotating_angle)
        
        s_angleL, s_angleR = desired_steering_angle, desired_steering_angle
        i_yaw = np.degrees(np.arctan((next_position[1] - current_position[1]) / (next_position[0] - current_position[0])))
        current_position = next_position

    # Extract trajectory points
    trajectory = np.array(trajectory)
    x_points = trajectory[:, 0]
    y_points = trajectory[:, 1]

    # Plot the trajectory
    plt.plot(path[:, 0], path[:, 1], "--", label="Bezier Path")
    plt.plot(path[0, 0], path[0, 1], "o", label="Start", color="blue")
    plt.plot(path[-1, 0], path[-1, 1], "o", label="End", color="green")
    
    # Plot the vehicle trajectory
    plt.plot(x_points, y_points, label="Predicted Path", color="red")
    for i, point in enumerate(zip(x_points, y_points)):
        plt.plot(point[0], point[1], "o", color="red")
        plot_arrow(point[0], point[1], np.degrees(rotation_angles[i]), 0.3, width=0.08)
    
    plt.xlabel("Y")
    plt.ylabel("X")
    plt.title("Vehicle Path and Bezier Trajectory")
    
    plt.legend()
    plt.grid(True)
    plt.show()

    # # plt.figure(figsize=(10, 6))
    # plt.plot(range(n_steps), torques, label="Torque")
    # plt.xlabel("Time (s)")
    # plt.ylabel("Torque (Nm)")
    # plt.title("Torque Feedback")
    # plt.grid(True)
    # plt.show()

    # # plt.figure(figsize=(10, 6))
    # plt.plot(range(n_steps), steering_angles[:-1], label="Steering Angle")
    # plt.xlabel("Time (s)")
    # plt.ylabel("Steering angle (rad)")
    # plt.title("Steering angle")
    # plt.grid(True)
    # plt.show()