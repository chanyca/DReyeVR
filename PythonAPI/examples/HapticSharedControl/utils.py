import math

import numpy as np
from scipy.spatial import KDTree


def calculate_steering_mechanism(inner_wheel_steering_angle: float, outer_wheel_steering_angle: float, vehicle_config:dict) -> float:
    """
    Calculate the steering angle of the steering mechanism
    :param inner_wheel_steering_angle: The steering angle of the inner wheel
    :param outer_wheel_steering_angle: The steering angle of the outer wheel
    :param L: The distance between the wheels
    :param W: The distance between the front and rear wheels
    :return: The steering angle of the steering mechanism
    """
    L = abs(vehicle_config["wheels"][0]["position"]["x"] - vehicle_config["wheels"][2]["position"]["x"])
    W = abs(vehicle_config["wheels"][1]["position"]["y"] - vehicle_config["wheels"][0]["position"]["y"])
    a2 = L * vehicle_config["center_of_mass"]["x"] # center of mass
    R1 = np.sqrt(a2**2 + (L**2) * (cot(inner_wheel_steering_angle)**2))
    R2 = np.sqrt(a2**2 + (L**2) * (cot(outer_wheel_steering_angle)**2))
    
    delta = arccot((cot(inner_wheel_steering_angle) + cot(outer_wheel_steering_angle)) / 2)
    R = np.sqrt((a2 ** 2) + (R1 **2))
    return {"Turning Radius": R, "Steering Angle": delta}

def simple_vehicle_model(steering_angles: float, vehicle_config: dict) -> dict:
    """
    Simple vehicle model to calculate the turning radius and steering angle
    :param steering_angle: The steering angle of the vehicle
    :param L: The distance between the wheels
    :param W: The distance between the front and rear wheels
    :return: The turning radius and steering angle
    """
    L = abs(vehicle_config["wheels"][0]["position"]["x"] - vehicle_config["wheels"][2]["position"]["x"])
    W = abs(vehicle_config["wheels"][1]["position"]["y"] - vehicle_config["wheels"][0]["position"]["y"])
    avg_steering_angle = sum(steering_angles) / len(steering_angles)
    avg_steering_angle = np.radians(avg_steering_angle)
    eps = 1e-6
    R = L / np.sin(avg_steering_angle + eps)
    return {"Turning Radius": R, "Steering Angle": avg_steering_angle}

def cot(x):
    return 1 / np.tan(x)

def arccot(x):
    # cspell: ignore arctan arccot
    return np.arctan(1 / x)

def sigmoid(x):
    return 1 / (1 + np.exp(-x))


def find_closest_point(given_point, list_of_points):
    # Convert list_of_points to a NumPy array for efficient processing
    points = np.array(list_of_points)
    
    # Build a KD-Tree from the points
    tree = KDTree(points)
    
    # Find the index of the nearest neighbor
    _, nnidx = tree.query(given_point, k=1)
    
    # Return the closest point using the found index
    return list_of_points[nnidx], nnidx


def getAngle(a, b, c, degrees=True):
    angle = math.atan2(c[1]-b[1], c[0]-b[0]) - math.atan2(a[1]-b[1], a[0]-b[0])
    return math.degrees(angle) if degrees else angle

if __name__ == "__main__":
    a = (5, 0)
    b = (0, 0)
    c = (0, -5)
    print(getAngle(a, b, c)) # result 90.0