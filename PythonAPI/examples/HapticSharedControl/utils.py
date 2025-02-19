import numpy as np


def calculate_steering_mechanism(inner_wheel_steering_angle: float, outer_wheel_steering_angle: float, vehicle_config:dict) -> float:
    """
    Calculate the steering angle of the steering mechanism
    :param inner_wheel_steering_angle: The steering angle of the inner wheel
    :param outer_wheel_steering_angle: The steering angle of the outer wheel
    :param L: The distance between the wheels
    :param W: The distance between the front and rear wheels
    :return: The steering angle of the steering mechanism
    """
    L = abs(vehicle_config["wheels"][0]["x"] - vehicle_config["wheels"][2]["x"])
    W = abs(vehicle_config["wheels"][1]["y"] - vehicle_config["wheels"][0]["y"])
    a2 = L * vehicle_config["center_of_mass"]["x"] # center of mass
    R1 = np.sqrt(a2**2 + (L**2) * (cot(inner_wheel_steering_angle)**2))
    R2 = np.sqrt(a2**2 + (L**2) * (cot(outer_wheel_steering_angle)**2))
    
    delta = arccot((cot(inner_wheel_steering_angle) + cot(outer_wheel_steering_angle)) / 2)
    R = np.sqrt((a2 ** 2) + (R1 **2))
    return {"Turning Radius": R, "Steering Angle": delta}

def cot(x):
    return 1 / np.tan(x)

def arccot(x):
    # cspell: ignore arctan arccot
    return np.arctan(1 / x)

def simple_vehicle_model(steering_angle: float, vehicle_config: dict) -> dict:
    """
    Simple vehicle model to calculate the turning radius and steering angle
    :param steering_angle: The steering angle of the vehicle
    :param L: The distance between the wheels
    :param W: The distance between the front and rear wheels
    :return: The turning radius and steering angle
    """
    L = abs(vehicle_config["wheels"][0]["x"] - vehicle_config["wheels"][2]["x"])
    W = abs(vehicle_config["wheels"][1]["y"] - vehicle_config["wheels"][0]["y"])
    R = L / np.sin(steering_angle)
    return {"Turning Radius": R, "Steering Angle": steering_angle}