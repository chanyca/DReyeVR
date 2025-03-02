import math

import numpy as np
from scipy.spatial import KDTree


# cspell: ignore Ackermann nnidx
def cot(x):
    return 1 / np.tan(x)


def arccot(x):
    # cspell: ignore arctan arccot
    return np.arctan(1 / x)


def sigmoid(x):
    return 1 / (1 + np.exp(-x))


def dist(p1, p2):
    # cspell: ignore linalg
    return np.linalg.norm(p1 - p2)


def find_closest_point(given_point, list_of_points, method="Brute"):
    # Convert list_of_points to a NumPy array for efficient processing
    points = np.array(list_of_points)

    if method == "KDTree":
        # Build a KD-Tree from the points
        tree = KDTree(points)

        # Find the index of the nearest neighbor
        _, nnidx = tree.query(given_point, k=1)
    else:
        # Find the index of the nearest neighbor
        nnidx = np.argmin(np.linalg.norm(points - given_point, axis=1))

    # Return the closest point using the found index
    return list_of_points[nnidx], nnidx


def getAngle(a, b, c, degrees=False):
    angle_rad = math.atan2(c[1] - b[1], c[0] - b[0]) - math.atan2(a[1] - b[1], a[0] - b[0])
    return math.degrees(angle_rad) if degrees else angle_rad


def rotation_matrix_cw(angle_rad):
    return np.array(
        [[np.cos(angle_rad), -np.sin(angle_rad)], [np.sin(angle_rad), np.cos(angle_rad)]]
    )


def rotation_matrix_ccw(angle_rad):
    return np.array(
        [[np.cos(angle_rad), np.sin(angle_rad)], [-np.sin(angle_rad), np.cos(angle_rad)]]
    )


class Vehicle:
    def __init__(self, vehicle_config: dict, model: str = "simple") -> None:
        self.vehicle_config = vehicle_config
        self.vehicle_model = model

        self.wheelbase = None
        self.tire_width = None
        self.center_of_mass_ratio = None
        self.minimum_turning_radius = None

        self.max_steering_angle_inner_deg = 69.99999237060547
        self.max_steering_angle_outer_deg = 47.425662994384766

        self.init_vehicle_model()

    def init_vehicle_model(self) -> dict:
        """
        Initialize the vehicle model parameters.
        """

        self.wheelbase = (
            np.abs(
                self.vehicle_config["wheels"][0]["position"]["y"]
                - self.vehicle_config["wheels"][2]["position"]["y"]
            )
            / 100
        )
        self.tire_width = (
            np.abs(
                self.vehicle_config["wheels"][1]["position"]["x"]
                - self.vehicle_config["wheels"][0]["position"]["x"]
            )
            / 100
        )

        self.center_of_mass_ratio = self.vehicle_config["center_of_mass"]["x"]
        self.center_of_mass = self.wheelbase * self.center_of_mass_ratio
        
        self.minimum_turning_radius = self.calc_turning_radius(
            [self.max_steering_angle_inner_deg, self.max_steering_angle_outer_deg]
        )["R"]

    def calc_turning_radius(self, steering_angles: tuple) -> list:
        """
        Calculate the turning radius and turning angle of the vehicle.
        This function calculates the turning radius and turning angle based on the steering angles of the inner and outer wheels.
        It supports two vehicle models: "simple" and "Ackermann".
        Args:
            steering_angles (tuple): The steering angles of the front wheels in degrees.
        Returns:
            dict: A dict containing the following values:
            - Turning radius at the center of mass (m)
            - Vehicle steering angle at the center of mass (degrees)
            - Center of rotation in vehicle coordinates (m)
        References:
            - https://www.theautopian.com/the-engineering-behind-why-some-cars-can-turn-tighter-than-others/
            - https://www.ijser.org/researchpaper/optimizing-the-turning-radius-of-a-vehicle-using-symmetric.pdf
        """
        if steering_angles[0] > 0:  # turn clockwise if go forward
            self.inner_wheel_steering_angle_deg = steering_angles[1]
            self.outer_wheel_steering_angle_deg = steering_angles[0]
        else:  # turn counter-clockwise if go forward
            self.inner_wheel_steering_angle_deg = steering_angles[0]
            self.outer_wheel_steering_angle_deg = steering_angles[1]

        self.inner_wheel_steering_angle_rad = np.radians(self.inner_wheel_steering_angle_deg)
        self.outer_wheel_steering_angle_rad = np.radians(self.outer_wheel_steering_angle_deg)

        if self.vehicle_model == "simple":
            # REF: https://www.theautopian.com/the-engineering-behind-why-some-cars-can-turn-tighter-than-others/
            steering_angle_rad = self.outer_wheel_steering_angle_rad
            steering_angle_rad += 1e-6 if steering_angle_rad % (np.pi) == 0 else 0
            
            turning_radius = np.sqrt(
                self.center_of_mass**2 + (self.wheelbase**2) / np.tan(steering_angle_rad) ** 2
            )
            vehicle_steering_angle = np.arctan(
                self.center_of_mass_ratio * np.tan(steering_angle_rad)
            )
        else:
            # Calculate the turning radius using the Ackermann steering model
            # REF: https://www.ijser.org/researchpaper/optimizing-the-turning-radius-of-a-vehicle-using-symmetric.pdf
            self.inner_wheel_steering_angle_rad += (
                1e-6 if self.inner_wheel_steering_angle_rad % (np.pi) == 0 else 0
            )
            self.outer_wheel_steering_angle_rad += (
                1e-6 if self.outer_wheel_steering_angle_rad % (np.pi) == 0 else 0
            )
            turning_angle_rad = arccot(
                (
                    cot(self.inner_wheel_steering_angle_rad)
                    + cot(self.outer_wheel_steering_angle_rad)
                )
                / 2
            )
            vehicle_steering_angle = np.sqrt(
                (self.center_of_mass**2) + (self.wheelbase**2) * (cot(turning_angle_rad) ** 2)
            )

        return {"R": np.abs(turning_radius), 
                "Delta:": np.degrees(vehicle_steering_angle), 
                "CoR": np.array(
                    [
                        self.wheelbase / np.tan(vehicle_steering_angle),
                        -self.center_of_mass,
                    ]
                )} 
    


def linear_fn(intercept, slope):
    return lambda x: slope * x + intercept

if __name__ == "__main__":
    a = (5, 0)
    b = (0, 0)
    c = (0, -5)
    print(getAngle(a, b, c))  # result 90.0
