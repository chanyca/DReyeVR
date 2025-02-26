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


def find_closest_point(given_point, list_of_points):
    # Convert list_of_points to a NumPy array for efficient processing
    points = np.array(list_of_points)

    # Build a KD-Tree from the points
    tree = KDTree(points)

    # Find the index of the nearest neighbor
    _, nnidx = tree.query(given_point, k=1)

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

        self.max_steering_angle_inner_deg = 69.99
        self.max_steering_angle_outer_deg = 47.95

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

        self.minimum_turning_radius = self.calc_turning_radius(
            self.max_steering_angle_inner_deg, self.max_steering_angle_outer_deg
        )[0]

    def calc_turning_radius(
        self, inner_wheel_steering_angle_deg: float, outer_wheel_steering_angle_deg: float
    ) -> list:
        """
        Calculate the turning radius and turning angle of the vehicle.
        This function calculates the turning radius and turning angle based on the steering angles of the inner and outer wheels.
        It supports two vehicle models: "simple" and "Ackermann".
        Args:
            inner_wheel_steering_angle (float): The steering angle of the inner wheel in degrees.
            outer_wheel_steering_angle (float): The steering angle of the outer wheel in degrees.
        Returns:
            list: A list containing the turning radius and the turning angle in degrees.
        References:
            - https://www.theautopian.com/the-engineering-behind-why-some-cars-can-turn-tighter-than-others/
            - https://www.ijser.org/researchpaper/optimizing-the-turning-radius-of-a-vehicle-using-symmetric.pdf
        """

        inner_wheel_steering_angle_rad = np.radians(inner_wheel_steering_angle_deg)
        outer_wheel_steering_angle_rad = np.radians(outer_wheel_steering_angle_deg)

        inner_wheel_steering_angle_rad += (
            1e-6 if inner_wheel_steering_angle_rad % (2 * np.pi) == 0 else 0
        )
        outer_wheel_steering_angle_rad += (
            1e-6 if outer_wheel_steering_angle_rad % (2 * np.pi) == 0 else 0
        )

        if self.vehicle_model == "simple":
            # REF: https://www.theautopian.com/the-engineering-behind-why-some-cars-can-turn-tighter-than-others/
            turning_angle_rad = outer_wheel_steering_angle_rad
            turning_radius = self.wheelbase / np.sin(turning_angle_rad) + self.tire_width / 2
        else:
            # Calculate the turning radius using the Ackermann steering model
            # REF: https://www.ijser.org/researchpaper/optimizing-the-turning-radius-of-a-vehicle-using-symmetric.pdf
            center_of_mass = self.wheelbase * self.center_of_mass_ratio
            turning_angle_rad = arccot(
                (cot(inner_wheel_steering_angle_rad) + cot(outer_wheel_steering_angle_rad)) / 2
            )
            turning_radius = np.sqrt(
                (center_of_mass**2) + (self.wheelbase**2) * (cot(turning_angle_rad) ** 2)
            )

        return np.abs(turning_radius), np.degrees(turning_angle_rad)


if __name__ == "__main__":
    a = (5, 0)
    b = (0, 0)
    c = (0, -5)
    print(getAngle(a, b, c))  # result 90.0
