import json
from pprint import pprint

import bezier as B
import matplotlib.pyplot as plt
import numpy as np
import scipy.special

with open("wheel_setting.json", "r") as f:
    Wheel_setting = json.load(f)
L = abs(Wheel_setting["wheels"][0]["position"]["y"] - Wheel_setting["wheels"][2]["position"]["y"])
T = abs(Wheel_setting["wheels"][0]["position"]["x"] - Wheel_setting["wheels"][1]["position"]["x"])
theta1 = np.radians(69.99)
theta2 = np.radians(47.95)

R = (L / np.sin(theta2) + np.sqrt((L / np.tan(theta1) + T) ** 2 + L**2)) / 2


def calc_control_points_bezier_path(sx, sy, s_yaw, ex, ey, e_yaw, l_d, l_f, n_points=100):
    """
    Compute control points and path given start and end position.

    :param sx: (float) x-coordinate of the starting point
    :param sy: (float) y-coordinate of the starting point
    :param s_yaw: (float) yaw angle at start
    :param ex: (float) x-coordinate of the ending point
    :param ey: (float) y-coordinate of the ending point
    :param e_yaw: (float) yaw angle at the end

    :return: (numpy array, numpy array)
    """

    control_points = np.array(
        [
            [sx, sy],
            [sx - l_d * np.cos(s_yaw), sy + l_d * np.sin(s_yaw)],
            [ex + l_f * np.cos(e_yaw), ey - l_f * np.sin(e_yaw)],
            [ex, ey],
        ]
    )

    paths = calc_bezier_path(control_points, n_points=n_points)

    return paths, control_points


def calc_bezier_path(control_points, n_points=100):
    """
    Compute bezier path (trajectory) given control points.

    :param control_points: (numpy array)
    :param n_points: (int) number of points in the trajectory
    :return: (numpy array)
    """
    trajectory = []
    d_trajectory = []
    dd_trajectory = []

    for u in np.linspace(0, 1, n_points):
        point = bezier(u, control_points)

        derivatives_cp = bezier_derivatives_control_points(control_points, 2)
        dt = bezier(u, derivatives_cp[1])
        ddt = bezier(u, derivatives_cp[2])

        trajectory.append(point)
        d_trajectory.append(dt)
        dd_trajectory.append(ddt)

    return [np.array(trajectory), np.array(d_trajectory), np.array(dd_trajectory)]


def bernstein_poly(n, i, u):
    """
    Bernstein polynomials.

    :param n: (int) polynomial degree
    :param i: (int)
    :param u: (float)
    :return: (float)
    """
    return scipy.special.comb(n, i) * (u**i) * ((1 - u) ** (n - i))


def bezier(u, control_points):
    """
    Return one point on the bezier curve.

    :param u: (float) number in [0, 1]
    :param control_points: (numpy array)
    :return: (numpy array) Coordinates of the point
    """
    n = len(control_points) - 1
    return np.sum([bernstein_poly(n, i, u) * control_points[i] for i in range(n + 1)], axis=0)


def bezier_derivatives_control_points(control_points, n_derivatives):
    """
    Compute control points of the successive derivatives of a given bezier curve.

    A derivative of a bezier curve is a bezier curve.
    See https://pomax.github.io/bezierinfo/#derivatives
    for detailed explanations

    :param control_points: (numpy array)
    :param n_derivatives: (int)
    e.g., n_derivatives=2 -> compute control points for first and second derivatives
    :return: ([numpy array])
    """
    w = {0: control_points}
    for i in range(n_derivatives):
        n = len(w[i])
        w[i + 1] = np.array([(n - 1) * (w[i][j + 1] - w[i][j]) for j in range(n - 1)])
    return w


def calc_curvature(dx, dy, ddx, ddy):
    """k_max
    Compute curvature at one point given first and second derivatives.

    :param dx: (float) First derivative along x axis
    :param dy: (float)
    :param ddx: (float) Second derivative along x axis
    :param ddy: (float)
    :return: (float)
    """
    return (dx * ddy - dy * ddx) / (dx**2 + dy**2) ** (3 / 2)


def plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):  # pragma: no cover
    """Plot arrow."""
    if not isinstance(x, float):
        for ix, iy, iyaw in zip(x, y, yaw):
            plot_arrow(ix, iy, iyaw)
    else:
        plt.arrow(
            x,
            y,
            length * np.cos(yaw),
            length * np.sin(yaw),
            fc=fc,
            ec=ec,
            head_width=width,
            head_length=width,
        )
        plt.plot(x, y)


def calculate_bezier_trajectory(
    start_pos,
    end_pos,
    start_yaw,
    end_yaw,
    n_points,
    turning_radius,
    control_param=0.86,
    max_res=10,
    show_animation=True,
):
    start_x, start_y = start_pos
    end_x, end_y = end_pos
    start_yaw = np.radians(start_yaw)
    end_yaw = np.radians(end_yaw)

    dist = np.hypot(end_x - start_x, end_y - start_y)  # distance between start and end

    trajectories = {}
    # Find the optimal l_d and l_f: assume that ld and lf in range of [dist/4, ]
    for l_d in np.linspace(dist / 5, dist, max_res):
        for l_f in np.linspace(dist / 5, dist, max_res):

            paths, control_points = calc_control_points_bezier_path(
                start_x, start_y, start_yaw, end_x, end_y, end_yaw, l_d, l_f, n_points=n_points
            )
            path, d_path, dd_path = paths
            control_points = np.asfortranarray(control_points)

            curve = B.Curve(control_points.T, degree=3)
            length = curve.length

            # Display the tangent, normal and radius of curvature at a given point
            curvatures = []
            for i, u in enumerate(np.linspace(0, 1, n_points)):
                curvatures.append(
                    calc_curvature(d_path[i][0], d_path[i][1], dd_path[i][0], dd_path[i][1])
                )

            max_curvature = max(curvatures)

            if abs(1 / max_curvature) >= turning_radius:
                trajectories[(l_d, l_f)] = length
                # print(f"l_d: {l_d}, l_f: {l_f}, length: {length}, radius: {abs(1 / max_curvature)}")

    l_f, l_d = min(trajectories, key=trajectories.get)

    paths, control_points = calc_control_points_bezier_path(
        start_x, start_y, start_yaw, end_x, end_y, end_yaw, l_d, l_f
    )
    path, _, __ = paths

    # Display the tangent, normal and radius of curvature at a given point
    u = control_param  # Number in [0, 1]
    x_target, y_target = bezier(u, control_points)
    derivatives_cp = bezier_derivatives_control_points(control_points, 2)
    point = bezier(u, control_points)
    dt = bezier(u, derivatives_cp[1])
    ddt = bezier(u, derivatives_cp[2])

    # Radius of curvature
    radius = 1 / calc_curvature(dt[0], dt[1], ddt[0], ddt[1])

    # Normalize derivative
    dt /= np.linalg.norm(dt, 2)
    tangent = np.array([point, point + dt])
    normal = np.array([point, point + [-dt[1], dt[0]]])
    curvature_center = point + np.array([-dt[1], dt[0]]) * radius
    circle = plt.Circle(
        tuple(curvature_center), radius, color=(0, 0.8, 0.8), fill=False, linewidth=1
    )

    assert path.T[0][0] == start_x, "path is invalid"
    assert path.T[1][0] == start_y, "path is invalid"
    assert path.T[0][-1] == end_x, "path is invalid"
    assert path.T[1][-1] == end_y, "path is invalid"

    plot_elements = {
        "path": path,
        "control_points": control_points,
        "tangent": tangent,
        "normal": normal,
        "circle": circle,
        "start_x": start_x,
        "start_y": start_y,
        "end_x": end_x,
        "end_y": end_y,
        "start_yaw": start_yaw,
        "end_yaw": end_yaw,
        "x_target": x_target,
        "y_target": y_target,
        "dist": dist,
        "radius": radius,
    }

    if show_animation:  # pragma: no cover
        fig, ax = plt.subplots()
        ax.plot(path.T[0], path.T[1], label="Bezier Path")
        ax.plot(control_points.T[0], control_points.T[1], "--o", label="Control Points")
        ax.plot(x_target, y_target)
        ax.plot(tangent[:, 0], tangent[:, 1], label="Tangent")
        ax.plot(normal[:, 0], normal[:, 1], label="Normal")
        ax.add_artist(circle)
        plot_arrow(start_x, start_y, np.pi - start_yaw, length=0.1 * dist, width=0.05 * dist)
        plot_arrow(end_x, end_y, np.pi - end_yaw, length=0.1 * dist, width=0.05 * dist)
        ax.legend()
        ax.axis("equal")
        ax.grid(True)
        plt.show()

    return path, control_points, plot_elements
