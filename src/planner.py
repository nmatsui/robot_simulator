import itertools

import numpy as np

from src.robot import Robot
from src import utils


class DWAwoObstacle:

    V_RESOLUTION = 0.05
    OMEGA_RESOLUTION = 0.05

    ERROR_ANGLE_GAIN = 1.0
    VELOCITY_GAIN = 0.5
    DISTANCE_GAIN = 0.01
    THETA_GAIN = 0.1

    @classmethod
    def get_input(cls, current, destination, current_input, delta):
        """
        Parameters:
        ----------
        current: np.array(x, y, theta)
            current pose
        destination: np.array(x, y, theta)
            destination pose
        current_input: np.array(v, omega)
            current input vector
        delta: float
            time delta of this tick

        Returns:
        ----------
        np.array(v, omega)
            next input vector of linear velocity and angular velocity
        """

        v_range, omega_range = cls._get_window(current_input, delta)

        input_list = np.array([[]]).reshape(0, 2)
        heading_list = np.array([])
        velocity_list = np.array([])
        distance_list = np.array([])
        theta_list = np.array([])

        for v, omega in itertools.product(v_range, omega_range):
            input = np.array((v, omega))
            next = Robot.move(current, input, delta)
            input_list = np.append(input_list, [input], axis=0)
            heading_list = np.append(heading_list, cls._eval_heading(next, destination))
            velocity_list = np.append(velocity_list, cls._eval_velocity(input))
            distance_list = np.append(distance_list, cls._eval_distance(next, destination))
            theta_list = np.append(theta_list, cls._eval_theta(next, destination))

        def _eval(a):
            return DWAwoObstacle.ERROR_ANGLE_GAIN * a[0] + DWAwoObstacle.VELOCITY_GAIN * a[1] \
                + DWAwoObstacle.DISTANCE_GAIN * a[2] + DWAwoObstacle.THETA_GAIN * a[3]

        candidate_list = np.apply_along_axis(_eval, 0, np.array([
            utils.normalize_min_max(heading_list),
            utils.normalize_min_max(velocity_list),
            utils.normalize_min_max(distance_list),
            utils.normalize_min_max(theta_list),
        ]))

        return input_list[np.argmin(candidate_list)]

    @classmethod
    def _get_window(cls, current_input, delta):
        """
        Parameters:
        ----------
        current_input: np.array(v, omega)
            current input vector
        delta: float
            time delta of this tick

        Returns:
        ----------
        tuple(np.array([v0, v1, ...]), np.array([omega0, omega1, ...]))
            list of possible linear velocities and list of possible angle velocities
            calculated from current input and robot specifications.
            caution) this candidate velocities do not consider any obstacles
        """

        delta_v = Robot.MAX_LIN_ACC * delta
        delta_omega = Robot.MAX_ANG_ACC * delta

        min_v = np.max((current_input[0] - delta_v, Robot.MIN_V))
        max_v = np.min((current_input[0] + delta_v, Robot.MAX_V))
        min_omega = np.max((current_input[1] - delta_omega, Robot.MIN_OMEGA))
        max_omega = np.min((current_input[1] + delta_omega, Robot.MAX_OMEGA))

        return np.arange(min_v, max_v, DWAwoObstacle.V_RESOLUTION), \
            np.arange(min_omega, max_omega, DWAwoObstacle.OMEGA_RESOLUTION)

    @classmethod
    def _eval_heading(cls, next, destination):
        """
        Parameters:
        ----------
        next: np.array(x, y, theta)
            candidate pose calculated from the Robot's motion model
        destination: np.array(x, y, theta)
            destination pose

        Returns:
        ----------
        float
            the difference angle between the direction to destination and candidate pose's theta
            (smaller is better)
        """

        angle = np.arctan2(destination[1] - next[1], destination[0] - next[0])
        return np.abs(utils.normalize_angle(angle - next[2]))

    @classmethod
    def _eval_velocity(cls, input):
        """
        Parameters:
        ----------
        input: np.array(v, omega)
            candidate input

        Returns:
        ----------
        float
            the difference velocity between max linear velocity and next linear velocity
            (smaller is better)
        """

        return Robot.MAX_V - input[0]

    @classmethod
    def _eval_distance(cls, next, destination):
        """
        Parameters:
        ----------
        next: np.array(x, y, theta)
            candidate pose calculated from the Robot's motion model
        destination: np.array(x, y, theta)
            destination pose

        Returns:
        ----------
        float
            the distance between the destination and candidate pose
            (smaller is better)
        """

        return np.linalg.norm(next[:2] - destination[:2])

    @classmethod
    def _eval_theta(cls, next, destination):
        """
        Parameters:
        ----------
        next: np.array(x, y, theta)
            candidate pose calculated from the Robot's motion model
        destination: np.array(x, y, theta)
            destination pose

        Returns:
        ----------
        float
            the difference angle between the destination's theta and candidate pose's theta
            (smaller is better)
        """

        return np.abs(utils.normalize_angle(next[2] - destination[2]))
