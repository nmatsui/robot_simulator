import abc

import numpy as np

from src.robot import Robot


class Agent(metaclass=abc.ABCMeta):
    actual_xy_sd = 0.005
    actual_theta_sd = 0.01
    ovserve_dist_sd = 0.02
    ovserve_angle_sd = 0.02

    def __init__(self, landmarks):
        """
        Parameters:
        ----------
        landmarks: list of tuple (x, y)
            list of the landmark coordination
        """

        self.actual = None
        self.observed_list = []
        self.landmarks = landmarks

    @abc.abstractmethod
    def get_ideal(self, current, t):
        """
        Parameters:
        ----------
        current: np.array(x, y, theta)
            current pose
        t: float
            elapsed time

        Returns:
        ----------
        np.array(x, y, theta)
            ideal pose of t

        Notes:
        ----------
        abstract method to calculate the ideal pose of t
        """

        raise NotImplementedError

    def get_max_accelarations(self, current):
        """
        Parameters:
        ----------
        current: np.array(x, y, theta)
            current pose (is not used in this default implement)

        Returns:
        ----------
        Tuple (float, float)
            tuple of the max linear accelaration and the max angular accelaration
        """
        return (Robot.MAX_LIN_ACC, Robot.MAX_ANG_ACC)

    def get_linear_velocities(self, current):
        """
        Parameters:
        ----------
        current: np.array(x, y, theta)
            current pose (is not used in this default implement)

        Returns:
        ----------
        Tuple (float, float)
            tuple of the max linear velocity and the min linear velocity
        """
        return (Robot.MAX_V, Robot.MIN_V)

    def get_angular_velocities(self, current):
        """
        Parameters:
        ----------
        current: np.array(x, y, theta)
            current pose (is not used in this default implement)

        Returns:
        ----------
        Tuple (float, float)
            tuple of the max angular velocity and the min angular velocity
        """
        return (Robot.MAX_OMEGA, Robot.MIN_OMEGA)

    def move(self, current, input, delta):
        """
        Parameters:
        ----------
        current: np.array(x, y, theta)
            current pose
        input: np.array(v, omega)
            input vector of linear velocity and angular velocity
        delta: float
            time delta of this tick

        Notes:
        ----------
        calculate actual pose with random noise
        """

        moved = Robot.move(current, input, delta)
        self.actual = np.array([np.random.normal(moved[0], Agent.actual_xy_sd),
                                np.random.normal(moved[1], Agent.actual_xy_sd),
                                np.random.normal(moved[2], Agent.actual_theta_sd)])

    def get_observations(self):
        """
        Returns:
        ----------
        list of tuple(landmark (x, y), np.array(distance, angle))
            list of observed landmark (tuple of (landmark pos, observed distance and angle))
        """

        self.observed_list = [(landmark, self._observe(landmark, self.actual)) for landmark in self.landmarks]
        return self.observed_list

    def _observe(self, landmark, actual):
        """
        Parameters:
        ----------
        actual: np.array(x, y, theta)
            actual pose

        Returns:
        ----------
        np.array(distance, angle)
            observed distance and angle of a landmark with noise
        """

        dist = np.random.normal(np.linalg.norm(np.array(landmark) - actual[:2]), Agent.ovserve_dist_sd)
        angle = np.random.normal(np.arctan2(landmark[1] - actual[1], landmark[0] - actual[0]) - actual[2],
                                 Agent.ovserve_angle_sd)
        return np.array([dist, angle])
