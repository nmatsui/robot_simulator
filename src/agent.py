import abc

import numpy as np

from src.robot import Robot


class Agent(metaclass=abc.ABCMeta):
    actual_sd = 0.01
    ovserve_dist_sd = 0.05
    ovserve_angle_sd = 0.04

    def __init__(self, landmarks):
        """
        Parameters:
        ----------
        landmarks: list of tuple (x, y)
            list of the landmark coordination
        """

        self.actual_list = np.array([()]).reshape(0, 3)
        self.observed_list = []
        self.landmarks = landmarks

    @abc.abstractmethod
    def cmd(self, t):
        """
        Parameters:
        ----------
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

    def next_tick(self, t):
        """
        Parameters:
        ----------
        t: float
            elapsed time

        Returns:
        ----------
        np.array(x, y, theta)
            ideal pose of t

        Notes:
        ----------
        when called 'next_tick', the ideal and actual pose is calculated
        """
        ideal = self.cmd(t)

        actual = np.random.normal(ideal, Agent.actual_sd, 3)
        self.actual_list = np.append(self.actual_list, np.array([actual]), axis=0)

        return ideal

    def get_observations(self):
        """
        Returns:
        ----------
        list of tuple(landmark (x, y), np.array(distance, angle))
            list of observed landmark (tuple of (landmark pos, observed distance and angle))
        """

        self.observed_list = [(landmark, self._observe(landmark, self.actual_list[-1])) for landmark in self.landmarks]
        return self.observed_list

    def get_input(self, current, destination, delta):
        """
        Parameters:
        ----------
        current: np.array(x, y, theta)
            current pose
        destination: np.array(x, y, theta)
            destination pose
        delta: float
            time delta of this tick

        Returns:
        ----------
        np.array(v, omega)
            input vector of linear velocity and angular velocity
        """

        theta = current[2]
        omega = (destination[2] - theta) / delta

        return np.linalg.pinv(Robot.T(theta, omega, delta)).dot((destination - current))

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
