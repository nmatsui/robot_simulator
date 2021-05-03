import numpy as np

from src import utils


class Robot:

    MAX_LIN_ACC = 1.0
    MAX_ANG_ACC = 1.0

    MAX_V = 0.5
    MIN_V = 0.0
    MAX_OMEGA = 0.5
    MIN_OMEGA = -0.5

    @classmethod
    def T(cls, theta, omega, delta):
        """
        Parameters:
        ----------
        theta: float
            current theta
        omega: float
            input angular vector
        delta: float
            time delta

        Returns:
        ----------
        np.array.shape(3, 2)
            matrix to calculate diff (x, y, theta) from (v, omega)
        """

        return np.array([[np.cos(theta + omega * delta / 2.0) * delta, 0],
                         [np.sin(theta + omega * delta / 2.0) * delta, 0],
                         [0, delta]])

    @classmethod
    def move(cls, current, input, delta):
        """
        Parameters:
        ----------
        current: np.array(x, y, theta)
            current pose
        input: np.array(v, omega)
            input vector
        delta: float
            time delta

        Returns:
        ----------
        np.array(x, y, theta)
            predict pose of next tick
        """

        theta = current[2]
        omega = input[1]

        next = current + Robot.T(theta, omega, delta).dot(input.T)
        next = utils.normalize_angle(next)
        return next

    @classmethod
    def F(cls, current, input, delta):
        """
        Parameters:
        ----------
        current: np.array(x, y, theta)
            current pose
        input: np.array(v, omega)
            input vector
        delta: float
            time delta

        Returns:
        ----------
        np.array.shape(3, 3)
            jacobians of state equation
        """

        theta = current[2]

        v = input[0]
        omega = input[1]

        return np.array([[1.0, 0.0, -1.0 * np.sin(theta + omega * delta / 2.0) * delta * v],
                         [0.0, 1.0,  1.0 * np.cos(theta + omega * delta / 2.0) * delta * v],
                         [0.0, 0.0,  1.0]])
