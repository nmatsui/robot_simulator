import time

import numpy as np

from src.robot import Robot
from src.camera import Camera
from src.planner import DWAwoObstacle


class EKF:
    q = 0.01
    r = 0.02

    def __init__(self, agent, initial):
        """
        Parameters:
        ----------
        agent: src.agent.Agent
            agent of robot
        initial: np.array(x, y, theta)
            initial pose

        Notes:
        ----------
        initialize xhat, P, Q, R and start_time
        """

        self.agent = agent
        self.xhat = np.array(initial)
        self.P = np.zeros((3, 3))
        self.Q = np.dot(EKF.q, np.identity(3))
        self.R = np.dot(EKF.r, np.identity(2))
        self.input = np.array((0, 0))
        self.start_t = time.time()
        self.t = self.start_t

    def predict(self, input, delta):
        """
        Parameters:
        ----------
        input: np.array(v, omega)
            input vector of linear velocity and angular velocity
        delta: float
            time delta

        Returns:
        ----------
        tuple(np.array(x, y, theta), np.array().size(3, 3))
            predicted pose and covariance
        """

        a_priori_x = Robot.move(self.xhat, input, delta)
        F = Robot.F(self.xhat, input, delta)
        a_priori_P = F.dot(self.P).dot(F.T) + self.Q
        return a_priori_x, a_priori_P

    def update(self, a_priori_x, a_priori_P, landmark, observed):
        """
        Parameters:
        ----------
        a_priori_x: np.array(x, y, theta)
            predicted pose
        a_priori_P: np.array().size(3, 3)
            predicted covariance
        landmark: tuple(x, y)
            coordinate of observed landmark
        observed: np.array(distance, angle)
            distance and angle of observed landmark

        Returns:
        ----------
        tuple(np.array(x, y, theta), np.array().size(3, 3), np.array().size(3, 2))
            updated pose, covariance and kalman gain
        """

        yhat = observed - Camera.observe(landmark, a_priori_x)
        H = Camera.H(landmark, a_priori_x)
        S = H.dot(a_priori_P).dot(H.T) + self.R
        K = a_priori_P.dot(H.T).dot(np.linalg.inv(S))
        xhat = a_priori_x + K.dot(yhat)
        P = (np.identity(3) - K.dot(H)).dot(a_priori_P)
        return xhat, P, K

    def step(self):
        """
        Returns:
        ----------
        tuple(np.array(x, y, theta), np.array(x, y, theta), np.array().size(3, 3), np.array().size(3, 2))
            ideal pose, estimated pose, covariance and kalman gain

        Notes:
        ----------
        estimate estimated pose of this time tick by using kalman filter
        """

        t = time.time()
        delta = t - self.t
        ideal = self.agent.get_ideal(t - self.start_t)
        input = DWAwoObstacle.get_input(self.xhat, ideal, self.input, delta)
        self.agent.move(self.xhat, input, delta)
        xhat, P = self.predict(input, delta)
        K = None
        for landmark, observed in self.agent.get_observations():
            xhat, P, K = self.update(xhat, P, landmark, observed)

        self.xhat = xhat
        self.P = P
        self.t = t
        self.input = input
        return ideal, xhat, P, K
