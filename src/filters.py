import time

import numpy as np

from src.robot import Robot
from src.camera import Camera


class EKF:
    q = 0.01
    r = 0.04

    def __init__(self, agent):
        """
        Parameters:
        ----------
        agent: src.agent.Agent
            agent of robot

        Notes:
        ----------
        initialize xhat_list, P_list, K_list
        """

        self.ideal_list = np.array([()]).reshape(0, 3)
        self.xhat_list = np.array([()]).reshape(0, 3)
        self.P_list = np.array([()]).reshape(0, 3, 3)
        self.K_list = np.array([()]).reshape(0, 3, 2)
        self.agent = agent

    def set_initial_values(self, initial):
        """
        Parameters:
        ----------
        initial: np.array(x, y, theta)
            initial pose

        Notes:
        ----------
        prepare xhat_list, P_list, K_list, Q, R
        """

        self.xhat_list = np.append(self.xhat_list, np.array([initial]), axis=0)
        self.P_list = np.append(self.P_list, np.array([np.zeros((3, 3))]), axis=0)
        self.K_list = np.append(self.K_list, np.array([np.zeros((3, 2))]), axis=0)
        self.Q = np.dot(EKF.q, np.identity(3))
        self.R = np.dot(EKF.r, np.identity(2))
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
            predicted pose and predicted covariance
        """

        a_priori_x = Robot.move(self.xhat_list[-1], input, delta)
        F = Robot.F(self.xhat_list[-1], input, delta)
        a_priori_P = F.dot(self.P_list[-1]).dot(F.T) + self.Q
        return a_priori_x, a_priori_P

    def update(self, a_priori_x, a_priori_P, landmark, observed):
        """
        Parameters:
        ----------
        a_priori_x: np.array(x, y, theta)
            predicted pose
        a_priori_p: np.array().size(3, 3)
            predicted covariance
        landmark: tuple(x, y)
            coordinate of observed landmark
        observed: np.array(distance, angle)
            distance and angle of observed landmark

        Returns:
        ----------
        tuple(np.array(x, y, theta), np.array().size(3, 3), np.array().size(3, 2))
            updated pose ,covariance and kalman gain
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
        Parameters:
        ----------
        t: float
            current unixtime

        Notes:
        ----------
        calculate pose, covariance and kalman gain of this tick
        """

        t = time.time()
        delta = t - self.t
        ideal = self.agent.next_tick(t - self.start_t)
        input = self.agent.get_input(self.xhat_list[-1], ideal, delta)
        xhat, P = self.predict(input, delta)
        K = None
        for landmark, observed in self.agent.get_observations():
            xhat, P, K = self.update(xhat, P, landmark, observed)

        self.ideal_list = np.append(self.ideal_list, np.array([ideal]), axis=0)
        self.xhat_list = np.append(self.xhat_list, np.array([xhat]), axis=0)
        self.P_list = np.append(self.P_list, np.array([P]), axis=0)
        self.K_list = np.append(self.K_list, np.array([K]), axis=0)
        self.t = t
