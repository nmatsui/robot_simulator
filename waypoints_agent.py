#!/usr/bin/env python

import itertools

import numpy as np

from src.environment import LANDMARKS
from src.robot import Robot
from src.agent import Agent
from src.filters import EKF
from src import utils, timer

INITIAL_POSE = (1.0, 0.0, np.pi / 2.0)
WAYPOINTS = [np.array((1.0, 0.5, np.pi * 3.0/4.0)), np.array((0.5, 1.0, -np.pi)), np.array((-0.5, 1.0, -np.pi/2.0)),
             np.array((-0.5, -1.0, 0.0)), np.array((1.0, -1.0, np.pi / 2.0))]

DISTANCE_THRESHOLD = 0.2
ANGLE_THRESHOLD = np.pi/18.0
NEAR_ACC_MAGNIFICATION = 0.5
NEAR_LINEAR_MAGNIFICATION = 0.3
NEAR_ANGULAR_MAGNIFICATION = 0.8


class WaypointsAgent(Agent):

    def __init__(self, landmarks):
        """
        Parameters:
        ----------
        landmarks: list of tuple (x, y)
            list of the landmark coordination
        """

        super().__init__(landmarks)
        self.itr = itertools.cycle(WAYPOINTS)
        self.target = np.array(INITIAL_POSE)

    def get_ideal(self, current, t):
        """
        Parameters:
        ----------
        current: np.array(x, y, theta)
            current pose
        t: float
            elapsed time (is not used in this agent)

        Returns:
        ----------
        np.array(x, y, theta)
            ideal pose of t
        """

        if np.linalg.norm(self.target[:2] - current[:2]) < DISTANCE_THRESHOLD and \
            np.abs(utils.normalize_angle(self.target[2] - current[2])) < ANGLE_THRESHOLD:

            self.target = next(self.itr)

        return self.target

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
        m = NEAR_ACC_MAGNIFICATION if np.linalg.norm(self.target[:2] - current[:2]) < DISTANCE_THRESHOLD else 1.0
        return (Robot.MAX_LIN_ACC * m, Robot.MAX_ANG_ACC * m)

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
        m = NEAR_LINEAR_MAGNIFICATION if np.linalg.norm(self.target[:2] - current[:2]) < DISTANCE_THRESHOLD else 1.0
        return (Robot.MAX_V * m, Robot.MIN_V * m)

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
        m = NEAR_ANGULAR_MAGNIFICATION if np.linalg.norm(self.target[:2] - current[:2]) < DISTANCE_THRESHOLD else 1.0
        return (Robot.MAX_OMEGA * m, Robot.MIN_OMEGA * m)


if __name__ == '__main__':
    agent = WaypointsAgent(LANDMARKS)
    ekf = EKF(agent, INITIAL_POSE)
    timer.start(ekf)
