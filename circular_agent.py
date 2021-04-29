#!/usr/bin/env python

import numpy as np

from src.environment import LANDMARKS
from src.agent import Agent
from src.filters import EKF
from src import utils, timer

INPUT_OMEGA = 0.2
INITIAL_POSE = (1.0, 0.0, np.pi / 2.0)


class CircularAgent(Agent):

    def get_ideal(self, t):
        """
        Parameters:
        ----------
        t: float
            elapsed time

        Returns:
        ----------
        np.array(x, y, theta)
            ideal pose of t
        """

        ideal = np.array((0, 0, np.pi/2.0)) + np.array((np.cos(INPUT_OMEGA * t),
                                                        np.sin(INPUT_OMEGA * t),
                                                        INPUT_OMEGA * t))
        ideal[2] = utils.normalize_angle(ideal[2])
        return ideal


if __name__ == '__main__':
    agent = CircularAgent(LANDMARKS)
    ekf = EKF(agent, INITIAL_POSE)
    timer.start(ekf)
