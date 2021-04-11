#!/usr/bin/env python

import numpy as np

from src.environment import LANDMARKS
from src.agent import Agent
from src.filters import EKF
from src.plotter import plot

INPUT_OMEGA = 0.2
INITIAL_POSE = (1.0, 0.0, np.pi / 2.0)


class CircularAgent(Agent):

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
        """

        ideal = np.array((0, 0, np.pi/2.0)) + np.array((np.cos(INPUT_OMEGA * t),
                                                        np.sin(INPUT_OMEGA * t),
                                                        INPUT_OMEGA * t))
        # ideal[2] = np.mod(ideal[2], 2.0 * np.pi)
        return ideal


if __name__ == '__main__':
    agent = CircularAgent(LANDMARKS)
    ekf = EKF(agent)
    ekf.set_initial_values(INITIAL_POSE)
    plot(ekf)
