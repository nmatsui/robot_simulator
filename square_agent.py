#!/usr/bin/env python

import numpy as np

from src.environment import LANDMARKS
from src.agent import Agent
from src.filters import EKF
from src.plotter import Plotter
from src import utils

INPUT_V = 0.2
INPUT_OMEGA = 0.2
INITIAL_POSE = (1.0, 0.0, np.pi / 2.0)


class SquareAgent(Agent):

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

        d0 = 0.0
        d1 = d0 + 1.0 / INPUT_V
        d2 = d1 + np.pi / 2.0 / INPUT_OMEGA
        d3 = d2 + 2.0 / INPUT_V
        d4 = d3 + np.pi / 2.0 / INPUT_OMEGA
        d5 = d4 + 2.0 / INPUT_V
        d6 = d5 + np.pi / 2.0 / INPUT_OMEGA
        d7 = d6 + 2.0 / INPUT_V
        d8 = d7 + np.pi / 2.0 / INPUT_OMEGA
        d9 = d8 + 1.0 / INPUT_V

        delta = t % d9

        if d0 <= delta < d1:
            ideal = np.array((1.0, 0 + INPUT_V * delta, np.pi / 2.0))
        elif d1 <= delta < d2:
            ideal = np.array((1.0, 1.0, np.pi / 2.0 + INPUT_OMEGA * (delta - d1)))
        elif d2 <= delta < d3:
            ideal = np.array((1.0 - INPUT_V * (delta - d2), 1.0, np.pi))
        elif d3 <= delta < d4:
            ideal = np.array((-1.0, 1.0, np.pi + INPUT_OMEGA * (delta - d3)))
        elif d4 <= delta < d5:
            ideal = np.array((-1.0, 1.0 - INPUT_V * (delta - d4), np.pi * 3.0 / 2.0))
        elif d5 <= delta < d6:
            ideal = np.array((-1.0, -1.0, np.pi * 3.0 / 2.0 + INPUT_OMEGA * (delta - d5)))
        elif d6 <= delta < d7:
            ideal = np.array((-1.0 + INPUT_V * (delta - d6), -1.0, 0.0))
        elif d7 <= delta < d8:
            ideal = np.array((1.0, -1.0, INPUT_OMEGA * (delta - d7)))
        elif d8 <= delta < d9:
            ideal = np.array((1.0, -1.0 + INPUT_V * (delta - d8), np.pi / 2.0))
        else:
            raise NotImplementedError
        ideal[2] = utils.normalize_angle(ideal[2])
        return ideal


if __name__ == '__main__':
    agent = SquareAgent(LANDMARKS)
    ekf = EKF(agent, INITIAL_POSE)
    Plotter(agent, ekf).plot()
