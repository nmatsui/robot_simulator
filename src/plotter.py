import numpy as np

import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation


class Plotter:

    def __init__(self, agent, ekf):
        """
        Parameters:
        ----------
        agent: extended class of src.agent.Agent
            an instance of the agent which implements 'cmd'
        ekf: src.filter.EKF
            an instance of EKF
        """

        self.agent = agent
        self.ekf = ekf
        self.ideal_list = np.array([()]).reshape(0, 3)
        self.xhat_list = np.array([()]).reshape(0, 3)

    def plot(self):
        """
        Notes:
        ----------
        * plot below trajectories
            * the ideal trajectory
            * the actual trajectory with randome noises
            * the estimated trajectory by using EKF
        * plot observed landmarks
        * plot estimated pose (x, y, theta) of robot
        """

        fig = plt.figure(figsize=(12.0, 12.0))
        ax = fig.add_subplot(111)

        def update(frame):
            ideal, xhat, P, K = self.ekf.step()
            self.ideal_list = np.append(self.ideal_list, np.array([ideal]), axis=0)
            self.xhat_list = np.append(self.xhat_list, np.array([xhat]), axis=0)

            ax.cla()
            ax.set_xlim([-1.2, 1.2])
            ax.set_ylim([-1.2, 1.2])

            for observed in self.agent.observed_list:
                self._plot_observed(ax, observed, 'green', 'gray')
            self._plot_ideal(ax, 'black')
            self._plot_actual(ax, 'blue')
            self._plot_estimated(ax, 'red')

            print(f'P={P}')
            print(f'K={K}')

        anim = FuncAnimation(fig, update, interval=100)

        def on_click(event):
            anim.event_source.stop()

        plt.connect('button_press_event', on_click)
        plt.show()

    def _plot_ideal(self, ax, color):
        """
        Parameters:
        ----------
        ax: Axes
            the axes to plot
        color: str
            the color of trajectory

        Notes:
        ----------
        plot the ideal trajectory
        """

        ax.plot(self.ideal_list[:, 0], self.ideal_list[:, 1], color=color)

    def _plot_actual(self, ax, color):
        """
        Parameters:
        ----------
        ax: Axes
            the axes to plot
        color: str
            the color of trajectory

        Notes:
        ----------
        plot the actual trajectory
        """

        ax.plot(self.agent.actual_list[:, 0], self.agent.actual_list[:, 1], color=color)

    def _plot_estimated(self, ax, color):
        """
        Parameters:
        ----------
        ax: Axes
            the axes to plot
        color: str
            the color of trajectory

        Notes:
        ----------
        plot the estimated trajectory and current pose
        """

        ax.plot(self.xhat_list[:, 0], self.xhat_list[:, 1], color=color)
        c = patches.Circle(xy=tuple(self.xhat_list[-1][:2]), radius=0.05, fc='none', ec=color)
        ax.add_patch(c)
        nose_x = [self.xhat_list[-1][0], self.xhat_list[-1][0] + 0.1 * np.cos(self.xhat_list[-1][2])]
        nose_y = [self.xhat_list[-1][1], self.xhat_list[-1][1] + 0.1 * np.sin(self.xhat_list[-1][2])]
        ax.plot(nose_x, nose_y, color=color, linewidth=5.0)

    def _plot_observed(self, ax, observed, color, mark_color):
        """
        Parameters:
        ----------
        ax: Axes
            the axes to plot
        observed: tuple(tuple(x, y), np.array(distance, angle))
            list of observed landmark (tuple of (landmark pos, observed distance and angle))
        color: str
            the color of observed sight line
        mark_color: str
            the color of observed landmark

        Notes:
        ----------
        plot the landmark and observed sight line
        """

        actual = self.agent.actual_list[-1]
        ax.plot(observed[0][0], observed[0][1], 's', color=mark_color)
        xs = [actual[0], actual[0] + observed[1][0] * np.cos(observed[1][1] + actual[2])]
        ys = [actual[1], actual[1] + observed[1][0] * np.sin(observed[1][1] + actual[2])]
        ax.plot(xs, ys, color=color)
