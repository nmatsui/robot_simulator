#!/usr/bin/env python

import json
import threading

import numpy as np

import zmq

import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation

HOST = 'localhost'
PORT = 5556


class Plotter:

    def __init__(self):
        """
        Notes:
        ----------
        prepare ideal_list, actual_list, xhat_list, observed_list to plot them
        """

        self.ideal_list = np.array([()]).reshape(0, 3)
        self.actual_list = np.array([()]).reshape(0, 3)
        self.xhat_list = np.array([()]).reshape(0, 3)
        self.observed_list = []

    def start(self):
        """
        Notes:
        ----------
        start subscribering data by using ZMQ and plot them
        """

        subscriber = threading.Thread(target=self.subscribe)
        subscriber.start()
        self.plot()

    def subscribe(self):
        """
        Notes:
        ----------
        subscribe data by using zmq
        """

        context = zmq.Context()
        subscriber = context.socket(zmq.SUB)
        subscriber.connect(f'tcp://{HOST}:{PORT}')
        subscriber.setsockopt(zmq.SUBSCRIBE, ''.encode('utf-8'))

        while True:
            data = subscriber.recv()
            msg = json.loads(data.decode('utf-8'))
            self.ideal_list = np.append(self.ideal_list,
                                        np.array([[msg['ideal']['x'], msg['ideal']['y'], msg['ideal']['theta']]]),
                                        axis=0)
            self.actual_list = np.append(self.actual_list,
                                         np.array([[msg['actual']['x'], msg['actual']['y'], msg['actual']['theta']]]),
                                         axis=0)
            self.xhat_list = np.append(self.xhat_list,
                                       np.array([[msg['xhat']['x'], msg['xhat']['y'], msg['xhat']['theta']]]),
                                       axis=0)
            self.observed_list = [(o['landmark']['x'], o['landmark']['y'], o['distance'], o['angle']) for o in msg['observed']]
            print(f'covariance : {msg["covariance"]}')
            print(f'kalman gain: {msg["kalmanGain"]}')

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
            ax.cla()
            ax.set_xlim([-1.2, 1.2])
            ax.set_ylim([-1.2, 1.2])

            for observed in self.observed_list:
                self._plot_observed(ax, observed, 'green', 'gray')
            if len(self.ideal_list) > 0:
                self._plot_ideal(ax, 'black')
            if len(self.actual_list):
                self._plot_actual(ax, 'blue')
            if len(self.xhat_list):
                self._plot_estimated(ax, 'red')

        anim = FuncAnimation(fig, update, interval=500)

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
        c = patches.Circle(xy=tuple(self.ideal_list[-1][:2]), radius=0.02, fc='none', ec=color)
        ax.add_patch(c)
        nose_x = [self.ideal_list[-1][0], self.ideal_list[-1][0] + 0.03 * np.cos(self.ideal_list[-1][2])]
        nose_y = [self.ideal_list[-1][1], self.ideal_list[-1][1] + 0.03 * np.sin(self.ideal_list[-1][2])]
        ax.plot(nose_x, nose_y, color=color, linewidth=2.0)

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

        ax.plot(self.actual_list[:, 0], self.actual_list[:, 1], color=color)

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
        observed: tuple(landmark x, landmark y, distance, angle)
            tuple of observed landmark
        color: str
            the color of observed sight line
        mark_color: str
            the color of observed landmark

        Notes:
        ----------
        plot the landmark and observed sight line
        """

        actual = self.actual_list[-1]
        ax.plot(observed[0], observed[1], 's', color=mark_color)
        xs = [actual[0], actual[0] + observed[2] * np.cos(observed[3] + actual[2])]
        ys = [actual[1], actual[1] + observed[2] * np.sin(observed[3] + actual[2])]
        ax.plot(xs, ys, color=color)


if __name__ == '__main__':
    Plotter().start()
