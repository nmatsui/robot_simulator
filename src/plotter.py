import numpy as np

import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation


def plot(ekf):
    """
    Parameters:
    ----------
    ekf: src.filters.EKF
        ekf class

    Notes:
    ----------
    plot below trajectories
        * the ideal trajectory
        * the actual trajectory with randome noises
        * the estimated trajectory by using EKF
    plot observed landmarks
    plot estimated robot pose (x, y, theta)
    """

    fig = plt.figure(figsize=(12.0, 12.0))
    ax = fig.add_subplot(111)

    def update(frame):
        ekf.step()
        ax.cla()
        ax.set_xlim([-1.2, 1.2])
        ax.set_ylim([-1.2, 1.2])

        for observed in ekf.agent.observed_list:
            ax.plot(observed[0][0], observed[0][1], 's', color='gray')
            xs = [ekf.agent.actual_list[-1][0],
                  ekf.agent.actual_list[-1][0] + observed[1][0] * np.cos(observed[1][1] + ekf.agent.actual_list[-1][2])]
            ys = [ekf.agent.actual_list[-1][1],
                  ekf.agent.actual_list[-1][1] + observed[1][0] * np.sin(observed[1][1] + ekf.agent.actual_list[-1][2])]
            ax.plot(xs, ys, color='green')
        ax.plot(ekf.ideal_list[:, 0], ekf.ideal_list[:, 1], color='black')
        ax.plot(ekf.agent.actual_list[:, 0], ekf.agent.actual_list[:, 1], color='blue')
        ax.plot(ekf.xhat_list[:, 0], ekf.xhat_list[:, 1], color='red')
        c = patches.Circle(xy=tuple(ekf.xhat_list[-1][:2]), radius=0.05, fc='none', ec='red')
        ax.add_patch(c)
        nose_x = [ekf.xhat_list[-1][0], ekf.xhat_list[-1][0] + 0.1 * np.cos(ekf.xhat_list[-1][2])]
        nose_y = [ekf.xhat_list[-1][1], ekf.xhat_list[-1][1] + 0.1 * np.sin(ekf.xhat_list[-1][2])]
        ax.plot(nose_x, nose_y, color='red', linewidth=5.0)

        print(f'P={ekf.P_list[-1]}')
        print(f'K={ekf.K_list[-1]}')

    anim = FuncAnimation(fig, update, interval=100)

    def on_click(event):
        anim.event_source.stop()

    plt.connect('button_press_event', on_click)
    plt.show()
