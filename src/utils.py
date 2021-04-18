import numpy as np


def normalize_angle(r):
    """
    Parameters:
    ----------
    r: float
        angle (radian)

    Returns:
    ----------
    float
        angle between -pi and +pi
    """

    return np.mod(r + np.pi, 2.0 * np.pi) - np.pi
