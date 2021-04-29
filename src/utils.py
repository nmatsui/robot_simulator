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


def normalize_min_max(x, axis=None):
    """
    Parameters:
    ----------
    x: np.array()
        target array
    axis: int
        target axis

    Returns:
    ----------
    np.array()
        normalized array so that the maximum value is 1 and the minimum value is 0
    """

    min = x.min(axis=axis, keepdims=True)
    max = x.max(axis=axis, keepdims=True)
    if max - min == 0:
        return np.full_like(x, 1.0)
    else:
        return (x - min) / (max - min)
