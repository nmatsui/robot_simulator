import numpy as np


class Camera:

    @classmethod
    def observe(cls, landmark, current):
        """
        Parameters:
        ----------
        landmark: tuple(x, y)
            coordinate of landmark
        current: np.array(x, y, theta)
            current pose

        Returns:
        ----------
        np.array(distance, angle)
            predict observation of landmark
        """

        return np.array([np.linalg.norm(np.array(landmark) - current[:2]),
                         np.arctan2(landmark[1] - current[1], landmark[0] - current[0]) - current[2]])

    @classmethod
    def H(cls, landmark, current):
        """
        Parameters:
        ----------
        landmark: tuple(x, y)
            coordinate of landmark
        current: np.array(x, y, theta)
            current pose

        Returns:
        ----------
        np.array.shape(2, 3)
            jacobians of observe equation
        """

        q = (landmark[0] - current[0]) ** 2 + (landmark[1] - current[1]) ** 2
        return np.array([[(current[0] - landmark[0])/np.sqrt(q), (current[1] - landmark[1])/np.sqrt(q), 0.0],
                         [(landmark[1] - current[1])/q, (current[0] - landmark[0])/q, -1.0]])
