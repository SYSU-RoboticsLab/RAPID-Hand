import numpy as np


class UR10Node:
    """
    Skeleton class for low-level UR10 communication interface.

    This class should be implemented by the user to wrap socket or RTDE
    communication with the UR10 robot.
    """

    def __init__(self, robot_ip: str) -> None:
        """
        Initialize connection to UR10.

        Parameters
        ----------
        robot_ip : str
            IP address of the UR10 robot.
        """
        self.robot_ip = robot_ip

    def move_to_joint_positions(
        self,
        joint_positions: np.ndarray,
        acceleration: float = 1.2,
        velocity: float = 0.25,
    ) -> None:
        """
        Send a command to move the UR10 to the given joint positions.

        Parameters
        ----------
        joint_positions : np.ndarray
            Desired joint angles (6 values, in radians).
        acceleration : float
            Optional acceleration setting.
        velocity : float
            Optional velocity setting.
        """
        pass

    def get_feedback(self) -> dict:
        """
        Return current joint positions and relevant state information.

        Returns
        -------
        dict
            A dictionary containing the current state.
        """
        return {
            "joint_positions": np.zeros(6),
            "joint_velocities": np.zeros(6),
            "joint_current": np.zeros(6),
            "eef_speed": np.zeros(6),
            "ee_pos_quat": np.zeros(6),
        }

    def close(self) -> None:
        """
        Cleanly close connection to the UR10.
        """
        pass
