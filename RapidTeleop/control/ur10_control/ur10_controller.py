from __future__ import annotations

import time
from typing import Optional
import numpy as np

from .ur10_node import UR10Node


class UR10Controller:
    """
    High-level synchronous controller for UR10 robotic arm.

    Args:
        robot_ip (str): IP address of the UR10 robot.
        init_qpos (Optional[np.ndarray]): Initial joint configuration in radians. If None, defaults to zero pose.
        control_dt (float): Control loop timestep (seconds).
    """

    def __init__(
        self,
        robot_ip: str,
        init_qpos: Optional[np.ndarray] = None,
        control_dt: float = 0.01,
    ) -> None:
        self._arm = UR10Node(robot_ip=robot_ip)
        self._control_dt = control_dt

        # Use default zero configuration if not provided
        self._init_qpos = np.zeros(6) if init_qpos is None else np.asarray(init_qpos, dtype=float)
        if self._init_qpos.shape != (6,):
            raise ValueError("init_qpos must have shape (6,)")

        self.reset()

    def reset(self) -> None:
        """Move the arm to its initial joint configuration."""
        self.set_joint_positions(self._init_qpos)

    def set_joint_positions(self, joint_positions: np.ndarray) -> None:
        """
        Command the robot to move to the specified joint configuration.

        Args:
            joint_positions (np.ndarray): Target joint angles (6-element array).
        """
        joint_positions = np.asarray(joint_positions, dtype=float)
        if joint_positions.shape != (6,):
            raise ValueError(f"Expected joint_positions shape (6,), got {joint_positions.shape}")
        self._arm.move_to_joint_positions(joint_positions)

    # Alias for compatibility or convenience
    control_arm_qpos = set_joint_positions

    def get_arm_data(self) -> dict:
        """
        Retrieve current joint state and feedback data.

        Returns:
            dict: Contains current joint positions and other sensor feedback.
        """
        return self._arm.get_feedback()

    def stop(self) -> None:
        """
        Safely reset and shutdown the controller.
        """
        try:
            self.reset()
            time.sleep(0.1)
        finally:
            self._arm.close()

    def __enter__(self) -> UR10Controller:
        """Context manager enter."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> bool:
        """Context manager exit, ensuring safe shutdown."""
        self.stop()
        return False  # Propagate any exceptions
