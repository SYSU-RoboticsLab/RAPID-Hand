from __future__ import annotations
import time
from typing import Dict, Optional

import numpy as np

from .rapid_node import RapidNode


class RapidHandController:
    """High-level synchronous controller for the *Rapid Hand*."""

    def __init__(
        self,
        motor_port: str,
        pcb_port: str,
        use_camera: bool,
        tactile_order_dict: dict,
        joint_command_order_list: list,
        init_qpos: Optional[np.ndarray] = None,
    ) -> None:
        # Low‑level device driver
        self._hand = RapidNode(
            motor_port=motor_port,
            pcb_port=pcb_port,
            use_camera=use_camera,
            tactile_order_dict=tactile_order_dict,
            joint_command_order_list=joint_command_order_list,
        )

        # Default posture: 20‑DoF fully open hand
        self._init_qpos: np.ndarray = (
            np.zeros(20) if init_qpos is None else np.asarray(init_qpos, float)
        )
        if self._init_qpos.shape != (20,):
            raise ValueError("init_qpos must have shape (20,)")

        self.reset()

    def reset(self) -> None:
        """Move the hand back to its initial pose."""
        self.set_joint_positions(self._init_qpos)

    # Friendly alias for symmetry with other robot APIs
    control_hand_qpos = set_joint_positions = lambda self, q: (
        self._set_joint_positions(q)
    )

    def _set_joint_positions(self, joint_positions: np.ndarray) -> None:
        """Send an absolute position command to every finger joint."""
        joint_positions = np.asarray(joint_positions, float)
        if joint_positions.shape != (20,):
            raise ValueError(
                f"Expected joint_positions with shape (20,), got {joint_positions.shape}"
            )
        self._hand.set_joint_angle(joint_positions)

    def get_hand_data(self) -> Dict[str, np.ndarray]:
        """Return the latest sensor packet received from the hand."""
        return self._hand.get_data()

    def stop(self) -> None:
        """Safely disengage the controller and shut down serial links."""
        try:
            self.reset()
            time.sleep(0.1)
        finally:
            # Ensure low‑level resources are cleaned even on error
            self._hand.stop_process()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.stop()
        return False
