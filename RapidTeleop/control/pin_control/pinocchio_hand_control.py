from threading import Lock
from typing import List, Optional, Dict
import os
import numpy as np
import pinocchio as pin


class PinocchioHandControl:
    """
    A motion control class for hand models using the Pinocchio library.
    Supports joint and link transformations based on a URDF description.
    """

    def __init__(self, hand_name: str, robot_urdf_file: str):
        """
        Initialize the hand model by loading a URDF and building the kinematic model.

        Args:
            hand_name (str): Name identifier for the hand model.
            robot_urdf_file (str): Relative path to the URDF file under 'assets/robots/hands'.
        """
        self.robot_name = hand_name
        self._qpos_lock = Lock()

        urdf_path = os.path.abspath(os.path.join(
            os.path.dirname(__file__), "../..", "assets/robots/hands", robot_urdf_file
        ))

        # Load URDF into Pinocchio model and data
        self.model: pin.Model = pin.buildModelFromUrdf(urdf_path)
        self.data: pin.Data = self.model.createData()

        # Build frame-to-index mappings
        self.link_mapping: Dict[str, int] = {}
        self.joint_mapping: Dict[str, int] = {}
        for i, frame in enumerate(self.model.frames):
            if frame.type == pin.FrameType.BODY:
                self.link_mapping[frame.name] = i
            elif frame.type == pin.FrameType.JOINT:
                self.joint_mapping[frame.name] = i

        # Set initial joint configuration
        self.qpos = pin.neutral(self.model)
        self.set_current_qpos(self.qpos)

    def get_current_qpos(self) -> np.ndarray:
        """
        Get the current joint configuration.

        Returns:
            np.ndarray: Joint position vector.
        """
        with self._qpos_lock:
            return self.qpos.copy()

    def set_current_qpos(self, qpos: np.ndarray):
        """
        Set a new joint configuration and update all frame placements.

        Args:
            qpos (np.ndarray): Joint position vector.
        """
        with self._qpos_lock:
            self.qpos = qpos
            pin.forwardKinematics(self.model, self.data, qpos)
            pin.updateFramePlacements(self.model, self.data)

    def get_link_transform(self, link_name: str) -> np.ndarray:
        """
        Retrieve the world transformation matrix for a specified link.

        Args:
            link_name (str): Name of the link frame.

        Returns:
            np.ndarray: 4x4 homogeneous transformation matrix.
        """
        if link_name not in self.link_mapping:
            raise ValueError(f"Link '{link_name}' not found in URDF.")
        frame_id = self.link_mapping[link_name]
        return self.data.oMf[frame_id].homogeneous

    def get_joint_transform(self, joint_name: str) -> np.ndarray:
        """
        Retrieve the world transformation matrix for a specified joint.

        Args:
            joint_name (str): Name of the joint frame.

        Returns:
            np.ndarray: 4x4 homogeneous transformation matrix.
        """
        if joint_name not in self.joint_mapping:
            raise ValueError(f"Joint '{joint_name}' not found in URDF.")
        frame_id = self.joint_mapping[joint_name]
        return self.data.oMf[frame_id].homogeneous

    def get_dof(self) -> int:
        """
        Get the number of degrees of freedom (DoF) for the hand model.

        Returns:
            int: DoF count.
        """
        return pin.neutral(self.model).shape[0]

    def get_joint_names(self) -> List[str]:
        """
        Get the list of joint names (excluding the root 'universe').

        Returns:
            List[str]: List of joint name strings.
        """
        return self.model.names[1:]

    def get_link_names(self) -> List[str]:
        """
        Get the list of all link (body) names.

        Returns:
            List[str]: List of link name strings.
        """
        return list(self.link_mapping.keys())


if __name__ == "__main__":
    model = PinocchioHandControl(
        hand_name="rapid_hand",
        robot_urdf_file="rapid_hand_right.urdf",
    )
    link_names = model.get_link_names()
    print(f"Loaded {len(link_names)} links:\n", link_names)

    transforms = {
        name: model.get_link_transform(name)[:3, 3]
        for name in link_names
    }

    print("Link positions:")
    for name, pos in transforms.items():
        print(f"{name}: {pos}")

    # Example query
    try:
        base_transform = model.get_link_transform("BASE")
        print("BASE transform:\n", base_transform)
    except ValueError as e:
        print("Error:", e)
