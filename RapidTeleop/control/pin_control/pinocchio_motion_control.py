from threading import Lock
from pathlib import Path
from typing import List, Optional, Dict

import numpy as np
import pinocchio as pin
import yaml
import os

from .base import BaseMotionControl


class PinocchioMotionControl(BaseMotionControl):
    """
    A motion control class based on the Pinocchio library, providing
    forward and inverse kinematics for robotic systems.
    """

    def __init__(
        self, robot_config_path: str, robot_config_dict: Optional[dict] = None
    ):
        """
        Initialize the motion controller using a robot config file or dictionary.

        Args:
            robot_config_path (str): Absolute path to the robot YAML configuration file.
            robot_config_dict (dict, optional): Pre-loaded configuration dictionary.
        """
        self._qpos_lock = Lock()

        if robot_config_dict is None:
            robot_config_path = Path(robot_config_path)
            if not robot_config_path.is_absolute():
                raise RuntimeError(
                    f"Robot config path must be absolute: {robot_config_path}"
                )

            with robot_config_path.open("r") as f:
                cfg = yaml.safe_load(f)["robot_cfg"]
            urdf_path = self.get_urdf_absolute_path(cfg, robot_config_path)
        else:
            cfg = robot_config_dict["robot_cfg"]
            urdf_path = os.path.abspath(
                os.path.join(
                    os.path.dirname(__file__),
                    "../..",
                    "assets/robots",
                    cfg["urdf_path"],
                )
            )

        # Load kinematics configuration
        self.ik_damping = float(cfg["kinematics"]["ik_damping"]) * np.eye(6)
        self.ik_eps = float(cfg["kinematics"]["eps"])
        self.dt = float(cfg["dt"])
        self.ee_name = cfg["kinematics"]["ee_link"]

        # Build model and initialize data
        self.model: pin.Model = pin.buildModelFromUrdf(str(urdf_path))
        self.data: pin.Data = self.model.createData()

        # Map frame names to indices
        self.frame_mapping: Dict[str, int] = {
            frame.name: i for i, frame in enumerate(self.model.frames)
        }
        if self.ee_name not in self.frame_mapping:
            raise ValueError(
                f"End effector '{self.ee_name}' not found in URDF: {urdf_path}"
            )
        self.ee_frame_id = self.frame_mapping[self.ee_name]

        # Initialize joint positions and compute initial forward kinematics
        self.qpos = pin.neutral(self.model)
        pin.forwardKinematics(self.model, self.data, self.qpos)
        self.ee_pose: pin.SE3 = pin.updateFramePlacement(
            self.model, self.data, self.ee_frame_id
        )

    def step(self, pose: Optional[np.ndarray], repeat: int = 1):
        """
        Perform inverse kinematics to move the end-effector toward the target pose.

        Args:
            pose (np.ndarray): 7D target pose [x, y, z, qx, qy, qz, qw].
            repeat (int): Number of internal IK iterations multiplier.
        """
        if pose is None:
            return  # No-op if no target

        oMdes = pin.XYZQUATToSE3(pose)

        with self._qpos_lock:
            qpos = self.qpos.copy()

        for _ in range(100 * repeat):
            pin.forwardKinematics(self.model, self.data, qpos)
            ee_pose = pin.updateFramePlacement(self.model, self.data, self.ee_frame_id)
            J = pin.computeFrameJacobian(self.model, self.data, qpos, self.ee_frame_id)
            iMd = ee_pose.actInv(oMdes)
            err = pin.log(iMd).vector

            if np.linalg.norm(err) < self.ik_eps:
                break

            dq = J.T @ np.linalg.solve(J @ J.T + self.ik_damping, err)
            qpos = pin.integrate(self.model, qpos, dq * self.dt)

        self.set_current_qpos(qpos)

    def compute_ee_pose(self, qpos: np.ndarray) -> np.ndarray:
        """
        Compute the current end-effector pose given a joint configuration.

        Args:
            qpos (np.ndarray): Joint positions.

        Returns:
            np.ndarray: 7D pose as [x, y, z, qw, qx, qy, qz].
        """
        pin.forwardKinematics(self.model, self.data, qpos)
        oMf: pin.SE3 = pin.updateFramePlacement(self.model, self.data, self.ee_frame_id)
        xyzw = pin.SE3ToXYZQUAT(oMf)
        return np.concatenate([xyzw[:3], [xyzw[6], xyzw[3], xyzw[4], xyzw[5]]])

    def get_current_qpos(self) -> np.ndarray:
        """
        Return the current joint configuration.

        Returns:
            np.ndarray: Joint positions.
        """
        with self._qpos_lock:
            return self.qpos.copy()

    def set_current_qpos(self, qpos: np.ndarray):
        """
        Set the current joint configuration and update the forward kinematics.

        Args:
            qpos (np.ndarray): New joint positions.
        """
        with self._qpos_lock:
            self.qpos = qpos
            pin.forwardKinematics(self.model, self.data, qpos)
            self.ee_pose = pin.updateFramePlacement(
                self.model, self.data, self.ee_frame_id
            )

    def get_ee_name(self) -> str:
        """
        Return the name of the end-effector link.

        Returns:
            str: End-effector frame name.
        """
        return self.ee_name

    def get_dof(self) -> int:
        """
        Return the number of degrees of freedom (DoF) of the robot.

        Returns:
            int: DoF.
        """
        return pin.neutral(self.model).shape[0]

    def get_timestep(self) -> float:
        """
        Return the control timestep.

        Returns:
            float: Time step in seconds.
        """
        return self.dt

    def get_joint_names(self) -> List[str]:
        """
        Return the list of joint names (excluding the 'universe' joint).

        Returns:
            List[str]: Joint names.
        """
        return self.model.names[1:]

    def is_use_gpu(self) -> bool:
        """
        Indicate whether the controller uses GPU (always False in this implementation).

        Returns:
            bool: GPU usage flag.
        """
        return False
