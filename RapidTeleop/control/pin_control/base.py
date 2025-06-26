from abc import abstractmethod
from typing import List, Optional
from pathlib import Path

import numpy as np


class BaseMotionControl:
    @abstractmethod
    def step(self, pos: Optional[np.ndarray], quat: Optional[np.ndarray], repeat=1):
        pass

    @abstractmethod
    def get_current_qpos(self) -> np.ndarray:
        pass

    @abstractmethod
    def set_current_qpos(self, qpos: np.ndarray):
        pass

    @abstractmethod
    def compute_ee_pose(self, qpos: np.ndarray) -> np.ndarray:
        pass

    @abstractmethod
    def get_ee_name(self) -> str:
        pass

    @abstractmethod
    def get_dof(self) -> int:
        pass

    @abstractmethod
    def get_timestep(self) -> float:
        pass

    @abstractmethod
    def get_joint_names(self) -> List[str]:
        pass

    @abstractmethod
    def is_use_gpu(self) -> bool:
        pass

    @staticmethod
    def get_urdf_absolute_path(cfg: dict, robot_config_path: Path) -> Path:
        urdf_path = Path(cfg["urdf_path"])
        if not urdf_path.is_absolute():
            base_path = robot_config_path.parent.parent / "assets/robots"
            urdf_path = (base_path / urdf_path).resolve().absolute()
        return urdf_path