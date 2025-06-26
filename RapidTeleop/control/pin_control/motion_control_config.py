from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Dict, Union

import yaml

from .base import BaseMotionControl


@dataclass
class MotionControlConfig:
    type: str

    robot_name: str
    robot_config_path: str

    # Low pass filter
    low_pass_alpha: float = 0.1

    # Orientation control
    disable_orientation_control: bool = False

    # Motion scaling factor, factor smaller than 1 means that robot hand will move smaller distance than human hand
    motion_scaling_factor: float = 1

    _TYPE = ["pinocchio"]
    _DEFAULT_ROBOT_CONFIG_DIR = "."

    def __post_init__(self):
        # Motion control type check
        if self.type not in self._TYPE:
            raise ValueError(f"Motion control type must be one of {self._TYPE}")

        # Robot configs file path check
        robot_config_path = Path(self.robot_config_path)
        if not robot_config_path.is_absolute():
            robot_config_path = self._DEFAULT_ROBOT_CONFIG_DIR / robot_config_path
            robot_config_path = robot_config_path.absolute()
        if not robot_config_path.exists():
            raise ValueError(f"Config path {robot_config_path} does not exist")
        self.robot_config_path = str(robot_config_path)

    @classmethod
    def from_file(cls, config_path):
        path = Path(config_path)
        if not path.is_absolute():
            path = path.absolute()

        with path.open("r") as f:
            yaml_config = yaml.load(f, Loader=yaml.FullLoader)
            cfg = yaml_config["control"]
            if cfg is None:
                return None
            else:
                config = MotionControlConfig(**cfg)
                return config

    @classmethod
    def from_dict(cls, cfg: Dict):
        if cfg is None:
            return None
        else:
            config = MotionControlConfig(**cfg)
            return config

    def build(self, device="cuda:0") -> BaseMotionControl:

        if self.type == "pinocchio":
            from .pinocchio_motion_control import (
                PinocchioMotionControl,
            )

            motion_control = PinocchioMotionControl(
                robot_name=self.robot_name,
                robot_config_path=self.robot_config_path,
            )
        else:
            raise ValueError(f"Motion control type must be one of {self._TYPE}")

        return motion_control

    @classmethod
    def set_default_robot_config_dir(cls, robot_config_dir: Union[str, Path]):
        path = Path(robot_config_dir)
        if not path.exists():
            raise ValueError(f"URDF dir {robot_config_dir} not exists.")
        cls._DEFAULT_ROBOT_CONFIG_DIR = robot_config_dir


def get_motion_control_config(config_path) -> Optional[MotionControlConfig]:
    config = MotionControlConfig.from_file(config_path)
    return config
