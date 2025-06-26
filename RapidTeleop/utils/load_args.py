from __future__ import annotations

from pathlib import Path
from typing import Tuple, Dict, Any

import yaml

ARGS_DIR = Path(__file__).resolve().parent.parent / "args"


def _load_yaml(path: Path) -> Dict[str, Any]:
    """Read a YAML file into a dict."""
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def _config_path(*subdirs: str, suffix: str = ".yaml") -> Path:
    """Compose a Path inside the project‑level *args* folder.
    """
    return (ARGS_DIR.joinpath(*subdirs)).with_suffix(suffix)


def load_teleop_args(teleop_args_path: str | Path) -> Tuple[dict, dict, dict, dict]:
    """Load teleoperation configs & enrich retarget settings with runtime info."""

    # --- main YAML --------------------------------------------------------
    teleop_args = _load_yaml(Path(teleop_args_path))

    robot_arm = teleop_args["robot_arm"].lower()
    robot_hand = teleop_args["robot_hand"].lower()
    input_type = teleop_args["input_type"]

    # --- auxiliary file paths --------------------------------------------
    input_cfg_path = _config_path("inputstream_args", input_type)
    arm_cfg_path = _config_path("robot_args", "arm_args", f"{robot_arm}_{robot_hand}")
    retarget_cfg_path = _config_path("retarget_args", f"{robot_hand}_hand_right")

    # --- load auxiliary YAMLs --------------------------------------------
    input_cfg = _load_yaml(input_cfg_path)
    arm_cfg = _load_yaml(arm_cfg_path)
    retarget_cfg = _load_yaml(retarget_cfg_path)

    # --- enrich retarget config ------------------------------------------
    mapping = retarget_cfg["transform"]["robot_human_link_mapping"]
    human_robot_map = {idx: name for name, idx in mapping.items()}

    retarget_cfg["transform"]["user"] = teleop_args["user"]
    retarget_cfg["transform"].update(
        {
            "human_robot_link_map": human_robot_map,
            "robot_base_link": human_robot_map[0],
        }
    )

    from control import PinocchioHandControl

    hand_model = PinocchioHandControl(
        hand_name=f"{robot_hand}_hand",
        robot_urdf_file=retarget_cfg["retargeting"]["urdf_path"],
    )

    link_positions = {
        name: hand_model.get_link_transform(name)[:3, 3]
        for name in hand_model.get_link_names()
    }
    retarget_cfg["transform"]["robot_link_pos_dict"] = link_positions

    return teleop_args, input_cfg, arm_cfg, retarget_cfg


def load_robot_args(robot_args_path: str | Path) -> Tuple[dict, dict, dict]:
    """Load robot‑side arguments (arm & hand)."""

    robot_args = _load_yaml(Path(robot_args_path))

    robot_arm = robot_args["robot_arm"].lower()
    robot_hand = robot_args["robot_hand"].lower()

    arm_cfg_path = _config_path("robot_args", "arm_args", f"{robot_arm}_{robot_hand}")
    hand_cfg_path = _config_path("robot_args", "hand_args", f"{robot_hand}_args")

    arm_cfg = _load_yaml(arm_cfg_path)
    hand_cfg = _load_yaml(hand_cfg_path)

    return robot_args, arm_cfg, hand_cfg
