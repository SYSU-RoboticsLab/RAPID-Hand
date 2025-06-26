"""teleop_system_refactored.py
---------------------------------
Pure‑Python refactor of the tele‑operation publisher node:
converts joystick / glove / motion‑capture input to robot
joint commands and publishes them on a ROS topic.

Key goals:
* **Modular** – encapsulated in a reusable `TeleopSystem` class.
* **Testable** – business logic separated from ROS runtime.
* **PEP 8** + type hints for readability and tooling support.
* **Consistent** – mirrors the architecture of
  ``robot_system_refactored.py`` for a coherent codebase.

Run via CLI:
$ python teleop_system_refactored.py path/to/teleop_args.yaml

"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import List

import numpy as np
import rospy
from sensor_msgs.msg import JointState

from teleop import TeleopProcessor
from utils.load_args import load_teleop_args


class TeleopSystem:
    """Convert raw tele‑op input to robot joint commands and publish."""

    def __init__(self, teleop_args_path: str | Path) -> None:
        (
            self.teleop_args,
            self.input_cfg,
            self.arm_cfg,
            self.retarget_cfg,
        ) = load_teleop_args(teleop_args_path)

        # Core processor translating human motion → robot qpos
        self.processor = TeleopProcessor(
            inputstream_config=self.input_cfg,
            arm_config=self.arm_cfg,
            retarget_config=self.retarget_cfg,
        )
        self.joint_names: List[str] = self.processor.get_robot_joint_name()

        self._init_ros()

    # ---------------------------------------------------------------- ROS
    def _init_ros(self) -> None:
        rospy.init_node("teleoperation_system", anonymous=True)
        self._publisher = rospy.Publisher(
            self.teleop_args["joint_command_topic"], JointState, queue_size=10
        )
        rospy.loginfo("[teleop] ROS node initialised")

    # ----------------------------------------------------------- Publishing
    def _publish_joint_state(self, qpos: np.ndarray) -> None:
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = self.joint_names
        msg.position = qpos
        self._publisher.publish(msg)
        # print(f"[teleop] publishing joint_command:{qpos}")

    # ---------------------------------------------------------------- Main
    def run(self, rate_hz: int = 100) -> None:
        rospy.loginfo("[teleop] Ready — spinning …")
        rate = rospy.Rate(rate_hz)
        while not rospy.is_shutdown():
            qpos = self.processor.step()  # heavy lifting inside TeleopProcessor
            self._publish_joint_state(qpos)
            rate.sleep()
        rospy.loginfo("[teleop] Shutdown complete")


# --------------------------------------------------------------------------
# CLI entry‑point
# --------------------------------------------------------------------------


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Launch the tele‑operation node that publishes robot joint commands"
        )
    )
    parser.add_argument(
        "--teleop_args",
        type=str,
        help="Path to teleop_args YAML configuration file",
        default="./args/teleop_args.yaml",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    teleop_system = TeleopSystem(args.teleop_args)
    try:
        teleop_system.run()
    except rospy.ROSInterruptException:
        pass  # Graceful exit on Ctrl‑C within  ROS


if __name__ == "__main__":  # pragma: no cover
    main()
