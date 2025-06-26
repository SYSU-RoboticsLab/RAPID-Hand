"""robot_system_refactored.py
---------------------------------
High‑level wrapper around UR10 + RapidHand tele‑operation.

This module turns the original monolithic script into a
clean, reusable class with clear single‑responsibility
methods.  It keeps ROS spin‑blocking behaviour out of the
business logic so the code can be imported and unit‑tested.

USAGE
-----
$ python robot_system_refactored.py path/to/robot_args.yaml

The YAML file layout is expected to be identical to that
accepted by ``utils.load_args.load_robot_args`` in the
original project.

"""

from __future__ import annotations

import argparse
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional

import numpy as np
import rospy
from sensor_msgs.msg import JointState

from control import RapidHandController, UR10Controller
from utils.load_args import load_robot_args


@dataclass
class IOControllers:
    """Small container for optional I/O helpers."""

    datasaver: Optional[object] = None  # PklSaver | None
    publisher: Optional[object] = None  # RobotDataPublisher | None

    @property
    def should_output(self) -> bool:
        """Determine whether I/O output should be performed based on available devices."""
        return self.datasaver is not None or self.publisher is not None


class RobotSystem:
    """Handle UR10 + RapidHand tele‑op in a ROS node."""

    def __init__(self, robot_args_path: str | Path) -> None:
        self.robot_args, self.arm_cfg, self.hand_cfg = load_robot_args(robot_args_path)

        # Low‑level hardware drivers
        self.ur10 = UR10Controller(robot_ip=None)
        self.rapid_hand = RapidHandController(**self.hand_cfg)

        # Joint name ordering & command buffers
        self.ur10_joint_order: List[str] = self.arm_cfg["ur10_command_order"]
        self.rapid_joint_order: List[str] = self.hand_cfg["joint_command_order_list"]
        self.ur10_cmd: np.ndarray = np.array(list(self.arm_cfg["init_qpos"].values()))
        self.rapid_cmd: np.ndarray = np.zeros(20)

        # Mapping from tele‑op indices → robot indices (lazily initialised)
        self._teleop2ur10: Optional[np.ndarray] = None
        self._teleop2rapid: Optional[np.ndarray] = None

        # Optional data saver / publisher helpers
        self.io = IOControllers()

        self._init_ros()
        self._init_io_helpers()

    # ------------------------------------------------------------------ ROS
    def _init_ros(self) -> None:
        rospy.init_node("robot_system", anonymous=True)
        rospy.Subscriber(
            self.robot_args["joint_command_topic"],
            JointState,
            self._joint_angle_cb,
            queue_size=1,
        )
        rospy.loginfo("[robot] ROS node initialised")

    # ---------------------------------------------------------------- IO‑helpers
    def _init_io_helpers(self) -> None:
        save_cfg = self.robot_args["save_config"]
        publish_cfg = self.robot_args["publish_config"]

        if save_cfg["save_robot_data"]:
            from data.scripts import PklSaver

            self.io.datasaver = PklSaver(
                task_name=save_cfg["task_dir"], root_dir=save_cfg["root_dir"]
            )

        if publish_cfg["publish_robot_data"]:
            from data.scripts import RobotDataPublisher

            self.io.publisher = RobotDataPublisher(
                rate_hz=50,
                init_node=False,
                **publish_cfg["robot_data_topic"],
            )
            self.io.publisher.start()

    # ---------------------------------------------------------- Callbacks
    def _joint_angle_cb(self, msg: JointState) -> None:
        """Convert incoming tele‑op message into robot command arrays."""

        teleop_names = msg.name
        if self._teleop2ur10 is None:
            self._teleop2ur10 = np.array(
                [teleop_names.index(j) for j in self.ur10_joint_order], dtype=int
            )
            rospy.loginfo_once(f"UR10 mapping: {self._teleop2ur10.tolist()}")

        if self._teleop2rapid is None:
            self._teleop2rapid = np.array(
                [teleop_names.index(j) for j in self.rapid_joint_order], dtype=int
            )
            rospy.loginfo_once(f"RapidHand mapping: {self._teleop2rapid.tolist()}")

        teleop_pos = np.array(msg.position)
        self.ur10_cmd = teleop_pos[self._teleop2ur10]
        self.rapid_cmd = teleop_pos[self._teleop2rapid]

    # -------------------------------------------------------------- Helpers
    @property
    def _should_collect_data(self) -> bool:
        return self.io.should_output

    def _send_robot_commands(self) -> None:
        self.ur10.control_arm_qpos(self.ur10_cmd)
        self.rapid_hand.control_hand_qpos(self.rapid_cmd)

    def _handle_robot_data(self) -> None:
        real_arm = self.ur10.get_arm_data()
        real_hand = self.rapid_hand.get_hand_data()

        if self.io.datasaver:
            self.io.datasaver.save_frame(
                real_arm_data=real_arm,
                real_hand_data=real_hand,
                command=np.concatenate([self.ur10_cmd, self.rapid_cmd]),
            )
            rospy.logdebug("[robot] Frame saved")

        if self.io.publisher:
            self.io.publisher.update_data(
                joint_angles=np.concatenate(
                    [real_arm["joint_positions"], real_hand["joint_positions"]]
                ),
                joint_names=self.ur10_joint_order + self.rapid_joint_order,
                image=real_hand["base_rgb"][0],
                tactile_data=real_hand["touch"],
                tactile_data_3D=real_hand["touch_3D"],
            )

    def _shutdown(self) -> None:
        self.ur10.stop()
        self.rapid_hand.stop()
        rospy.loginfo("[robot] Shutdown complete")

    # ----------------------------------------------------------------- Main
    def run(self) -> None:
        rospy.loginfo("[robot] Ready — spinning …")
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            self._send_robot_commands()
            if self._should_collect_data:
                self._handle_robot_data()
            rate.sleep()

        self._shutdown()


# ---------------------------------------------------------------------------
# CLI entry‑point
# ---------------------------------------------------------------------------


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="UR10 + RapidHand ROS node")
    parser.add_argument(
        "robot_args",
        type=str,
        help="Path to robot_args YAML configuration file",
        default="./args/teleop_args.yaml",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    robot_system = RobotSystem(args.robot_args)
    try:
        robot_system.run()
    except rospy.ROSInterruptException:
        pass  # Graceful exit on Ctrl‑C within  ROS


if __name__ == "__main__":  # pragma: no cover
    main()
