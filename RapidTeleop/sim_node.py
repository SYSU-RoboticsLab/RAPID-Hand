"""simulator_system_refactored.py
---------------------------------
Refactored standalone ROS node for running a Sapien physics
simulator in lock‑step with tele‑operation joint commands.

Highlights
~~~~~~~~~~
* **SimulatorSystem** class – encapsulates loading URDF, ROS
  subscription and render loop.
* **Lazy joint‑name mapping** from tele‑op → simulator order.
* **PEP 8** compliant, full type hints + docstrings.
* **CLI** interface mirrors other refactored nodes.

Run via CLI:
$ python simulator_system_refactored.py path/to/teleop_args.yaml

"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import List, Optional

import numpy as np
import rospy
from sensor_msgs.msg import JointState

from control import SimSapien as Sim
from utils.load_args import _load_yaml


class SimulatorSystem:
    """Synchronise Sapien simulator with incoming joint commands."""

    def __init__(self, teleop_args_path: str | Path) -> None:
        # ------------------------------------------------------------------
        # Load configuration
        self.teleop_args = _load_yaml(Path(teleop_args_path))
        robot_name = f"{self.teleop_args['robot_arm']}_{self.teleop_args['robot_hand']}"

        # ------------------------------------------------------------------
        urdf_rel = (
            f"./assets/robots/hands/{self.teleop_args['robot_hand'].lower()}_hand/"
            f"{self.teleop_args['robot_hand'].lower()}_hand_right.urdf"
        )
        self.sim = Sim(urdf_rel)
        
        self.sim_joint_names: List[str] = self.sim.sapien_joint_names

        # Joint command buffer and mapping indices
        self._cmd: np.ndarray = np.zeros(len(self.sim_joint_names))
        self._teleop2sim: Optional[np.ndarray] = None  # lazy lookup

        self._init_ros()

    # -------------------------------------------------------------- ROS
    def _init_ros(self) -> None:
        rospy.init_node("simulator_system", anonymous=True)
        rospy.Subscriber(
            self.teleop_args["joint_command_topic"],
            JointState,
            self._joint_angle_cb,
            queue_size=1,
        )
        rospy.loginfo("[sim] ROS node initialised")

    def _joint_angle_cb(self, msg: JointState) -> None:
        if self._teleop2sim is None:
            self._teleop2sim = np.array(
                [msg.name.index(n) for n in self.sim_joint_names], dtype=int
            )
            rospy.loginfo_once(f"Mapping tele‑op → sim: {self._teleop2sim.tolist()}")

        teleop_pos = np.array(msg.position)
        self._cmd = teleop_pos[self._teleop2sim]

    # ------------------------------------------------------------- Main
    def run(self) -> None:
        rospy.loginfo("[sim] Ready — spinning …")
        # Sapien internally runs at its own fixed‑step; we simply drive it
        # with the most recent joint command each tick until ROS shutdown.
        rate = rospy.Rate(100)  # 100 Hz visual update
        while not rospy.is_shutdown():
            # Render / physics step & capture cameras if needed
            _ = self.sim.step(self._cmd)  # (left_img, right_img)
            rate.sleep()
        rospy.loginfo("[sim] Shutdown complete")


# ------------------------------------------------------------------------
# CLI entry‑point
# ------------------------------------------------------------------------


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Run Sapien simulator ROS node")
    p.add_argument(
        "--teleop_args",
        type=str,
        help="Path to teleop_args YAML configuration file",
        default="./args/teleop_args.yaml",
    )
    return p.parse_args()


def main() -> None:
    args = parse_args()
    sim_system = SimulatorSystem(args.teleop_args)
    try:
        sim_system.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":  # pragma: no cover
    main()
