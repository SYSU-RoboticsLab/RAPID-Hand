import time
import numpy as np
import matplotlib.pyplot as plt
from abc import ABC, abstractmethod
from pytransform3d import rotations


class BaseInput(ABC):
    @abstractmethod
    def get_hand_keypoints(self) -> np.ndarray:
        """
        Returns:
            np.ndarray: Shape (21, 3), 3D coordinates of hand keypoints.
        """
        pass

    @abstractmethod
    def get_wrist_pose(self) -> np.ndarray:
        """
        Returns:
            np.ndarray: Shape (7,), wrist pose as (x, y, z, qx, qy, qz, qw).
        """
        pass

    def test_hand_keypoints(self, stop_condition=lambda: False):
        """
        Live 3D visualization of hand keypoints.

        Args:
            stop_condition: Callable returning bool to terminate the loop (e.g., rospy.is_shutdown)
        """
        plt.ion()
        fig = plt.figure("Hand Keypoints")
        ax = fig.add_subplot(111, projection="3d")

        while not stop_condition():
            keypoints = self.get_hand_keypoints()
            if keypoints is not None:
                ax.cla()
                ax.scatter(keypoints[:, 0], keypoints[:, 1], keypoints[:, 2],
                           c="r", marker="o", s=50)
                ax.set_xlabel("X")
                ax.set_ylabel("Y")
                ax.set_zlabel("Z")
                ax.set_title("Hand Keypoints (Local Frame)")
                lim = 0.16
                ax.set_xlim([-lim, lim])
                ax.set_ylim([-lim, lim])
                ax.set_zlim([0, lim])
                plt.pause(0.01)
            time.sleep(0.1)

    def test_wrist_pose(self, stop_condition=lambda: False):
        """
        Live 3D visualization of wrist pose axes.

        Args:
            stop_condition: Callable returning bool to terminate the loop (e.g., rospy.is_shutdown)
        """
        plt.ion()
        fig = plt.figure("Wrist Pose")
        ax = fig.add_subplot(111, projection="3d")

        while not stop_condition():
            pose = self.get_wrist_pose()
            if pose is not None:
                ax.cla()
                position = pose[:3]
                quat_ros = pose[3:]  # (qx, qy, qz, qw)
                quat_tf3d = [quat_ros[3], quat_ros[0], quat_ros[1], quat_ros[2]]
                rot_matrix = rotations.matrix_from_quaternion(quat_tf3d)

                axis_len = 0.8
                x_axis = position + rot_matrix[:, 0] * axis_len
                y_axis = position + rot_matrix[:, 1] * axis_len
                z_axis = position + rot_matrix[:, 2] * axis_len

                ax.scatter(*position, c="r", s=50, label="Wrist")
                ax.plot([position[0], x_axis[0]], [position[1], x_axis[1]], [position[2], x_axis[2]], 'r', label="X")
                ax.plot([position[0], y_axis[0]], [position[1], y_axis[1]], [position[2], y_axis[2]], 'g', label="Y")
                ax.plot([position[0], z_axis[0]], [position[1], z_axis[1]], [position[2], z_axis[2]], 'b', label="Z")

                ax.set_xlabel("X")
                ax.set_ylabel("Y")
                ax.set_zlabel("Z")
                ax.set_title("Wrist Orientation")
                lim = 1.5
                ax.set_xlim([-lim, lim])
                ax.set_ylim([-lim, lim])
                ax.set_zlim([-lim, lim])
                ax.legend()
                plt.pause(0.01)
            time.sleep(0.1)
