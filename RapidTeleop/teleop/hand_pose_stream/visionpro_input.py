import os
import json
import time
import threading
import numpy as np
from pytransform3d import rotations
from .base_input import BaseInput
from avp_stream import VisionProStreamer


class VisionProInput(BaseInput):
    """
    Input stream class for retrieving hand and wrist pose data from VisionPro.
    """

    input_stream_type = "visionpro"

    # Transformation matrices from AVP frame to standard frame
    hand_transform = np.array([[0, -1, 0], [0, 0, 1], [-1, 0, 0]])
    wrist_transform = np.array([[-1, 0, 0], [0, 0, 1], [0, 1, 0]])
    wrist_rotation = rotations.matrix_from_angle(2, np.deg2rad(-90))  # Z axis
    wrist_position_transform = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])

    avp_to_standard_order = [
        0, 1, 2, 3, 4, 6, 7, 8, 9,
        11, 12, 13, 14,
        16, 17, 18, 19,
        21, 22, 23, 24
    ]

    def __init__(self, avp_ip: str):
        self.avp_ip = avp_ip
        self.datastream = VisionProStreamer(ip=avp_ip, record=True)
        self.latest_data = self.datastream.get_latest()

        self._stop_thread = False
        self._thread = threading.Thread(target=self._update_loop, daemon=True)
        self._thread.start()

        self.data_dir = os.path.abspath(
            os.path.join(os.path.dirname(__file__), "../..", "data/user_data/visionpro")
        )

    def _update_loop(self):
        """Continuously fetches latest data from the AVP stream."""
        while not self._stop_thread:
            self.latest_data = self.datastream.get_latest()
            time.sleep(0.001)  # ~30Hz

    def get_hand_keypoints(self) -> np.ndarray:
        """
        Returns:
            np.ndarray: Shape (21, 3), 3D coordinates of right hand keypoints.
        """
        if self.latest_data is None:
            raise RuntimeError("No data received from AVP. Waiting for updates.")

        finger_poses = self.latest_data["right_fingers"]
        selected_positions = finger_poses[:, :3, 3][self.avp_to_standard_order]
        return selected_positions @ self.hand_transform.T

    def get_wrist_pose(self) -> np.ndarray:
        """
        Returns:
            np.ndarray: Shape (7,), 3D position + quaternion (x, y, z, qx, qy, qz, qw)
        """
        if self.latest_data is None:
            raise RuntimeError("No data received from AVP. Waiting for updates.")

        wrist_mat = self.latest_data["right_wrist"].squeeze()
        position = wrist_mat[:3, 3] @ self.wrist_position_transform.T
        rotation = self.wrist_rotation @ wrist_mat[:3, :3] @ self.wrist_transform
        quaternion = rotations.quaternion_from_matrix(rotation)[[1, 2, 3, 0]]  # (x, y, z, w)
        return np.concatenate([position, quaternion])

    def store_data(self, user_name: str, duration: float, frame_rate: float):
        """
        Collects and stores data samples for a given duration and frame rate.

        Args:
            user_name (str): Name for the user/session.
            duration (float): Time in seconds to record.
            frame_rate (float): Sampling frequency in Hz.
        """
        os.makedirs(self.data_dir, exist_ok=True)
        file_path = os.path.join(self.data_dir, f"{user_name}.txt")
        interval = 1.0 / frame_rate
        end_time = time.time() + duration

        with open(file_path, "w") as f:
            while time.time() < end_time:
                try:
                    timestamp = time.time()
                    keypoints = self.get_hand_keypoints()
                    wrist_pose = self.get_wrist_pose()
                    data = {
                        "timestamp": timestamp,
                        "hand_keypoints": keypoints.tolist(),
                        "wrist_pose": wrist_pose.tolist()
                    }
                    f.write(json.dumps(data) + "\n")
                except Exception as e:
                    print(f"Data capture error: {e}")
                time.sleep(interval)

        print(f"[VisionProInput] Data saved to {file_path}")

    def read_data(self, user_name: str):
        """
        Reads stored data for the given user.

        Returns:
            list[dict]: Each dict includes timestamp, hand_keypoints, wrist_pose
        """
        file_path = os.path.join(self.data_dir, f"{user_name}.txt")
        if not os.path.isfile(file_path):
            raise FileNotFoundError(f"Data file not found for user: {user_name}")

        samples = []
        with open(file_path, "r") as f:
            for line in f:
                try:
                    sample = json.loads(line.strip())
                    samples.append(sample)
                except json.JSONDecodeError as e:
                    print(f"Parse error: {e}")
        return samples

    def stop(self):
        """Stops the background data-fetching thread."""
        self._stop_thread = True
        self._thread.join()


if __name__ == "__main__":
    input_stream = VisionProInput(avp_ip="192.168.1.100")
    input_stream.store_data(user_name="User0", duration=5, frame_rate=10)
    input_stream.stop()
