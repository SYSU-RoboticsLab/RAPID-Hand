import os
import cv2
import time
import json
import threading
import numpy as np

from .base_input import BaseInput
from .single_hand_detector import SingleHandDetector
from pytransform3d import rotations


class ImageInput(BaseInput):
    input_stream_type = "image"

    def __init__(
        self,
        input_type: str,
        frame_rate: int = 120,
        target_resolution=(540, 870),
        settings=None,
    ):
        super().__init__()

        self.detector = SingleHandDetector(hand_type="Right", selfie=False)
        self.data_lock = threading.Lock()
        self.shared_data = {
            "right_hand_mat": None,
            "wrist_pose": None,
        }
        self._running = threading.Event()
        self._running.set()

        # Setup input producer based on type
        settings = settings or {}
        producer_map = {
            "k": "KinectInputProducer",
            "v": "VideoInputProducer",
            "i": "ImageInputProducer",
            "a": "AndroidInputProducer",
        }
        if input_type not in producer_map:
            raise ValueError(f"Invalid input type '{input_type}'. Choose from {list(producer_map.keys())}")

        producer_class = getattr(__import__(".image_loader", fromlist=[producer_map[input_type]]), producer_map[input_type])
        self.producer = producer_class(frame_rate=frame_rate, target_resolution=target_resolution, **settings)

        # Start consumer thread
        self.consumer_thread = threading.Thread(target=self._compute_loop, daemon=True)
        self.consumer_thread.start()
        self.producer.start()

        self.data_directory = os.path.abspath(
            os.path.join(os.path.dirname(__file__), "../..", "data/user_data/image")
        )
        print("ImageInput stream started.")

    def _compute_loop(self):
        while self._running.is_set():
            frame = self.producer.get_latest_frame()
            if frame is None:
                time.sleep(0.01)
                continue

            num_box, joint_pos, _, wrist_rot = self._detect_hand(frame)
            if num_box == 0:
                continue

            quat = rotations.quaternion_from_matrix(wrist_rot[:3, :3])
            wrist_pose = np.concatenate([np.zeros(3), quat[[1, 2, 3, 0]]])

            with self.data_lock:
                self.shared_data["right_hand_mat"] = joint_pos
                self.shared_data["wrist_pose"] = wrist_pose

    def _detect_hand(self, frame: np.ndarray):
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        return self.detector.detect(rgb)

    def get_hand_keypoints(self) -> np.ndarray:
        while self._running.is_set():
            with self.data_lock:
                data = self.shared_data["right_hand_mat"]
            if data is not None:
                return data
            time.sleep(0.05)
        raise RuntimeError("Hand keypoint stream has stopped unexpectedly.")

    def get_wrist_pose(self) -> np.ndarray:
        while self._running.is_set():
            with self.data_lock:
                pose = self.shared_data["wrist_pose"]
            if pose is not None:
                return pose
            time.sleep(0.05)
        raise RuntimeError("Wrist pose stream has stopped unexpectedly.")

    def stop(self):
        self._running.clear()
        self.producer.stop()
        if self.consumer_thread.is_alive():
            self.consumer_thread.join()
        print("ImageInput stream stopped.")

    def store_img_data(self, user_name: str):
        os.makedirs(self.data_directory, exist_ok=True)
        image = self.producer.get_latest_frame()
        if image is None:
            print("No image available to store.")
            return
        file_path = os.path.join(self.data_directory, f"{user_name}.png")
        cv2.imwrite(file_path, image)
        print(f"Image saved at {file_path}")

    def find_img(self, user_name: str) -> np.ndarray:
        import glob
        files = glob.glob(os.path.join(self.data_directory, f"{user_name}.*"))
        for f in files:
            if not f.endswith(".txt"):
                return cv2.imread(f)
        print(f"No image file found for user: {user_name}")
        return None

    def read_img_data(self, user_name: str) -> np.ndarray:
        img = self.find_img(user_name)
        if img is None:
            return None

        num_box, joint_pos, _, _ = self._detect_hand(img)
        retry = 0
        while num_box == 0 and retry < 5:
            print(f"Retrying detection for {user_name}'s image...")
            num_box, joint_pos, _, _ = self._detect_hand(img)
            retry += 1
        if num_box == 0:
            raise RuntimeError(f"Failed to detect hand in {user_name}'s image.")
        return joint_pos

    def store_data(self, user_name: str, save_time: float, frame_rate: float):
        os.makedirs(self.data_directory, exist_ok=True)
        file_path = os.path.join(self.data_directory, f"{user_name}.txt")

        interval = 1.0 / frame_rate
        end_time = time.time() + save_time

        with open(file_path, "w") as f:
            while time.time() < end_time:
                timestamp = time.time()
                keypoints = self.get_hand_keypoints()
                wrist_pose = self.get_wrist_pose()
                data = {
                    "timestamp": timestamp,
                    "hand_keypoints": keypoints.tolist(),
                    "wrist_pose": wrist_pose.tolist(),
                }
                f.write(json.dumps(data) + "\n")
                time.sleep(interval)
        print(f"Time-series data saved at {file_path}")

    def read_data(self, user_name: str):
        file_path = os.path.join(self.data_directory, f"{user_name}.txt")
        if not os.path.isfile(file_path):
            raise FileNotFoundError(f"No saved data for {user_name}")

        results = []
        with open(file_path, "r") as f:
            for line in f:
                try:
                    results.append(json.loads(line.strip()))
                except Exception as e:
                    print(f"Skipping line due to parse error: {e}")
        return results


if __name__ == "__main__":
    # Example: Android camera as source
    ip = "192.168.1.100"
    input_stream = ImageInput(input_type="a", settings={"android_ip": ip})
    input_stream.store_data(user_name="User0", save_time=5, frame_rate=10)
    input_stream.stop()
