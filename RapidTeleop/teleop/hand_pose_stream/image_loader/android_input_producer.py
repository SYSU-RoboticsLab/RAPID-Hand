import cv2
import time
import numpy as np
from .base_input_producer import BaseInputProducer


class AndroidInputProducer(BaseInputProducer):
    """
    A frame producer that receives MJPEG stream from an Android IP Camera.
    Assumes apps like "IP Webcam" are running on the phone.
    """

    def __init__(
        self,
        android_ip: str = "192.168.1.100",
        frame_rate: int = 30,
        target_resolution: tuple = None,
    ):
        """
        Args:
            android_ip: IP address of the Android device running MJPEG server
            frame_rate: Capture frame rate in Hz
            target_resolution: Resize image to (width, height) if specified
        """
        super().__init__(frame_rate, target_resolution)

        self.stream_url = f"http://admin:admin@{android_ip}:8081/"  # default IP Webcam port
        self.cap = cv2.VideoCapture(self.stream_url)

        if not self.cap.isOpened():
            raise ConnectionError(f"Failed to open video stream from {self.stream_url}")

        print(f"[AndroidInputProducer] Connected to {self.stream_url}")

    def produce_frames(self):
        """
        Continuously read frames from the IP stream and store them.
        """
        while self._running and self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret:
                print("[AndroidInputProducer] Frame read failed. Retrying...")
                time.sleep(0.05)
                continue

            if self.target_resolution is not None:
                frame = cv2.resize(frame, self.target_resolution)

            self._set_frame(frame)
            time.sleep(self.frame_interval)

        self.cap.release()
        print("[AndroidInputProducer] Stopped stream.")
