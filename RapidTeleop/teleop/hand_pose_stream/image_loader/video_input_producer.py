import time
import cv2
from .base_input_producer import BaseInputProducer


class VideoInputProducer(BaseInputProducer):
    """
    A frame producer that reads frames sequentially from a video file.
    """

    def __init__(
        self,
        video_path: str,
        frame_rate: int = 30,
        target_resolution: tuple = None,
        loop: bool = True,
    ):
        """
        Args:
            video_path: Path to the video file.
            frame_rate: Playback rate in Hz.
            target_resolution: Resize frame to (width, height) if specified.
            loop: If True, restarts video when it ends.
        """
        super().__init__(frame_rate=frame_rate, target_resolution=target_resolution)
        self.video_path = video_path
        self.loop = loop

    def produce_frames(self):
        """
        Continuously read frames from video and serve the latest frame.
        """
        cap = cv2.VideoCapture(self.video_path)
        if not cap.isOpened():
            raise FileNotFoundError(f"[VideoInputProducer] Failed to open video: {self.video_path}")

        print(f"[VideoInputProducer] Streaming from: {self.video_path}")

        try:
            while self._running:
                while self._running:
                    success, frame = cap.read()
                    if not success:
                        break

                    if self.target_resolution:
                        frame = cv2.resize(frame, self.target_resolution)

                    self._set_frame(frame)
                    time.sleep(self.frame_interval)

                if not self.loop:
                    break
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # Rewind video

        finally:
            cap.release()
            print("[VideoInputProducer] Video stream stopped.")
