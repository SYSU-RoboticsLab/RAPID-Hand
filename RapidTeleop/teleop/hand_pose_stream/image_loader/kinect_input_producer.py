import time
import cv2
from pyk4a import PyK4A, Config, ColorResolution, DepthMode
from .base_input_producer import BaseInputProducer


class KinectInputProducer(BaseInputProducer):
    """
    A frame producer using Azure Kinect via PyK4A.
    Captures color frames and optionally downsamples them.
    """

    def __init__(
        self,
        frame_rate: int = 30,
        target_resolution: tuple = None,
        camera_index: int = 0,
        downsample_half: bool = True,
    ):
        """
        Args:
            frame_rate: Capture rate in Hz.
            target_resolution: Resize output image to (width, height) if specified.
            camera_index: Optional device index (if multiple Kinect devices connected).
            downsample_half: If True, reduce resolution by 50% when target_resolution is None.
        """
        super().__init__(frame_rate=frame_rate, target_resolution=target_resolution)
        self.camera_index = camera_index
        self.downsample_half = downsample_half

    def produce_frames(self):
        """
        Continuously read frames from Kinect and provide the latest color image.
        """
        k4a = PyK4A(
            Config(
                color_resolution=ColorResolution.RES_1080P,
                depth_mode=DepthMode.NFOV_UNBINNED,
            )
        )

        try:
            k4a.start()
            print("[KinectInputProducer] Device started.")
        except Exception as e:
            raise RuntimeError(f"[KinectInputProducer] Failed to start Kinect: {e}")

        try:
            while self._running:
                capture = k4a.get_capture()
                color = capture.color

                if color is None:
                    time.sleep(0.01)
                    continue

                if color.shape[2] == 4:
                    color = cv2.cvtColor(color, cv2.COLOR_BGRA2BGR)

                if self.target_resolution:
                    color = cv2.resize(color, self.target_resolution)
                elif self.downsample_half:
                    h, w = color.shape[:2]
                    color = cv2.resize(color, (w // 2, h // 2))

                self._set_frame(color)
                time.sleep(self.frame_interval)

        except Exception as e:
            print(f"[KinectInputProducer] Error during capture: {e}")

        finally:
            k4a.stop()
            print("[KinectInputProducer] Device stopped.")
