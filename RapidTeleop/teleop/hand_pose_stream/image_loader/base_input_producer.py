import threading
import time
from abc import ABC, abstractmethod


class BaseInputProducer(ABC):
    """
    Abstract base class for all input producers (e.g., Kinect, Video, Image, Android).
    Handles threading and frame caching logic. Subclasses must implement produce_frames().
    """

    def __init__(self, frame_rate: int = 30, target_resolution=None):
        """
        Args:
            frame_rate (int): Frame capture rate in Hz
            target_resolution (tuple): Resize output frame to (width, height), if specified
        """
        self.frame_rate = frame_rate
        self.target_resolution = target_resolution

        self.frame_interval = 1.0 / self.frame_rate
        self._lock = threading.Lock()
        self._latest_frame = None

        self._running = True
        self.producer_thread = None  # Optionally used to spawn a background producer thread

    @abstractmethod
    def produce_frames(self):
        """
        Abstract method to be implemented by subclasses.
        Typical workflow:
            1. Open camera / video / stream
            2. While self._running:
                - Capture frame
                - Call self._set_frame(frame)
                - time.sleep(self.frame_interval)
            3. Release resource
        """
        pass

    def start(self):
        """
        Starts the background producer thread if not already running.
        """
        if self.producer_thread is not None and self.producer_thread.is_alive():
            print("[BaseInputProducer] Producer thread already running.")
            return

        self._running = True
        self.producer_thread = threading.Thread(target=self.produce_frames, daemon=True)
        self.producer_thread.start()
        print("[BaseInputProducer] Producer thread started.")

    def stop(self):
        """
        Signals the thread to stop and waits for cleanup.
        """
        self._running = False
        if self.producer_thread is not None:
            self.producer_thread.join(timeout=1.0)
            print("[BaseInputProducer] Producer thread stopped.")

    def _set_frame(self, frame):
        """
        Thread-safe method to update the latest captured frame.

        Args:
            frame: Captured image (usually np.ndarray)
        """
        with self._lock:
            self._latest_frame = frame

    def get_latest_frame(self):
        """
        Thread-safe method to retrieve the latest captured frame.

        Returns:
            The most recent image frame (usually np.ndarray), or None if unavailable.
        """
        with self._lock:
            return self._latest_frame
