import os
import time
import cv2

from .base_input_producer import BaseInputProducer


class ImageInputProducer(BaseInputProducer):
    """
    A frame producer that sequentially loads images from a folder as input.
    Useful for debugging or offline simulation.
    """

    def __init__(
        self,
        image_folder: str,
        frame_rate: int = 30,
        target_resolution: tuple = None,
        loop: bool = True,
    ):
        """
        Args:
            image_folder: Path to the folder containing input images.
            frame_rate: Playback rate in Hz.
            target_resolution: Resize image to (width, height) if specified.
            loop: If True, will repeat images after reaching the end.
        """
        super().__init__(frame_rate=frame_rate, target_resolution=target_resolution)
        self.image_folder = image_folder
        self.loop = loop

        self.images = sorted([
            f for f in os.listdir(self.image_folder)
            if f.lower().endswith((".png", ".jpg", ".jpeg"))
        ])

        if not self.images:
            raise FileNotFoundError(f"No image files found in: {self.image_folder}")

        print(f"[ImageInputProducer] {len(self.images)} image(s) loaded.")

    def produce_frames(self):
        """
        Reads images in order and pushes them as frames.
        If loop is True, restart from the beginning after one pass.
        """
        while self._running:
            for image_file in self.images:
                if not self._running:
                    break

                path = os.path.join(self.image_folder, image_file)
                image = cv2.imread(path)

                if image is None:
                    print(f"[ImageInputProducer] Failed to load image: {image_file}")
                    continue

                if self.target_resolution is not None:
                    image = cv2.resize(image, self.target_resolution)

                self._set_frame(image)
                time.sleep(self.frame_interval)

            if not self.loop:
                break

        print("[ImageInputProducer] Stopped.")
