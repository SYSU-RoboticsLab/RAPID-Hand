import json
import os
import time
import cv2
import numpy as np
from queue import Queue
from threading import Lock
from typing import List, Tuple, Optional

from .camera_sdk.pyorbbecsdk import *
from .utils import frame_to_bgr_image


class OrbbecCamera:
    MAX_DEVICES = 2
    MAX_QUEUE_SIZE = 5
    WRITE_CAMERA_CONFIG = True

    def __init__(self, config_path: Optional[str] = None):
        if config_path is None:
            config_path = os.path.abspath(
                os.path.join(os.path.dirname(__file__), "multi_device_sync_config.json")
            )
        self.config_path = config_path

        self.depth_frames_queue: List[Queue] = [Queue() for _ in range(self.MAX_DEVICES)]
        self.color_frames_buffer: List[List[VideoFrame]] = [[] for _ in range(self.MAX_DEVICES)]
        self.locks: List[Lock] = [Lock() for _ in range(self.MAX_DEVICES)]
        self.has_color_sensor: List[bool] = [False] * self.MAX_DEVICES

        self.stop_rendering = False
        self.multi_device_sync_config = {}
        self.curr_device_cnt = 0
        self.devices = []
        self.pipelines = []
        self.configs = []

    def _sync_mode_from_str(self, sync_mode_str: str) -> OBMultiDeviceSyncMode:
        """Convert sync mode string to OBMultiDeviceSyncMode enum."""
        try:
            return OBMultiDeviceSyncMode[sync_mode_str.upper()]
        except KeyError:
            raise ValueError(f"Invalid sync mode: {sync_mode_str}")

    def _on_new_frame_callback(self, frames: FrameSet, index: int):
        """Callback when new frames are received from the camera."""
        assert index < self.MAX_DEVICES
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        with self.locks[index]:
            if depth_frame is not None:
                if self.depth_frames_queue[index].qsize() >= self.MAX_QUEUE_SIZE:
                    self.depth_frames_queue[index].get()
                self.depth_frames_queue[index].put(depth_frame)

            if color_frame is not None:
                self.color_frames_buffer[index].append(color_frame)
                if len(self.color_frames_buffer[index]) > self.MAX_QUEUE_SIZE:
                    self.color_frames_buffer[index].pop(0)

    def _read_config(self):
        """Read the multi-device sync configuration from file."""
        with open(self.config_path, "r") as f:
            config = json.load(f)
        for device in config.get("devices", []):
            self.multi_device_sync_config[device["serial_number"]] = device

    def init_camera(self):
        """Initialize and configure connected cameras."""
        self._read_config()
        ctx = Context()
        device_list = ctx.query_devices()
        self.curr_device_cnt = device_list.get_count()

        if self.curr_device_cnt == 0:
            raise RuntimeError("No Orbbec devices connected.")
        if self.curr_device_cnt > self.MAX_DEVICES:
            raise RuntimeError(f"Too many devices connected (max {self.MAX_DEVICES}).")

        for i in range(self.curr_device_cnt):
            device = device_list.get_device_by_index(i)
            self.devices.append(device)
            pipeline = Pipeline(device)
            config = Config()

            if self.WRITE_CAMERA_CONFIG:
                serial = device.get_device_info().get_serial_number()
                sync_config_data = self.multi_device_sync_config.get(serial)
                if sync_config_data:
                    sync_config = device.get_multi_device_sync_config()
                    sync_data = sync_config_data["config"]
                    sync_config.mode = self._sync_mode_from_str(sync_data["mode"])
                    sync_config.color_delay_us = sync_data["color_delay_us"]
                    sync_config.depth_delay_us = sync_data["depth_delay_us"]
                    sync_config.trigger_out_enable = sync_data["trigger_out_enable"]
                    sync_config.trigger_out_delay_us = sync_data["trigger_out_delay_us"]
                    sync_config.frames_per_trigger = sync_data["frames_per_trigger"]
                    device.set_multi_device_sync_config(sync_config)

            # Enable color stream
            try:
                profile_list = pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
                color_profile = profile_list.get_video_stream_profile(640, 480, OBFormat.MJPG, 60)
                config.enable_stream(color_profile)
                self.has_color_sensor[i] = True
            except OBError:
                self.has_color_sensor[i] = False

            # Enable depth stream
            profile_list = pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
            depth_profile = profile_list.get_video_stream_profile(848, 100, OBFormat.Y16, 100)
            config.enable_stream(depth_profile)

            self.pipelines.append(pipeline)
            self.configs.append(config)

        self._start_streams()

    def _start_streams(self):
        """Start streaming from all pipelines."""
        for index, (pipeline, config) in enumerate(zip(self.pipelines, self.configs)):
            pipeline.start(config, lambda frames, i=index: self._on_new_frame_callback(frames, i))

    def stop_streams(self):
        """Stop all running pipelines."""
        for pipeline in self.pipelines:
            pipeline.stop()

    def release(self):
        """Release all camera resources."""
        self.stop_streams()

    def get_latest_frames(self, device_index: int, timeout: float = 1.0) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """Get the latest color and depth images from specified device.

        Args:
            device_index (int): Index of the device.
            timeout (float): Maximum waiting time in seconds.

        Returns:
            Tuple[np.ndarray, np.ndarray]: (color_image, depth_image), or (None, None) if timeout.
        """
        if device_index >= self.curr_device_cnt:
            raise IndexError("Device index out of range.")

        start_time = time.time()
        color_image, depth_image = None, None

        while time.time() - start_time < timeout:
            time.sleep(0.02)
            with self.locks[device_index]:
                if not self.depth_frames_queue[device_index].empty() and self.color_frames_buffer[device_index]:
                    while self.depth_frames_queue[device_index].qsize() > 1:
                        self.depth_frames_queue[device_index].get()
                    depth_frame = self.depth_frames_queue[device_index].get()
                    depth_timestamp = depth_frame.get_timestamp()

                    best_color_frame = None
                    best_diff = float("inf")
                    for cf in self.color_frames_buffer[device_index]:
                        diff = abs(cf.get_timestamp() - depth_timestamp)
                        if diff < best_diff:
                            best_diff = diff
                            best_color_frame = cf

                    self.color_frames_buffer[device_index].remove(best_color_frame)
                    color_image = frame_to_bgr_image(best_color_frame)

                    width, height = depth_frame.get_width(), depth_frame.get_height()
                    scale = depth_frame.get_depth_scale()
                    depth_data = np.frombuffer(depth_frame.get_data(), dtype=np.uint16).reshape((height, width))
                    depth_data = depth_data.astype(np.float32) * scale
                    depth_normalized = cv2.normalize(depth_data, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                    depth_image = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)

                    return color_image, depth_image

        return None, None
