from typing import Union, Any, Optional
import cv2
import numpy as np

from .camera_sdk.pyorbbecsdk import FormatConvertFilter, VideoFrame
from .camera_sdk.pyorbbecsdk import OBFormat, OBConvertFormat


def yuyv_to_bgr(frame: np.ndarray, width: int, height: int) -> np.ndarray:
    """Convert YUYV image to BGR."""
    yuyv_image = frame.reshape((height, width, 2))
    return cv2.cvtColor(yuyv_image, cv2.COLOR_YUV2BGR_YUY2)


def uyvy_to_bgr(frame: np.ndarray, width: int, height: int) -> np.ndarray:
    """Convert UYVY image to BGR."""
    uyvy_image = frame.reshape((height, width, 2))
    return cv2.cvtColor(uyvy_image, cv2.COLOR_YUV2BGR_UYVY)


def i420_to_bgr(frame: np.ndarray, width: int, height: int) -> np.ndarray:
    """Convert I420 image to BGR."""
    y_size = width * height
    u_size = v_size = y_size // 4
    y = frame[0:y_size].reshape((height, width))
    u = frame[y_size:y_size + u_size].reshape((height // 2, width // 2))
    v = frame[y_size + u_size:].reshape((height // 2, width // 2))
    yuv = cv2.merge([y, u, v])
    return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_I420)


def nv21_to_bgr(frame: np.ndarray, width: int, height: int) -> np.ndarray:
    """Convert NV21 image to BGR."""
    y = frame[:height * width].reshape((height, width))
    vu = frame[height * width:].reshape((height // 2, width))
    yuv = np.vstack((y, vu))
    return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_NV21)


def nv12_to_bgr(frame: np.ndarray, width: int, height: int) -> np.ndarray:
    """Convert NV12 image to BGR."""
    y = frame[:height * width].reshape((height, width))
    uv = frame[height * width:].reshape((height // 2, width))
    yuv = np.vstack((y, uv))
    return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_NV12)


def determine_convert_format(frame: VideoFrame) -> Optional[int]:
    """Map OBFormat to corresponding conversion format."""
    format_map = {
        OBFormat.I420: OBConvertFormat.I420_TO_RGB888,
        OBFormat.MJPG: OBConvertFormat.MJPG_TO_RGB888,
        OBFormat.YUYV: OBConvertFormat.YUYV_TO_RGB888,
        OBFormat.NV21: OBConvertFormat.NV21_TO_RGB888,
        OBFormat.NV12: OBConvertFormat.NV12_TO_RGB888,
        OBFormat.UYVY: OBConvertFormat.UYVY_TO_RGB888
    }
    return format_map.get(frame.get_format(), None)


def frame_to_rgb_frame(frame: VideoFrame) -> Optional[VideoFrame]:
    """Convert a frame to RGB format using FormatConvertFilter if needed."""
    if frame.get_format() == OBFormat.RGB:
        return frame

    convert_format = determine_convert_format(frame)
    if convert_format is None:
        print(f"Unsupported format: {frame.get_format()}")
        return None

    print(f"Converting frame using format: {convert_format}")
    convert_filter = FormatConvertFilter()
    convert_filter.set_format_convert_format(convert_format)

    rgb_frame = convert_filter.process(frame)
    if rgb_frame is None:
        print(f"Failed to convert {frame.get_format()} to RGB.")
    return rgb_frame


def frame_to_bgr_image(frame: VideoFrame) -> Optional[np.ndarray]:
    """Convert a VideoFrame to BGR image (numpy array)."""
    width, height = frame.get_width(), frame.get_height()
    format_type = frame.get_format()
    data = np.asanyarray(frame.get_data())

    if format_type == OBFormat.RGB:
        rgb_image = data.reshape((height, width, 3))
        return cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)

    if format_type == OBFormat.BGR:
        return data.reshape((height, width, 3))

    if format_type == OBFormat.YUYV:
        return yuyv_to_bgr(data, width, height)

    if format_type == OBFormat.UYVY:
        return uyvy_to_bgr(data, width, height)

    if format_type == OBFormat.I420:
        return i420_to_bgr(data, width, height)

    if format_type == OBFormat.NV12:
        return nv12_to_bgr(data, width, height)

    if format_type == OBFormat.NV21:
        return nv21_to_bgr(data, width, height)

    if format_type == OBFormat.MJPG:
        return cv2.imdecode(data, cv2.IMREAD_COLOR)

    print(f"Unsupported color format: {format_type}")
    return None
