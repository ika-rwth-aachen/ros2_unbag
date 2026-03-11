# MIT License

# Copyright (c) 2025 Institute for Automotive Engineering (ika), RWTH Aachen University

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import cv2
import numpy as np
from pathlib import Path
from typing import Optional

from ros2_unbag.core.routines.base import ExportRoutine, ExportMode, ExportMetadata
from ros2_unbag.core.utils.image_utils import convert_image
from ros2_unbag.core.utils.file_utils import get_time_from_msg
from ros2_unbag.core.utils.video_utils import ensure_bgr, write_video_frame, finalize_video

@ExportRoutine("sensor_msgs/msg/CompressedImage", ["video/mp4", "video/avi"], mode=ExportMode.SINGLE_FILE)
def export_compressed_video(
    msg,
    path: Path,
    fmt: str,
    metadata: ExportMetadata,
    target_fps: Optional[float] = None,
    resize_width: Optional[int] = None,
    resize_height: Optional[int] = None,
):
    """
    Export a sequence of compressed image ROS messages to a video file using OpenCV.

    Args:
        msg: CompressedImage ROS message instance.
        path: Output file path (without extension).
        fmt: Export format string ("video/mp4" or "video/avi").
        metadata: Export metadata including message index and max index.
        target_fps (float or None): Override the auto-detected frame rate. When None, FPS is
            estimated from the timestamp delta between the first two frames.
        resize_width (int or None): Target width in pixels. Height is computed to preserve aspect ratio when only one dimension is given.
        resize_height (int or None): Target height in pixels. Width is computed to preserve aspect ratio when only one dimension is given.

    Returns:
        None
    """
    target_fps = float(target_fps) if target_fps is not None else None
    resize_width = int(resize_width) if resize_width is not None else None
    resize_height = int(resize_height) if resize_height is not None else None

    np_arr = np.frombuffer(msg.data, dtype=np.uint8)
    img = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
    img = ensure_bgr(img)
    img = _apply_resize(img, resize_width, resize_height)

    ps = export_compressed_video.persistent_storage
    if target_fps is not None:
        ps["target_fps"] = target_fps
    ts_ns = get_time_from_msg(msg, return_datetime=False)

    write_video_frame(ps, img, ts_ns, path, fmt)

    if metadata.index == metadata.max_index:
        finalize_video(ps, path, fmt)


@ExportRoutine("sensor_msgs/msg/Image", ["video/mp4", "video/avi"], mode=ExportMode.SINGLE_FILE)
def export_video(
    msg,
    path: Path,
    fmt: str,
    metadata: ExportMetadata,
    target_fps: Optional[float] = None,
    resize_width: Optional[int] = None,
    resize_height: Optional[int] = None,
):
    """
    Export a sequence of raw Image ROS messages to a video file using OpenCV.

    Args:
        msg: Image ROS message instance.
        path: Output file path (without extension).
        fmt: Export format string ("video/mp4" or "video/avi").
        metadata: Export metadata including message index and max index.
        target_fps (float or None): Override the auto-detected frame rate. When None, FPS is
            estimated from the timestamp delta between the first two frames.
        resize_width (int or None): Target width in pixels. Height is computed to preserve aspect ratio when only one dimension is given.
        resize_height (int or None): Target height in pixels. Width is computed to preserve aspect ratio when only one dimension is given.

    Returns:
        None
    """
    target_fps = float(target_fps) if target_fps is not None else None
    resize_width = int(resize_width) if resize_width is not None else None
    resize_height = int(resize_height) if resize_height is not None else None

    raw = np.frombuffer(msg.data, dtype=np.uint8)
    img = convert_image(raw, msg.encoding, msg.width, msg.height)
    img = ensure_bgr(img)
    img = _apply_resize(img, resize_width, resize_height)

    ps = export_video.persistent_storage
    if target_fps is not None:
        ps["target_fps"] = target_fps
    ts_ns = get_time_from_msg(msg, return_datetime=False)

    write_video_frame(ps, img, ts_ns, path, fmt)

    if metadata.index == metadata.max_index:
        finalize_video(ps, path, fmt)


def _apply_resize(img, width: Optional[int], height: Optional[int]):
    """Resize img to (width, height), preserving aspect ratio when only one dimension is given."""
    if width is None and height is None:
        return img
    h, w = img.shape[:2]
    if width is None:
        width = int(round(w * height / h))
    elif height is None:
        height = int(round(h * width / w))
    return cv2.resize(img, (int(width), int(height)))
