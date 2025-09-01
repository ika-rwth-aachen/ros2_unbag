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

from ros2_unbag.core.routines.base import ExportRoutine, ExportMode, ExportMetadata
from ros2_unbag.core.utils.image_utils import convert_image
from ros2_unbag.core.utils.file_utils import get_time_from_msg
from ros2_unbag.core.utils.video_utils import ensure_bgr, write_video_frame, finalize_video

@ExportRoutine("sensor_msgs/msg/CompressedImage", ["video/mp4", "video/avi"], mode=ExportMode.SINGLE_FILE)
def export_compressed_video(msg, path: Path, fmt: str, metadata: ExportMetadata):
    """
    Export a sequence of compressed image ROS messages to a video file using OpenCV.

    Args:
        msg: CompressedImage ROS message instance.
        path: Output file path (without extension).
        fmt: Export format string ("video/mp4" or "video.avi").
        metadata: Export metadata including message index and max index.

    Returns:
        None
    """
    np_arr = np.frombuffer(msg.data, dtype=np.uint8)
    img = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
    img = ensure_bgr(img)

    ps = export_compressed_video.persistent_storage
    ts_ns = get_time_from_msg(msg, return_datetime=False)

    write_video_frame(ps, img, ts_ns, path, fmt)

    if metadata.index == metadata.max_index:
        finalize_video(ps, path, fmt)


@ExportRoutine("sensor_msgs/msg/Image", ["video/mp4", "video/avi"], mode=ExportMode.SINGLE_FILE)
def export_video(msg, path: Path, fmt: str, metadata: ExportMetadata):
    """
    Export a sequence of raw Image ROS messages to a video file using OpenCV.

    Args:
        msg: Image ROS message instance.
        path: Output file path (without extension).
        fmt: Export format string ("video/mp4" or "video.avi").
        metadata: Export metadata including message index and max index.

    Returns:
        None
    """
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    img = convert_image(raw, msg.encoding, msg.width, msg.height)
    img = ensure_bgr(img)

    ps = export_video.persistent_storage
    ts_ns = get_time_from_msg(msg, return_datetime=False)

    write_video_frame(ps, img, ts_ns, path, fmt)

    if metadata.index == metadata.max_index:
        finalize_video(ps, path, fmt)
