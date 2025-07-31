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

@ExportRoutine("sensor_msgs/msg/CompressedImage", ["video/mp4", "video/avi"], mode=ExportMode.SINGLE_FILE)
def export_compressed_image(msg, path: Path, fmt: str, metadata: ExportMetadata):
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
    fourcc_map = {
        "video/mp4": cv2.VideoWriter_fourcc(*'mp4v'),
        "video/avi": cv2.VideoWriter_fourcc(*'XVID')
    }

    ext_map = {
        "video/mp4": ".mp4",
        "video/avi": ".avi"
    }


    if fmt not in fourcc_map:
        raise ValueError(f"Unsupported export format: {fmt}")

    np_arr = np.frombuffer(msg.data, dtype=np.uint8)
    img = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)

    if len(img.shape) == 2 or img.shape[2] == 1:
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    ps = export_video_image.persistent_storage

    image_size = img.shape[:2]

    if "frame_size" not in ps:
        ps["frame_size"] = image_size

    if image_size != ps["frame_size"]:
        raise ValueError("All images must have the same dimensions")

    if "writer" not in ps:
        height, width = image_size
        writer = cv2.VideoWriter(
            str(path.with_suffix(ext_map[fmt])),
            fourcc_map[fmt],
            30.0,
            (width, height)
        )
        ps["writer"] = writer

    ps["writer"].write(img)

    if metadata.index == metadata.max_index:
        ps["writer"].release()
        del ps["writer"]


@ExportRoutine("sensor_msgs/msg/Image", ["video/mp4", "video/avi"], mode=ExportMode.SINGLE_FILE)
def export_video_image(msg, path: Path, fmt: str, metadata: ExportMetadata):
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
    fourcc_map = {
        "video/mp4": cv2.VideoWriter_fourcc(*'mp4v'),
        "video/avi": cv2.VideoWriter_fourcc(*'XVID')
    }

    ext_map = {
        "video/mp4": ".mp4",
        "video/avi": ".avi"
    }

    if fmt not in fourcc_map:
        raise ValueError(f"Unsupported export format: {fmt}")

    raw = np.frombuffer(msg.data, dtype=np.uint8)
    img = convert_image(raw, msg.encoding, msg.width, msg.height)

    if len(img.shape) == 2 or img.shape[2] == 1:
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    ps = export_video_image.persistent_storage

    image_size = img.shape[:2]

    if "frame_size" not in ps:
        ps["frame_size"] = image_size

    if image_size != ps["frame_size"]:
        raise ValueError("All images must have the same dimensions")

    if "writer" not in ps:
        height, width = image_size
        writer = cv2.VideoWriter(
            str(path.with_suffix(ext_map[fmt])),
            fourcc_map[fmt],
            30.0,
            (width, height)
        )
        ps["writer"] = writer

    ps["writer"].write(img)

    if metadata.index == metadata.max_index:
        ps["writer"].release()
        del ps["writer"]