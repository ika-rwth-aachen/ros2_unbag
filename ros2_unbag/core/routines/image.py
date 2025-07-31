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


@ExportRoutine("sensor_msgs/msg/CompressedImage", ["image/png", "image/jpeg"], mode=ExportMode.MULTI_FILE)
def export_compressed_image(msg, path: Path, fmt: str, metadata: ExportMetadata):
    """
    Export a CompressedImage ROS message to PNG or JPEG.
    If the message is already in the desired format, write raw data; otherwise decode and re-encode with OpenCV.

    Args:
        msg: CompressedImage ROS message instance.
        path: Output file path (without extension).
        fmt: Export format string ("image/png" or "image/jpeg").
        metadata: Export metadata including message index and max index.

    Returns:
        None
    """
    desired_fmt = "jpeg" if fmt == "image/jpeg" else "png"
    msg_fmt = msg.format.lower()

    if desired_fmt in msg_fmt:
        # If message is already in the desired format, write directly
        ext = ".jpg" if desired_fmt == "jpeg" else ".png"
        with open(path.with_suffix(ext), "wb") as f:
            f.write(msg.data)
    else:
        # Decode and re-encode to desired format
        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
        ext = ".jpg" if desired_fmt == "jpeg" else ".png"
        cv2.imwrite(path.with_suffix(ext), img)


@ExportRoutine("sensor_msgs/msg/Image", ["image/png", "image/jpeg"], mode=ExportMode.MULTI_FILE)
def export_raw_image(msg, path: Path, fmt: str, metadata: ExportMetadata):
    """
    Export a raw Image ROS message to PNG or JPEG using OpenCV.

    Args:
        msg: Image ROS message instance.
        path: Output file path (without extension).
        fmt: Export format string ("image/png" or "image/jpeg").
        metadata: Export metadata including message index and max index.

    Returns:
        None

    Raises:
        ValueError: If encoding or export format is unsupported.
    """

    raw = np.frombuffer(msg.data, dtype=np.uint8)
    img = convert_image(raw, msg.encoding, msg.width, msg.height)

    ext = { "image/png": ".png", "image/jpeg": ".jpg" }.get(fmt)
    if not ext:
        raise ValueError(f"Unsupported export format: {fmt}")

    cv2.imwrite(path.with_suffix(ext), img)
