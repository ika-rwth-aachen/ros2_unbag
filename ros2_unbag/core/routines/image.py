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


@ExportRoutine("sensor_msgs/msg/CompressedImage", ["image/png", "image/jpeg"], mode=ExportMode.MULTI_FILE)
def export_compressed_image(
    msg,
    path: Path,
    fmt: str,
    metadata: ExportMetadata,
    jpeg_quality: int = 95,
    png_compression: int = 3,
    resize_width: Optional[int] = None,
    resize_height: Optional[int] = None,
):
    """
    Export a CompressedImage ROS message to PNG or JPEG.
    If the message is already in the desired format and no resize/quality params are set,
    write raw data directly; otherwise decode and re-encode with OpenCV.

    Args:
        msg: CompressedImage ROS message instance.
        path: Output file path (without extension).
        fmt: Export format string ("image/png" or "image/jpeg").
        metadata: Export metadata including message index and max index.
        jpeg_quality (int): JPEG compression quality (1–100). Only used when exporting as JPEG.
        png_compression (int): PNG compression level (0–9, where 0 = no compression, 9 = maximum). Only used when exporting as PNG.
        resize_width (int or None): Target width in pixels. Height is computed to preserve aspect ratio when only one dimension is given.
        resize_height (int or None): Target height in pixels. Width is computed to preserve aspect ratio when only one dimension is given.

    Returns:
        None
    """
    jpeg_quality = int(jpeg_quality)
    png_compression = int(png_compression)
    resize_width = int(resize_width) if resize_width is not None else None
    resize_height = int(resize_height) if resize_height is not None else None

    desired_fmt = "jpeg" if fmt == "image/jpeg" else "png"
    msg_fmt = msg.format.lower()
    ext = ".jpg" if desired_fmt == "jpeg" else ".png"

    needs_reencode = (
        desired_fmt not in msg_fmt
        or resize_width is not None
        or resize_height is not None
        or (desired_fmt == "jpeg" and jpeg_quality != 95)
        or (desired_fmt == "png" and png_compression != 3)
    )

    if not needs_reencode:
        with open(path.with_suffix(ext), "wb") as f:
            f.write(msg.data)
        return

    np_arr = np.frombuffer(msg.data, np.uint8)
    img = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
    img = _apply_resize(img, resize_width, resize_height)
    if desired_fmt == "jpeg":
        encode_params = [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality]
    else:
        encode_params = [cv2.IMWRITE_PNG_COMPRESSION, png_compression]
    cv2.imwrite(str(path.with_suffix(ext)), img, encode_params)


@ExportRoutine("sensor_msgs/msg/Image", ["image/png", "image/jpeg"], mode=ExportMode.MULTI_FILE)
def export_raw_image(
    msg,
    path: Path,
    fmt: str,
    metadata: ExportMetadata,
    jpeg_quality: int = 95,
    png_compression: int = 3,
    resize_width: Optional[int] = None,
    resize_height: Optional[int] = None,
):
    """
    Export a raw Image ROS message to PNG or JPEG using OpenCV.

    Args:
        msg: Image ROS message instance.
        path: Output file path (without extension).
        fmt: Export format string ("image/png" or "image/jpeg").
        metadata: Export metadata including message index and max index.
        jpeg_quality (int): JPEG compression quality (1–100). Only used when exporting as JPEG.
        png_compression (int): PNG compression level (0–9, where 0 = no compression, 9 = maximum). Only used when exporting as PNG.
        resize_width (int or None): Target width in pixels. Height is computed to preserve aspect ratio when only one dimension is given.
        resize_height (int or None): Target height in pixels. Width is computed to preserve aspect ratio when only one dimension is given.

    Returns:
        None

    Raises:
        ValueError: If encoding or export format is unsupported.
    """
    jpeg_quality = int(jpeg_quality)
    png_compression = int(png_compression)
    resize_width = int(resize_width) if resize_width is not None else None
    resize_height = int(resize_height) if resize_height is not None else None

    raw = np.frombuffer(msg.data, dtype=np.uint8)
    img = convert_image(raw, msg.encoding, msg.width, msg.height)
    img = _apply_resize(img, resize_width, resize_height)

    ext = {"image/png": ".png", "image/jpeg": ".jpg"}.get(fmt)
    if not ext:
        raise ValueError(f"Unsupported export format: {fmt}")

    if fmt == "image/jpeg":
        encode_params = [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality]
    else:
        encode_params = [cv2.IMWRITE_PNG_COMPRESSION, png_compression]
    cv2.imwrite(str(path.with_suffix(ext)), img, encode_params)


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
