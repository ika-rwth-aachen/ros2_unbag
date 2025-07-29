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

from sensor_msgs.msg import CompressedImage, Image

from ros2_unbag.core.processors.base import Processor
from ros2_unbag.core.utils.image_utils import convert_image


@Processor(["sensor_msgs/msg/CompressedImage", "sensor_msgs/msg/Image"], ["apply_color_map"])
def apply_color_map(msg, color_map: int = 1):
    """
    Apply a cv2 color map to an image.

    Args:
        msg: CompressedImage or Image ROS 2 message instance.
        color_map: Integer specifying cv2 colormap. For available colormaps, see: https://docs.opencv.org/4.x/d3/d50/group__imgproc__colormap.html

    Returns:
        CompressedImage or Image ROS 2 message instance with the color map applied.

    Raises:
        ValueError: If color_map is not an integer.
        RuntimeError: If image encoding fails.
    """

    # Get color map as integer
    try:
        color_map = int(color_map)
    except ValueError:
        raise ValueError(
            f"Invalid color map value: {color_map}. Must be an integer.")

    # Decode incoming message into a cv2 image
    if isinstance(msg, CompressedImage):
        arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(arr, cv2.IMREAD_GRAYSCALE)
        if cv_image is None:
            raise RuntimeError("Failed to decode CompressedImage")
    elif isinstance(msg, Image):
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        cv_image = convert_image(raw, msg.encoding.lower(), msg.width, msg.height)
    else:
        raise TypeError(f"Unsupported message type: {type(msg)}")

    # Normalize to uint8 if needed
    if cv_image.dtype != np.uint8:
        cv_image = cv2.normalize(cv_image, None, 0, 255, cv2.NORM_MINMAX)
        cv_image = cv_image.astype(np.uint8)

    # Apply the color map
    recolored = cv2.applyColorMap(cv_image, color_map)

    # Reencode the recolored image back to the original format
    if isinstance(msg, CompressedImage):
        ext = '.jpg' if 'jpeg' in msg.format.lower() else '.png'
        success, encoded = cv2.imencode(ext, recolored)
        if not success:
            raise RuntimeError("Failed to encode recolored image")
        msg.data = encoded.tobytes()
    elif isinstance(msg, Image):
        # recolored is H×W×3, BGR
        msg.encoding = "bgr8"
        msg.height = recolored.shape[0]
        msg.width = recolored.shape[1]
        msg.is_bigendian = msg.is_bigendian  # preserve original
        msg.step = msg.width * 3
        # flatten and assign
        msg.data = recolored.reshape(-1).tobytes()

    return msg
