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


@Processor(["sensor_msgs/msg/CompressedImage", "sensor_msgs/msg/Image"], ["apply_color_map"])
def apply_color_map(msg, color_map):
    """
    Apply a cv2 color map to an image.

    Args:
        msg: CompressedImage or Image ROS 2 message instance.
        color_map: Integer or string convertible to integer specifying cv2 colormap.

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

    # Decode incoming message into a single-channel gray cv2 image
    if isinstance(msg, CompressedImage):
        arr = np.frombuffer(msg.data, np.uint8)
        gray = cv2.imdecode(arr, cv2.IMREAD_GRAYSCALE)
        if gray is None:
            raise RuntimeError("Failed to decode CompressedImage")
    elif isinstance(msg, Image):
        # raw Image: interpret bytes according to encoding
        h, w = msg.height, msg.width
        enc = msg.encoding.lower()

        if enc == "mono8":
            gray = np.frombuffer(msg.data, np.uint8).reshape(h, w)
        elif enc in ("rgb8", "bgr8"):
            # reshape into H×W×3
            arr = np.frombuffer(msg.data, np.uint8).reshape(h, w, 3)
            # convert RGB→GRAY or BGR→GRAY
            code = cv2.COLOR_RGB2GRAY if enc == "rgb8" else cv2.COLOR_BGR2GRAY
            gray = cv2.cvtColor(arr, code)
        else:
            raise RuntimeError(f"Unsupported raw image encoding: {msg.encoding}")
    else:
        raise TypeError(f"Unsupported message type: {type(msg)}")

    # Apply the color map
    recolored = cv2.applyColorMap(gray, color_map)

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
        msg.height = gray.shape[0]
        msg.width = gray.shape[1]
        msg.is_bigendian = msg.is_bigendian  # preserve original
        msg.step = msg.width * 3
        # flatten and assign
        msg.data = recolored.reshape(-1).tobytes()

    return msg
