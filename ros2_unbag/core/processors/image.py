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

from ros2_unbag.core.processors.base import Processor


@Processor("sensor_msgs/msg/CompressedImage", ["recolor"])
def recolor_compressed_image(msg, color_map):
    """
    Recolor a compressed image using a cv2 color map.

    Args:
        msg: CompressedImage ROS message instance.
        color_map: Integer or string convertible to integer specifying cv2 colormap.

    Returns:
        CompressedImage: Modified message with recolored image data.

    Raises:
        ValueError: If color_map is not an integer.
        RuntimeError: If image encoding fails.
    """
    try:
        color_map = int(color_map)
    except ValueError:
        raise ValueError(
            f"Invalid color map value: {color_map}. Must be an integer.")

    img_array = np.frombuffer(msg.data, np.uint8)
    img = cv2.imdecode(
        img_array,
        cv2.IMREAD_GRAYSCALE)  # assuming single-channel for colormaps

    recolored = cv2.applyColorMap(img, color_map)

    ext = '.jpg' if 'jpeg' in msg.format.lower() else '.png'
    success, encoded = cv2.imencode(ext, recolored)

    if not success:
        raise RuntimeError("Failed to encode recolored image")

    msg.data = encoded.tobytes()
    return msg
