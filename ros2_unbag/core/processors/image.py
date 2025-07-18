import cv2
import numpy as np

from ros2_unbag.core.processors.base import Processor


@Processor("sensor_msgs/msg/CompressedImage", ["recolor"])
def recolor_compressed_image(msg, color_map):
    """Recolor a compressed image using a cv2 color map
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
