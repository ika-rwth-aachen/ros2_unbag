import cv2
import numpy as np

from ros2_unbag.core.routines.base import ExportRoutine


@ExportRoutine("sensor_msgs/msg/CompressedImage", ["image/png", "image/jpeg"])
def export_compressed_image(msg, path, fmt="image/png"):
    """
    Export a CompressedImage ROS message to PNG or JPEG.
    If the message is already in the desired format, write raw data; otherwise decode and re-encode with OpenCV.

    Args:
        msg: CompressedImage ROS message instance.
        path: Output file path (without extension).
        fmt: Export format string ("image/png" or "image/jpeg").

    Returns:
        None
    """
    desired_fmt = "jpeg" if fmt == "image/jpeg" else "png"
    msg_fmt = msg.format.lower()

    if desired_fmt in msg_fmt:
        # If message is already in the desired format, write directly
        ext = ".jpg" if desired_fmt == "jpeg" else ".png"
        with open(path + ext, "wb") as f:
            f.write(msg.data)
    else:
        # Decode and re-encode to desired format
        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
        ext = ".jpg" if desired_fmt == "jpeg" else ".png"
        cv2.imwrite(path + ext, img)


@ExportRoutine("sensor_msgs/msg/Image", ["image/png", "image/jpeg"])
def export_raw_image(msg, path, fmt="image/png"):
    """
    Export a raw Image ROS message to PNG or JPEG.
    Convert supported encodings (bgr8, rgb8, bgra8) to BGR, then write with OpenCV; error on unsupported formats.

    Args:
        msg: Image ROS message instance.
        path: Output file path (without extension).
        fmt: Export format string ("image/png" or "image/jpeg").

    Returns:
        None

    Raises:
        ValueError: If encoding or export format is unsupported.
    """
    if msg.encoding in ("bgr8", "rgb8", "bgra8"):
        img_array = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)

        if msg.encoding == "rgb8":
            img_array = cv2.cvtColor(img_array, cv2.COLOR_RGB2BGR)
        elif msg.encoding == "bgra8":
            img_array = cv2.cvtColor(img_array, cv2.COLOR_BGRA2BGR)
    else:
        raise ValueError(f"Unsupported encoding: {msg.encoding}")

    ext_map = {"image/png": ".png", "image/jpeg": ".jpg"}
    ext = ext_map.get(fmt)
    if not ext:
        raise ValueError(f"Unsupported export format: {fmt}")

    cv2.imwrite(path + ext, img_array)
