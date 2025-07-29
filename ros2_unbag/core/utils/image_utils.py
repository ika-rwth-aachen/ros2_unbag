import cv2
import numpy as np
import re

def convert_image(img, encoding, width, height):
    """
    Convert a raw image buffer to a numpy array based on the specified encoding.
    Supports various encodings including BGR, RGB, mono, Bayer, and abstract types.

    Args:
        img: Raw image buffer (numpy array).
        encoding: Image encoding string (e.g., "bgr8", "rgb8", "mono8", etc.).
        width: Width of the image.
        height: Height of the image.

    Returns:
        numpy.ndarray: Converted image as a numpy array.

    Raises:
        ValueError: If the encoding is unsupported.
    """

    converters = {
        "bgr8":        lambda: img.reshape(height, width, 3),
        "rgb8":        lambda: cv2.cvtColor(img.reshape(height, width, 3), cv2.COLOR_RGB2BGR),
        "bgra8":       lambda: cv2.cvtColor(img.reshape(height, width, 4), cv2.COLOR_BGRA2BGR),
        "rgba8":       lambda: cv2.cvtColor(img.reshape(height, width, 4), cv2.COLOR_RGBA2BGR),
        "mono8":       lambda: img.reshape(height, width),
        "mono16":      lambda: img.view(np.uint16).reshape(height, width),
        "bgr16":       lambda: img.view(np.uint16).reshape(height, width, 3),
        "rgb16":       lambda: cv2.cvtColor(img.view(np.uint16).reshape(height, width, 3), cv2.COLOR_RGB2BGR),
        "bgra16":      lambda: cv2.cvtColor(img.view(np.uint16).reshape(height, width, 4), cv2.COLOR_BGRA2BGR),
        "rgba16":      lambda: cv2.cvtColor(img.view(np.uint16).reshape(height, width, 4), cv2.COLOR_RGBA2BGR),
        "yuv422":      lambda: cv2.cvtColor(img.reshape(height, width, 2), cv2.COLOR_YUV2BGR_YUY2),
        "bayer_rggb8": lambda: cv2.cvtColor(img.reshape(height, width), cv2.COLOR_BAYER_RG2BGR),
        "bayer_bggr8": lambda: cv2.cvtColor(img.reshape(height, width), cv2.COLOR_BAYER_BG2BGR),
        "bayer_gbrg8": lambda: cv2.cvtColor(img.reshape(height, width), cv2.COLOR_BAYER_GB2BGR),
        "bayer_grbg8": lambda: cv2.cvtColor(img.reshape(height, width), cv2.COLOR_BAYER_GR2BGR),
        "bayer_rggb16": lambda: cv2.cvtColor(img.view(np.uint16).reshape(height, width), cv2.COLOR_BAYER_RG2BGR),
        "bayer_bggr16": lambda: cv2.cvtColor(img.view(np.uint16).reshape(height, width), cv2.COLOR_BAYER_BG2BGR),
        "bayer_gbrg16": lambda: cv2.cvtColor(img.view(np.uint16).reshape(height, width), cv2.COLOR_BAYER_GB2BGR),
        "bayer_grbg16": lambda: cv2.cvtColor(img.view(np.uint16).reshape(height, width), cv2.COLOR_BAYER_GR2BGR),
    }

    if encoding in converters:
        return converters[encoding]()

    # Abstract types: e.g., 8UC3, 32FC1, etc.
    match = re.match(r"(\d+)([USF]C)(\d+)", encoding.upper())
    if match:
        depth, _, channels = match.groups()
        depth = int(depth)
        channels = int(channels)
        dtype = {
            (8, "UC"): np.uint8,
            (8, "SC"): np.int8,
            (16, "UC"): np.uint16,
            (16, "SC"): np.int16,
            (32, "SC"): np.int32,
            (32, "FC"): np.float32,
            (64, "FC"): np.float64,
        }.get((depth, match.group(2)))

        # If dtype is found, reshape and convert the image to a saveable format
        if dtype is not None:
            shape = (height, width) if channels == 1 else (height, width, channels)
            result = img.view(dtype).reshape(shape)
            if np.issubdtype(result.dtype, np.floating):
                result = np.clip(result, 0.0, 1.0) if result.max() <= 1.0 else np.clip(result / result.max(), 0.0, 1.0)
                result = (result * 255.0).astype(np.uint8)
            elif result.dtype not in (np.uint8, np.uint16):
                result = np.clip(result, 0, 255).astype(np.uint8)

            if result.ndim == 3 and result.shape[2] > 3:
                result = result[:, :, :3]

            return result

    raise ValueError(f"Unsupported encoding: {encoding}")
