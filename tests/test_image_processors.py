# MIT License

# Copyright (c) 2026 Institute for Automotive Engineering (ika), RWTH Aachen University

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

"""Real codec checks for lossless compressed-image processor intermediates."""

import importlib.util
from pathlib import Path
import sys
import types

import numpy as np
import pytest


cv2 = pytest.importorskip("cv2")
ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def image_modules(monkeypatch):
    """Load image processing modules without registering unrelated processors."""
    try:
        from sensor_msgs.msg import CompressedImage, Image
    except ImportError:
        messages = types.ModuleType("sensor_msgs.msg")
        messages.CompressedImage = type("CompressedImage", (), {})
        messages.Image = type("Image", (), {})
        package = types.ModuleType("sensor_msgs")
        package.msg = messages
        monkeypatch.setitem(sys.modules, "sensor_msgs", package)
        monkeypatch.setitem(sys.modules, "sensor_msgs.msg", messages)
        CompressedImage, Image = messages.CompressedImage, messages.Image

    def load(name, relative):
        spec = importlib.util.spec_from_file_location(name, ROOT / relative)
        module = importlib.util.module_from_spec(spec)
        monkeypatch.setitem(sys.modules, name, module)
        spec.loader.exec_module(module)
        return module

    load("ros2_unbag.core.processors.base", "ros2_unbag/core/processors/base.py")
    load("ros2_unbag.core.routines.base", "ros2_unbag/core/routines/base.py")
    load("ros2_unbag.core.utils.image_utils", "ros2_unbag/core/utils/image_utils.py")
    processor = load("_image_processors_test", "ros2_unbag/core/processors/image.py")
    routine = load("_image_routines_test", "ros2_unbag/core/routines/image.py")
    return types.SimpleNamespace(
        processor=processor,
        routine=routine,
        CompressedImage=CompressedImage,
        Image=Image,
    )


def compressed_message(modules, extension):
    """
    Create a deterministic compressed image for codec checks.

    Args:
        modules: Loaded image modules and ROS message classes.
        extension: Codec extension, such as ".jpg" or ".png".

    Returns:
        CompressedImage: Encoded test image with matching format metadata.
    """
    pixels = np.random.default_rng(42).integers(0, 256, (48, 64, 3), dtype=np.uint8)
    success, encoded = cv2.imencode(extension, pixels)
    assert success
    msg = modules.CompressedImage()
    if not hasattr(msg, "header"):
        msg.header = object()
    msg.data = encoded.tobytes()
    msg.format = "rgb8; jpeg compressed bgr8" if extension == ".jpg" else "png"
    return msg


@pytest.mark.parametrize("extension", [".jpg", ".png"])
def test_colormap_encodes_pixels_losslessly(image_modules, extension):
    """Preserve the colormap's exact pixels and ROS header through PNG encoding."""
    msg = compressed_message(image_modules, extension)
    header = msg.header
    decoded = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_GRAYSCALE)
    expected = cv2.applyColorMap(decoded, 2)
    result = image_modules.processor.apply_color_map.func(msg, 2)
    assert result is msg
    assert result.header is header
    assert result.format == "bgr8; png compressed bgr8"
    assert bytes(result.data).startswith(b"\x89PNG\r\n\x1a\n")
    actual = cv2.imdecode(np.frombuffer(result.data, np.uint8), cv2.IMREAD_UNCHANGED)
    np.testing.assert_array_equal(actual, expected)


def test_chained_colormaps_use_only_png(image_modules, monkeypatch):
    """Keep every compressed intermediate lossless across a processor chain."""
    msg = compressed_message(image_modules, ".jpg")
    encode = cv2.imencode
    extensions = []

    def record_encode(extension, pixels):
        extensions.append(extension)
        return encode(extension, pixels)

    monkeypatch.setattr(cv2, "imencode", record_encode)
    for color_map in (1, 2, 3):
        before = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_GRAYSCALE)
        expected = cv2.applyColorMap(before, color_map)
        msg = image_modules.processor.apply_color_map.func(msg, color_map)
        assert isinstance(msg, image_modules.CompressedImage)
        after = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_UNCHANGED)
        np.testing.assert_array_equal(after, expected)
    assert extensions == [".png"] * 3


def test_png_export_preserves_intermediate_bytes(image_modules, tmp_path):
    """Write matching PNG payloads directly without another encoding pass."""
    msg = compressed_message(image_modules, ".jpg")
    msg = image_modules.processor.apply_color_map.func(msg, 2)
    image_modules.routine.export_compressed_image(msg, tmp_path / "frame", "image/png", None)
    assert (tmp_path / "frame.png").read_bytes() == bytes(msg.data)


def test_jpeg_export_encodes_only_at_export(image_modules, monkeypatch, tmp_path):
    """Convert the lossless intermediate to JPEG only at the export boundary."""
    msg = compressed_message(image_modules, ".jpg")
    msg = image_modules.processor.apply_color_map.func(msg, 2)
    expected = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_UNCHANGED)
    writes = []
    write = cv2.imwrite

    def record_write(path, pixels):
        writes.append(path)
        np.testing.assert_array_equal(pixels, expected)
        return write(path, pixels)

    monkeypatch.setattr(cv2, "imwrite", record_write)
    image_modules.routine.export_compressed_image(msg, tmp_path / "frame", "image/jpeg", None)
    assert writes == [str(tmp_path / "frame.jpg")]
    assert (tmp_path / "frame.jpg").read_bytes().startswith(b"\xff\xd8")


def test_failed_png_encoding_preserves_message(image_modules, monkeypatch):
    """Leave payload and metadata unchanged when PNG encoding fails."""
    msg = compressed_message(image_modules, ".jpg")
    original = (bytes(msg.data), msg.format)
    monkeypatch.setattr(cv2, "imencode", lambda *args: (False, None))
    with pytest.raises(RuntimeError, match="Failed to encode"):
        image_modules.processor.apply_color_map.func(msg, 2)
    assert (bytes(msg.data), msg.format) == original


def test_raw_image_stays_raw(image_modules):
    """Keep raw-image processors compatible with the Image message contract."""
    pixels = np.arange(48, dtype=np.uint8).reshape(6, 8)
    msg = image_modules.Image()
    msg.data, msg.encoding = pixels.tobytes(), "mono8"
    msg.height, msg.width, msg.step, msg.is_bigendian = 6, 8, 8, False
    expected = cv2.applyColorMap(pixels, 2)
    result = image_modules.processor.apply_color_map.func(msg, 2)
    assert result is msg
    assert isinstance(result, image_modules.Image)
    assert (result.encoding, result.step) == ("bgr8", 24)
    assert bytes(result.data) == expected.tobytes()
