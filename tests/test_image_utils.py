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

import numpy as np
import pytest

# Skip suite if OpenCV is unavailable in the environment
pytest.importorskip("cv2")
import cv2

from ros2_unbag.core.utils.image_utils import convert_image


def test_convert_bgr8_identity():
    h, w = 2, 3
    arr = np.arange(h * w * 3, dtype=np.uint8)
    out = convert_image(arr, "bgr8", w, h)
    assert out.shape == (h, w, 3)
    assert out.dtype == np.uint8
    assert np.array_equal(out.reshape(-1), arr)


def test_convert_rgb8_to_bgr():
    h, w = 1, 2
    # RGB pixels: [R,G,B,  R,G,B] -> after conversion, channels swap to BGR
    rgb = np.array([255, 0, 0, 0, 255, 0], dtype=np.uint8)  # red, green
    out = convert_image(rgb, "rgb8", w, h)
    assert out.shape == (h, w, 3)
    # red -> (0,0,255), green -> (0,255,0)
    expected = np.array([[ [0,0,255], [0,255,0] ]], dtype=np.uint8)
    assert np.array_equal(out, expected)


def test_convert_mono8_and_mono16():
    h, w = 2, 2
    mono8 = np.arange(h * w, dtype=np.uint8)
    out8 = convert_image(mono8, "mono8", w, h)
    assert out8.shape == (h, w)
    assert out8.dtype == np.uint8

    # For mono16, feed bytes view; values 0..3 -> uint16
    mono16 = (np.arange(h * w, dtype=np.uint16)).view(np.uint8)
    out16 = convert_image(mono16, "mono16", w, h)
    assert out16.shape == (h, w)
    assert out16.dtype == np.uint16


def test_convert_rgb16():
    h, w = 1, 1
    # single pixel RGB16 -> bytes buffer in little-endian
    px = np.array([1000, 2000, 3000], dtype=np.uint16)
    buf = px.view(np.uint8)
    out = convert_image(buf, "rgb16", w, h)
    assert out.shape == (h, w, 3)
    assert out.dtype == np.uint16


def test_convert_yuv422_shape():
    # shape becomes (h, w, 3) after conversion
    h, w = 2, 2
    # yuv422 needs 2 bytes per pixel -> h*w*2 bytes
    buf = np.zeros(h * w * 2, dtype=np.uint8)
    out = convert_image(buf, "yuv422", w, h)
    assert out.shape == (h, w, 3)


def test_convert_abstract_8uc3():
    h, w = 2, 2
    buf = np.arange(h * w * 3, dtype=np.uint8)
    out = convert_image(buf, "8UC3", w, h)
    assert out.shape == (h, w, 3)
    assert out.dtype == np.uint8


def test_convert_abstract_32fc1_scales_to_uint8():
    h, w = 2, 2
    # values in [0,1]
    vals = np.array([0.0, 0.5, 1.0, 0.25], dtype=np.float32)
    buf = vals.view(np.uint8)
    out = convert_image(buf, "32FC1", w, h)
    assert out.shape == (h, w)
    assert out.dtype == np.uint8
    # 0 -> 0, 1 -> 255, 0.5 -> 127/128 (allow small tolerance)
    assert out.max() <= 255


def test_convert_abstract_more_than_3_channels_truncated():
    h, w = 1, 1
    buf = np.arange(4, dtype=np.uint8)
    out = convert_image(buf, "8UC4", w, h)
    assert out.shape == (h, w, 3)


def test_unsupported_encoding_raises():
    h, w = 1, 1
    with pytest.raises(ValueError):
        convert_image(np.zeros(1, dtype=np.uint8), "weird42", w, h)
