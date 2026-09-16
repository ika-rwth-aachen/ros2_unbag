# MIT License
#
# Copyright (c) 2026 Institute for Automotive Engineering (ika), RWTH Aachen University
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import sys
import types

import numpy as np


def _install_dependency_stubs():
    rosidl_runtime_py = types.ModuleType("rosidl_runtime_py")
    rosidl_runtime_py.message_to_ordereddict = lambda msg: {}
    rosidl_runtime_py.message_to_yaml = lambda msg: ""
    sys.modules.setdefault("rosidl_runtime_py", rosidl_runtime_py)

    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")

    class PointField:
        INT8 = 1
        UINT8 = 2
        INT16 = 3
        UINT16 = 4
        INT32 = 5
        UINT32 = 6
        FLOAT32 = 7
        FLOAT64 = 8

    sensor_msgs_msg.PointField = PointField
    sensor_msgs_msg.PointCloud2 = type("PointCloud2", (), {})
    sensor_msgs_msg.CompressedImage = type("CompressedImage", (), {})
    sensor_msgs_msg.Image = type("Image", (), {})
    sensor_msgs.msg = sensor_msgs_msg
    sys.modules.setdefault("sensor_msgs", sensor_msgs)
    sys.modules.setdefault("sensor_msgs.msg", sensor_msgs_msg)

_install_dependency_stubs()

from ros2_unbag.core.routines import image  # noqa: E402


def test_raw_image_export_passes_string_filename_to_opencv(monkeypatch, tmp_path):
    filenames = []
    monkeypatch.setattr(
        image.cv2, "imwrite", lambda filename, img: filenames.append(filename), raising=False
    )
    msg = types.SimpleNamespace(
        data=bytes([0, 0, 0]), encoding="bgr8", width=1, height=1
    )

    image.export_raw_image(msg, tmp_path / "frame", "image/png", metadata=None)

    assert filenames == [str(tmp_path / "frame.png")]


def test_transcoded_compressed_image_passes_string_filename_to_opencv(monkeypatch, tmp_path):
    filenames = []
    monkeypatch.setattr(
        image.cv2,
        "imdecode",
        lambda data, flags: np.zeros((1, 1, 3), dtype=np.uint8),
        raising=False,
    )
    monkeypatch.setattr(
        image.cv2, "imwrite", lambda filename, img: filenames.append(filename), raising=False
    )
    msg = types.SimpleNamespace(data=b"compressed", format="jpeg")

    image.export_compressed_image(msg, tmp_path / "frame", "image/png", metadata=None)

    assert filenames == [str(tmp_path / "frame.png")]
