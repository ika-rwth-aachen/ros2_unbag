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

"""Point cloud regression coverage, runnable without a ROS installation."""

from array import array
import importlib.util
from pathlib import Path
import struct
import sys
import types

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def pointcloud_module(monkeypatch):
    """Load point cloud processors without importing image processors."""
    # Load only these modules; package discovery also imports image processors.
    try:
        from sensor_msgs.msg import PointCloud2, PointField
    except ImportError:
        class PointField:
            INT8, UINT8, INT16, UINT16, INT32, UINT32, FLOAT32, FLOAT64 = range(1, 9)

            def __init__(self, name="", offset=0, datatype=0, count=1):
                self.name, self.offset, self.datatype, self.count = name, offset, datatype, count

        class PointCloud2:
            pass

        sensor_msgs = types.ModuleType("sensor_msgs")
        messages = types.ModuleType("sensor_msgs.msg")
        messages.PointCloud2, messages.PointField = PointCloud2, PointField
        sensor_msgs.msg = messages
        monkeypatch.setitem(sys.modules, "sensor_msgs", sensor_msgs)
        monkeypatch.setitem(sys.modules, "sensor_msgs.msg", messages)

    def load(name, path):
        spec = importlib.util.spec_from_file_location(name, ROOT / path)
        module = importlib.util.module_from_spec(spec)
        monkeypatch.setitem(sys.modules, name, module)
        spec.loader.exec_module(module)
        return module

    load("ros2_unbag.core.processors.base", "ros2_unbag/core/processors/base.py")
    load("ros2_unbag.core.utils.pointcloud_utils", "ros2_unbag/core/utils/pointcloud_utils.py")
    return load("_pointcloud_processors_test", "ros2_unbag/core/processors/pointcloud.py")


def make_cloud(module, *, endian="<", width=2, buffer_type=bytes):
    """
    Build a cloud with mixed field types, vector fields, and storage padding.

    Args:
        module: Loaded point cloud module supplying ROS message classes.
        endian: Byte order marker, "<" or ">".
        width: Number of points per row.
        buffer_type: Callable converting the byte buffer to the desired type.

    Returns:
        tuple: Input cloud and expected packed bytes after intensity removal.
    """
    field = module.PointField
    msg = module.PointCloud2()
    if not hasattr(msg, "header"):
        msg.header = object()
    msg.width, msg.height = width, 2
    msg.point_step, msg.row_step = 24, width * 24 + 7
    msg.is_bigendian, msg.is_dense = endian == ">", False
    # Deliberately use a different declaration order from the storage order.
    msg.fields = [
        field(name="normal", offset=12, datatype=field.FLOAT32, count=2),
        field(name="intensity", offset=4, datatype=field.UINT16, count=1),
        field(name="x", offset=0, datatype=field.FLOAT32, count=1),
        field(name="ring", offset=8, datatype=field.UINT8, count=1),
    ]
    data = bytearray([0xCD]) * (msg.height * msg.row_step)
    expected = bytearray()
    for row in range(msg.height):
        for column in range(width):
            index = row * width + column
            base = row * msg.row_step + column * msg.point_step
            struct.pack_into(endian + "f", data, base, index + 0.25)
            struct.pack_into(endian + "H", data, base + 4, 100 + index)
            struct.pack_into("B", data, base + 8, index)
            struct.pack_into(endian + "ff", data, base + 12, index + 0.5, -index - 1)
            expected.extend(
                data[base:base + 4]
                + data[base + 8:base + 9]
                + data[base + 12:base + 20]
            )
    msg.data = buffer_type(data)
    return msg, bytes(expected)


@pytest.mark.parametrize("endian", ["<", ">"])
@pytest.mark.parametrize("buffer_type", [bytes, bytearray, lambda data: array("B", data)])
def test_remove_fields_preserves_values_and_metadata(pointcloud_module, endian, buffer_type):
    """Compact retained fields while preserving metadata and the input cloud."""
    module = pointcloud_module
    msg, expected = make_cloud(module, endian=endian, buffer_type=buffer_type)
    original = bytes(msg.data)
    original_fields = [(f.name, f.offset, f.datatype, f.count) for f in msg.fields]
    out = module.pointcloud_remove_fields.func(msg, " intensity, absent ")

    assert bytes(out.data) == expected
    assert out is not msg
    assert out.header is msg.header
    assert (out.width, out.height, out.point_step, out.row_step) == (2, 2, 13, 26)
    assert (out.is_bigendian, out.is_dense) == (msg.is_bigendian, msg.is_dense)
    assert [(f.name, f.offset, f.datatype, f.count) for f in out.fields] == [
        ("x", 0, module.PointField.FLOAT32, 1),
        ("ring", 4, module.PointField.UINT8, 1),
        ("normal", 5, module.PointField.FLOAT32, 2),
    ]
    assert bytes(msg.data) == original
    assert [(f.name, f.offset, f.datatype, f.count) for f in msg.fields] == original_fields


@pytest.mark.parametrize("selection", ["", "absent", "x,normal,ring,intensity"])
def test_remove_fields_noop(pointcloud_module, selection):
    """Return the original cloud when selection cannot remove usable fields."""
    msg, _ = make_cloud(pointcloud_module)
    assert pointcloud_module.pointcloud_remove_fields.func(msg, selection) is msg


def test_remove_fields_empty_cloud(pointcloud_module):
    """Preserve an empty cloud's dimensions while updating its field layout."""
    msg, _ = make_cloud(pointcloud_module, width=0)
    msg.data = b""
    out = pointcloud_module.pointcloud_remove_fields.func(msg, "intensity")
    assert bytes(out.data) == b""
    assert (out.width, out.height, out.point_step, out.row_step) == (0, 2, 13, 0)


@pytest.mark.parametrize("endian", ["<", ">"])
def test_remove_multiple_fields(pointcloud_module, endian):
    """Remove multiple fields and retain vector components in storage order."""
    msg, _ = make_cloud(pointcloud_module, endian=endian)
    out = pointcloud_module.pointcloud_remove_fields.func(msg, "x, ring")
    expected = b"".join(struct.pack(endian + "Hff", 100 + i, i + 0.5, -i - 1) for i in range(4))
    assert bytes(out.data) == expected
    assert [(f.name, f.offset) for f in out.fields] == [("intensity", 0), ("normal", 2)]
    assert (out.point_step, out.row_step) == (10, 20)


@pytest.fixture
def transform_utils(pointcloud_module):
    """Return the transform utilities loaded by the point cloud fixture."""
    return sys.modules["ros2_unbag.core.utils.pointcloud_utils"]


def scalar_transform(module, msg, translation, rotation):
    """
    Reference the original per-point arithmetic with correct layout traversal.

    Args:
        module: Transform utilities supplying the quaternion matrix helper.
        msg: Input PointCloud2 message.
        translation: Translation vector containing x, y, and z.
        rotation: Quaternion containing x, y, z, and w.

    Returns:
        bytes: Transformed cloud buffer with unrelated storage preserved.
    """
    matrix = module.quaternion_matrix(rotation)
    matrix[:3, 3] = translation
    endian = ">" if msg.is_bigendian else "<"
    fields = {f.name: f for f in msg.fields}
    formats = {name: endian + ("f" if fields[name].datatype == module.PointField.FLOAT32 else "d")
               for name in ("x", "y", "z")}
    data = bytearray(msg.data)
    with np.errstate(invalid="ignore"):
        for row in range(msg.height):
            for column in range(msg.width):
                base = row * msg.row_step + column * msg.point_step
                xyz = [struct.unpack_from(formats[name], data, base + fields[name].offset)[0]
                       for name in ("x", "y", "z")]
                result = matrix @ np.array([*xyz, 1.0])
                for axis, name in enumerate(("x", "y", "z")):
                    struct.pack_into(formats[name], data, base + fields[name].offset, result[axis])
    return bytes(data)


def make_transform_cloud(
    module, *, endian="<", kinds=("f", "f", "f"), width=3, height=2, padding=7
):
    """
    Build a cloud with separate coordinate offsets and configurable storage.

    Args:
        module: Transform utilities supplying ROS message classes.
        endian: Byte order marker, "<" or ">".
        kinds: Struct format codes for the x, y, and z coordinate types.
        width: Number of points per row.
        height: Number of rows.
        padding: Extra bytes following each row.

    Returns:
        PointCloud2: Cloud with deterministic coordinates and padding bytes.
    """
    msg = module.PointCloud2()
    if not hasattr(msg, "header"):
        msg.header = object()
    msg.width, msg.height = width, height
    msg.point_step, msg.row_step = 40, width * 40 + padding
    msg.is_bigendian, msg.is_dense = endian == ">", False
    offsets = (24, 12, 0)
    msg.fields = [
        module.PointField(
            name=name,
            offset=offset,
            datatype=module.PointField.FLOAT32 if kind == "f" else module.PointField.FLOAT64,
            count=1,
        )
        for name, offset, kind in zip(("x", "y", "z"), offsets, kinds)
    ]
    msg.fields.reverse()
    data = bytearray([0xCD]) * (height * msg.row_step)
    for row in range(height):
        for column in range(width):
            base = row * msg.row_step + column * msg.point_step
            for axis, (offset, kind) in enumerate(zip(offsets, kinds)):
                value = (row * width + column + 0.25) * (axis + 1)
                struct.pack_into(endian + kind, data, base + offset, value)
    msg.data = bytes(data)
    return msg


@pytest.mark.parametrize("endian", ["<", ">"])
@pytest.mark.parametrize("kinds", [("f", "f", "f"), ("d", "d", "d"), ("f", "d", "f")])
def test_transform_layout_and_preservation(transform_utils, endian, kinds):
    """Respect coordinate types and layout while preserving unrelated bytes."""
    module = transform_utils
    msg = make_transform_cloud(module, endian=endian, kinds=kinds)
    original = bytes(msg.data)
    translation, rotation = [1, -2, 0.5], [0.1, -0.2, 0.3, 0.9]
    expected = scalar_transform(module, msg, translation, rotation)
    out = module.apply_pointcloud_transform(msg, translation, rotation)
    # Exact full-buffer equality also checks every padding and non-XYZ byte.
    assert bytes(out.data) == expected
    assert bytes(msg.data) == original
    assert out is not msg
    assert out.header is msg.header
    assert out.fields is msg.fields
    for name in ("width", "height", "point_step", "row_step", "is_bigendian", "is_dense"):
        assert getattr(out, name) == getattr(msg, name)


@pytest.mark.parametrize("rotation", [[0, 0, 0, 1], [0, 0, 0, 0], [0.1, -0.2, 0.3, 0.9]])
def test_transform_matches_original_float32_bytes(transform_utils, rotation):
    """Preserve the original FLOAT32 rounding, including nonfinite values."""
    module = transform_utils
    msg = make_transform_cloud(module, width=20000, height=1, padding=0)
    rng = np.random.default_rng(42)
    data = bytearray(msg.data)
    # Include a broad range of magnitudes and nonfinite coordinates.
    xyz = (
        rng.normal(size=(msg.width, 3))
        * np.exp2(rng.integers(-30, 30, (msg.width, 3)))
    ).astype("f4")
    xyz[:4] = [[0.0, -0.0, 0.0], [np.inf, 1, 2], [1, -np.inf, 2], [np.nan, 1, 2]]
    for axis, name in enumerate(("x", "y", "z")):
        field = next(f for f in msg.fields if f.name == name)
        view = np.ndarray(
            (msg.width,), dtype="<f4", buffer=data,
            offset=field.offset, strides=(msg.point_step,),
        )
        view[:] = xyz[:, axis]
    msg.data = bytes(data)
    translation = [1, -2, 0.5]
    expected = scalar_transform(module, msg, translation, rotation)
    with np.errstate(invalid="ignore"):
        actual = module.apply_pointcloud_transform(msg, translation, rotation)
    assert bytes(actual.data) == expected


@pytest.mark.parametrize("shape", [(0, 2), (3, 0), (0, 0)])
def test_transform_empty_cloud(transform_utils, shape):
    """Handle zero-width and zero-height clouds without accessing point data."""
    msg = make_transform_cloud(transform_utils, width=shape[0], height=shape[1], padding=0)
    out = transform_utils.apply_pointcloud_transform(msg, [1, 2, 3], [0, 0, 0, 1])
    assert bytes(out.data) == bytes(msg.data) == b""
    assert (out.width, out.height) == shape


@pytest.mark.parametrize("problem", ["missing", "integer", "count", "offset"])
def test_transform_rejects_invalid_coordinates(transform_utils, problem):
    """Reject missing, nonscalar, unsupported, and out-of-bounds coordinates."""
    msg = make_transform_cloud(transform_utils)
    if problem == "missing":
        msg.fields = [f for f in msg.fields if f.name != "x"]
    elif problem == "integer":
        msg.fields[0].datatype = transform_utils.PointField.UINT32
    elif problem == "count":
        msg.fields[0].count = 2
    else:
        msg.fields[0].offset = msg.point_step
    with pytest.raises(ValueError):
        transform_utils.apply_pointcloud_transform(msg, [1, 2, 3], [0, 0, 0, 1])
