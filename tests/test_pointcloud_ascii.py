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

"""Byte compatibility and layout checks for native point cloud ASCII export."""

import math
import struct
import types

import numpy as np
import pytest

from test_native_lzf import load_utils, native


FIELD_FORMATS = {1: "b", 2: "B", 3: "h", 4: "H", 5: "i", 6: "I", 7: "f", 8: "d"}


def reference_ascii(msg, *, xyz_only=False):
    """
    Format fields using the original Python scalar-to-string conversions.

    Args:
        msg: Cloud whose point and row padding should be respected.
        xyz_only: Whether to emit XYZ and apply the non-dense NaN rule.

    Returns:
        bytes: Expected ASCII rows with Python decimal formatting.
    """
    fields = msg.fields
    if xyz_only:
        fields = [next(field for field in fields if field.name == name) for name in ("x", "y", "z")]
    endian = ">" if msg.is_bigendian else "<"
    rows = []
    for row in range(msg.height):
        for column in range(msg.width):
            base = row * msg.row_step + column * msg.point_step
            values = []
            for field in fields:
                values.extend(struct.unpack_from(
                    endian + FIELD_FORMATS[field.datatype] * max(field.count, 1),
                    msg.data, base + field.offset,
                ))
            if xyz_only and not msg.is_dense and any(math.isnan(value) for value in values):
                continue
            rows.append(" ".join(str(value) for value in values) + "\n")
    return "".join(rows).encode("ascii")


def make_cloud(*, bigendian=False, dense=True):
    """
    Build padded points containing every supported type and a vector field.

    Args:
        bigendian: Whether source scalar bytes should use big-endian order.
        dense: Whether XYZ export should retain NaN rows.

    Returns:
        SimpleNamespace: PointCloud2-compatible message for layout tests.
    """
    # The declaration order deliberately differs from the byte offset order.
    specs = [("normal", 36, 7, 3), ("x", 4, 7, 1), ("y", 12, 7, 1),
             ("z", 20, 7, 1), ("int8", 0, 1, 1), ("uint8", 1, 2, 1),
             ("int16", 2, 3, 1), ("uint16", 8, 4, 1), ("int32", 16, 5, 1),
             ("uint32", 24, 6, 1), ("float64", 28, 8, 1)]
    msg = types.SimpleNamespace(width=2, height=2, point_step=52, row_step=111,
                                is_bigendian=bigendian, is_dense=dense)
    msg.fields = [types.SimpleNamespace(name=name, offset=offset, datatype=kind, count=count)
                  for name, offset, kind, count in specs]
    data = bytearray([0xCD]) * (msg.row_step * msg.height)
    coordinates = [(0.0, -0.0, 1e-5), (float("nan"), 2, 3),
                   (float("inf"), -float("inf"), 1e16), (0.1, -2.25, 4.5)]
    endian = ">" if bigendian else "<"
    for index, xyz in enumerate(coordinates):
        base = (index // msg.width) * msg.row_step + (index % msg.width) * msg.point_step
        values = {"x": (xyz[0],), "y": (xyz[1],), "z": (xyz[2],),
                  "normal": (0.1, -0.0, 1e-30), "int8": (-128,), "uint8": (255,),
                  "int16": (-32768,), "uint16": (65535,), "int32": (-2147483648,),
                  "uint32": (4294967295,), "float64": (1.2345678901234567,)}
        for field in msg.fields:
            struct.pack_into(endian + FIELD_FORMATS[field.datatype] * field.count,
                             data, base + field.offset, *values[field.name])
    msg.data = bytes(data)
    return msg


@pytest.mark.parametrize("bigendian", [False, True])
def test_pcd_ascii_all_field_types_and_padding(native, load_utils, bigendian):
    """Match Python bytes for all scalar types, vectors, and padded storage."""
    utils = load_utils(native)
    msg = make_cloud(bigendian=bigendian)
    assert utils.format_pcd_ascii_data(msg) == reference_ascii(msg)


@pytest.mark.parametrize("bigendian", [False, True])
@pytest.mark.parametrize("dense", [False, True])
def test_xyz_ascii_padding_and_nan_filtering(native, load_utils, bigendian, dense):
    """Respect row padding and keep the existing XYZ NaN and infinity rules."""
    utils = load_utils(native)
    msg = make_cloud(bigendian=bigendian, dense=dense)
    assert utils.format_pointcloud_xyz_data(msg) == reference_ascii(msg, xyz_only=True)


@pytest.mark.parametrize("datatype,dtype", [(7, "<f4"), (8, "<f8")])
def test_float_text_matches_python_across_bit_patterns(native, datatype, dtype):
    """Match shortest decimal notation for broad FLOAT32/FLOAT64 bit patterns."""
    rng = np.random.default_rng(42)
    integer_dtype = "<u4" if datatype == 7 else "<u8"
    maximum = 2**32 if datatype == 7 else 2**64
    values = rng.integers(0, maximum, 100000, dtype=integer_dtype).view(dtype)
    expected = "".join(str(value.item()) + "\n" for value in values).encode("ascii")
    actual = native.format_pointcloud_ascii(values.tobytes(), ((datatype, 1),), len(values))
    assert actual == expected


def test_float_text_at_notation_and_rounding_boundaries(native):
    """Preserve signed zero and Python's fixed/scientific exponent boundaries."""
    values = [0.0, -0.0, float("nan"), -float("nan"), float("inf"), -float("inf")]
    for exponent in range(-323, 309):
        value = 10.0**exponent
        values.extend([value, -value, math.nextafter(value, 0), math.nextafter(value, math.inf)])
    data = np.asarray(values, dtype="<f8")
    expected = "".join(str(value) + "\n" for value in values).encode("ascii")
    assert native.format_pointcloud_ascii(data.tobytes(), ((8, 1),), len(data)) == expected


def test_empty_ascii_clouds(native, load_utils):
    """Emit no rows for empty clouds and retain empty-field row behavior."""
    utils = load_utils(native)
    msg = make_cloud()
    msg.width, msg.data = 0, b""
    assert utils.format_pcd_ascii_data(msg) == b""
    assert utils.format_pointcloud_xyz_data(msg) == b""
    assert native.format_pointcloud_ascii(b"", (), 3) == b"\n\n\n"


@pytest.mark.parametrize("data,schema,count", [
    (b"", ((0, 1),), 0), (b"", ((9, 1),), 0), (b"", ((7, 0),), 0),
    (b"", ((7, -1),), 0), (b"123", ((7, 1),), 1), (b"12345", ((7, 1),), 1),
    (b"", ((7, 1),), -1), (b"x", ((7, 1),), 0),
])
def test_ascii_rejects_invalid_layouts(native, data, schema, count):
    """Reject inconsistent lengths and field descriptions before reading data."""
    with pytest.raises(ValueError):
        native.format_pointcloud_ascii(data, schema, count)


def test_ascii_rejects_mutable_input_and_malformed_schema(native):
    """Require immutable bytes and tuple schemas for safe GIL-free access."""
    with pytest.raises(TypeError):
        native.format_pointcloud_ascii(bytearray(4), ((7, 1),), 1)
    with pytest.raises(TypeError):
        native.format_pointcloud_ascii(b"", (None,), 0)
    with pytest.raises(OverflowError):
        native.format_pointcloud_ascii(b"", ((8, 2**63 - 1),), 0)
