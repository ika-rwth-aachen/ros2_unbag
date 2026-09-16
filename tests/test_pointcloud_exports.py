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

from pathlib import Path
import struct
import sys
import types

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def _install_dependency_stubs():
    rosidl_runtime_py = types.ModuleType("rosidl_runtime_py")
    rosidl_runtime_py.message_to_ordereddict = lambda msg: {}
    rosidl_runtime_py.message_to_yaml = lambda msg: ""
    sys.modules.setdefault("rosidl_runtime_py", rosidl_runtime_py)

def _install_sensor_msgs_stubs():
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

        def __init__(self, name="", offset=0, datatype=0, count=1):
            self.name = name
            self.offset = offset
            self.datatype = datatype
            self.count = count

    class PointCloud2:
        pass

    sensor_msgs_msg.CompressedImage = type("CompressedImage", (), {})
    sensor_msgs_msg.Image = type("Image", (), {})
    sensor_msgs_msg.PointField = PointField
    sensor_msgs_msg.PointCloud2 = PointCloud2
    sensor_msgs.msg = sensor_msgs_msg
    sys.modules.setdefault("sensor_msgs", sensor_msgs)
    sys.modules.setdefault("sensor_msgs.msg", sensor_msgs_msg)

    return PointCloud2, PointField


_install_dependency_stubs()
PointCloud2, PointField = _install_sensor_msgs_stubs()

from ros2_unbag.core.routines.pointcloud import export_pointcloud_pcd, export_pointcloud_xyz  # noqa: E402


def _build_pointcloud(points, *, is_bigendian=False):
    msg = PointCloud2()
    msg.width = 2
    msg.height = 2
    msg.is_bigendian = is_bigendian
    msg.is_dense = True
    msg.point_step = 32
    msg.row_step = 72
    msg.fields = [
        PointField("x", 0, PointField.FLOAT32, 1),
        PointField("y", 4, PointField.FLOAT32, 1),
        PointField("z", 8, PointField.FLOAT32, 1),
        PointField("intensity", 12, PointField.UINT16, 1),
        PointField("ring", 14, PointField.UINT8, 1),
        PointField("normal", 16, PointField.FLOAT32, 3),
    ]

    data = bytearray(msg.row_step * msg.height)
    endian = ">" if is_bigendian else "<"
    for index, point in enumerate(points):
        row = index // msg.width
        column = index % msg.width
        base = row * msg.row_step + column * msg.point_step
        struct.pack_into(endian + "f", data, base + 0, point["x"])
        struct.pack_into(endian + "f", data, base + 4, point["y"])
        struct.pack_into(endian + "f", data, base + 8, point["z"])
        struct.pack_into(endian + "H", data, base + 12, point["intensity"])
        struct.pack_into(endian + "B", data, base + 14, point["ring"])
        struct.pack_into(endian + "fff", data, base + 16, *point["normal"])

    msg.data = bytes(data)
    return msg


def _parse_pcd(path: Path):
    content = path.read_bytes()
    lines = []
    offset = 0
    while True:
        newline = content.index(b"\n", offset)
        line = content[offset:newline].decode("utf-8")
        offset = newline + 1
        lines.append(line)
        if line.startswith("DATA "):
            break

    header = {}
    for line in lines:
        if not line or line.startswith("#"):
            continue
        key, value = line.split(" ", 1)
        header[key] = value

    return header, content[offset:]


def _lzf_decompress(data: bytes, expected_size: int) -> bytes:
    output = bytearray()
    index = 0
    while index < len(data):
        ctrl = data[index]
        index += 1
        if ctrl < 32:
            length = ctrl + 1
            output.extend(data[index:index + length])
            index += length
            continue

        length = ctrl >> 5
        ref = len(output) - ((ctrl & 0x1F) << 8) - 1
        if length == 7:
            length += data[index]
            index += 1
        ref -= data[index]
        index += 1
        length += 2
        for _ in range(length):
            output.append(output[ref])
            ref += 1

    assert len(output) == expected_size
    return bytes(output)


def _field_specs(header):
    fields = header["FIELDS"].split()
    sizes = [int(value) for value in header["SIZE"].split()]
    types_ = header["TYPE"].split()
    counts = [int(value) for value in header["COUNT"].split()]
    return list(zip(fields, sizes, types_, counts))


def _unpack_scalar(type_code, size, data):
    format_map = {
        ("I", 1): "b",
        ("I", 2): "h",
        ("I", 4): "i",
        ("U", 1): "B",
        ("U", 2): "H",
        ("U", 4): "I",
        ("F", 4): "f",
        ("F", 8): "d",
    }
    return struct.unpack("<" + format_map[(type_code, size)], data)[0]


def _parse_binary_points(header, payload: bytes):
    specs = _field_specs(header)
    point_count = int(header["POINTS"])
    point_size = sum(size * count for _, size, _, count in specs)
    rows = []
    offset = 0
    for _ in range(point_count):
        point = {}
        for name, size, type_code, count in specs:
            values = []
            for _ in range(count):
                chunk = payload[offset:offset + size]
                offset += size
                values.append(_unpack_scalar(type_code, size, chunk))
            point[name] = values[0] if count == 1 else values
        rows.append(point)
    assert offset == point_count * point_size
    return rows


def _parse_binary_compressed_points(header, payload: bytes):
    compressed_size, uncompressed_size = struct.unpack("<II", payload[:8])
    compressed = payload[8:8 + compressed_size]
    uncompressed = _lzf_decompress(compressed, uncompressed_size)

    specs = _field_specs(header)
    point_count = int(header["POINTS"])
    rows = [dict() for _ in range(point_count)]
    offset = 0
    for name, size, type_code, count in specs:
        field_width = size * count
        block = uncompressed[offset:offset + point_count * field_width]
        offset += point_count * field_width
        for point_index in range(point_count):
            values = []
            base = point_index * field_width
            for value_index in range(count):
                start = base + value_index * size
                values.append(_unpack_scalar(type_code, size, block[start:start + size]))
            rows[point_index][name] = values[0] if count == 1 else values
    assert offset == uncompressed_size
    return rows


def _normalize_points(points):
    normalized = []
    for point in points:
        normalized.append(
            {
                "x": point["x"],
                "y": point["y"],
                "z": point["z"],
                "intensity": point["intensity"],
                "ring": point["ring"],
                "normal": list(point["normal"]),
            }
        )
    return normalized


def _assert_points_close(actual, expected):
    assert len(actual) == len(expected)
    for actual_point, expected_point in zip(actual, expected):
        assert actual_point["intensity"] == expected_point["intensity"]
        assert actual_point["ring"] == expected_point["ring"]
        assert abs(actual_point["x"] - expected_point["x"]) < 1e-6
        assert abs(actual_point["y"] - expected_point["y"]) < 1e-6
        assert abs(actual_point["z"] - expected_point["z"]) < 1e-6
        for actual_value, expected_value in zip(actual_point["normal"], expected_point["normal"]):
            assert abs(actual_value - expected_value) < 1e-6


def test_export_pointcloud_pcd_binary_preserves_fields(tmp_path):
    points = [
        {"x": 1.0, "y": 2.0, "z": 3.0, "intensity": 10, "ring": 1, "normal": [0.1, 0.2, 0.3]},
        {"x": 4.0, "y": 5.0, "z": 6.0, "intensity": 20, "ring": 2, "normal": [0.4, 0.5, 0.6]},
        {"x": 7.0, "y": 8.0, "z": 9.0, "intensity": 30, "ring": 3, "normal": [0.7, 0.8, 0.9]},
        {"x": 10.0, "y": 11.0, "z": 12.0, "intensity": 40, "ring": 4, "normal": [1.0, 1.1, 1.2]},
    ]
    msg = _build_pointcloud(points)
    export_pointcloud_pcd(msg, tmp_path / "cloud", "pointcloud/pcd", None)

    header, payload = _parse_pcd(tmp_path / "cloud.pcd")
    assert header["FIELDS"] == "x y z intensity ring normal"
    assert header["SIZE"] == "4 4 4 2 1 4"
    assert header["TYPE"] == "F F F U U F"
    assert header["COUNT"] == "1 1 1 1 1 3"
    assert header["WIDTH"] == "2"
    assert header["HEIGHT"] == "2"
    assert header["POINTS"] == "4"
    assert header["DATA"] == "binary"
    _assert_points_close(_parse_binary_points(header, payload), _normalize_points(points))


def test_export_pointcloud_pcd_binary_compressed_handles_big_endian_input(tmp_path):
    points = [
        {"x": -1.5, "y": 2.25, "z": 3.5, "intensity": 11, "ring": 5, "normal": [1.5, 2.5, 3.5]},
        {"x": -4.5, "y": 5.25, "z": 6.5, "intensity": 22, "ring": 6, "normal": [4.5, 5.5, 6.5]},
        {"x": -7.5, "y": 8.25, "z": 9.5, "intensity": 33, "ring": 7, "normal": [7.5, 8.5, 9.5]},
        {"x": -10.5, "y": 11.25, "z": 12.5, "intensity": 44, "ring": 8, "normal": [10.5, 11.5, 12.5]},
    ]
    msg = _build_pointcloud(points, is_bigendian=True)
    export_pointcloud_pcd(msg, tmp_path / "cloud", "pointcloud/pcd_compressed", None)

    header, payload = _parse_pcd(tmp_path / "cloud.pcd")
    assert header["DATA"] == "binary_compressed"
    _assert_points_close(_parse_binary_compressed_points(header, payload), _normalize_points(points))


def test_export_pointcloud_pcd_binary_compressed_supports_larger_clouds(tmp_path):
    points = []
    for index in range(1024):
        points.append(
            {
                "x": float(index),
                "y": float(index + 1),
                "z": float(index + 2),
                "intensity": index % 65535,
                "ring": index % 255,
                "normal": [0.1, 0.2, 0.3],
            }
        )

    msg = PointCloud2()
    msg.width = len(points)
    msg.height = 1
    msg.is_bigendian = False
    msg.is_dense = True
    msg.point_step = 32
    msg.row_step = msg.width * msg.point_step
    msg.fields = [
        PointField("x", 0, PointField.FLOAT32, 1),
        PointField("y", 4, PointField.FLOAT32, 1),
        PointField("z", 8, PointField.FLOAT32, 1),
        PointField("intensity", 12, PointField.UINT16, 1),
        PointField("ring", 14, PointField.UINT8, 1),
        PointField("normal", 16, PointField.FLOAT32, 3),
    ]
    data = bytearray(msg.row_step)
    for index, point in enumerate(points):
        base = index * msg.point_step
        struct.pack_into("<f", data, base + 0, point["x"])
        struct.pack_into("<f", data, base + 4, point["y"])
        struct.pack_into("<f", data, base + 8, point["z"])
        struct.pack_into("<H", data, base + 12, point["intensity"])
        struct.pack_into("<B", data, base + 14, point["ring"])
        struct.pack_into("<fff", data, base + 16, *point["normal"])
    msg.data = bytes(data)

    export_pointcloud_pcd(msg, tmp_path / "cloud", "pointcloud/pcd_compressed", None)

    header, payload = _parse_pcd(tmp_path / "cloud.pcd")
    assert header["DATA"] == "binary_compressed"
    _assert_points_close(_parse_binary_compressed_points(header, payload), _normalize_points(points))


def test_export_pointcloud_pcd_ascii_writes_readable_rows(tmp_path):
    points = [
        {"x": 1.25, "y": 2.5, "z": 3.75, "intensity": 12, "ring": 1, "normal": [0.0, 1.0, 2.0]},
        {"x": 4.25, "y": 5.5, "z": 6.75, "intensity": 34, "ring": 2, "normal": [3.0, 4.0, 5.0]},
        {"x": 7.25, "y": 8.5, "z": 9.75, "intensity": 56, "ring": 3, "normal": [6.0, 7.0, 8.0]},
        {"x": 10.25, "y": 11.5, "z": 12.75, "intensity": 78, "ring": 4, "normal": [9.0, 10.0, 11.0]},
    ]
    msg = _build_pointcloud(points)
    export_pointcloud_pcd(msg, tmp_path / "cloud", "pointcloud/pcd_ascii", None)

    header, payload = _parse_pcd(tmp_path / "cloud.pcd")
    lines = payload.decode("utf-8").strip().splitlines()
    assert header["DATA"] == "ascii"
    assert lines[0] == "1.25 2.5 3.75 12 1 0.0 1.0 2.0"
    assert len(lines) == 4


def test_export_pointcloud_xyz_skips_nan_points_when_not_dense(tmp_path):
    msg = PointCloud2()
    msg.width = 3
    msg.height = 1
    msg.is_bigendian = False
    msg.is_dense = False
    msg.point_step = 12
    msg.row_step = 36
    msg.fields = [
        PointField("x", 0, PointField.FLOAT32, 1),
        PointField("y", 4, PointField.FLOAT32, 1),
        PointField("z", 8, PointField.FLOAT32, 1),
    ]

    data = bytearray(msg.row_step)
    struct.pack_into("<fff", data, 0, 1.0, 2.0, 3.0)
    struct.pack_into("<fff", data, 12, float("nan"), 5.0, 6.0)
    struct.pack_into("<fff", data, 24, 7.0, 8.0, 9.0)
    msg.data = bytes(data)

    export_pointcloud_xyz(msg, tmp_path / "cloud", "pointcloud/xyz", None)
    lines = (tmp_path / "cloud.xyz").read_text().strip().splitlines()

    assert lines == ["1.0 2.0 3.0", "7.0 8.0 9.0"]
