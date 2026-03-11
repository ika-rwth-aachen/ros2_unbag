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

import struct
import math
from pathlib import Path
import pickle

from pypcd4 import Encoding
from sensor_msgs.msg import PointField

from ros2_unbag.core.routines.base import ExportRoutine, ExportMode, ExportMetadata
from ros2_unbag.core.utils.pointcloud_utils import convert_pointcloud2_to_pypcd


@ExportRoutine("sensor_msgs/msg/PointCloud2", ["pointcloud/pkl"], mode=ExportMode.MULTI_FILE)
def export_pointcloud_pkl(msg, path: Path, fmt: str, metadata: ExportMetadata):
    """
    Export PointCloud2 message as a raw pickle file by dumping the message object to a .pkl.

    Args:
        msg: PointCloud2 message instance.
        path: Output file path (without extension).
        fmt: Export format string (default "pointcloud/pkl").
        metadata: Export metadata including message index and max index.

    Returns:
        None
    """
    with open(path.with_suffix(".pkl"), 'wb') as f:
        pickle.dump(msg, f)


@ExportRoutine("sensor_msgs/msg/PointCloud2", ["pointcloud/xyz"], mode=ExportMode.MULTI_FILE)
def export_pointcloud_xyz(
    msg,
    path: Path,
    fmt: str,
    metadata: ExportMetadata,
    extra_fields: str = "",
    filter_nan: bool = True,
):
    """
    Export PointCloud2 message as an XYZ text file by unpacking x, y, z floats from each point and writing lines.

    Args:
        msg: PointCloud2 message instance.
        path: Output file path (without extension).
        fmt: Export format string (default "pointcloud/xyz").
        metadata: Export metadata including message index and max index.
        extra_fields (str): Space-separated list of additional field names to append after X Y Z on each line
            (e.g. "intensity ring"). Each field must exist in the message.
        filter_nan (bool): When True, points containing NaN in x, y, or z are skipped.

    Returns:
        None
    """
    if isinstance(filter_nan, str):
        filter_nan = filter_nan.lower() not in ("false", "0", "no")

    # Validate required fields
    field_by_name = {f.name: f for f in msg.fields}
    for name in ("x", "y", "z"):
        if name not in field_by_name:
            raise ValueError(f"PointCloud2 missing '{name}' field")

    # Require FLOAT32 for x,y,z
    for name in ("x", "y", "z"):
        if field_by_name[name].datatype != PointField.FLOAT32:
            raise ValueError(f"Field '{name}' must be FLOAT32")

    # Resolve extra field names
    extra_field_names = [f for f in extra_fields.split() if f] if extra_fields else []
    extra_field_infos = []
    for name in extra_field_names:
        if name not in field_by_name:
            raise ValueError(f"PointCloud2 missing extra field '{name}'")
        extra_field_infos.append(field_by_name[name])

    offx, offy, offz = field_by_name["x"].offset, field_by_name["y"].offset, field_by_name["z"].offset
    endian = ">" if msg.is_bigendian else "<"

    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path.with_suffix(".xyz"), "w") as f:
        step = msg.point_step
        data = msg.data  # bytes/array('B')
        for i in range(0, len(data), step):
            x = struct.unpack_from(endian + "f", data, i + offx)[0]
            y = struct.unpack_from(endian + "f", data, i + offy)[0]
            z = struct.unpack_from(endian + "f", data, i + offz)[0]
            if filter_nan and (math.isnan(x) or math.isnan(y) or math.isnan(z)):
                continue
            if extra_field_infos:
                extra_vals = " ".join(
                    str(_unpack_field(endian, data, i, fi)) for fi in extra_field_infos
                )
                f.write(f"{x} {y} {z} {extra_vals}\n")
            else:
                f.write(f"{x} {y} {z}\n")


@ExportRoutine("sensor_msgs/msg/PointCloud2", ["pointcloud/pcd", "pointcloud/pcd_compressed", "pointcloud/pcd_ascii"], mode=ExportMode.MULTI_FILE)
def export_pointcloud_pcd(msg, path: Path, fmt: str, metadata: ExportMetadata):
    """
    Export PointCloud2 message as a binary PCD v0.7 file.
    Construct and write PCD header from message fields and metadata, then pack and write each point’s data.

    Args:
        msg: PointCloud2 message instance.
        path: Output file path (without extension).
        fmt: Export format string (default "pointcloud/xyz").
        metadata: Export metadata including message index and max index.

    Returns:
        None
    """

    # Build point cloud
    pc = convert_pointcloud2_to_pypcd(msg)


    # Save the point cloud to a PCD file
    if fmt == "pointcloud/pcd":
        pc.save(path.with_suffix(".pcd"), encoding=Encoding.BINARY)
    elif fmt == "pointcloud/pcd_compressed":
        pc.save(path.with_suffix(".pcd"), encoding=Encoding.BINARY_COMPRESSED)
    elif fmt == "pointcloud/pcd_ascii":
        pc.save(path.with_suffix(".pcd"), encoding=Encoding.ASCII)


_FIELD_STRUCT = {
    PointField.INT8:    "b",
    PointField.UINT8:   "B",
    PointField.INT16:   "h",
    PointField.UINT16:  "H",
    PointField.INT32:   "i",
    PointField.UINT32:  "I",
    PointField.FLOAT32: "f",
    PointField.FLOAT64: "d",
}


def _unpack_field(endian: str, data, point_offset: int, field):
    """Unpack a single scalar value from a PointCloud2 data buffer for the given field."""
    fmt_char = _FIELD_STRUCT.get(field.datatype)
    if fmt_char is None:
        raise ValueError(f"Unsupported PointField datatype {field.datatype} for field '{field.name}'")
    return struct.unpack_from(endian + fmt_char, data, point_offset + field.offset)[0]