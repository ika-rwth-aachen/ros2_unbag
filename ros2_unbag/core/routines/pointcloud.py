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
import numpy as np
from pathlib import Path
import pickle

from pypcd4 import PointCloud, Encoding
from pypcd4.pointcloud2 import build_dtype_from_msg

from ros2_unbag.core.routines.base import ExportRoutine, ExportMode, ExportMetadata


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
def export_pointcloud_xyz(msg, path: Path, fmt: str, metadata: ExportMetadata):
    """
    Export PointCloud2 message as an XYZ text file by unpacking x, y, z floats from each point and writing lines.

    Args:
        msg: PointCloud2 message instance.
        path: Output file path (without extension).
        fmt: Export format string (default "pointcloud/xyz").
        metadata: Export metadata including message index and max index.

    Returns:
        None
    """
    with open(path.with_suffix(".xyz"), 'w') as f:
        for i in range(0, len(msg.data), msg.point_step):
            x, y, z = struct.unpack_from("fff", msg.data, offset=i)
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

    # Build dtype from message fields
    dtype_fields = build_dtype_from_msg(msg)
    dtype = np.dtype(dtype_fields)

    # Get field names and types
    field_names = tuple(f.name for f in msg.fields)
    np_types = tuple(dtype[name].type for name in field_names)
    structured_array = np.frombuffer(msg.data, dtype=dtype)
    points_np = np.vstack([structured_array[name] for name in field_names]).T

    # Build point cloud
    pc = PointCloud.from_points(points_np, field_names, np_types)

    # Save the point cloud to a PCD file
    if fmt == "pointcloud/pcd":
        pc.save(path.with_suffix(".pcd"), encoding=Encoding.BINARY)
    elif fmt == "pointcloud/pcd_compressed":
        pc.save(path.with_suffix(".pcd"), encoding=Encoding.BINARY_COMPRESSED)
    elif fmt == "pointcloud/pcd_ascii":
        pc.save(path.with_suffix(".pcd"), encoding=Encoding.ASCII)