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
from pathlib import Path
import pickle

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


@ExportRoutine("sensor_msgs/msg/PointCloud2", ["pointcloud/pcd"], mode=ExportMode.MULTI_FILE)
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
    # Map ROS2 field data types to struct format and PCD types
    type_map = {
        1: ('B', 'U', 1),
        2: ('H', 'U', 2),
        3: ('I', 'U', 4),
        4: ('b', 'I', 1),
        5: ('h', 'I', 2),
        6: ('i', 'I', 4),
        7: ('f', 'F', 4)
    }

    fields = msg.fields
    num_points = len(msg.data) // msg.point_step

    # Extract PCD metadata
    names = [f.name for f in fields]
    fmts = [type_map[f.datatype][0] for f in fields]
    types = [type_map[f.datatype][1] for f in fields]
    sizes = [type_map[f.datatype][2] for f in fields]
    counts = [f.count for f in fields]
    offsets = [f.offset for f in fields]

    # Build PCD header
    header = [
        "# .PCD v0.7 - Point Cloud Data file format", "VERSION 0.7",
        f"FIELDS {' '.join(names)}", f"SIZE {' '.join(map(str, sizes))}",
        f"TYPE {' '.join(types)}", f"COUNT {' '.join(map(str, counts))}",
        f"WIDTH {msg.width}", f"HEIGHT {msg.height}", "VIEWPOINT 0 0 0 1 0 0 0",
        f"POINTS {num_points}", "DATA binary"
    ]

    with open(path.with_suffix(".pcd"), "wb") as f:
        # Write header to file
        for line in header:
            f.write((line + "\n").encode("ascii"))

        # Write point data
        for i in range(0, len(msg.data), msg.point_step):
            for fmt, off in zip(fmts, offsets):
                val = struct.unpack_from(fmt, msg.data, offset=i + off)[0]
                f.write(struct.pack(fmt, val))
