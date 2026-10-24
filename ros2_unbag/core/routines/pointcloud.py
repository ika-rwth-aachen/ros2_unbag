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

from pathlib import Path
import pickle

from ros2_unbag.core.routines.base import ExportRoutine, ExportMode, ExportMetadata
from ros2_unbag.core.utils.pointcloud_utils import format_pointcloud_xyz_data, write_pointcloud_pcd


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
    Export PointCloud2 message as XYZ text using native scalar formatting.

    Args:
        msg: PointCloud2 message instance.
        path: Output file path (without extension).
        fmt: Export format string (default "pointcloud/xyz").
        metadata: Export metadata including message index and max index.

    Returns:
        None
    """
    payload = format_pointcloud_xyz_data(msg)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path.with_suffix(".xyz"), "wb") as f:
        f.write(payload)


@ExportRoutine("sensor_msgs/msg/PointCloud2", ["pointcloud/pcd", "pointcloud/pcd_compressed", "pointcloud/pcd_ascii"], mode=ExportMode.MULTI_FILE)
def export_pointcloud_pcd(msg, path: Path, fmt: str, metadata: ExportMetadata):
    """
    Export PointCloud2 message as a PCD v0.7 file.

    Supports ASCII, binary, and binary-compressed PCD output while preserving
    the original message fields.

    Args:
        msg: PointCloud2 message instance.
        path: Output file path (without extension).
        fmt: Export format string.
        metadata: Export metadata including message index and max index.

    Returns:
        None
    """

    if fmt == "pointcloud/pcd":
        write_pointcloud_pcd(msg, path.with_suffix(".pcd"), "binary")
    elif fmt == "pointcloud/pcd_compressed":
        write_pointcloud_pcd(msg, path.with_suffix(".pcd"), "binary_compressed")
    elif fmt == "pointcloud/pcd_ascii":
        write_pointcloud_pcd(msg, path.with_suffix(".pcd"), "ascii")
    else:
        raise ValueError(f"Unsupported point cloud export format: {fmt}")
