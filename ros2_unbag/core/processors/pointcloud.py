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

import os
import struct

import numpy as np
import yaml

from ros2_unbag.core.processors.base import Processor
from sensor_msgs.msg import PointCloud2


@Processor("sensor_msgs/msg/PointCloud2", ["transform_from_yaml"])
def apply_transform_from_yaml(msg, custom_frame_path):
    """
    Apply a rigid-body transform from a YAML file to all points in a PointCloud2 message.

    Args:
        msg: PointCloud2 message instance.
        custom_frame_path: Path to YAML file containing translation and rotation.

    Returns:
        PointCloud2: Transformed PointCloud2 message.

    Raises:
        ValueError: If file path is invalid or message fields are missing.
    """
    # Check if the provided path is valid
    if not os.path.isfile(custom_frame_path):
        raise ValueError(
            f"The provided custom_frame_path '{custom_frame_path}' is not a valid file path"
        )

    # Load transformation from YAML
    with open(custom_frame_path, 'r') as file:
        custom_frame = yaml.safe_load(file)

    t = custom_frame["translation"]
    r = custom_frame["rotation"]
    translation = np.array([t["x"], t["y"], t["z"]])
    rotation = np.array([r["x"], r["y"], r["z"], r["w"]])

    # Compute transformation matrix
    transform_matrix = quaternion_matrix(rotation)
    transform_matrix[0:3, 3] = translation

    # Find offsets of x, y, z fields
    offsets = {}
    for field in msg.fields:
        if field.name in ('x', 'y', 'z'):
            offsets[field.name] = field.offset

    if not all(k in offsets for k in ('x', 'y', 'z')):
        raise ValueError("PointCloud2 message does not contain x, y, z fields")

    x_off = offsets['x']
    y_off = offsets['y']
    z_off = offsets['z']

    # Transform the point data
    data = bytearray(msg.data)  # mutable copy

    for i in range(0, len(data), msg.point_step):
        # Unpack x, y, z from their respective offsets
        x = struct.unpack_from('f', data, i + x_off)[0]
        y = struct.unpack_from('f', data, i + y_off)[0]
        z = struct.unpack_from('f', data, i + z_off)[0]

        # Transform the point
        point = np.array([x, y, z, 1.0])
        transformed = transform_matrix @ point

        # Write back transformed coordinates
        struct.pack_into('f', data, i + x_off, transformed[0])
        struct.pack_into('f', data, i + y_off, transformed[1])
        struct.pack_into('f', data, i + z_off, transformed[2])

    # Construct the new PointCloud2 message
    transformed_msg = PointCloud2()
    transformed_msg.header = msg.header
    transformed_msg.height = msg.height
    transformed_msg.width = msg.width
    transformed_msg.fields = msg.fields
    transformed_msg.is_bigendian = msg.is_bigendian
    transformed_msg.point_step = msg.point_step
    transformed_msg.row_step = msg.row_step
    transformed_msg.is_dense = msg.is_dense
    transformed_msg.data = bytes(data)

    return transformed_msg

def quaternion_matrix(quaternion):
    """
    Compute a 4×4 transformation matrix from a quaternion [x, y, z, w].

    Args:
        quaternion: Sequence of 4 floats [x, y, z, w].

    Returns:
        numpy.ndarray: 4x4 transformation matrix.
    """
    x, y, z, w = quaternion
    N = x*x + y*y + z*z + w*w
    if N < np.finfo(float).eps:
        return np.eye(4)
    s = 2.0 / N
    xx, yy, zz = x*x*s, y*y*s, z*z*s
    xy, xz, yz = x*y*s, x*z*s, y*z*s
    wx, wy, wz = w*x*s, w*y*s, w*z*s

    M = np.eye(4)
    M[0,0] = 1 - (yy + zz)
    M[0,1] =     xy - wz
    M[0,2] =     xz + wy
    M[1,0] =     xy + wz
    M[1,1] = 1 - (xx + zz)
    M[1,2] =     yz - wx
    M[2,0] =     xz - wy
    M[2,1] =     yz + wx
    M[2,2] = 1 - (xx + yy)
    return M
