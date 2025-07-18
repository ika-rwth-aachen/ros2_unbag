import os
import yaml
import numpy as np
import struct
import tf_transformations

from ros2_unbag.core.processors.base import Processor
from sensor_msgs.msg import PointCloud2


@Processor("sensor_msgs/msg/PointCloud2", ["transform_from_yaml"])
def apply_transform_from_yaml(msg, custom_frame_path):

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
    transform_matrix = tf_transformations.quaternion_matrix(rotation)
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
