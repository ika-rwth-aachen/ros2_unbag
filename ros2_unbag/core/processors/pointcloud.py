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

import os

import numpy as np
import yaml

from ros2_unbag.core.processors.base import Processor
from ros2_unbag.core.utils.pointcloud_utils import apply_pointcloud_transform
from sensor_msgs.msg import PointCloud2, PointField


@Processor("sensor_msgs/msg/PointCloud2", ["field_mapping"])
def pointcloud_apply_field_mapping(msg, field_mapping: str):
    """
    Apply a field mapping to a PointCloud2 message.

    Args:
        msg: PointCloud2 message instance.
        field_mapping: "field_name: new_field_name, field_name2: new_field_name2, ..."

    Returns:
        PointCloud2: Modified PointCloud2 message with remapped fields.

    Raises:
        ValueError: If field_mapping is invalid or fields do not exist in the message.
    """
    if not field_mapping:
        return msg

    existing_field_names = {field.name for field in msg.fields}
    mapping = {}

    for pair in field_mapping.split(","):
        if ":" not in pair:
            raise ValueError(f"Invalid mapping format: '{pair}' (expected 'old:new, ...')")
        old_field, new_field = map(str.strip, pair.split(":"))

        if old_field not in existing_field_names:
            raise ValueError(f"Field '{old_field}' does not exist in PointCloud2 message.")
        if not new_field or not new_field.isidentifier():
            raise ValueError(f"Invalid new field name: '{new_field}'")

        mapping[old_field] = new_field

    for field in msg.fields:
        if field.name in mapping:
            field.name = mapping[field.name]

    return msg


@Processor("sensor_msgs/msg/PointCloud2", ["remove_fields"])
def pointcloud_remove_fields(msg, fields_to_remove: str):
    """
    Remove specified fields from a PointCloud2 message.

    Args:
        msg: PointCloud2 message instance.
        fields_to_remove: "field_name, field_name2, ..."

    Returns:
        PointCloud2: Modified PointCloud2 message with specified fields removed.
    """

    # Check if the user actually specified fields to remove
    if not fields_to_remove:
        return msg

    remove_set = {f.strip() for f in fields_to_remove.split(",") if f.strip()}

    # Build list of fields to keep in original order
    original_fields = list(msg.fields)
    kept = [f for f in original_fields if f.name not in remove_set]

    # Nothing to remove or removing all would produce invalid cloud
    if len(kept) == len(original_fields) or len(kept) == 0:
        return msg

    # Only support height >= 1
    if msg.height < 1:
        raise ValueError("When removing fields, PointCloud2.height must be >= 1")

    # Helper: datatype sizes (bytes)
    type_size = {
        PointField.INT8: 1,
        PointField.UINT8: 1,
        PointField.INT16: 2,
        PointField.UINT16: 2,
        PointField.INT32: 4,
        PointField.UINT32: 4,
        PointField.FLOAT32: 4,
        PointField.FLOAT64: 8,
    }

    # Precompute source segments and construct new fields with compacted offsets
    kept_sorted = sorted(kept, key=lambda f: f.offset)
    segments = []  # (src_offset, size, dst_offset)
    new_fields = []
    dst_offset = 0
    for field in kept_sorted:
        size = type_size.get(field.datatype, 0) * (field.count if getattr(field, "count", 1) else 1)
        if size == 0:
            raise ValueError(f"Unsupported PointField datatype for field removal: {field.datatype}")
        segments.append((field.offset, size, dst_offset))
        nf = PointField()
        nf.name = field.name
        nf.offset = dst_offset
        nf.datatype = field.datatype
        # Some ROS2 PointField definitions may not include 'count' explicitly; default to 1
        nf.count = field.count if hasattr(field, "count") and field.count else 1
        new_fields.append(nf)
        dst_offset += size

    # If all segments were skipped due to unknown types, return original message
    if not segments:
        return msg

    new_point_step = dst_offset
    new_row_step = new_point_step * msg.width

    # View each point as raw bytes, respecting source row and point padding.
    # Copy whole fields across all points without interpreting their byte order.
    new_data = bytearray(new_row_step * msg.height)
    if msg.width:
        source = np.ndarray(
            shape=(msg.height, msg.width, msg.point_step),
            dtype=np.uint8,
            buffer=memoryview(msg.data),
            strides=(msg.row_step, msg.point_step, 1),
        )
        destination = np.frombuffer(new_data, dtype=np.uint8).reshape(
            msg.height, msg.width, new_point_step
        )
        for src_offset, size, dst_offset in segments:
            destination[..., dst_offset:dst_offset + size] = source[..., src_offset:src_offset + size]

    # Assemble new PointCloud2 message
    out = PointCloud2()
    out.header = msg.header
    out.height = msg.height
    out.width = msg.width
    out.fields = new_fields
    out.is_bigendian = msg.is_bigendian
    out.point_step = new_point_step
    out.row_step = new_row_step
    out.is_dense = msg.is_dense
    out.data = bytes(new_data)

    return out


@Processor("sensor_msgs/msg/PointCloud2", ["transform_from_yaml"])
def pointcloud_apply_transform_from_yaml(msg, custom_frame_path: str):
    """
    Apply a rigid-body transform from a YAML file to all points in a PointCloud2 message.

    Args:
        msg: PointCloud2 message instance.
        custom_frame_path: Path to YAML file containing translation as x, y, z and rotation as x, y, z, w.

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

    return apply_pointcloud_transform(msg, translation, rotation)


@Processor("sensor_msgs/msg/PointCloud2", ["transform"])
def pointcloud_apply_transform(
    msg,
    translation_x: float,
    translation_y: float,
    translation_z: float,
    rotation_x: float,
    rotation_y: float,
    rotation_z: float,
    rotation_w: float,
):
    """
    Apply a rigid-body transform directly from translation and quaternion components.

    Args:
        msg: PointCloud2 message instance.
        translation_x: Translation along x-axis.
        translation_y: Translation along y-axis.
        translation_z: Translation along z-axis.
        rotation_x: Quaternion x component.
        rotation_y: Quaternion y component.
        rotation_z: Quaternion z component.
        rotation_w: Quaternion w component.

    Returns:
        PointCloud2: Transformed PointCloud2 message.

    Raises:
        ValueError: If inputs are not numeric or message fields are missing.
    """
    try:
        translation = np.array(
            [float(translation_x), float(translation_y), float(translation_z)]
        )
        rotation = np.array(
            [float(rotation_x), float(rotation_y), float(rotation_z), float(rotation_w)]
        )
    except (TypeError, ValueError):
        raise ValueError("Translation and rotation arguments must be numeric.")

    return apply_pointcloud_transform(msg, translation, rotation)
