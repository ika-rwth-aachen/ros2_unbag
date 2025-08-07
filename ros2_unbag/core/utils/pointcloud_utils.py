
import numpy as np
from pypcd4 import PointCloud
from pypcd4.pointcloud2 import build_dtype_from_msg

def convert_pointcloud2_to_pypcd(msg):
    """Convert a PointCloud2 message to a Pypcd PointCloud object.
    Args:
        msg (sensor_msgs.msg.PointCloud2): PointCloud2 message instance.
    Returns:
        pypcd4.PointCloud: Pypcd PointCloud object.
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

    return pc