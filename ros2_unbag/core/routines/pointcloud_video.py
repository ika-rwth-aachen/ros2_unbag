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

"""
PointCloud Video Export Routine.

Renders each PointCloud2 frame into a colourised image and encodes all frames
into a single MP4 or AVI video file.  Colour is driven by any scalar field
present in the cloud (e.g. z-height, intensity, ring id) and a matplotlib
colormap.  Multiple orthographic projections (top-down, front, side) and a
full 3-D matplotlib scatter view are supported.
"""

from pathlib import Path
from typing import Optional

from ros2_unbag.core.routines.base import ExportRoutine, ExportMode, ExportMetadata
from ros2_unbag.core.utils.file_utils import get_time_from_msg
from ros2_unbag.core.utils.video_utils import write_video_frame, finalize_video
from ros2_unbag.core.utils.pointcloud_video_utils import render_frame


@ExportRoutine(
    "sensor_msgs/msg/PointCloud2",
    ["pointcloud/video_mp4", "pointcloud/video_avi"],
    mode=ExportMode.SINGLE_FILE,
)
def export_pointcloud_video(
    msg,
    path: Path,
    fmt: str,
    metadata: ExportMetadata,
    color_field: str = "intensity",
    colormap: str = "viridis",
    width: int = 1280,
    height: int = 720,
    point_size: int = 2,
    range_min: Optional[float] = None,
    range_max: Optional[float] = None,
    projection: str = "topdown",
    x_range: float = 50.0,
    y_range: float = 50.0,
    view_azimuth: float = 45.0,
    view_elevation: float = 30.0,
    bg_color: str = "black",
):
    """
    Export a sequence of PointCloud2 messages as a colourised video.

    Each message is rendered into a BGR image frame according to the chosen
    projection and colormap, then encoded into a video file via OpenCV.

    Args:
        msg: PointCloud2 ROS message instance.
        path (Path): Output file path (without extension).
        fmt (str): Export format string ("pointcloud/video_mp4" or "pointcloud/video_avi").
        metadata (ExportMetadata): Export metadata including message index and max index.
        color_field (str): Point field used to drive the colormap (e.g. "z", "intensity").
        colormap (str): Matplotlib colormap name (e.g. "jet", "viridis", "turbo").
        width (int): Output frame width in pixels.
        height (int): Output frame height in pixels.
        point_size (int): Rendered point radius in pixels.
        range_min (float or None): Lower bound for colormap normalisation. Auto-detected per frame when None.
        range_max (float or None): Upper bound for colormap normalisation. Auto-detected per frame when None.
        projection (str): View mode: "topdown", "front", "side", or "matplotlib3d".
        x_range (float): Half-width of the visible scene in metres (orthographic modes).
        y_range (float): Half-height of the visible scene in metres (orthographic modes).
        view_azimuth (float): Camera azimuth angle in degrees (matplotlib3d only).
        view_elevation (float): Camera elevation angle in degrees (matplotlib3d only).
        bg_color (str): Background colour: "black" or "white".

    Returns:
        None
    """
    ps = export_pointcloud_video.persistent_storage
    ts_ns = get_time_from_msg(msg, return_datetime=False)

    # Convert string values coming from CLI / JSON config to the correct types
    width = int(width)
    height = int(height)
    point_size = int(point_size)
    x_range = float(x_range)
    y_range = float(y_range)
    view_azimuth = float(view_azimuth)
    view_elevation = float(view_elevation)
    range_min = float(range_min) if range_min is not None else None
    range_max = float(range_max) if range_max is not None else None

    # Map format key to the video format expected by video_utils
    _FMT_MAP = {
        "pointcloud/video_mp4": "video/mp4",
        "pointcloud/video_avi": "video/avi",
    }
    video_fmt = _FMT_MAP.get(fmt, "video/mp4")

    img = render_frame(
        msg,
        color_field=color_field,
        colormap=colormap,
        width=width,
        height=height,
        point_size=point_size,
        range_min=range_min,
        range_max=range_max,
        projection=projection,
        x_range=x_range,
        y_range=y_range,
        view_azimuth=view_azimuth,
        view_elevation=view_elevation,
        bg_color=bg_color,
    )

    write_video_frame(ps, img, ts_ns, path, video_fmt)

    if metadata.index == metadata.max_index:
        finalize_video(ps, path, video_fmt)
