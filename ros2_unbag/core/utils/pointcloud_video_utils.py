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
PointCloud Video Rendering Utilities.

Provides helpers for extracting point data from PointCloud2 messages and
rendering them into BGR image frames suitable for video export.
"""

import struct

import cv2
import numpy as np

from sensor_msgs.msg import PointField

# Mapping from PointField datatype to (struct_char, numpy_dtype, byte_size)
_FIELD_TYPE_MAP = {
    PointField.INT8:    ("b", np.int8,    1),
    PointField.UINT8:   ("B", np.uint8,   1),
    PointField.INT16:   ("h", np.int16,   2),
    PointField.UINT16:  ("H", np.uint16,  2),
    PointField.INT32:   ("i", np.int32,   4),
    PointField.UINT32:  ("I", np.uint32,  4),
    PointField.FLOAT32: ("f", np.float32, 4),
    PointField.FLOAT64: ("d", np.float64, 8),
}

# Colormaps available in the GUI combo box (must be valid matplotlib cmap names)
AVAILABLE_COLORMAPS = [
    "viridis", "jet", "plasma", "inferno", "turbo",
    "magma", "rainbow", "hot", "coolwarm", "hsv",
]

# Projection modes available in the GUI combo box
AVAILABLE_PROJECTIONS = ["topdown", "front", "side", "matplotlib3d"]


def extract_field(msg, field_name: str) -> np.ndarray:
    """
    Extract the values of a named field from a PointCloud2 message as a float32 array.

    Args:
        msg: PointCloud2 ROS message instance.
        field_name (str): Name of the point field to extract (e.g. "z", "intensity").

    Returns:
        numpy.ndarray: 1-D float32 array with one value per point, NaN for invalid points.

    Raises:
        ValueError: If the requested field is not present in the message.
    """
    field_by_name = {f.name: f for f in msg.fields}
    if field_name not in field_by_name:
        available = list(field_by_name.keys())
        raise ValueError(
            f"Field '{field_name}' not found in PointCloud2. Available fields: {available}"
        )

    field = field_by_name[field_name]
    type_info = _FIELD_TYPE_MAP.get(field.datatype)
    if type_info is None:
        raise ValueError(f"Unsupported PointField datatype: {field.datatype}")

    fmt_char, np_dtype, _ = type_info
    endian = ">" if msg.is_bigendian else "<"
    full_fmt = endian + fmt_char
    offset = field.offset
    step = msg.point_step
    data = msg.data

    values = np.empty(msg.width * msg.height, dtype=np.float32)
    for i in range(len(values)):
        raw = struct.unpack_from(full_fmt, data, i * step + offset)[0]
        values[i] = float(raw)

    return values


def extract_xyz(msg) -> tuple:
    """
    Extract x, y, z coordinate arrays from a PointCloud2 message.

    Args:
        msg: PointCloud2 ROS message instance.

    Returns:
        tuple: (x, y, z) each a 1-D float32 numpy array, one element per point.

    Raises:
        ValueError: If x, y, or z fields are missing.
    """
    return extract_field(msg, "x"), extract_field(msg, "y"), extract_field(msg, "z")


def apply_colormap(values: np.ndarray, cmap_name: str, vmin=None, vmax=None) -> np.ndarray:
    """
    Map scalar values to BGR colours using a matplotlib colormap.

    Values are clipped to [vmin, vmax] before colormap normalisation, so any
    point whose color_field value falls outside the specified range will receive
    the colormap's edge colour (lowest or highest colour).

    Args:
        values (numpy.ndarray): 1-D array of scalar values.
        cmap_name (str): Matplotlib colormap name (e.g. "jet", "viridis").
        vmin (float or None): Lower clip/normalisation bound. Uses array min when None.
        vmax (float or None): Upper clip/normalisation bound. Uses array max when None.

    Returns:
        numpy.ndarray: Shape (N, 3) uint8 array of BGR colours.
    """
    import matplotlib.cm as cm
    import matplotlib.colors as mcolors

    finite = values[np.isfinite(values)]
    lo = float(vmin) if vmin is not None else (float(finite.min()) if len(finite) else 0.0)
    hi = float(vmax) if vmax is not None else (float(finite.max()) if len(finite) else 1.0)
    if hi == lo:
        hi = lo + 1.0

    # Explicitly clip: values below lo → lo, values above hi → hi
    clipped = np.clip(values, lo, hi)

    norm = mcolors.Normalize(vmin=lo, vmax=hi, clip=False)
    cmap = cm.get_cmap(cmap_name)
    # rgba float [0,1] -> uint8 RGB, then swap to BGR
    rgba = cmap(norm(clipped))  # (N, 4) float
    rgb = (rgba[:, :3] * 255).astype(np.uint8)
    bgr = rgb[:, ::-1].copy()  # RGB -> BGR
    return bgr


def render_frame(
    msg,
    color_field: str = "z",
    colormap: str = "jet",
    width: int = 1280,
    height: int = 720,
    point_size: int = 2,
    range_min=None,
    range_max=None,
    projection: str = "topdown",
    x_range: float = 50.0,
    y_range: float = 50.0,
    z_range: float = 10.0,
    view_azimuth: float = 45.0,
    view_elevation: float = 30.0,
    view_roll: float = 0.0,
    zoom: float = 1.0,
    bg_color: str = "black",
) -> np.ndarray:
    """
    Render a PointCloud2 message into a BGR image frame.

    Args:
        msg: PointCloud2 ROS message instance.
        color_field (str): Point field used to drive the colormap.
        colormap (str): Matplotlib colormap name.
        width (int): Output frame width in pixels.
        height (int): Output frame height in pixels.
        point_size (int): Rendered point radius in pixels.
        range_min (float or None): Lower clip bound for color_field values; values below this are clipped to this colour (auto-detect per frame when None).
        range_max (float or None): Upper clip bound for color_field values; values above this are clipped to this colour (auto-detect per frame when None).
        projection (str): View mode: "topdown", "front", "side", or "matplotlib3d".
        x_range (float): Half-width of the visible scene in metres (x-axis).
        y_range (float): Half-height of the visible scene in metres (y-axis).
        z_range (float): Half-depth of the visible scene in metres (z-axis, matplotlib3d only).
        view_azimuth (float): Camera azimuth in degrees (matplotlib3d only).
        view_elevation (float): Camera elevation in degrees (matplotlib3d only).
        view_roll (float): Camera roll in degrees (matplotlib3d only).
        zoom (float): Zoom factor; > 1 zooms in, < 1 zooms out (matplotlib3d only).
        bg_color (str): Background colour, "black" or "white".

    Returns:
        numpy.ndarray: H×W×3 BGR uint8 image.
    """
    x, y, z = extract_xyz(msg)
    color_values = extract_field(msg, color_field)
    bgr_colors = apply_colormap(color_values, colormap, range_min, range_max)

    if projection == "matplotlib3d":
        return _render_matplotlib3d(
            x, y, z, color_values, colormap, width, height,
            point_size, range_min, range_max, view_azimuth, view_elevation,
            view_roll, zoom, bg_color, x_range, y_range, z_range,
        )

    # Select which axes to project for each orthographic mode
    if projection == "topdown":
        horiz, vert = x, y
        h_range, v_range = x_range, y_range
        h_label, v_label = "X", "Y"
    elif projection == "front":
        horiz, vert = x, z
        h_range, v_range = x_range, y_range
        h_label, v_label = "X", "Z"
    elif projection == "side":
        horiz, vert = y, z
        h_range, v_range = x_range, y_range
        h_label, v_label = "Y", "Z"
    else:
        raise ValueError(
            f"Unknown projection '{projection}'. "
            f"Choose from: {AVAILABLE_PROJECTIONS}"
        )

    return _render_ortho(
        horiz, vert, bgr_colors, width, height,
        h_range, v_range, point_size, bg_color,
    )


def _render_ortho(
    horiz: np.ndarray,
    vert: np.ndarray,
    bgr_colors: np.ndarray,
    width: int,
    height: int,
    h_range: float,
    v_range: float,
    point_size: int,
    bg_color: str,
) -> np.ndarray:
    """
    Render an orthographic (bird's-eye / front / side) projection to a BGR image.

    Args:
        horiz (numpy.ndarray): Horizontal axis values (one per point).
        vert (numpy.ndarray): Vertical axis values (one per point).
        bgr_colors (numpy.ndarray): (N, 3) uint8 BGR colour per point.
        width (int): Frame width in pixels.
        height (int): Frame height in pixels.
        h_range (float): Half-width of visible range in metres.
        v_range (float): Half-height of visible range in metres.
        point_size (int): Point radius in pixels.
        bg_color (str): "black" or "white".

    Returns:
        numpy.ndarray: H×W×3 BGR uint8 image.
    """
    bg = (0, 0, 0) if bg_color == "black" else (255, 255, 255)
    img = np.full((height, width, 3), bg, dtype=np.uint8)

    # Pixel coordinates: centre of image = (0, 0) in scene space
    px = ((horiz / h_range + 1.0) * 0.5 * width).astype(np.int32)
    py = ((-vert / v_range + 1.0) * 0.5 * height).astype(np.int32)

    # Filter to visible region
    mask = (px >= 0) & (px < width) & (py >= 0) & (py < height)
    mask &= np.isfinite(horiz) & np.isfinite(vert)

    px_v = px[mask]
    py_v = py[mask]
    colors_v = bgr_colors[mask]

    if point_size <= 1:
        img[py_v, px_v] = colors_v
    else:
        r = max(1, point_size // 2)
        for i in range(len(px_v)):
            color = (int(colors_v[i, 0]), int(colors_v[i, 1]), int(colors_v[i, 2]))
            cv2.circle(img, (int(px_v[i]), int(py_v[i])), r, color, -1, cv2.LINE_AA)

    return img


def _render_matplotlib3d(
    x: np.ndarray,
    y: np.ndarray,
    z: np.ndarray,
    color_values: np.ndarray,
    colormap: str,
    width: int,
    height: int,
    point_size: int,
    vmin,
    vmax,
    azimuth: float,
    elevation: float,
    roll: float,
    zoom: float,
    bg_color: str,
    x_range: float = 50.0,
    y_range: float = 50.0,
    z_range: float = 10.0,
) -> np.ndarray:
    """
    Render a 3-D scatter plot of the point cloud via matplotlib and return a BGR image.

    Uses the Agg non-interactive backend so it is safe to call from worker processes.

    Args:
        x (numpy.ndarray): X coordinates.
        y (numpy.ndarray): Y coordinates.
        z (numpy.ndarray): Z coordinates.
        color_values (numpy.ndarray): Scalar values for colormap.
        colormap (str): Matplotlib colormap name.
        width (int): Frame width in pixels.
        height (int): Frame height in pixels.
        point_size (int): Marker size in matplotlib points.
        vmin (float or None): Lower clip bound for colour values.
        vmax (float or None): Upper clip bound for colour values.
        azimuth (float): Camera azimuth in degrees.
        elevation (float): Camera elevation in degrees.
        roll (float): Camera roll in degrees.
        zoom (float): Zoom factor; > 1 zooms in, < 1 zooms out.
        bg_color (str): "black" or "white".
        x_range (float): Half-extent for the X axis in metres.
        y_range (float): Half-extent for the Y axis in metres.
        z_range (float): Half-extent for the Z axis in metres.

    Returns:
        numpy.ndarray: H×W×3 BGR uint8 image.
    """
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    dpi = 100
    fig_w = width / dpi
    fig_h = height / dpi

    bg = "black" if bg_color == "black" else "white"

    fig = plt.figure(figsize=(fig_w, fig_h), dpi=dpi)
    fig.patch.set_facecolor(bg)
    ax = fig.add_subplot(111, projection="3d")
    ax.set_facecolor(bg)

    finite = np.isfinite(x) & np.isfinite(y) & np.isfinite(z) & np.isfinite(color_values)

    # Explicitly clip color values to [vmin, vmax] before passing to matplotlib
    clipped_color = color_values.copy()
    if vmin is not None:
        clipped_color = np.maximum(clipped_color, float(vmin))
    if vmax is not None:
        clipped_color = np.minimum(clipped_color, float(vmax))

    scatter_kwargs = dict(
        c=clipped_color[finite],
        cmap=colormap,
        s=max(1, point_size),
        linewidths=0,
        alpha=0.85,
    )
    if vmin is not None:
        scatter_kwargs["vmin"] = float(vmin)
    if vmax is not None:
        scatter_kwargs["vmax"] = float(vmax)

    ax.scatter(x[finite], y[finite], z[finite], **scatter_kwargs)
    ax.view_init(elev=elevation, azim=azimuth, roll=roll)

    # Fixed axis limits so the view does not change between frames.
    # Dividing by zoom shrinks the visible range, producing a zoom-in effect.
    effective_zoom = max(zoom, 1e-3)
    ax.set_xlim(-x_range / effective_zoom, x_range / effective_zoom)
    ax.set_ylim(-y_range / effective_zoom, y_range / effective_zoom)
    ax.set_zlim(-z_range / effective_zoom, z_range / effective_zoom)

    # Hide all axes, ticks, labels and panes
    ax.set_axis_off()

    fig.tight_layout(pad=0)
    fig.canvas.draw()

    buf = np.frombuffer(fig.canvas.buffer_rgba(), dtype=np.uint8)
    buf = buf.reshape(height, width, 4)
    plt.close(fig)

    # RGBA -> BGR
    bgr = buf[:, :, 2::-1].copy()
    return bgr
