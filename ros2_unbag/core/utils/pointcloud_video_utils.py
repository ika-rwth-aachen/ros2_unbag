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

# Per-colormap BGR lookup table cache: computed once, reused for every frame
_COLORMAP_LUT_CACHE: dict = {}


def _get_colormap_lut(cmap_name: str) -> np.ndarray:
    """Return a cached (256, 3) uint8 BGR LUT for *cmap_name*.

    The LUT is built on the first call for each colormap name and stored in
    ``_COLORMAP_LUT_CACHE`` so that subsequent frames pay no matplotlib overhead.
    """
    if cmap_name not in _COLORMAP_LUT_CACHE:
        import matplotlib.cm as cm
        try:
            cmap = cm.colormaps[cmap_name]       # matplotlib >= 3.7
        except AttributeError:
            cmap = cm.get_cmap(cmap_name)        # matplotlib < 3.7
        indices = np.linspace(0.0, 1.0, 256)
        rgba = cmap(indices)                          # (256, 4) float64
        rgb = (rgba[:, :3] * 255).astype(np.uint8)    # (256, 3) uint8
        _COLORMAP_LUT_CACHE[cmap_name] = rgb[:, ::-1].copy()  # RGB -> BGR
    return _COLORMAP_LUT_CACHE[cmap_name]


# Colormaps available in the GUI combo box (must be valid matplotlib cmap names)
AVAILABLE_COLORMAPS = [
    "viridis", "jet", "plasma", "inferno", "turbo",
    "magma", "rainbow", "hot", "coolwarm", "hsv",
]


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

    _fmt_char, np_dtype, byte_size = type_info
    endian_char = ">" if msg.is_bigendian else "<"
    offset = field.offset
    step = msg.point_step
    n_points = msg.width * msg.height

    # Vectorised extraction: build a (n_points, byte_size) uint8 view, then
    # reinterpret as the target dtype in one shot — no Python loop required.
    raw_data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    point_starts = np.arange(n_points, dtype=np.int64) * step + offset
    byte_idx = point_starts[:, None] + np.arange(byte_size, dtype=np.int64)[None, :]
    target_dtype = np.dtype(np_dtype).newbyteorder(endian_char)
    values = raw_data[byte_idx].reshape(-1).view(target_dtype).astype(np.float32)

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
    finite = values[np.isfinite(values)]
    lo = float(vmin) if vmin is not None else (float(finite.min()) if len(finite) else 0.0)
    hi = float(vmax) if vmax is not None else (float(finite.max()) if len(finite) else 1.0)
    if hi == lo:
        hi = lo + 1.0

    # Map values to [0, 255] LUT indices using the cached BGR LUT.
    # NaN inputs are suppressed and clamped to index 0 (lowest colour).
    lut = _get_colormap_lut(cmap_name)
    clipped = np.clip(values, lo, hi)
    with np.errstate(invalid="ignore"):
        idx = ((clipped - lo) / (hi - lo) * 255.0).astype(np.int32)
    idx = np.clip(idx, 0, 255)
    return lut[idx]


def render_frame(
    msg,
    color_field: str = "z",
    colormap: str = "jet",
    width: int = 1280,
    height: int = 720,
    point_size: int = 2,
    range_min=None,
    range_max=None,
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
    Render a PointCloud2 message into a BGR image frame using a 3-D matplotlib scatter plot.

    Args:
        msg: PointCloud2 ROS message instance.
        color_field (str): Point field used to drive the colormap.
        colormap (str): Matplotlib colormap name.
        width (int): Output frame width in pixels.
        height (int): Output frame height in pixels.
        point_size (int): Rendered point radius in pixels.
        range_min (float or None): Lower clip bound for color_field values; values below this are clipped to this colour (auto-detect per frame when None).
        range_max (float or None): Upper clip bound for color_field values; values above this are clipped to this colour (auto-detect per frame when None).
        x_range (float): Half-width of the visible scene in metres (x-axis).
        y_range (float): Half-height of the visible scene in metres (y-axis).
        z_range (float): Half-depth of the visible scene in metres (z-axis).
        view_azimuth (float): Camera azimuth in degrees.
        view_elevation (float): Camera elevation in degrees.
        view_roll (float): Camera roll in degrees.
        zoom (float): Zoom factor; > 1 zooms in, < 1 zooms out.
        bg_color (str): Background colour, "black" or "white".

    Returns:
        numpy.ndarray: H×W×3 BGR uint8 image.
    """
    x, y, z = extract_xyz(msg)
    color_values = extract_field(msg, color_field)

    return _render_matplotlib3d(
        x, y, z, color_values, colormap, width, height,
        point_size, range_min, range_max, view_azimuth, view_elevation,
        view_roll, zoom, bg_color, x_range, y_range, z_range,
    )



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
    # Use the Agg canvas explicitly so we never touch the global backend
    # registry (which would conflict with the interactive Qt canvas used by
    # CameraPreviewDialog when the GUI is running).
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.figure import Figure as MplFigure
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    dpi = 100
    fig_w = width / dpi
    fig_h = height / dpi

    bg = "black" if bg_color == "black" else "white"

    fig = MplFigure(figsize=(fig_w, fig_h), dpi=dpi)
    FigureCanvasAgg(fig)  # attach Agg raster backend
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

    # RGBA -> BGR
    bgr = buf[:, :, 2::-1].copy()
    return bgr
