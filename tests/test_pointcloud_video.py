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
Tests for the PointCloud-to-video export routine and its rendering utilities.
"""

import struct
import tempfile
from pathlib import Path

import cv2
import numpy as np
import pytest

import ros2_unbag.core.routines  # ensure all routines are registered

from ros2_unbag.core.routines.base import ExportRoutine, ExportMetadata
from ros2_unbag.core.utils.pointcloud_video_utils import (
    extract_field,
    extract_xyz,
    apply_colormap,
    render_frame,
    AVAILABLE_COLORMAPS,
    AVAILABLE_PROJECTIONS,
)


# ---------------------------------------------------------------------------
# Helpers to build synthetic PointCloud2 messages
# ---------------------------------------------------------------------------

def _make_pc2(points: list, fields_order=("x", "y", "z", "intensity")):
    """
    Create a minimal synthetic sensor_msgs/PointCloud2 message for testing.

    Args:
        points: List of dicts with field-name → float value per point.
        fields_order: Field names in declaration order.

    Returns:
        A mock PointCloud2-like object with the same API as the real one.
    """
    from sensor_msgs.msg import PointCloud2, PointField

    DTYPE_MAP = {
        "x": PointField.FLOAT32,
        "y": PointField.FLOAT32,
        "z": PointField.FLOAT32,
        "intensity": PointField.FLOAT32,
        "ring": PointField.UINT16,
    }
    STRUCT_MAP = {
        PointField.FLOAT32: ("f", 4),
        PointField.UINT16: ("H", 2),
    }

    # Build fields list
    fields = []
    offset = 0
    field_meta = {}
    for name in fields_order:
        dt = DTYPE_MAP[name]
        _, size = STRUCT_MAP[dt]
        field = PointField()
        field.name = name
        field.offset = offset
        field.datatype = dt
        field.count = 1
        fields.append(field)
        field_meta[name] = (dt, offset, size)
        offset += size

    point_step = offset

    # Pack binary data
    data = bytearray()
    for pt in points:
        row = bytearray(point_step)
        for name, (dt, off, size) in field_meta.items():
            fmt_char = STRUCT_MAP[dt][0]
            val = pt.get(name, 0.0)
            struct.pack_into(f"<{fmt_char}", row, off, val)
        data.extend(row)

    msg = PointCloud2()
    msg.height = 1
    msg.width = len(points)
    msg.fields = fields
    msg.is_bigendian = False
    msg.point_step = point_step
    msg.row_step = point_step * len(points)
    msg.is_dense = True
    msg.data = bytes(data)

    # Inject a fake header with a timestamp
    from std_msgs.msg import Header
    from builtin_interfaces.msg import Time
    h = Header()
    t = Time()
    t.sec = 1000
    t.nanosec = 0
    h.stamp = t
    msg.header = h

    return msg


# ---------------------------------------------------------------------------
# Unit tests: extract_field
# ---------------------------------------------------------------------------

class TestExtractField:
    def test_extracts_z(self):
        pts = [{"x": 1.0, "y": 2.0, "z": 3.0, "intensity": 10.0},
               {"x": 4.0, "y": 5.0, "z": 6.0, "intensity": 20.0}]
        msg = _make_pc2(pts)
        values = extract_field(msg, "z")
        np.testing.assert_allclose(values, [3.0, 6.0], atol=1e-5)

    def test_extracts_intensity(self):
        pts = [{"x": 0.0, "y": 0.0, "z": 0.0, "intensity": 42.0}]
        msg = _make_pc2(pts)
        values = extract_field(msg, "intensity")
        np.testing.assert_allclose(values, [42.0], atol=1e-5)

    def test_missing_field_raises(self):
        pts = [{"x": 0.0, "y": 0.0, "z": 0.0, "intensity": 0.0}]
        msg = _make_pc2(pts)
        with pytest.raises(ValueError, match="nonexistent"):
            extract_field(msg, "nonexistent")

    def test_extract_xyz(self):
        pts = [{"x": 1.0, "y": 2.0, "z": 3.0, "intensity": 0.0},
               {"x": -1.0, "y": -2.0, "z": -3.0, "intensity": 0.0}]
        msg = _make_pc2(pts)
        x, y, z = extract_xyz(msg)
        np.testing.assert_allclose(x, [1.0, -1.0], atol=1e-5)
        np.testing.assert_allclose(y, [2.0, -2.0], atol=1e-5)
        np.testing.assert_allclose(z, [3.0, -3.0], atol=1e-5)


# ---------------------------------------------------------------------------
# Unit tests: apply_colormap
# ---------------------------------------------------------------------------

class TestApplyColormap:
    def test_output_shape(self):
        values = np.linspace(0.0, 1.0, 10, dtype=np.float32)
        bgr = apply_colormap(values, "jet")
        assert bgr.shape == (10, 3)
        assert bgr.dtype == np.uint8

    def test_range_clipping(self):
        values = np.array([0.0, 5.0, 10.0], dtype=np.float32)
        bgr_full = apply_colormap(values, "jet", vmin=0.0, vmax=10.0)
        assert bgr_full.shape == (3, 3)

    def test_all_colormaps_run(self):
        values = np.linspace(0.0, 1.0, 5, dtype=np.float32)
        for cmap in AVAILABLE_COLORMAPS:
            result = apply_colormap(values, cmap)
            assert result.shape == (5, 3)


# ---------------------------------------------------------------------------
# Unit tests: render_frame (orthographic)
# ---------------------------------------------------------------------------

class TestRenderFrame:
    def _simple_msg(self):
        pts = [{"x": float(i), "y": float(i), "z": float(i), "intensity": float(i)}
               for i in range(100)]
        return _make_pc2(pts)

    def test_output_shape_topdown(self):
        msg = self._simple_msg()
        img = render_frame(msg, projection="topdown", width=320, height=240)
        assert img.shape == (240, 320, 3)
        assert img.dtype == np.uint8

    def test_output_shape_front(self):
        msg = self._simple_msg()
        img = render_frame(msg, projection="front", width=320, height=240)
        assert img.shape == (240, 320, 3)

    def test_output_shape_side(self):
        msg = self._simple_msg()
        img = render_frame(msg, projection="side", width=320, height=240)
        assert img.shape == (240, 320, 3)

    def test_black_background(self):
        # Background pixels should be (0,0,0) for bg_color="black"
        pts = [{"x": 1000.0, "y": 1000.0, "z": 1000.0, "intensity": 0.0}]
        msg = _make_pc2(pts)
        img = render_frame(msg, projection="topdown", width=64, height=64,
                           bg_color="black", x_range=1.0, y_range=1.0)
        # All points are out of range, so entire frame should be black
        assert np.all(img == 0)

    def test_white_background(self):
        pts = [{"x": 1000.0, "y": 1000.0, "z": 1000.0, "intensity": 0.0}]
        msg = _make_pc2(pts)
        img = render_frame(msg, projection="topdown", width=64, height=64,
                           bg_color="white", x_range=1.0, y_range=1.0)
        assert np.all(img == 255)

    def test_invalid_projection(self):
        msg = self._simple_msg()
        with pytest.raises(ValueError, match="projection"):
            render_frame(msg, projection="invalid")


# ---------------------------------------------------------------------------
# Unit tests: ExportRoutine.get_args for the new routine
# ---------------------------------------------------------------------------

class TestRoutineGetArgs:
    def test_get_args_returns_dict(self):
        args = ExportRoutine.get_args(
            "sensor_msgs/msg/PointCloud2", "pointcloud/video_mp4"
        )
        assert isinstance(args, dict)

    def test_expected_params_present(self):
        args = ExportRoutine.get_args(
            "sensor_msgs/msg/PointCloud2", "pointcloud/video_mp4"
        )
        for expected in (
            "color_field", "colormap", "width", "height", "point_size",
            "range_min", "range_max", "projection",
            "x_range", "y_range", "view_azimuth", "view_elevation", "bg_color",
        ):
            assert expected in args, f"Missing parameter: {expected}"

    def test_fixed_args_not_exposed(self):
        args = ExportRoutine.get_args(
            "sensor_msgs/msg/PointCloud2", "pointcloud/video_mp4"
        )
        for forbidden in ("msg", "path", "fmt", "metadata"):
            assert forbidden not in args

    def test_defaults_are_set(self):
        import inspect as _inspect
        args = ExportRoutine.get_args(
            "sensor_msgs/msg/PointCloud2", "pointcloud/video_mp4"
        )
        param_colormap, _ = args["colormap"]
        assert param_colormap.default == "jet"
        param_w, _ = args["width"]
        assert param_w.default == 1280


# ---------------------------------------------------------------------------
# Integration test: routine produces a valid video file
# ---------------------------------------------------------------------------

class TestExportPointcloudVideoIntegration:
    def test_single_frame_produces_video(self, tmp_path):
        pts = [{"x": float(i * 0.5 - 10), "y": float(j * 0.5 - 10), "z": float(i + j),
                "intensity": float(i * j)}
               for i in range(20) for j in range(20)]
        msg = _make_pc2(pts)

        from ros2_unbag.core.routines.pointcloud_video import export_pointcloud_video

        out_path = tmp_path / "test_pc_video"
        metadata = ExportMetadata(index=0, max_index=0)

        export_pointcloud_video(
            msg, out_path, "pointcloud/video_mp4", metadata,
            width=320, height=240, point_size=2,
            projection="topdown", x_range=15.0, y_range=15.0,
        )

        result = tmp_path / "test_pc_video.mp4"
        assert result.exists(), "Expected .mp4 output not created"
        assert result.stat().st_size > 0

        cap = cv2.VideoCapture(str(result))
        assert cap.isOpened()
        ret, frame = cap.read()
        assert ret, "Could not read a frame from the output video"
        assert frame.shape == (240, 320, 3)
        cap.release()

    def test_multi_frame_video(self, tmp_path):
        def make_msg(z_offset):
            pts = [{"x": float(i - 5), "y": float(j - 5), "z": float(i + j + z_offset),
                    "intensity": float(i)}
                   for i in range(10) for j in range(10)]
            return _make_pc2(pts)

        from ros2_unbag.core.routines.pointcloud_video import export_pointcloud_video

        out_path = tmp_path / "multi_frame"
        n_frames = 5
        for idx in range(n_frames):
            msg = make_msg(float(idx))
            metadata = ExportMetadata(index=idx, max_index=n_frames - 1)
            export_pointcloud_video(
                msg, out_path, "pointcloud/video_mp4", metadata,
                width=160, height=120, color_field="z",
                colormap="viridis", projection="topdown",
            )

        result = tmp_path / "multi_frame.mp4"
        assert result.exists()
        cap = cv2.VideoCapture(str(result))
        frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        cap.release()
        assert frame_count == n_frames
