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
Camera Preview Dialog.

Opens the first PointCloud2 frame as an interactive matplotlib 3-D scatter plot
alongside a live parameter panel.  The panel lets the user edit every relevant
rendering parameter directly and always stays in sync with the view:

* **Camera group** – azimuth, elevation, roll, zoom: bidirectionally synced
  with the matplotlib 3-D axes.  Dragging/scrolling the plot updates the
  spinboxes; editing a spinbox applies the change to the axes instantly
  *without* rebuilding the scatter plot (fast).

* **Scene Ranges group** – x_range, y_range, z_range: a fast axes limit update
  happens immediately; a full scatter re-render follows after a short debounce.

* **Color Range group** – range_min, range_max, each with an "auto" checkbox
  that disables the spinbox and passes ``None`` to the renderer.

* **Appearance group** – bg_color combo.

Clicking "Apply to Settings" emits ``camera_params_applied`` with a dict
containing **all** editable keys and closes the dialog.
"""

import numpy as np
from PySide6 import QtCore, QtWidgets

from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qtagg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 – registers the 3d projection

from ros2_unbag.core.utils.pointcloud_video_utils import extract_xyz, extract_field
from sensor_msgs.msg import PointCloud2

__all__ = ["CameraPreviewDialog"]

_PANEL_WIDTH = 260       # fixed width of the right-hand control panel (px)
_RERENDER_DELAY_MS = 250  # debounce before full scatter rebuild (ms)


class CameraPreviewDialog(QtWidgets.QDialog):
    """
    Interactive 3-D point cloud camera preview dialog with a live parameter panel.

    The left side shows the matplotlib scatter canvas with a navigation toolbar.
    The right side contains editable controls for all render parameters grouped
    into *Camera*, *Scene Ranges*, *Color Range*, and *Appearance*.

    Camera spinboxes (azim / elev / roll / zoom) are bidirectionally synced
    with the matplotlib axes:

    * Dragging or scrolling the plot → spinboxes update (no redraw cost).
    * Editing a spinbox → axes updated via ``view_init`` + ``draw_idle`` (fast,
      no scatter rebuild).

    Scene ranges, color range, and appearance changes trigger a debounced full
    scatter re-render that preserves the current view angle.

    Signals:
        camera_params_applied (dict): Emitted on "Apply to Settings".  Keys:
            ``view_azimuth``, ``view_elevation``, ``view_roll``, ``zoom``,
            ``x_range``, ``y_range``, ``z_range``,
            ``range_min`` (float or None), ``range_max`` (float or None),
            ``bg_color``.
    """

    camera_params_applied = QtCore.Signal(dict)

    def __init__(self, msg, args: dict, parent=None):
        """
        Initialise the dialog, extract point data, build UI, and render.

        Args:
            msg: Deserialised PointCloud2 ROS 2 message (first frame).
            args (dict): Current routine argument values from
                ``RoutineArgsWidget.get_args()``.  Used to seed all controls.
            parent: Optional Qt parent widget.

        Returns:
            None
        """
        super().__init__(parent)
        self.setWindowTitle(
            "Camera Preview — drag to rotate · scroll to zoom · edit panel values"
        )
        self.resize(1220, 780)

        self._msg = msg
        if not isinstance(msg, PointCloud2):
            # Conditionally import only if needed to avoid the dependency for non-cloudini-users
            from ros2_unbag.core.routines.cloudini_pointcloud import decode_cloudini_compressed_pointcloud
            self._msg = decode_cloudini_compressed_pointcloud(self._msg)
        self._args = dict(args)
        self._ax = None

        # Extract point cloud data once; reused by every re-render.
        self._color_field = str(args.get("color_field", "intensity"))
        self._px = self._py = self._pz = self._color_values = None
        self._extract_points()

        # Build UI — all spinboxes get their initial values BEFORE signals are
        # connected, so no premature callbacks fire during construction.
        self._init_ui()

        # First render (spinboxes already hold the correct initial values).
        self._render()

    # ------------------------------------------------------------------
    # Point extraction (once)
    # ------------------------------------------------------------------

    def _extract_points(self) -> None:
        """Extract XYZ + colour field from the ROS message; falls back to z."""
        try:
            self._px, self._py, self._pz = extract_xyz(self._msg)
            self._color_values = extract_field(self._msg, self._color_field)
        except Exception:
            try:
                self._px, self._py, self._pz = extract_xyz(self._msg)
                self._color_values = self._pz.copy()
            except Exception:
                self._px = self._py = self._pz = self._color_values = None

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _init_ui(self) -> None:
        """Build the full dialog layout."""
        args = self._args
        root = QtWidgets.QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(6)

        # ── top row: canvas (left, expanding) + panel (right, fixed) ────
        top_row = QtWidgets.QHBoxLayout()
        top_row.setSpacing(8)
        root.addLayout(top_row, 1)

        # Canvas column
        canvas_col = QtWidgets.QVBoxLayout()
        canvas_col.setSpacing(2)
        top_row.addLayout(canvas_col, 1)

        self._figure = Figure(figsize=(8, 6), dpi=100)
        self._canvas = FigureCanvas(self._figure)
        self._canvas.setSizePolicy(
            QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding
        )
        self._toolbar = NavigationToolbar(self._canvas, self)
        canvas_col.addWidget(self._toolbar)
        canvas_col.addWidget(self._canvas)

        # Right panel (scrollable, fixed width)
        panel_outer = QtWidgets.QFrame()
        panel_outer.setFrameShape(QtWidgets.QFrame.StyledPanel)
        panel_outer.setFixedWidth(_PANEL_WIDTH)
        panel_outer_layout = QtWidgets.QVBoxLayout(panel_outer)
        panel_outer_layout.setContentsMargins(0, 0, 0, 0)
        top_row.addWidget(panel_outer)

        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        panel_outer_layout.addWidget(scroll)

        inner = QtWidgets.QWidget()
        inner_layout = QtWidgets.QVBoxLayout(inner)
        inner_layout.setContentsMargins(6, 6, 6, 6)
        inner_layout.setSpacing(8)
        scroll.setWidget(inner)

        # ── Group: Camera ─────────────────────────────────────────────────
        cam_group = QtWidgets.QGroupBox("Camera")
        cam_form = QtWidgets.QFormLayout(cam_group)
        cam_form.setSpacing(4)
        cam_form.setContentsMargins(6, 8, 6, 6)

        self._sb_azim = self._dsb(-360.0, 360.0, 1.0, 1)
        self._sb_azim.setValue(float(args.get("view_azimuth") or 45.0))
        cam_form.addRow("Azimuth °", self._sb_azim)

        self._sb_elev = self._dsb(-90.0, 90.0, 1.0, 1)
        self._sb_elev.setValue(float(args.get("view_elevation") or 30.0))
        cam_form.addRow("Elevation °", self._sb_elev)

        self._sb_roll = self._dsb(-180.0, 180.0, 1.0, 1)
        self._sb_roll.setValue(float(args.get("view_roll") or 0.0))
        cam_form.addRow("Roll °", self._sb_roll)

        self._sb_zoom = self._dsb(0.01, 100.0, 0.1, 3)
        self._sb_zoom.setValue(float(args.get("zoom") or 1.0))
        cam_form.addRow("Zoom", self._sb_zoom)

        inner_layout.addWidget(cam_group)

        # ── Group: Scene Ranges ───────────────────────────────────────────
        range_group = QtWidgets.QGroupBox("Scene Ranges")
        range_form = QtWidgets.QFormLayout(range_group)
        range_form.setSpacing(4)
        range_form.setContentsMargins(6, 8, 6, 6)

        self._sb_xrange = self._dsb(0.1, 1e6, 5.0, 1)
        self._sb_xrange.setValue(float(args.get("x_range") or 50.0))
        range_form.addRow("X range (m)", self._sb_xrange)

        self._sb_yrange = self._dsb(0.1, 1e6, 5.0, 1)
        self._sb_yrange.setValue(float(args.get("y_range") or 50.0))
        range_form.addRow("Y range (m)", self._sb_yrange)

        self._sb_zrange = self._dsb(0.1, 1e6, 5.0, 1)
        self._sb_zrange.setValue(float(args.get("z_range") or 10.0))
        range_form.addRow("Z range (m)", self._sb_zrange)

        inner_layout.addWidget(range_group)

        # ── Group: Color Range ────────────────────────────────────────────
        color_group = QtWidgets.QGroupBox("Color Range")
        color_form = QtWidgets.QFormLayout(color_group)
        color_form.setSpacing(4)
        color_form.setContentsMargins(6, 8, 6, 6)

        self._sb_rmin = self._dsb(-1e9, 1e9, 1.0, 3)
        self._cb_rmin_auto = QtWidgets.QCheckBox("auto")
        rmin_val = args.get("range_min")
        if rmin_val is None:
            self._cb_rmin_auto.setChecked(True)
            self._sb_rmin.setValue(0.0)
            self._sb_rmin.setEnabled(False)
        else:
            self._cb_rmin_auto.setChecked(False)
            self._sb_rmin.setValue(float(rmin_val))
        color_form.addRow("Min", self._inline(self._sb_rmin, self._cb_rmin_auto))

        self._sb_rmax = self._dsb(-1e9, 1e9, 1.0, 3)
        self._cb_rmax_auto = QtWidgets.QCheckBox("auto")
        rmax_val = args.get("range_max")
        if rmax_val is None:
            self._cb_rmax_auto.setChecked(True)
            self._sb_rmax.setValue(0.0)
            self._sb_rmax.setEnabled(False)
        else:
            self._cb_rmax_auto.setChecked(False)
            self._sb_rmax.setValue(float(rmax_val))
        color_form.addRow("Max", self._inline(self._sb_rmax, self._cb_rmax_auto))

        inner_layout.addWidget(color_group)

        # ── Group: Appearance ─────────────────────────────────────────────
        app_group = QtWidgets.QGroupBox("Appearance")
        app_form = QtWidgets.QFormLayout(app_group)
        app_form.setSpacing(4)
        app_form.setContentsMargins(6, 8, 6, 6)

        self._cb_bgcolor = QtWidgets.QComboBox()
        self._cb_bgcolor.addItems(["black", "white"])
        bg_val = str(args.get("bg_color", "black"))
        self._cb_bgcolor.setCurrentIndex(max(0, self._cb_bgcolor.findText(bg_val)))
        app_form.addRow("Background", self._cb_bgcolor)

        self._sb_density = QtWidgets.QSpinBox()
        self._sb_density.setRange(1, 100)
        self._sb_density.setSingleStep(5)
        self._sb_density.setSuffix(" %")
        self._sb_density.setMinimumWidth(80)
        self._sb_density.setValue(int(args.get("preview_point_density", 10)))
        self._sb_density.setToolTip(
            "Percentage of points shown in the preview (lower = faster).\n"
            "The full dataset is always preserved for export."
        )
        app_form.addRow("Point density", self._sb_density)

        self._sb_point_size = QtWidgets.QSpinBox()
        self._sb_point_size.setRange(1, 50)
        self._sb_point_size.setSingleStep(1)
        self._sb_point_size.setMinimumWidth(80)
        self._sb_point_size.setValue(max(1, int(args.get("point_size") or 2)))
        app_form.addRow("Point size", self._sb_point_size)

        self._cb_color_field = QtWidgets.QComboBox()
        self._cb_color_field.setEditable(True)
        _available_fields = [f.name for f in self._msg.fields]
        self._cb_color_field.addItems(_available_fields)
        _cf_idx = self._cb_color_field.findText(self._color_field)
        if _cf_idx >= 0:
            self._cb_color_field.setCurrentIndex(_cf_idx)
        else:
            self._cb_color_field.setCurrentText(self._color_field)
        app_form.addRow("Color field", self._cb_color_field)

        self._cb_colormap = QtWidgets.QComboBox()
        self._cb_colormap.setEditable(True)
        _common_colormaps = [
            "viridis", "plasma", "inferno", "magma", "cividis",
            "turbo", "jet", "hot", "cool", "gray",
            "RdYlGn", "Spectral", "coolwarm",
        ]
        self._cb_colormap.addItems(_common_colormaps)
        _cm_val = str(args.get("colormap", "viridis"))
        _cm_idx = self._cb_colormap.findText(_cm_val)
        if _cm_idx >= 0:
            self._cb_colormap.setCurrentIndex(_cm_idx)
        else:
            self._cb_colormap.setCurrentText(_cm_val)
        app_form.addRow("Colormap", self._cb_colormap)

        inner_layout.addWidget(app_group)
        inner_layout.addStretch()

        # ── info bar ──────────────────────────────────────────────────────
        info = QtWidgets.QLabel(
            "Drag / scroll the plot  ·  or type values in the panel  ·  "
            "\u201cApply\u201d writes all parameters back to the settings form"
        )
        info.setAlignment(QtCore.Qt.AlignCenter)
        info.setStyleSheet("color: #6b7280; font-style: italic; padding: 2px;")
        info.setWordWrap(True)
        root.addWidget(info)

        # ── bottom buttons ────────────────────────────────────────────────
        btn_row = QtWidgets.QHBoxLayout()
        btn_row.addStretch()
        self._btn_apply = QtWidgets.QPushButton("\u2714  Apply to Settings")
        self._btn_apply.setDefault(True)
        self._btn_apply.setMinimumWidth(170)
        self._btn_apply.clicked.connect(self._apply)
        btn_row.addWidget(self._btn_apply)
        btn_cancel = QtWidgets.QPushButton("Cancel")
        btn_cancel.clicked.connect(self.reject)
        btn_row.addWidget(btn_cancel)
        root.addLayout(btn_row)

        # ── connect signals (after all setValue calls) ────────────────────

        # Camera spinboxes → instant axes update (no scatter rebuild)
        self._sb_azim.valueChanged.connect(self._on_camera_spinbox_changed)
        self._sb_elev.valueChanged.connect(self._on_camera_spinbox_changed)
        self._sb_roll.valueChanged.connect(self._on_camera_spinbox_changed)
        self._sb_zoom.valueChanged.connect(self._on_camera_spinbox_changed)

        # Scene-range spinboxes → instant axes limit update only
        # (ranges only affect set_xlim/ylim/zlim, not the scatter data)
        self._sb_xrange.valueChanged.connect(self._on_camera_spinbox_changed)
        self._sb_yrange.valueChanged.connect(self._on_camera_spinbox_changed)
        self._sb_zrange.valueChanged.connect(self._on_camera_spinbox_changed)

        # Debounce timer used by color range and appearance changes
        self._rerender_timer = QtCore.QTimer(self)
        self._rerender_timer.setSingleShot(True)
        self._rerender_timer.setInterval(_RERENDER_DELAY_MS)
        self._rerender_timer.timeout.connect(self._render)

        # Color range spinboxes + auto checkboxes → full re-render only
        self._sb_rmin.valueChanged.connect(self._schedule_rerender)
        self._sb_rmax.valueChanged.connect(self._schedule_rerender)
        self._cb_rmin_auto.stateChanged.connect(
            lambda s: (self._sb_rmin.setEnabled(s == 0), self._schedule_rerender())
        )
        self._cb_rmax_auto.stateChanged.connect(
            lambda s: (self._sb_rmax.setEnabled(s == 0), self._schedule_rerender())
        )

        # Appearance → full re-render
        self._cb_bgcolor.currentTextChanged.connect(self._schedule_rerender)
        self._sb_density.valueChanged.connect(self._schedule_rerender)
        self._sb_point_size.valueChanged.connect(self._schedule_rerender)
        # Color field → re-extract points, then re-render
        self._cb_color_field.currentTextChanged.connect(self._on_color_field_changed)
        self._cb_colormap.currentTextChanged.connect(self._schedule_rerender)

        # matplotlib event hooks
        self._canvas.mpl_connect("button_release_event", self._on_mpl_release)
        self._canvas.mpl_connect("motion_notify_event", self._on_mpl_motion)
        self._canvas.mpl_connect(
            "scroll_event",
            lambda _e: QtCore.QTimer.singleShot(80, self._sync_spinboxes_from_axes),
        )

    # ------------------------------------------------------------------
    # Widget factory helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _dsb(lo: float, hi: float, step: float, decimals: int) -> QtWidgets.QDoubleSpinBox:
        """Return a ``QDoubleSpinBox`` configured with the given parameters."""
        sb = QtWidgets.QDoubleSpinBox()
        sb.setRange(lo, hi)
        sb.setSingleStep(step)
        sb.setDecimals(decimals)
        sb.setMinimumWidth(80)
        return sb

    @staticmethod
    def _inline(*widgets) -> QtWidgets.QWidget:
        """Pack *widgets* side-by-side in a zero-margin container."""
        w = QtWidgets.QWidget()
        h = QtWidgets.QHBoxLayout(w)
        h.setContentsMargins(0, 0, 0, 0)
        h.setSpacing(4)
        for i, wgt in enumerate(widgets):
            h.addWidget(wgt, 1 if i == 0 else 0)
        return w

    # ------------------------------------------------------------------
    # Rendering
    # ------------------------------------------------------------------

    def _render(self) -> None:
        """
        Fully rebuild the scatter plot using the current panel values.

        Preserves the view angle (azim / elev / roll / zoom) so that a
        re-render triggered by a color or range change does not reset the
        user's view.

        Returns:
            None
        """
        if self._px is None:
            self._show_error("No point data could be extracted from this message.")
            return

        azimuth   = self._sb_azim.value()
        elevation = self._sb_elev.value()
        roll      = self._sb_roll.value()
        zoom      = max(self._sb_zoom.value(), 1e-3)
        x_range   = self._sb_xrange.value()
        y_range   = self._sb_yrange.value()
        z_range   = self._sb_zrange.value()
        vmin      = None if self._cb_rmin_auto.isChecked() else self._sb_rmin.value()
        vmax      = None if self._cb_rmax_auto.isChecked() else self._sb_rmax.value()
        bg_color  = self._cb_bgcolor.currentText()
        colormap  = self._cb_colormap.currentText() or "viridis"
        point_size = self._sb_point_size.value()

        bg = "black" if bg_color == "black" else "white"
        x, y, z, cv = self._px, self._py, self._pz, self._color_values

        finite = np.isfinite(x) & np.isfinite(y) & np.isfinite(z) & np.isfinite(cv)

        # Subsample for the preview only – the full data arrays are never modified.
        density = self._sb_density.value() / 100.0
        if density < 1.0:
            valid_idx = np.where(finite)[0]
            n_keep = max(1, int(len(valid_idx) * density))
            rng = np.random.default_rng(42)  # fixed seed → stable preview
            chosen = rng.choice(valid_idx, size=n_keep, replace=False)
            finite = np.zeros(len(x), dtype=bool)
            finite[chosen] = True

        clipped = cv.copy()
        if vmin is not None:
            clipped = np.maximum(clipped, float(vmin))
        if vmax is not None:
            clipped = np.minimum(clipped, float(vmax))

        self._figure.clear()
        self._figure.patch.set_facecolor(bg)
        ax = self._figure.add_subplot(111, projection="3d")
        ax.set_facecolor(bg)

        scatter_kwargs: dict = dict(
            c=clipped[finite], cmap=colormap, s=point_size, linewidths=0, alpha=0.85,
        )
        if vmin is not None:
            scatter_kwargs["vmin"] = float(vmin)
        if vmax is not None:
            scatter_kwargs["vmax"] = float(vmax)

        ax.scatter(x[finite], y[finite], z[finite], **scatter_kwargs)
        ax.view_init(elev=elevation, azim=azimuth, roll=roll)
        ax.set_xlim(-x_range / zoom, x_range / zoom)
        ax.set_ylim(-y_range / zoom, y_range / zoom)
        ax.set_zlim(-z_range / zoom, z_range / zoom)
        ax.set_box_aspect([1, 1, 1])
        ax.set_axis_off()

        self._figure.tight_layout(pad=0)
        self._ax = ax
        self._canvas.draw()

    def _show_error(self, message: str) -> None:
        """Replace canvas content with a centred error string."""
        self._figure.clear()
        self._figure.text(
            0.5, 0.5, message,
            ha="center", va="center", color="red", fontsize=12, wrap=True,
        )
        self._canvas.draw()

    def _apply_camera_to_axes(self) -> None:
        """
        Push the current camera + range spinbox values to the matplotlib axes
        and call ``draw_idle``.  Does *not* rebuild the scatter (fast path).

        Returns:
            None
        """
        if self._ax is None:
            return
        self._ax.view_init(
            elev=self._sb_elev.value(),
            azim=self._sb_azim.value(),
            roll=self._sb_roll.value(),
        )
        zoom = max(self._sb_zoom.value(), 1e-3)
        xr, yr, zr = self._sb_xrange.value(), self._sb_yrange.value(), self._sb_zrange.value()
        self._ax.set_xlim(-xr / zoom, xr / zoom)
        self._ax.set_ylim(-yr / zoom, yr / zoom)
        self._ax.set_zlim(-zr / zoom, zr / zoom)
        self._canvas.draw_idle()

    def _sync_spinboxes_from_axes(self) -> None:
        """
        Read azim / elev / roll / zoom from the matplotlib axes and update the
        camera spinboxes.  Uses ``blockSignals`` to avoid re-entrant callbacks.

        Returns:
            None
        """
        if self._ax is None:
            return

        azim = round(float(self._ax.azim), 1)
        elev = round(float(self._ax.elev), 1)
        roll = round(float(getattr(self._ax, "roll", 0.0)), 1)

        xlim = self._ax.get_xlim()
        half = abs(xlim[1] - xlim[0]) / 2.0
        zoom = round(self._sb_xrange.value() / half, 3) if half > 1e-6 else 1.0

        for sb, val in (
            (self._sb_azim, azim),
            (self._sb_elev, elev),
            (self._sb_roll, roll),
            (self._sb_zoom, zoom),
        ):
            sb.blockSignals(True)
            sb.setValue(val)
            sb.blockSignals(False)

    # ------------------------------------------------------------------
    # Collect final parameter dict
    # ------------------------------------------------------------------

    def _collect_params(self) -> dict:
        """
        Return all current panel values as a dict ready for
        ``RoutineArgsWidget.set_args()``.

        Returns:
            dict: Keys match ``export_pointcloud_video`` argument names.
        """
        return {
            "view_azimuth":   round(self._sb_azim.value(), 1),
            "view_elevation": round(self._sb_elev.value(), 1),
            "view_roll":      round(self._sb_roll.value(), 1),
            "zoom":           round(self._sb_zoom.value(), 3),
            "x_range":        self._sb_xrange.value(),
            "y_range":        self._sb_yrange.value(),
            "z_range":        self._sb_zrange.value(),
            "range_min":      None if self._cb_rmin_auto.isChecked() else self._sb_rmin.value(),
            "range_max":      None if self._cb_rmax_auto.isChecked() else self._sb_rmax.value(),
            "bg_color":       self._cb_bgcolor.currentText(),
            "point_size":     self._sb_point_size.value(),
            "color_field":    self._cb_color_field.currentText(),
            "colormap":       self._cb_colormap.currentText(),
        }

    # ------------------------------------------------------------------
    # Signal handlers
    # ------------------------------------------------------------------

    def _schedule_rerender(self, *_) -> None:
        """(Re-)start the debounce timer for a full scatter rebuild."""
        self._rerender_timer.start()

    def _on_camera_spinbox_changed(self, _value=None) -> None:
        """Apply the updated spinbox value to the axes immediately (fast path)."""
        self._apply_camera_to_axes()

    def _on_mpl_release(self, _event) -> None:
        """Sync spinboxes after the user releases the mouse."""
        self._sync_spinboxes_from_axes()

    def _on_mpl_motion(self, event) -> None:
        """Sync spinboxes while the user is dragging."""
        if event.button is not None:
            self._sync_spinboxes_from_axes()

    def _on_color_field_changed(self, field_name: str) -> None:
        """
        Update the active colour field, re-extract colour values from the
        stored message, and trigger a full re-render.

        Args:
            field_name (str): Name of the PointCloud2 field to use for colour.

        Returns:
            None
        """
        if not field_name:
            return
        self._color_field = field_name
        self._extract_points()
        self._render()

    # ------------------------------------------------------------------
    # Apply / close
    # ------------------------------------------------------------------

    def _apply(self) -> None:
        """
        Emit ``camera_params_applied`` with all current parameter values and
        close the dialog with an *Accepted* result.

        Returns:
            None
        """
        self.camera_params_applied.emit(self._collect_params())
        self.accept()
