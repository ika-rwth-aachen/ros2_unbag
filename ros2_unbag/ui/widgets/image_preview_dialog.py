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
Image Preview Dialog.

Opens the first Image or CompressedImage frame alongside an interactive
parameter panel.  Supported controls are determined by which parameter keys
are present in the *args* dict passed from the routine arguments form:

* **resize_width / resize_height** – always shown; live resize preview.
* **jpeg_quality** – JPEG quality spinbox; the preview encodes and decodes
  the image in memory so JPEG compression artefacts are visible in real time.
* **png_compression** – PNG compression level spinbox; PNG is lossless so no
  visual artefacts, but the output size estimate updates live.
* **target_fps** – FPS spinbox with "auto" checkbox (video export only).

Clicking "Apply to Settings" emits ``params_applied`` with a dict of all
currently visible parameter values and closes the dialog.
"""

import cv2
import numpy as np
from PySide6 import QtCore, QtGui, QtWidgets

__all__ = ["ImagePreviewDialog"]

_PANEL_WIDTH = 250
_UPDATE_DELAY_MS = 120  # debounce before refreshing displayed image


class ImagePreviewDialog(QtWidgets.QDialog):
    """
    Interactive image preview dialog for Image and CompressedImage export routines.

    The left pane displays the decoded, resized (and, for JPEG export, re-compressed)
    image so that the effect of every parameter change is immediately visible.
    The right pane contains controls for all parameters that are present in the
    *args* dict supplied at construction time.

    Signals:
        params_applied (dict): Emitted on "Apply to Settings".  Keys match the
            parameter names of the export routine (e.g. ``resize_width``,
            ``jpeg_quality``).
    """

    params_applied = QtCore.Signal(dict)

    def __init__(self, msg, args: dict, parent=None):
        """
        Initialise the dialog, decode the first frame, build UI, and render.

        Args:
            msg: Deserialised ``sensor_msgs/msg/Image`` or
                ``sensor_msgs/msg/CompressedImage`` ROS 2 message (first frame).
            args (dict): Current routine argument values from
                ``RoutineArgsWidget.get_args()``.  Used to seed all controls
                and to determine which parameter groups to show.
            parent: Optional Qt parent widget.

        Returns:
            None
        """
        super().__init__(parent)
        self.setWindowTitle(
            "Image Preview — edit values on the right · preview updates automatically"
        )
        self.resize(1000, 650)

        self._args = dict(args)
        self._fmt = self._args.pop("__fmt__", "")
        self._raw_bgr = self._decode_msg(msg)

        # Widget references (set conditionally in _init_ui)
        self._sb_width = self._auto_w = None
        self._sb_height = self._auto_h = None
        self._sb_jpeg_quality = None
        self._sb_png_compression = None
        self._sb_fps = self._auto_fps = None
        self._lbl_out_size = None

        self._init_ui()
        self._update_preview()

    # ------------------------------------------------------------------
    # Message decoding
    # ------------------------------------------------------------------

    @staticmethod
    def _decode_msg(msg) -> np.ndarray | None:
        """Decode a ROS Image or CompressedImage message to a BGR numpy array."""
        try:
            # CompressedImage
            if hasattr(msg, "format") and hasattr(msg, "data") and not hasattr(msg, "encoding"):
                np_arr = np.frombuffer(msg.data, np.uint8)
                img = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
            else:
                # sensor_msgs/msg/Image
                from ros2_unbag.core.utils.image_utils import convert_image
                raw = np.frombuffer(msg.data, dtype=np.uint8)
                img = convert_image(raw, msg.encoding, msg.width, msg.height)

            if img is None:
                return None
            # Normalise to 3-channel BGR uint8
            if img.dtype != np.uint8:
                # Scale 16-bit or float to 8-bit
                img = cv2.normalize(img, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
            if img.ndim == 2:
                img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            elif img.shape[2] == 4:
                img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
            return img
        except Exception:
            return None

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _init_ui(self) -> None:
        """Build the full dialog layout."""
        args = self._args

        root = QtWidgets.QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(6)

        # ── top row: image (left, expanding) + panel (right, fixed) ────
        top_row = QtWidgets.QHBoxLayout()
        top_row.setSpacing(8)
        root.addLayout(top_row, 1)

        # Image display
        self._img_label = QtWidgets.QLabel()
        self._img_label.setAlignment(QtCore.Qt.AlignCenter)
        self._img_label.setMinimumSize(400, 300)
        self._img_label.setSizePolicy(
            QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding
        )
        self._img_label.setStyleSheet("background-color: #111;")
        top_row.addWidget(self._img_label, 1)

        # Right panel (scrollable, fixed width)
        panel_frame = QtWidgets.QFrame()
        panel_frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        panel_frame.setFixedWidth(_PANEL_WIDTH)
        panel_outer = QtWidgets.QVBoxLayout(panel_frame)
        panel_outer.setContentsMargins(0, 0, 0, 0)
        top_row.addWidget(panel_frame)

        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        panel_outer.addWidget(scroll)

        inner = QtWidgets.QWidget()
        inner_layout = QtWidgets.QVBoxLayout(inner)
        inner_layout.setContentsMargins(6, 6, 6, 6)
        inner_layout.setSpacing(8)
        scroll.setWidget(inner)

        # ── Group: Image Info ──────────────────────────────────────────
        info_group = QtWidgets.QGroupBox("Image Info")
        info_form = QtWidgets.QFormLayout(info_group)
        info_form.setSpacing(4)
        info_form.setContentsMargins(6, 8, 6, 6)

        if self._raw_bgr is not None:
            oh, ow = self._raw_bgr.shape[:2]
            info_form.addRow("Original", QtWidgets.QLabel(f"{ow} × {oh} px"))
        else:
            info_form.addRow("Original", QtWidgets.QLabel("(decode failed)"))

        self._lbl_out_size = QtWidgets.QLabel("—")
        info_form.addRow("Output", self._lbl_out_size)

        self._lbl_file_est = QtWidgets.QLabel("—")
        self._lbl_file_est.setToolTip(
            "Rough in-memory estimate of the encoded file size."
        )
        info_form.addRow("Est. size", self._lbl_file_est)

        inner_layout.addWidget(info_group)

        # ── Group: Resize ──────────────────────────────────────────────
        resize_group = QtWidgets.QGroupBox("Resize")
        resize_form = QtWidgets.QFormLayout(resize_group)
        resize_form.setSpacing(4)
        resize_form.setContentsMargins(6, 8, 6, 6)

        raw_w = self._raw_bgr.shape[1] if self._raw_bgr is not None else 1920
        raw_h = self._raw_bgr.shape[0] if self._raw_bgr is not None else 1080

        self._sb_width = QtWidgets.QSpinBox()
        self._sb_width.setRange(1, 99999)
        self._auto_w = QtWidgets.QCheckBox("auto")
        w_val = args.get("resize_width")
        if w_val is None:
            self._sb_width.setValue(raw_w)
            self._sb_width.setEnabled(False)
            self._auto_w.setChecked(True)
        else:
            self._sb_width.setValue(int(w_val))
            self._auto_w.setChecked(False)
        resize_form.addRow("Width px", _inline(self._sb_width, self._auto_w))

        self._sb_height = QtWidgets.QSpinBox()
        self._sb_height.setRange(1, 99999)
        self._auto_h = QtWidgets.QCheckBox("auto")
        h_val = args.get("resize_height")
        if h_val is None:
            self._sb_height.setValue(raw_h)
            self._sb_height.setEnabled(False)
            self._auto_h.setChecked(True)
        else:
            self._sb_height.setValue(int(h_val))
            self._auto_h.setChecked(False)
        resize_form.addRow("Height px", _inline(self._sb_height, self._auto_h))

        inner_layout.addWidget(resize_group)

        # ── Group: JPEG Quality (image export only) ────────────────────
        if "jpeg_quality" in args:
            qual_group = QtWidgets.QGroupBox("JPEG Quality")
            qual_form = QtWidgets.QFormLayout(qual_group)
            qual_form.setSpacing(4)
            qual_form.setContentsMargins(6, 8, 6, 6)
            self._sb_jpeg_quality = QtWidgets.QSpinBox()
            self._sb_jpeg_quality.setRange(1, 100)
            self._sb_jpeg_quality.setSuffix(" %")
            self._sb_jpeg_quality.setMinimumWidth(80)
            self._sb_jpeg_quality.setValue(int(args.get("jpeg_quality", 95)))
            self._sb_jpeg_quality.setToolTip(
                "JPEG compression quality (1–100).\n"
                "Lower values produce smaller files but visible artefacts.\n"
                "The preview re-compresses and decodes the image in memory so\n"
                "you can see the artefacts before exporting."
            )
            qual_form.addRow("Quality", self._sb_jpeg_quality)
            inner_layout.addWidget(qual_group)

        # ── Group: PNG Compression (image export only) ─────────────────
        if "png_compression" in args:
            comp_group = QtWidgets.QGroupBox("PNG Compression")
            comp_form = QtWidgets.QFormLayout(comp_group)
            comp_form.setSpacing(4)
            comp_form.setContentsMargins(6, 8, 6, 6)
            self._sb_png_compression = QtWidgets.QSpinBox()
            self._sb_png_compression.setRange(0, 9)
            self._sb_png_compression.setMinimumWidth(80)
            self._sb_png_compression.setValue(int(args.get("png_compression", 3)))
            self._sb_png_compression.setToolTip(
                "PNG compression level (0 = no compression, 9 = maximum).\n"
                "PNG is lossless — higher levels produce smaller files at the\n"
                "cost of longer write time; the image is visually identical."
            )
            comp_form.addRow("Level (0–9)", self._sb_png_compression)
            inner_layout.addWidget(comp_group)

        # ── Group: Frame Rate (video export only) ──────────────────────
        if "target_fps" in args:
            fps_group = QtWidgets.QGroupBox("Frame Rate")
            fps_form = QtWidgets.QFormLayout(fps_group)
            fps_form.setSpacing(4)
            fps_form.setContentsMargins(6, 8, 6, 6)
            self._sb_fps = QtWidgets.QDoubleSpinBox()
            self._sb_fps.setRange(1.0, 240.0)
            self._sb_fps.setSingleStep(1.0)
            self._sb_fps.setDecimals(2)
            self._sb_fps.setSuffix(" fps")
            self._sb_fps.setMinimumWidth(80)
            self._auto_fps = QtWidgets.QCheckBox("auto")
            fps_val = args.get("target_fps")
            if fps_val is None:
                self._sb_fps.setValue(30.0)
                self._sb_fps.setEnabled(False)
                self._auto_fps.setChecked(True)
            else:
                self._sb_fps.setValue(float(fps_val))
                self._auto_fps.setChecked(False)
            self._sb_fps.setToolTip(
                "Override the auto-detected frame rate.\n"
                "When set to auto, FPS is inferred from the timestamp\n"
                "delta between the first two messages."
            )
            fps_form.addRow("FPS", _inline(self._sb_fps, self._auto_fps))
            inner_layout.addWidget(fps_group)

        inner_layout.addStretch()

        # ── info bar ──────────────────────────────────────────────────
        info_bar = QtWidgets.QLabel(
            "Edit values in the panel  ·  "
            "\u201cApply\u201d writes all parameters back to the settings form"
        )
        info_bar.setAlignment(QtCore.Qt.AlignCenter)
        info_bar.setStyleSheet("color: #6b7280; font-style: italic; padding: 2px;")
        info_bar.setWordWrap(True)
        root.addWidget(info_bar)

        # ── bottom buttons ────────────────────────────────────────────
        btn_row = QtWidgets.QHBoxLayout()
        btn_row.addStretch()
        btn_apply = QtWidgets.QPushButton("\u2714  Apply to Settings")
        btn_apply.setDefault(True)
        btn_apply.setMinimumWidth(160)
        btn_apply.clicked.connect(self._apply)
        btn_row.addWidget(btn_apply)
        btn_cancel = QtWidgets.QPushButton("Cancel")
        btn_cancel.clicked.connect(self.reject)
        btn_row.addWidget(btn_cancel)
        root.addLayout(btn_row)

        # ── debounce timer ────────────────────────────────────────────
        self._update_timer = QtCore.QTimer(self)
        self._update_timer.setSingleShot(True)
        self._update_timer.setInterval(_UPDATE_DELAY_MS)
        self._update_timer.timeout.connect(self._update_preview)

        # ── connect signals (after all setValue calls) ────────────────
        self._sb_width.valueChanged.connect(self._schedule_update)
        self._sb_height.valueChanged.connect(self._schedule_update)
        self._auto_w.stateChanged.connect(
            lambda s: (self._sb_width.setEnabled(s == 0), self._schedule_update())
        )
        self._auto_h.stateChanged.connect(
            lambda s: (self._sb_height.setEnabled(s == 0), self._schedule_update())
        )
        if self._sb_jpeg_quality is not None:
            self._sb_jpeg_quality.valueChanged.connect(self._schedule_update)
        if self._sb_png_compression is not None:
            self._sb_png_compression.valueChanged.connect(self._schedule_update)
        if self._sb_fps is not None:
            self._sb_fps.valueChanged.connect(self._schedule_update)
            self._auto_fps.stateChanged.connect(
                lambda s: (self._sb_fps.setEnabled(s == 0), self._schedule_update())
            )

    # ------------------------------------------------------------------
    # Preview rendering
    # ------------------------------------------------------------------

    def _schedule_update(self, *_) -> None:
        """(Re-)start the debounce timer."""
        self._update_timer.start()

    def _build_output_image(self) -> np.ndarray | None:
        """
        Apply resize (and, if JPEG quality is shown, JPEG round-trip encode) to the
        cached raw image and return the result as a BGR uint8 array.
        """
        if self._raw_bgr is None:
            return None

        img = self._raw_bgr.copy()

        # Resize
        w = None if self._auto_w.isChecked() else self._sb_width.value()
        h = None if self._auto_h.isChecked() else self._sb_height.value()
        if w is not None or h is not None:
            orig_h, orig_w = img.shape[:2]
            if w is None:
                w = max(1, int(round(orig_w * h / orig_h)))
            elif h is None:
                h = max(1, int(round(orig_h * w / orig_w)))
            img = cv2.resize(img, (int(w), int(h)))

        # JPEG round-trip: only when the selected output format is JPEG so the
        # user can see compression artefacts in the preview window.
        if self._sb_jpeg_quality is not None and "jpeg" in self._fmt:
            quality = self._sb_jpeg_quality.value()
            ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, quality])
            if ok:
                img = cv2.imdecode(buf, cv2.IMREAD_COLOR)

        return img

    def _update_preview(self) -> None:
        """Render the output image and update all info labels."""
        out = self._build_output_image()
        if out is None:
            self._img_label.setText(
                "<span style='color:#ef4444'>Failed to decode image.</span>"
            )
            return

        out_h, out_w = out.shape[:2]
        self._lbl_out_size.setText(f"{out_w} × {out_h} px")

        # File-size estimate
        self._lbl_file_est.setText(self._estimate_size(out))

        # Convert BGR → RGB for Qt
        rgb = cv2.cvtColor(out, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        qimg = QtGui.QImage(
            rgb.data.tobytes(), w, h, ch * w, QtGui.QImage.Format_RGB888
        )
        pixmap = QtGui.QPixmap.fromImage(qimg)

        # Scale to fit the label while keeping aspect ratio
        label_size = self._img_label.size()
        scaled = pixmap.scaled(
            label_size, QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation
        )
        self._img_label.setPixmap(scaled)

    def _estimate_size(self, img: np.ndarray) -> str:
        """Return a human-readable estimate of the encoded file size."""
        try:
            if "jpeg" in self._fmt and self._sb_jpeg_quality is not None:
                quality = self._sb_jpeg_quality.value()
                ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, quality])
            elif "png" in self._fmt and self._sb_png_compression is not None:
                level = self._sb_png_compression.value()
                ok, buf = cv2.imencode(".png", img, [cv2.IMWRITE_PNG_COMPRESSION, level])
            else:
                return "—"
            if not ok:
                return "—"
            size = len(buf)
            if size >= 1024 * 1024:
                return f"~{size / (1024 * 1024):.1f} MiB"
            return f"~{size / 1024:.0f} KiB"
        except Exception:
            return "—"

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        QtCore.QTimer.singleShot(10, self._update_preview)

    # ------------------------------------------------------------------
    # Collect params & apply
    # ------------------------------------------------------------------

    def _collect_params(self) -> dict:
        """Return all current panel values as a dict for ``RoutineArgsWidget.set_args()``."""
        params: dict = {}
        params["resize_width"] = None if self._auto_w.isChecked() else self._sb_width.value()
        params["resize_height"] = None if self._auto_h.isChecked() else self._sb_height.value()
        if self._sb_jpeg_quality is not None:
            params["jpeg_quality"] = self._sb_jpeg_quality.value()
        if self._sb_png_compression is not None:
            params["png_compression"] = self._sb_png_compression.value()
        if self._sb_fps is not None:
            params["target_fps"] = (
                None if self._auto_fps.isChecked() else round(self._sb_fps.value(), 3)
            )
        return params

    def _apply(self) -> None:
        """Emit ``params_applied`` and close the dialog."""
        self.params_applied.emit(self._collect_params())
        self.accept()


# ---------------------------------------------------------------------------
# Utility
# ---------------------------------------------------------------------------

def _inline(*widgets) -> QtWidgets.QWidget:
    """Pack *widgets* side-by-side in a zero-margin container."""
    w = QtWidgets.QWidget()
    h = QtWidgets.QHBoxLayout(w)
    h.setContentsMargins(0, 0, 0, 0)
    h.setSpacing(4)
    for i, wgt in enumerate(widgets):
        h.addWidget(wgt, 1 if i == 0 else 0)
    return w
