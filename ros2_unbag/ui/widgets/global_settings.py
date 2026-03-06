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
Global Settings Widget Module.

Provides the GlobalSettingsWidget class for configuring application-wide export
settings in the ros2_unbag GUI. This widget appears in the right column and manages
CPU usage limits, resampling configuration, and the main export action.
"""

from pathlib import Path

from PySide6 import QtCore, QtWidgets

from ros2_unbag.ui.styles import (
    CANCEL_BANNER_STYLE,
    CANCEL_BUTTON_STYLE,
    EXPORT_BUTTON_STYLE,
    SUCCESS_BANNER_STYLE,
)

__all__ = ["GlobalSettingsWidget"]


class GlobalSettingsWidget(QtWidgets.QWidget):
    """
    Widget for global export settings and summary display.
    
    This widget provides controls for:
    - CPU usage limit configuration (percentage slider)
    - Error handling strategy (fail-fast vs continue-on-error)
    - Global resampling settings (master topic, association strategy, epsilon)
    - Base output directory selection
    - Export summary showing selected topic count
    - Export action button to trigger the export process
    
    The resampling feature allows synchronizing multiple topics to a master topic's
    timeline, useful for temporal alignment of sensor data.
    
    Signals:
        export_clicked: Emitted when the export button is clicked
        base_dir_changed (str): Emitted when the base output directory is changed
    """
    
    export_clicked = QtCore.Signal()
    cancel_clicked = QtCore.Signal()
    base_dir_changed = QtCore.Signal(str)

    def __init__(self, parent=None):
        """
        Initialize the GlobalSettingsWidget with UI components.

        Args:
            parent: Optional parent widget.

        Returns:
            None
        """
        super().__init__(parent)
        self.selected_topics = []
        self.base_dir = Path.cwd()
        self._has_selection = False
        self._is_export_running = False
        self.init_ui()

    def init_ui(self):
        """
        Build the global settings UI with CPU controls, resampling options, summary, and export button.

        Args:
            None

        Returns:
            None
        """
        layout = QtWidgets.QVBoxLayout(self)
        layout.setAlignment(QtCore.Qt.AlignTop)

        # Global Settings Group
        self.gb_settings = QtWidgets.QGroupBox("Global Settings")
        form_layout = QtWidgets.QFormLayout(self.gb_settings)

        # CPU Usage
        self.cpu_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.cpu_slider.setRange(0, 100)
        self.cpu_slider.setValue(80)
        self.cpu_slider.setSingleStep(10)
        self.cpu_slider.setPageStep(10)
        self.cpu_slider.setTickInterval(10)
        self.cpu_slider.setTickPosition(QtWidgets.QSlider.TicksBelow)
        self.cpu_spin = QtWidgets.QDoubleSpinBox()
        self.cpu_spin.setRange(0.0, 100.0)
        self.cpu_spin.setSingleStep(1.0)
        self.cpu_spin.setDecimals(1)
        self.cpu_spin.setValue(80.0)
        
        def _slider_to_spin(val):
            # snap slider to 10% steps, keep spinbox in sync with snapped value
            snapped = round(val / 10) * 10
            if snapped != val:
                self.cpu_slider.blockSignals(True)
                self.cpu_slider.setValue(snapped)
                self.cpu_slider.blockSignals(False)
            self.cpu_spin.setValue(float(snapped))

        def _spin_to_slider(val):
            # reflect manual input on the slider without snapping the spin value
            self.cpu_slider.blockSignals(True)
            self.cpu_slider.setValue(int(round(val)))
            self.cpu_slider.blockSignals(False)

        self.cpu_slider.valueChanged.connect(_slider_to_spin)
        self.cpu_spin.valueChanged.connect(_spin_to_slider)
        
        cpu_layout = QtWidgets.QHBoxLayout()
        cpu_layout.addWidget(self.cpu_slider)
        cpu_layout.addWidget(self.cpu_spin)
        cpu_label = QtWidgets.QLabel("CPU Usage %")
        cpu_label.setToolTip("Set the maximum CPU percentage allowed for export workers.")
        form_layout.addRow(cpu_label, cpu_layout)

        self.continue_on_error_check = QtWidgets.QCheckBox("Continue on error")
        self.continue_on_error_check.setToolTip(
            "If enabled, item-level processor/routine failures are logged and skipped instead of aborting the export."
        )
        form_layout.addRow("Error Handling", self.continue_on_error_check)

        # Resampling
        self.assoc_combo = QtWidgets.QComboBox()
        self.assoc_combo.addItems(["no resampling", "last", "nearest"])
        self.assoc_combo.currentTextChanged.connect(self._on_assoc_changed)
        assoc_label = QtWidgets.QLabel("Association")
        assoc_label.setToolTip("Choose how to align messages to a master timeline (or disable resampling).")
        form_layout.addRow(assoc_label, self.assoc_combo)

        self.eps_edit = QtWidgets.QLineEdit()
        self.eps_edit.setPlaceholderText("e.g. 0.5")
        self.eps_edit.setEnabled(False)
        eps_label = QtWidgets.QLabel("Discard Eps (s)")
        eps_label.setToolTip("Discard messages with timestamp offsets larger than this (seconds).")
        form_layout.addRow(eps_label, self.eps_edit)

        self.master_combo = QtWidgets.QComboBox()
        self.master_combo.setEnabled(False)
        master_label = QtWidgets.QLabel("Master Topic")
        master_label.setToolTip("Select the master topic used as the timing reference when resampling.")
        form_layout.addRow(master_label, self.master_combo)

        layout.addWidget(self.gb_settings)

        # Base Directory Group
        self.gb_base = QtWidgets.QGroupBox("Base Directory")
        self.gb_base.setToolTip("Changes the base directory for all topic exports at once.")
        base_layout = QtWidgets.QVBoxLayout(self.gb_base)
        base_layout.setSpacing(6)
        base_layout.setContentsMargins(10, 10, 10, 10)
        self.base_dir_label = QtWidgets.QLabel(str(self.base_dir))
        self.base_dir_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.base_dir_label.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)
        self.base_dir_full = str(self.base_dir)
        self.base_dir_label.setToolTip(self.base_dir_full)
        self.btn_base_dir = QtWidgets.QPushButton("Change")
        self.btn_base_dir.clicked.connect(self._choose_base_dir)
        row = QtWidgets.QHBoxLayout()
        row.setContentsMargins(0, 0, 0, 0)
        row.addWidget(self.base_dir_label)
        row.addWidget(self.btn_base_dir)
        base_layout.addLayout(row)
        desc = QtWidgets.QLabel("Changes the base directory for all topic exports at once.")
        desc.setWordWrap(True)
        desc.setStyleSheet("color: #475569;")
        base_layout.addWidget(desc)
        layout.addWidget(self.gb_base)

        # Summary Group
        self.gb_summary = QtWidgets.QGroupBox("Summary")
        self.gb_summary.setToolTip("Shows how many topics are selected for export out of the total loaded.")
        self.summary_layout = QtWidgets.QVBoxLayout(self.gb_summary)
        self.summary_label = QtWidgets.QLabel("No bag loaded.")
        self.summary_layout.addWidget(self.summary_label)
        layout.addWidget(self.gb_summary)

        layout.addStretch()

        # Inline feedback banner above export button
        self.feedback_banner = QtWidgets.QFrame()
        self.feedback_banner.setObjectName("feedbackBanner")
        self.feedback_banner.setVisible(False)
        self.feedback_banner.setStyleSheet(SUCCESS_BANNER_STYLE)
        banner_layout = QtWidgets.QHBoxLayout(self.feedback_banner)
        banner_layout.setContentsMargins(10, 6, 10, 6)
        banner_layout.setSpacing(6)
        self.feedback_label = QtWidgets.QLabel("")
        banner_layout.addWidget(self.feedback_label)
        banner_layout.addStretch()
        self.feedback_close = QtWidgets.QToolButton()
        self.feedback_close.setText("x")
        self.feedback_close.setCursor(QtCore.Qt.PointingHandCursor)
        self.feedback_close.setStyleSheet(
            "QToolButton { border: none; padding: 2px; font-weight: 700; color: #475569; }"
            "QToolButton:hover { color: #0f172a; }"
        )
        self.feedback_close.clicked.connect(self.hide_feedback)
        banner_layout.addWidget(self.feedback_close)
        self.feedback_timer = QtCore.QTimer(self)
        self.feedback_timer.setSingleShot(True)
        self.feedback_timer.timeout.connect(self.hide_feedback)
        layout.addWidget(self.feedback_banner)

        # Export Button
        self.btn_export = QtWidgets.QPushButton("Unbag")
        self.btn_export.setMinimumHeight(56)
        self.btn_export.setStyleSheet(EXPORT_BUTTON_STYLE)
        self.btn_export.clicked.connect(self.export_clicked)
        self.btn_export.setEnabled(False)
        layout.addWidget(self.btn_export)

        # Cancel Button (initially hidden, shown when export is running)
        self.btn_cancel = QtWidgets.QPushButton("Cancel Export")
        self.btn_cancel.setMinimumHeight(56)
        self.btn_cancel.setStyleSheet(CANCEL_BUTTON_STYLE)
        self.btn_cancel.clicked.connect(self.cancel_clicked)
        self.btn_cancel.setVisible(False)
        layout.addWidget(self.btn_cancel)

    def show_feedback(self, message: str, duration_ms: int = 3500, feedback_type: str = "success"):
        """
        Display a transient banner message above the export button.

        The banner text is set to the provided message and the banner is shown.
        Any running feedback timer is stopped and a new single-shot timer is
        started if duration_ms is non-zero, after which the banner will be hidden.

        Args:
            message: Text to display in the feedback banner.
            duration_ms: Duration in milliseconds to keep the banner visible.
                         If 0, the banner will remain visible until manually hidden.

        Returns:
            None
        """
        if feedback_type == "cancel":
            self.feedback_banner.setStyleSheet(CANCEL_BANNER_STYLE)
        else:
            self.feedback_banner.setStyleSheet(SUCCESS_BANNER_STYLE)
        self.feedback_label.setText(message)
        self.feedback_banner.setVisible(True)
        self.feedback_timer.stop()
        if duration_ms:
            self.feedback_timer.start(duration_ms)

    def hide_feedback(self):
        """
        Hide the inline feedback banner and stop its timer.

        Stops the feedback timer (if running) and hides the banner so it is no
        longer visible to the user.

        Args:
            None

        Returns:
            None
        """
        self.feedback_timer.stop()
        self.feedback_banner.setVisible(False)

    def _on_assoc_changed(self, text):
        """
        Handle association strategy changes and update epsilon field state.
        
        Enables/disables the epsilon field based on whether resampling is active,
        and sets a default epsilon value when 'nearest' strategy is selected.

        Args:
            text: Selected association strategy string.

        Returns:
            None
        """
        enable = text != "no resampling"
        self.eps_edit.setEnabled(enable)
        self._refresh_master_combo(resampling_enabled=enable)
        if text == "nearest" and not self.eps_edit.text():
            self.eps_edit.setText("0.5")

    def _refresh_master_combo(self, resampling_enabled=False):
        """
        Populate the master topic dropdown with currently selected topics.
        """
        current = self.master_combo.currentText()
        self.master_combo.blockSignals(True)
        self.master_combo.clear()
        for topic in self.selected_topics:
            self.master_combo.addItem(topic)
        restore_idx = self.master_combo.findText(current)
        if restore_idx >= 0:
            self.master_combo.setCurrentIndex(restore_idx)
        elif self.master_combo.count() > 0:
            self.master_combo.setCurrentIndex(0)
        self.master_combo.setEnabled(resampling_enabled and self.master_combo.count() > 0)
        self.master_combo.blockSignals(False)

    def update_summary(self, selected_count, total_count, selected_topics=None):
        """
        Update the summary display with current topic selection counts.
        
        Updates the summary label and enables/disables the export button
        based on whether any topics are selected.

        Args:
            selected_count: Number of topics selected for export.
            total_count: Total number of topics in the bag.

        Returns:
            None
        """
        if selected_topics is not None:
            self.selected_topics = selected_topics
            self._refresh_master_combo(resampling_enabled=self.assoc_combo.currentText() != "no resampling")

        self.summary_label.setText(
            f"Selected Topics: {selected_count}\n"
            f"Total Topics: {total_count}"
        )
        self._has_selection = selected_count > 0
        if not self._is_export_running:
            self.btn_export.setEnabled(self._has_selection)
        if self._has_selection:
            self.btn_export.setToolTip("Start unbagging")
        else:
            self.btn_export.setToolTip("Select at least one topic to unbag.")

    def set_controls_enabled(self, enabled: bool):
        """
        Enable or disable global configuration controls while keeping action buttons available.

        Args: 
            enabled: If True, enable configuration controls; if False, disable them.

        Returns:
            None
        """
        self.gb_settings.setEnabled(enabled)
        self.gb_base.setEnabled(enabled)
        self.gb_summary.setEnabled(enabled)
        self.feedback_close.setEnabled(enabled)

    def set_export_running(self, running: bool):
        """
        Toggle action buttons between export and cancel state.

        Args:
            running: If True, export is running; if False, export is not running.

        Returns:
            None
        """
        self._is_export_running = running
        self.btn_export.setVisible(not running)
        self.btn_cancel.setVisible(running)
        if running:
            self.btn_cancel.setEnabled(False)
        else:
            self.btn_export.setEnabled(self._has_selection)

    def _choose_base_dir(self):
        """
        Open a directory selection dialog to choose the base directory.

        Args:
            None

        Returns:
            None
        """
        start_dir = str(self.base_dir) if self.base_dir else str(Path.cwd())
        new_dir = QtWidgets.QFileDialog.getExistingDirectory(self, "Select Base Directory", start_dir)
        if new_dir:
            self.set_base_dir(new_dir)
            self.base_dir_changed.emit(new_dir)

    def set_base_dir(self, path: str | Path):
        """
        Update the base directory path and refresh the label display.

        Args:
            path: New base directory path as a string or Path object.
        
        Returns:
            None
        """
        self.base_dir = Path(path)
        self.base_dir_full = str(self.base_dir)
        self.base_dir_label.setToolTip(self.base_dir_full)
        self._update_base_dir_label()

    def set_base_dir_enabled(self, enabled: bool):
        """
        Enable or disable the base directory selection controls.

        Args:
            enabled: If True, enable the base directory controls; if False, disable them.

        Returns:
            None
        """
        self.btn_base_dir.setEnabled(enabled)

    def get_config(self):
        """
        Retrieve current global configuration from UI widgets.
        
        Collects CPU usage and resampling settings into a configuration dictionary.

        Args:
            None

        Returns:
            dict: Configuration dictionary with keys 'cpu_percentage',
                 'continue_on_error', and optionally 'resample_config'
                 (if resampling is enabled).
        """
        cfg = {
            "cpu_percentage": float(self.cpu_spin.value()),
            "continue_on_error": self.continue_on_error_check.isChecked(),
        }
        assoc = self.assoc_combo.currentText()
        if assoc != "no resampling":
            eps_text = self.eps_edit.text().strip()
            if eps_text:
                try:
                    eps = float(eps_text)
                except (TypeError, ValueError):
                    eps = None
            else:
                eps = None
            
            master_topic = self.master_combo.currentText().strip()
            cfg["resample_config"] = {
                "association": assoc,
                "discard_eps": eps,
                "master_topic": master_topic if master_topic else None
            }
        return cfg
    
    def set_config(self, config):
        """
        Populate UI widgets from a global configuration dictionary.
        
        Restores CPU usage and resampling settings from a previously saved configuration.

        Args:
            config: Configuration dictionary with keys 'cpu_percentage',
                   'continue_on_error', and optionally 'resample_config'.

        Returns:
            None
        """
        if not isinstance(config, dict):
            return

        cpu_percentage = config.get("cpu_percentage")
        if cpu_percentage is not None:
            try:
                cpu_value = float(cpu_percentage)
            except (TypeError, ValueError):
                cpu_value = float(self.cpu_spin.value())
            cpu_value = max(0.0, min(100.0, cpu_value))
            self.cpu_spin.setValue(cpu_value)
            self.cpu_slider.blockSignals(True)
            self.cpu_slider.setValue(int(round(cpu_value)))
            self.cpu_slider.blockSignals(False)

        self.continue_on_error_check.setChecked(bool(config.get("continue_on_error", False)))
        
        rcfg = config.get("resample_config")
        if isinstance(rcfg, dict):
            assoc = rcfg.get("association", "no resampling")
            idx = self.assoc_combo.findText(assoc)
            if idx >= 0:
                self.assoc_combo.setCurrentIndex(idx)
            else:
                assoc = "no resampling"
                self.assoc_combo.setCurrentIndex(0)
            discard_eps = rcfg.get("discard_eps")
            if discard_eps is None:
                self.eps_edit.clear()
            else:
                self.eps_edit.setText(str(discard_eps))
            self._refresh_master_combo(resampling_enabled=assoc != "no resampling")
            if "master_topic" in rcfg and rcfg["master_topic"]:
                midx = self.master_combo.findText(rcfg["master_topic"])
                if midx >= 0:
                    self.master_combo.setCurrentIndex(midx)
        else:
            self.assoc_combo.setCurrentIndex(0)
            self.eps_edit.clear()

    def resizeEvent(self, event):
        """
        Handle widget resize and update elided base directory label.

        Args:
            event: Qt resize event delivered by the Qt framework.

        Returns:
            None
        """
        super().resizeEvent(event)
        self._update_base_dir_label()

    def _update_base_dir_label(self):
        """
        Elide the base directory path for display while preserving the full path in the tooltip.

        Ensures the label text is shortened with an ellipsis in the middle if it does
        not fit within the available width. The full path remains available via the
        label tooltip.

        Args:
            None

        Returns:
            None
        """
        if not hasattr(self, "base_dir_full"):
            self.base_dir_full = str(self.base_dir)
        fm = self.base_dir_label.fontMetrics()
        available = max(40, self.base_dir_label.width())
        elided = fm.elidedText(self.base_dir_full, QtCore.Qt.ElideMiddle, available)
        self.base_dir_label.setText(elided)
