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
Topic Settings Widget Module.

Provides the TopicSettingsWidget class for configuring per-topic export settings
in the ros2_unbag GUI. This widget appears in the middle column and adapts its
controls based on the selected topic's message type.
"""

from PySide6 import QtCore, QtWidgets, QtGui
from pathlib import Path

from ros2_unbag.ui.styles import (
    TS_HEADER_STYLE,
    TS_TOPIC_STYLE,
    EXPORT_BADGE_SELECTED_STYLE,
    EXPORT_BADGE_UNSELECTED_STYLE,
    HELP_TEXT_STYLE,
    EMPTY_HINT_STYLE,
    INPUT_ERROR_STYLE,
)
from ros2_unbag.core.processors import Processor
from ros2_unbag.core.routines import ExportRoutine, ExportMode
from .processor_chain import ProcessorChainWidget
from .routine_args import RoutineArgsWidget

__all__ = ["TopicSettingsWidget"]


class TopicSettingsWidget(QtWidgets.QWidget):
    """
    Widget for configuring export settings for a single ROS2 topic.
    
    This widget displays and manages all export configuration options for the currently
    selected topic, including:
    - Export format selection
    - Export mode (single file vs multi-file)
    - Output directory and subdirectory
    - File naming scheme with placeholder support
    - Master topic designation for resampling
    - Processor chain configuration
    
    The widget dynamically adapts its UI based on the topic's message type, showing
    only relevant formats and processors. When no topic is selected, displays a
    centered banner image that scales to fit the scroll area viewport.
    
    Signals:
        export_toggle_requested (str): Emitted when the export badge is clicked, passes topic name
        settings_changed (str, dict): Emitted when any setting changes, passes topic name and updated config dict
    """
    
    # Signals
    export_toggle_requested = QtCore.Signal(str)
    settings_changed = QtCore.Signal(str, dict)  # topic_name, new_config

    def __init__(self, default_folder, parent=None):
        """
        Initialize the TopicSettingsWidget with default output folder.

        Args:
            default_folder: Default output directory path for exports.
            parent: Optional parent widget.

        Returns:
            None
        """
        super().__init__(parent)
        self.default_folder = default_folder
        self.current_topic = None
        self.current_type = None
        self._scroll_area = None  # Will be set when added to scroll area
        self._bag_path = None     # Set by main_window after loading a bag
        self.init_ui()

    def set_bag_path(self, bag_path) -> None:
        """
        Provide the path to the currently loaded bag file.

        Called by the main window whenever a new bag is loaded so that the
        "Preview Camera" feature can open an independent reader to fetch the
        first frame for the current topic.

        Args:
            bag_path: ``pathlib.Path`` (or str) to the ``.db3`` / ``.mcap`` file.

        Returns:
            None
        """
        self._bag_path = bag_path

    def init_ui(self):
        """
        Build the topic settings UI with form layout for all configuration options.
        
        Creates a form with format selection, mode selection, path configuration,
        naming scheme, and processor chain widget.
        Initially hidden until a topic is selected.

        Args:
            None

        Returns:
            None
        """
        self.layout = QtWidgets.QVBoxLayout(self)
        self.layout.setAlignment(QtCore.Qt.AlignTop)
        self.setStyleSheet(INPUT_ERROR_STYLE)  # enables property-based error styling for child inputs

        # Header row with title/topic and export badge on the right
        header_row = QtWidgets.QHBoxLayout()
        header_row.setContentsMargins(0, 0, 0, 0)
        header_row.setSpacing(8)

        header_block = QtWidgets.QVBoxLayout()
        header_block.setContentsMargins(0, 0, 0, 0)
        header_block.setSpacing(2)

        self.header_label = QtWidgets.QLabel("Export Settings")
        self.header_label.setStyleSheet(TS_HEADER_STYLE)
        header_block.addWidget(self.header_label)

        self.topic_label = QtWidgets.QLabel("No Topic Selected")
        self.topic_label.setStyleSheet(TS_TOPIC_STYLE)
        self.topic_label.setVisible(False)
        header_block.addWidget(self.topic_label)

        header_row.addLayout(header_block)
        header_row.addStretch()

        self.export_state = QtWidgets.QPushButton("•")
        self.export_state.setFlat(True)
        self.export_state.setCursor(QtCore.Qt.PointingHandCursor)
        self.export_state.setStyleSheet(EXPORT_BADGE_UNSELECTED_STYLE)
        self.export_state.setFixedSize(40, 40)
        self.export_state.setVisible(False)
        self.export_state.clicked.connect(self._on_export_badge_clicked)
        header_row.addWidget(self.export_state)

        self.layout.addLayout(header_row)

        # Content Container (hidden when no topic selected)
        self.content_widget = QtWidgets.QWidget()
        self.form_layout = QtWidgets.QFormLayout(self.content_widget)
        
        # Format
        format_row = QtWidgets.QWidget()
        format_layout = QtWidgets.QHBoxLayout(format_row)
        format_layout.setContentsMargins(0, 0, 0, 0)
        format_layout.setSpacing(8)
        self.fmt_combo = QtWidgets.QComboBox()
        self.fmt_combo.currentTextChanged.connect(self._on_format_changed)
        self.fmt_combo.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)
        format_layout.addWidget(self.fmt_combo)

        # Mode (inline with format)
        mode_container = QtWidgets.QWidget()
        mode_layout = QtWidgets.QHBoxLayout(mode_container)
        mode_layout.setContentsMargins(0, 0, 0, 0)
        mode_layout.setSpacing(4)
        self.mode_label = QtWidgets.QLabel("Mode")
        self.mode_combo = QtWidgets.QComboBox()
        self.mode_combo.currentIndexChanged.connect(self._on_mode_changed)
        self.mode_combo.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)
        mode_layout.addWidget(self.mode_label)
        mode_layout.addWidget(self.mode_combo)
        format_layout.addWidget(mode_container)
        format_layout.setStretch(0, 1)
        format_layout.setStretch(1, 1)
        self.form_layout.addRow("Format", format_row)

        # Format Options (routine-specific parameters, shown only when the
        # selected routine declares extra keyword arguments)
        self.routine_args_container = QtWidgets.QWidget()
        self.routine_args_layout = QtWidgets.QVBoxLayout(self.routine_args_container)
        self.routine_args_layout.setContentsMargins(0, 0, 0, 0)
        self.routine_args_layout.setSpacing(0)
        self.routine_args_row_label = QtWidgets.QLabel("Format Options")
        self.form_layout.addRow(self.routine_args_row_label, self.routine_args_container)
        self.routine_args_row_label.setVisible(False)
        self.routine_args_container.setVisible(False)
        self.routine_args_widget = None

        # Output Directory
        self.path_edit = QtWidgets.QLineEdit()
        self.browse_btn = QtWidgets.QPushButton("Browse")
        self.browse_btn.clicked.connect(self._browse_path)
        path_layout = QtWidgets.QHBoxLayout()
        path_layout.addWidget(self.path_edit)
        path_layout.addWidget(self.browse_btn)
        self.form_layout.addRow("Output Directory", path_layout)

        # Subdirectory
        self.subdir_edit = QtWidgets.QLineEdit()
        self.form_layout.addRow("Subdirectory", self.subdir_edit)

        # Naming
        self.naming_edit = QtWidgets.QLineEdit()
        self.form_layout.addRow("Naming", self.naming_edit)

        # Processor Chain Placeholder
        self.processor_container = QtWidgets.QWidget()
        self.processor_layout = QtWidgets.QVBoxLayout(self.processor_container)
        self.processor_layout.setContentsMargins(0,0,0,0)
        self.form_layout.addRow("Processors", self.processor_container)

        self.layout.addWidget(self.content_widget)
        
        # Placeholder image when no topic is selected
        self.placeholder = QtWidgets.QLabel()
        self.placeholder.setAlignment(QtCore.Qt.AlignCenter)
        self.placeholder.setVisible(True)
        self.placeholder.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Expanding)
        self.placeholder.setScaledContents(False)  # We'll handle scaling manually
        img_path = Path(__file__).resolve().parent.parent / "assets" / "title.png"
        self._placeholder_pixmap = QtGui.QPixmap(str(img_path)) if img_path.exists() else None
        self.layout.addWidget(self.placeholder)
        self.layout.setAlignment(self.placeholder, QtCore.Qt.AlignCenter)
        self.placeholder_hint = QtWidgets.QLabel("Please select topic to configure export")
        self.placeholder_hint.setAlignment(QtCore.Qt.AlignCenter)
        self.placeholder_hint.setStyleSheet(EMPTY_HINT_STYLE)
        self.placeholder_hint.setVisible(True)
        self.layout.addWidget(self.placeholder_hint)
        self.layout.setAlignment(self.placeholder_hint, QtCore.Qt.AlignCenter)
        
        # Help Text (hidden until a topic is selected)
        self.layout.addStretch()
        self.help_text = QtWidgets.QLabel(
            "Placeholders:\n"
            "  %name (topic name)\n"
            "  %index (msg idx)\n"
            "  %timestamp (msg timestamp in nanoseconds)\n"
            "  %master_timestamp (master topic timestamp when resampling)\n"
            "  %Y-%m-%d_%H-%M-%S (timestamp)"
        )
        self.help_text.setStyleSheet(HELP_TEXT_STYLE)
        self.help_text.setVisible(False)
        self.layout.addWidget(self.help_text)

        # Connect change signals
        self.path_edit.textChanged.connect(self._apply_validation_styles)
        self.naming_edit.textChanged.connect(self._apply_validation_styles)
        self.path_edit.editingFinished.connect(self._emit_change)
        self.subdir_edit.editingFinished.connect(self._emit_change)
        self.naming_edit.editingFinished.connect(self._emit_change)
        
        self.content_widget.setVisible(False)
        self.placeholder.setVisible(True)

    def _get_active_mode(self):
        """
        Return the currently selected or forced export mode.
        """
        mode = self.mode_combo.currentData()
        if mode is None:
            mode = self.mode_combo.property("forced_mode")
        return mode

    def _sync_naming_with_mode(self, initial=False):
        """
        Adjust the naming field to match the active mode when it is still at a default value.

        Args:
            initial: If True, only override when the naming text is empty or a default value.
        """
        mode = self._get_active_mode()
        current = self.naming_edit.text().strip()

        if mode == ExportMode.SINGLE_FILE:
            if not initial or current in ("", "%name_%index"):
                self.naming_edit.setText("%name")
        else:
            if not initial or current in ("", "%name"):
                self.naming_edit.setText("%name_%index")

    def set_topic(self, topic, topic_type, config):
        """
        Load and display settings for a specific topic.
        
        Updates the UI to show configuration options for the given topic,
        populating all fields from the provided config dictionary. Dynamically
        adjusts available formats and processors based on the topic's message type.

        Args:
            topic: Topic name string.
            topic_type: ROS2 message type string.
            config: Configuration dictionary with keys 'format', 'path', 'subfolder',
                   'naming', 'processors', 'resample_config'.

        Returns:
            None
        """
        self.current_topic = topic
        self.current_type = topic_type
        
        self.topic_label.setText(topic)
        self.topic_label.setVisible(True)
        self.content_widget.setVisible(True)
        self.help_text.setVisible(True)
        self.export_state.setVisible(True)
        self.placeholder.setVisible(False)
        self.placeholder_hint.setVisible(False)
        # Export state will be set by caller based on selection

        # Block signals to prevent auto-saving during load
        self.blockSignals(True)

        # 1. Setup Formats
        self.fmt_combo.blockSignals(True)
        self.fmt_combo.clear()
        formats = ExportRoutine.get_formats(topic_type)
        self.fmt_combo.addItems(formats)
        
        # Select current format
        current_fmt = config.get("format", "")
        # If format has @mode suffix, strip it for selection but keep mode in mind
        resolution = ExportRoutine.resolve(topic_type, current_fmt)
        if resolution:
            _, canonical_fmt, mode = resolution
            idx = self.fmt_combo.findText(canonical_fmt)
            if idx >= 0:
                self.fmt_combo.setCurrentIndex(idx)
            self.mode_combo.setProperty("pending_mode", mode)
        else:
            if self.fmt_combo.count() > 0:
                self.fmt_combo.setCurrentIndex(0)
        
        self.fmt_combo.blockSignals(False)

        # 2. Update Mode options based on format
        self._refresh_mode_controls(self.fmt_combo.currentText())

        # 2b. Rebuild routine args widget for the current format
        self._rebuild_routine_args(self.fmt_combo.currentText())
        if self.routine_args_widget and config.get("routine_args"):
            self.routine_args_widget.set_args(config["routine_args"])

        # 3. Set other fields
        self.path_edit.setText(config.get("path", str(self.default_folder)))
        subdir_value = config.get("subfolder", "%name")
        if subdir_value is None:
            subdir_value = ""
        subdir = subdir_value.strip("/")
        self.subdir_edit.setText(subdir)
        self.naming_edit.setText(config.get("naming", "%name"))
        self._sync_naming_with_mode(initial=True)
        
        # 4. Processor Chain
        # Clear old
        while self.processor_layout.count():
            item = self.processor_layout.takeAt(0)
            if item.widget(): item.widget().deleteLater()
        
        available_processors = Processor.get_formats(topic_type)
        if available_processors:
            self.chain_widget = ProcessorChainWidget(topic_type, available_processors)
            # Load chain config
            chain_cfg = config.get("processors", [])
            # Normalize chain config if needed (same as in ExportOptions)
            normalized_chain = []
            for entry in chain_cfg:
                if isinstance(entry, str):
                    normalized_chain.append({"name": entry, "args": {}})
                elif isinstance(entry, dict):
                    normalized_chain.append(entry)
            
            self.chain_widget.set_chain(normalized_chain)
            # Connect signal if ProcessorChainWidget has one, otherwise we might need to poll it
            # Assuming ProcessorChainWidget doesn't emit change signal, we might need to add one or check on save
            self.processor_layout.addWidget(self.chain_widget)
        else:
            self.chain_widget = None
            lbl = QtWidgets.QLabel("No processors available")
            self.processor_layout.addWidget(lbl)

        self._apply_validation_styles()
        self.blockSignals(False)

    def get_config(self):
        """
        Retrieve current configuration from UI widgets.
        
        Collects all settings from the form and returns them as a configuration
        dictionary. Handles format string construction with mode suffix if needed.

        Args:
            None

        Returns:
            dict: Configuration dictionary with keys 'format', 'path', 'subfolder',
                 'naming', 'resample_config', and optionally 'processors'.
                 Returns empty dict if no topic is currently selected.
        """
        if not self.current_topic: return {}
        
        fmt = self.fmt_combo.currentText()
        mode = self.mode_combo.currentData()
        if mode is None: mode = self.mode_combo.property("forced_mode")
        
        # Construct format string (e.g. "csv@single_file")
        # Logic from ExportOptions
        available_modes = self.mode_combo.property("available_modes") or tuple()
        if mode == ExportMode.SINGLE_FILE and len(available_modes) > 1:
            fmt = f"{fmt}@single_file"

        cfg = {
            "format": fmt,
            "path": self.path_edit.text().strip(),
            "subfolder": self.subdir_edit.text().strip("/"),
            "naming": self.naming_edit.text().strip()
        }
        
        if self.chain_widget:
            cfg["processors"] = self.chain_widget.get_chain()

        cfg["routine_args"] = self.routine_args_widget.get_args() if self.routine_args_widget else {}

        return cfg

    def set_export_state(self, checked: bool):
        """
        Update the export state badge to reflect whether the topic is selected for export.
        """
        if checked:
            self.export_state.setText("✓")
            self.export_state.setStyleSheet(EXPORT_BADGE_SELECTED_STYLE)
        else:
            self.export_state.setText("•")
            self.export_state.setStyleSheet(EXPORT_BADGE_UNSELECTED_STYLE)

    def _on_export_badge_clicked(self):
        """
        Toggle export state for the current topic via badge click.
        """
        if not self.current_topic:
            return
        self.export_toggle_requested.emit(self.current_topic)

    def _browse_path(self):
        """
        Open directory selection dialog and update output path field.
        
        Displays a file dialog for the user to select an output directory,
        then updates the path field and emits a settings change signal.

        Args:
            None

        Returns:
            None
        """
        directory = QtWidgets.QFileDialog.getExistingDirectory(self, "Select Directory", self.path_edit.text())
        if directory:
            self.path_edit.setText(directory)
            self._emit_change()

    def _on_format_changed(self, text):
        """
        Handle format selection changes.
        
        Updates available export modes based on the selected format and
        emits a settings change signal.

        Args:
            text: Selected format string.

        Returns:
            None
        """
        self._refresh_mode_controls(text)
        self._rebuild_routine_args(text)
        self._emit_change()

    def _rebuild_routine_args(self, fmt: str) -> None:
        """
        Rebuild the RoutineArgsWidget for the currently selected format.

        Clears any existing widget, queries ExportRoutine.get_args() for the
        new format and, if extra arguments are declared, creates a new
        RoutineArgsWidget and adds it to the form.

        Args:
            fmt (str): The newly selected export format string.

        Returns:
            None
        """
        # Remove the old widget
        while self.routine_args_layout.count():
            item = self.routine_args_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()
        self.routine_args_widget = None

        if not self.current_type:
            self.routine_args_row_label.setVisible(False)
            self.routine_args_container.setVisible(False)
            return

        extra_args = ExportRoutine.get_args(self.current_type, fmt)
        if extra_args:
            self.routine_args_widget = RoutineArgsWidget(self.current_type, fmt)
            self.routine_args_widget.args_changed.connect(self._emit_change)
            self.routine_args_widget.preview_camera_requested.connect(self._open_camera_preview)
            self.routine_args_layout.addWidget(self.routine_args_widget)
            self.routine_args_row_label.setVisible(True)
            self.routine_args_container.setVisible(True)
        else:
            self.routine_args_row_label.setVisible(False)
            self.routine_args_container.setVisible(False)

    def _on_mode_changed(self, idx):
        """
        Handle export mode changes and update naming scheme accordingly.
        
        When switching between single-file and multi-file modes, automatically
        adjusts the naming scheme to include or exclude the %index placeholder.

        Args:
            idx: Selected mode combo box index.

        Returns:
            None
        """
        # Update default naming if changed
        mode = self.mode_combo.currentData()
        if mode is None:
            mode = self.mode_combo.property("forced_mode")
        
        if mode == ExportMode.SINGLE_FILE:
            if "%index" in self.naming_edit.text():
                self.naming_edit.setText("%name")
        else:
            if self.naming_edit.text() == "%name":
                self.naming_edit.setText("%name_%index")
        
        self._emit_change()

    def _refresh_mode_controls(self, fmt):
        """
        Update mode selection controls based on available modes for the selected format.
        
        Queries the export routine system for available modes (single-file vs multi-file)
        for the current format and topic type. Shows/hides the mode selector accordingly.
        If only one mode is available, it's set as a forced mode and the selector is hidden.

        Args:
            fmt: Export format string.

        Returns:
            None
        """
        if not self.current_type:
            return
        
        modes = list(ExportRoutine.get_modes_for_format(self.current_type, fmt))
        if not modes:
            modes = [ExportMode.MULTI_FILE]
        
        self.mode_combo.blockSignals(True)
        self.mode_combo.clear()
        
        ordered_modes = sorted(modes, key=lambda m: 0 if m == ExportMode.MULTI_FILE else 1)
        
        if len(ordered_modes) > 1:
            for m in ordered_modes:
                label = "Multi file" if m == ExportMode.MULTI_FILE else "Single file"
                self.mode_combo.addItem(label, m)
            
            # Restore pending mode if set
            pending = self.mode_combo.property("pending_mode")
            if pending in ordered_modes:
                idx = self.mode_combo.findData(pending)
                self.mode_combo.setCurrentIndex(idx)
            
            self.mode_combo.setVisible(True)
            self.mode_label.setVisible(True)
        else:
            self.mode_combo.setProperty("forced_mode", ordered_modes[0])
            self.mode_combo.setVisible(False)
            self.mode_label.setVisible(False)
            
        self.mode_combo.setProperty("available_modes", tuple(ordered_modes))
        self.mode_combo.setProperty("pending_mode", None)
        self.mode_combo.blockSignals(False)
        self._sync_naming_with_mode(initial=True)

    def showEvent(self, event):
        """
        Handle widget show events and set up scroll area connection.
        """
        super().showEvent(event)
        # Find and monitor the scroll area parent for resize events
        if not self._scroll_area:
            parent = self.parent()
            if isinstance(parent, QtWidgets.QScrollArea):
                self._scroll_area = parent
                # Install event filter to catch scroll area resize events
                self._scroll_area.installEventFilter(self)
        # Update placeholder on first show
        if self._placeholder_pixmap:
            QtCore.QTimer.singleShot(0, self._update_placeholder_pixmap)
    
    def eventFilter(self, obj, event):
        """
        Filter events from the parent scroll area to detect resizes.
        """
        if obj == self._scroll_area and event.type() == QtCore.QEvent.Resize:
            # Delay update slightly to ensure layout is complete
            if self._placeholder_pixmap and self.placeholder.isVisible():
                QtCore.QTimer.singleShot(10, self._update_placeholder_pixmap)
        return super().eventFilter(obj, event)
    
    def resizeEvent(self, event):
        """
        Handle widget resize events.
        """
        super().resizeEvent(event)
        # Update placeholder when widget itself resizes
        if self._placeholder_pixmap and self.placeholder.isVisible():
            QtCore.QTimer.singleShot(10, self._update_placeholder_pixmap)

    def _update_placeholder_pixmap(self):
        """
        Scale placeholder image to fit the scroll area viewport width precisely.
        
        Uses the scroll area's viewport width to calculate the exact available
        width for the banner image, accounting for layout margins.
        """
        if not self._placeholder_pixmap or not self.placeholder.isVisible():
            return
        
        # Calculate available width from scroll area viewport
        available_width = 400  # Fallback default
        
        if self._scroll_area:
            # Get viewport width from scroll area
            viewport_width = self._scroll_area.viewport().width()
            # Account for scroll area's internal spacing and margins
            margins = self.layout.contentsMargins()
            # Subtract scroll bar width if visible
            scrollbar_width = 0
            if self._scroll_area.verticalScrollBar().isVisible():
                scrollbar_width = self._scroll_area.verticalScrollBar().width()
            available_width = viewport_width - (margins.left() + margins.right()) - scrollbar_width - 20  # Extra padding
        else:
            # Fallback: try to use widget width
            margins = self.layout.contentsMargins()
            available_width = self.width() - (margins.left() + margins.right()) - 20
        
        # Ensure minimum width
        available_width = max(200, available_width)
        
        # Scale pixmap to fit width while maintaining aspect ratio
        scaled = self._placeholder_pixmap.scaledToWidth(
            int(available_width), 
            QtCore.Qt.SmoothTransformation
        )
        self.placeholder.setPixmap(scaled)

    def _open_camera_preview(self) -> None:
        """
        Read the first PointCloud2 message for the current topic and open
        ``CameraPreviewDialog`` so the user can interactively set camera
        parameters.

        A ``QProgressDialog`` is shown while the message is being read from
        disk.  Any error is surfaced as a ``QMessageBox``.

        Returns:
            None
        """
        if not self._bag_path:
            QtWidgets.QMessageBox.warning(
                self,
                "No Bag Loaded",
                "Please load a bag file before opening the camera preview.",
            )
            return

        if not self.current_topic:
            return

        # Show a transient progress dialog while reading the bag
        progress = QtWidgets.QProgressDialog(
            f"Reading first frame from '{self.current_topic}'…",
            None, 0, 0, self,
        )
        progress.setWindowModality(QtCore.Qt.WindowModal)
        progress.setMinimumDuration(0)
        progress.setValue(0)
        progress.show()
        QtWidgets.QApplication.processEvents()

        msg = None
        try:
            from ros2_unbag.core.bag_reader import BagReader
            reader = BagReader(str(self._bag_path))
            for _, m, _ in reader.read_messages([self.current_topic]):
                msg = m
                break
        except Exception as exc:
            progress.close()
            QtWidgets.QMessageBox.critical(
                self,
                "Preview Failed",
                f"Could not read the first message:\n{exc}",
            )
            return
        finally:
            progress.close()

        if msg is None:
            QtWidgets.QMessageBox.information(
                self,
                "No Messages",
                f"No messages found for topic '{self.current_topic}'.",
            )
            return

        args = self.routine_args_widget.get_args() if self.routine_args_widget else {}

        from ros2_unbag.ui.widgets.camera_preview_dialog import CameraPreviewDialog
        dlg = CameraPreviewDialog(msg, args, parent=self)
        dlg.camera_params_applied.connect(self._on_camera_params_applied)
        dlg.exec()

    def _on_camera_params_applied(self, params: dict) -> None:
        """
        Apply camera parameters returned by ``CameraPreviewDialog`` to the
        routine args form and emit a settings-changed signal.

        Args:
            params (dict): Dict with keys ``view_azimuth``, ``view_elevation``,
                ``view_roll``, ``zoom``.

        Returns:
            None
        """
        if self.routine_args_widget:
            self.routine_args_widget.set_args(params)
        self._emit_change()

    def _emit_change(self):
        """
        Emit settings_changed signal with current topic and configuration.
        
        Called whenever any setting is modified to notify the main window
        that the configuration has changed.

        Args:
            None

        Returns:
            None
        """
        self._apply_validation_styles()
        if self.current_topic:
            self.settings_changed.emit(self.current_topic, self.get_config())

    def _apply_validation_styles(self):
        """
        Apply error styling to required fields and report validity.

        Args:
            None

        Returns:
            bool: True if all required fields are non-empty, False otherwise.
        """
        path_empty = not self.path_edit.text().strip()
        naming_empty = not self.naming_edit.text().strip()
        self._set_error_state(self.path_edit, path_empty)
        self._set_error_state(self.naming_edit, naming_empty)
        return not (path_empty or naming_empty)

    def _set_error_state(self, widget, is_error):
        """
        Apply or clear the error style for a widget while preserving its base style.

        Args:
            widget: The input widget to style.
            is_error: Whether to show the error styling.

        Returns:
            None
        """
        widget.setProperty("error", is_error)
        widget.style().unpolish(widget)
        widget.style().polish(widget)
        widget.update()

    def validate_inputs(self):
        """
        Validate required fields and update their styling.

        Args:
            None

        Returns:
            bool: True if required fields are valid, False otherwise.
        """
        return self._apply_validation_styles()
