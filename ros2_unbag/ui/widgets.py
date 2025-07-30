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

import inspect
import os

from PySide6 import QtCore, QtWidgets

from ros2_unbag.core.processors import Processor
from ros2_unbag.core.routines import ExportRoutine, ExportMode


class TopicSelector(QtWidgets.QWidget):
    # Widget to display and select available topics from the bag

    def __init__(self, bag_reader):
        """
        Initialize TopicSelector with a BagReader, retrieve topics and message counts, and build the UI.

        Args:
            bag_reader: BagReader instance for the ROS2 bag.

        Returns:
            None
        """
        super().__init__()
        self.bag_reader = bag_reader
        self.topics = self.bag_reader.get_topics()
        self.message_counts = self.bag_reader.get_message_count()
        self.checkboxes = {}
        self.select_all_state = True

        self.init_ui()

    def init_ui(self):
        """
        Build the topic selection UI: group topics by message type with checkboxes and message count labels.

        Args:
            None

        Returns:
            None
        """
        layout = QtWidgets.QVBoxLayout()

        # Create checkboxes grouped by message type
        for msg_type, topic_list in sorted(self.topics.items()):
            group_box = QtWidgets.QGroupBox(msg_type)
            group_layout = QtWidgets.QVBoxLayout()

            for topic in sorted(topic_list):
                container = QtWidgets.QWidget()
                h_layout = QtWidgets.QHBoxLayout()
                h_layout.setContentsMargins(0, 0, 0, 0)

                checkbox = QtWidgets.QCheckBox()
                checkbox.setCursor(QtCore.Qt.PointingHandCursor)
                label = QtWidgets.QLabel(topic)
                label.setCursor(QtCore.Qt.PointingHandCursor)
                label.mousePressEvent = self._make_label_toggle_cb(checkbox)
                count_label = QtWidgets.QLabel(str(self.message_counts.get(topic, 0))+ " Messages")
                count_label.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)

                h_layout.addWidget(checkbox)
                h_layout.addWidget(label)
                h_layout.addStretch()
                h_layout.addWidget(count_label)

                container.setLayout(h_layout)
                group_layout.addWidget(container)
                self.checkboxes[topic] = checkbox

            group_box.setLayout(group_layout)
            layout.addWidget(group_box)

        # Select All / Deselect All button
        self.select_all_button = QtWidgets.QPushButton("Select All")
        self.select_all_button.clicked.connect(self.toggle_select_all)
        layout.addWidget(self.select_all_button)

        self.setLayout(layout)

    def toggle_select_all(self):
        """
        Toggles the selection state of all checkboxes in the widget.

        Args:
            None

        Returns:
            None
        """
        for cb in self.checkboxes.values():
            cb.setChecked(self.select_all_state)
        self.select_all_state = not self.select_all_state
        self.select_all_button.setText("Deselect All" if not self.select_all_state else "Select All")

    def _make_label_toggle_cb(self, checkbox):
        """
        Creates a callback function that toggles the state of the given checkbox.

        Args:
            checkbox: QCheckBox instance to toggle.

        Returns:
            function: Callback function for mousePressEvent.
        """
        def toggle(_):
            checkbox.toggle()
        return toggle
    
    def get_selected_topics(self):
        """
        Return a list of topics whose checkboxes are currently checked.

        Args:
            None

        Returns:
            list: List of selected topic names.
        """
        return [
            topic for topic, cb in self.checkboxes.items() if cb.isChecked()
        ]


class ExportOptions(QtWidgets.QWidget):

    def __init__(self, selected_topics, all_topics, default_folder):
        """
        Initialize ExportOptions with selected topics, all topics mapping, and default output folder; prepare UI state.

        Args:
            selected_topics: List of selected topic names.
            all_topics: Dict mapping message types to topic lists.
            default_folder: Default output folder path.

        Returns:
            None
        """
        super().__init__()
        self.config_widgets = {}
        self.master_checkboxes = {}
        self.all_path_edits = []
        self.master_group = QtWidgets.QButtonGroup(self)
        self.master_group.setExclusive(True)  # ensure single master
        self.selected_topics = selected_topics
        self.all_topics = all_topics
        self.processor_args = {}
        self.default_folder = default_folder

        self.init_ui()

    def init_ui(self):
        """
        Build the export options UI: global settings (CPU, resampling) and per-topic controls for format, paths, naming, and processors.

        Args:
            None

        Returns:
            None
        """
        layout = QtWidgets.QVBoxLayout()

        # ────────── Global Sync Settings (TOP) ──────────
        global_group = QtWidgets.QGroupBox("Global Settings")
        global_layout = QtWidgets.QFormLayout()

        self.cpu_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.cpu_slider.setRange(0, 100)
        self.cpu_slider.setValue(80)
        self.cpu_slider.setTickInterval(5)
        self.cpu_slider.setTickPosition(QtWidgets.QSlider.TicksBelow)
        self.cpu_spinbox = QtWidgets.QSpinBox()
        self.cpu_spinbox.setRange(0, 100)
        self.cpu_spinbox.setValue(80)

        # Sync slider and spinbox
        self.cpu_slider.valueChanged.connect(self.cpu_spinbox.setValue)
        self.cpu_spinbox.valueChanged.connect(self.cpu_slider.setValue)
        cpu_layout = QtWidgets.QHBoxLayout()
        cpu_layout.addWidget(self.cpu_slider)
        cpu_layout.addWidget(self.cpu_spinbox)

        self.assoc_combo = QtWidgets.QComboBox()
        self.assoc_combo.addItems(["no resampling", "last", "nearest"])
        self.assoc_combo.currentTextChanged.connect(self._sync_mode_changed)

        self.eps_edit = QtWidgets.QLineEdit()
        self.eps_edit.setPlaceholderText("e.g., 0.5 (required for nearest)")
        self.eps_hint = QtWidgets.QLabel("Required for 'nearest' strategy.")
        self.eps_hint.setStyleSheet("color: gray; font-style: italic;")

        global_layout.addRow("CPU usage", cpu_layout)
        global_layout.addRow("Association Strategy", self.assoc_combo)
        global_layout.addRow("Discard Eps (s)", self.eps_edit)
        global_layout.addRow("", self.eps_hint)
        global_group.setLayout(global_layout)
        layout.addWidget(global_group)

        # ────────── Per-topic export options ──────────
        for idx, topic in enumerate(self.selected_topics):
            topic_type = next(
                (k for k, v in self.all_topics.items() if topic in v), None)

            group_box = QtWidgets.QGroupBox(topic)
            form_layout = QtWidgets.QFormLayout()

            # Format selection
            fmt_combo = QtWidgets.QComboBox()
            fmt_combo.addItems(ExportRoutine.get_formats(topic_type))

            # Output directory
            abs_path_edit = QtWidgets.QLineEdit()
            abs_path_edit.setText(str(self.default_folder))
            browse_button = QtWidgets.QPushButton("Browse")
            if idx == 0:
                browse_button.clicked.connect(lambda _, e=abs_path_edit: self.
                                              select_directory_and_apply(e))
            else:
                browse_button.clicked.connect(
                    lambda _, e=abs_path_edit: self.select_directory(e))

            path_layout = QtWidgets.QHBoxLayout()
            path_layout.addWidget(abs_path_edit)
            path_layout.addWidget(browse_button)
            self.all_path_edits.append(abs_path_edit)

            # Subdirectory and naming scheme
            rel_path_edit = QtWidgets.QLineEdit("%name")
            name_scheme_edit = QtWidgets.QLineEdit()
            
            # Dynamic update based on format selection
            def update_naming_and_checkbox(fmt, name_edit=name_scheme_edit, t_type=topic_type):
                mode = ExportRoutine.get_mode(t_type, fmt)
                if mode == ExportMode.SINGLE_FILE:
                    name_edit.setText("%name")
                else:
                    name_edit.setText("%name_%index")

            # Connect format selection to update naming and checkbox
            fmt_combo.currentTextChanged.connect(update_naming_and_checkbox)
            update_naming_and_checkbox(fmt_combo.currentText())

            # Master checkbox (mutually exclusive)
            is_master_check = QtWidgets.QCheckBox(
                "Set as Master for Resampling")
            self.master_group.addButton(is_master_check)
            self.master_checkboxes[topic] = is_master_check

            # Processing selection
            if Processor.get_formats(topic_type):
                proc_combo = QtWidgets.QComboBox()
                proc_combo.addItems(
                    ["No Processor", *Processor.get_formats(topic_type)])
                proc_combo.currentTextChanged.connect(
                    lambda selected_processor, fl=form_layout, t=topic, tt
                    =topic_type: self._processor_changed(
                        selected_processor, t, tt, fl))
            else:
                proc_combo = None

            form_layout.addRow("Format", fmt_combo)
            form_layout.addRow("Output Directory", path_layout)
            form_layout.addRow("Subdirectory", rel_path_edit)
            form_layout.addRow("Naming", name_scheme_edit)
            form_layout.addRow("Master Topic", is_master_check)
            if proc_combo:
                form_layout.addRow("Processor", proc_combo)

            group_box.setLayout(form_layout)
            layout.addWidget(group_box)

            self.config_widgets[topic] = (fmt_combo, abs_path_edit,
                                          rel_path_edit, name_scheme_edit,
                                          is_master_check, proc_combo)

        # ────────── Help ──────────
        note = QtWidgets.QLabel(
            "Naming and paths supports placeholders:\n"
            "  %name   → topic name without slashes\n"
            "  %index  → message index (starting from 0)\n"
            "  %Y, %m, %d, %H, %M, %S  → timestamp components from message header or receive-time if there is no header\n"
            "    (e.g. %Y-%m-%d_%H-%M-%S → 2025-04-14_12-30-00)"
        )
        note.setWordWrap(True)
        note.setStyleSheet("color: gray; font-style: italic;")
        layout.addWidget(note)

        self.setLayout(layout)
        self._sync_mode_changed(
            self.assoc_combo.currentText())  # initialize state

    def _sync_mode_changed(self, mode):
        """
        Enable or disable discard epsilon and master-topic checkboxes based on the selected resampling mode; set default eps for 'nearest'.

        Args:
            mode: Selected association strategy (str).

        Returns:
            None
        """
        enable = mode != "no resampling"
        self.eps_edit.setEnabled(enable)
        self.eps_hint.setVisible(mode == "nearest")
        for cb in self.master_checkboxes.values():
            cb.setEnabled(enable)

        # Enable first checkbox by default if resampling is enabled
        if enable and not any(cb.isChecked() for cb in self.master_checkboxes.values()):
            first_topic = next(iter(self.master_checkboxes.values()))
            first_topic.setChecked(True)

        # Default epsilon if nearest is selected
        if mode == "nearest" and not self.eps_edit.text().strip():
            self.eps_edit.setText("0.5")

    def _processor_changed(self, selected_processor, topic, topic_type,
                           form_layout):
        """
        Update the form layout when the processor selection changes: clear old argument fields and add QLineEdits for new processor args.

        Args:
            selected_processor: Name of the selected processor (str).
            topic: Topic name (str).
            topic_type: Message type (str).
            form_layout: QFormLayout instance for the topic.

        Returns:
            None
        """
        # Safely clear existing argument rows
        for i in reversed(range(form_layout.rowCount())):
            item = form_layout.itemAt(i, QtWidgets.QFormLayout.LabelRole)
            if item:
                label = item.widget()
                if label and hasattr(
                        label, "is_argument_row") and label.is_argument_row:
                    # Remove the associated field widget
                    field_item = form_layout.itemAt(
                        i, QtWidgets.QFormLayout.FieldRole)
                    if field_item:
                        field_widget = field_item.widget()
                        if field_widget:
                            field_widget.setParent(None)
                            field_widget.deleteLater()
                    # Remove the label widget
                    label.setParent(None)
                    label.deleteLater()
                    form_layout.removeRow(i)

        if selected_processor != "No Processor":
            args = Processor.get_args(topic_type, selected_processor)
            self.processor_args[topic] = {}  # Store argument names and QLineEdit widgets

            for arg_name, (param, doc) in args.items():
                # Create a QLabel and QLineEdit for each argument
                label = QtWidgets.QLabel()
                label.setText(f"{arg_name} (optional)" if param.default != inspect.Parameter.empty else arg_name)
                label.is_argument_row = True  # Tag this label as an argument row

                # Build placeholder with doc, default, and type
                parts = []
                if doc:
                    parts.append(doc)
                if param.default != inspect.Parameter.empty:
                    parts.append(f"default: {param.default}")
                if param.annotation != inspect.Parameter.empty:
                    parts.append(f"Type: {param.annotation.__name__}")
                placeholder_text = " — ".join(parts)

                arg_edit = QtWidgets.QLineEdit()
                arg_edit.setPlaceholderText(placeholder_text)

                # Add to form layout
                form_layout.addRow(label, arg_edit)

                # Store input
                self.processor_args[topic][arg_name] = arg_edit
        else:
            # If no processor is selected, clear the stored arguments for this topic
            self.processor_args[topic] = {}

    def get_export_config(self):
        """
        Collect and return the export configuration dict for each topic and the global configuration from UI widget values.

        Args:
            None

        Returns:
            tuple: (topics_config: dict, global_config: dict)
        """
        topics_config = {}
        global_config = {}

        global_config["cpu_percentage"] = float(self.cpu_slider.value())
        assoc_mode = self.assoc_combo.currentText()

        if assoc_mode != "no resampling":
            # validate sync config
            try:
                eps = float(self.eps_edit.text().strip())
            except ValueError:
                eps = None

            if assoc_mode == "nearest" and eps is None:
                raise ValueError(
                    "Discard Eps is required for 'nearest' association strategy.")

            master_topic = None
            for topic, cb in self.master_checkboxes.items():
                if cb.isChecked():
                    master_topic = topic
                    break

            if not master_topic:
                raise ValueError(
                    "One topic must be marked as Master when synchronization is enabled."
                )
            
            global_config["resample_config"] = {
                "master_topic": master_topic,
                "association": assoc_mode,
                "discard_eps": eps
            }

        for topic, widgets in self.config_widgets.items():
            if assoc_mode == "no resampling":
                fmt, abs_path, rel_path, name, _, processor = widgets
            else:
                fmt, abs_path, rel_path, name, is_master, processor = widgets

            base = abs_path.text().strip()
            sub = rel_path.text().strip().lstrip("/")

            topic_cfg = {
                "format": fmt.currentText(),
                "path": base,
                "subfolder": sub,
                "naming": name.text().strip()
            }

            if processor and processor.currentText() != "No Processor":
                proc_name = processor.currentText()
                topic_cfg["processor"] = proc_name

                processor_config = {}
                processor_args = self.processor_args.get(topic, {})
                for arg_name, arg_edit in processor_args.items():
                    arg_value = arg_edit.text().strip()
                    if arg_value:
                        processor_config[arg_name] = arg_value
                topic_cfg["processor_args"] = processor_config

            topics_config[topic] = topic_cfg

        return topics_config, global_config

    def set_export_config(self, config, global_config=None):
        """
        Populate UI widgets from a given export configuration and optional global settings, restoring formats, paths, naming, and processors.

        Args:
            config: Dict of per-topic export configuration.
            global_config: Optional dict of global settings.

        Returns:
            None
        """
        if global_config is not None and "cpu_percentage" in global_config:
            self.cpu_slider.setValue(global_config["cpu_percentage"])

        for topic, topic_cfg in config.items():
            widgets = self.config_widgets.get(topic)
            if not widgets:
                continue

            # Check if the topic exists in the bag
            all_topics_list = [
                topic for topics in self.all_topics.values() for topic in topics
            ]
            if topic not in all_topics_list:
                raise ValueError(
                    f"Topic '{topic}' not found in the bag. Cannot set export config properly."
                )

            fmt_combo, abs_path_edit, rel_path_edit, name_scheme_edit, is_master_check, proc_combo = widgets
            # Set format
            fmt = topic_cfg.get("format", "")
            idx = fmt_combo.findText(fmt)
            if idx >= 0:
                fmt_combo.setCurrentIndex(idx)
            # Set output path and subdirectory
            path = topic_cfg.get("path", "")
            subdir = topic_cfg.get("subfolder", "").strip("/")
            abs_path_edit.setText(path)
            rel_path_edit.setText(subdir)
            # Set naming scheme
            name_scheme_edit.setText(topic_cfg.get("naming", ""))
            # Set master topic checkbox
            rcfg = topic_cfg.get("resample_config")
            if rcfg and rcfg.get("is_master"):
                is_master_check.setChecked(True)

            # Set processor and arguments
            if proc_combo:

                proc_name = topic_cfg.get("processor", "No Processor")
                idx = proc_combo.findText(proc_name)
                if idx >= 0:
                    proc_combo.setCurrentIndex(idx)

                # Restore processor arguments
                processor_config = topic_cfg.get("processor_args", {})
                topic_type = next(
                    (k for k, v in self.all_topics.items() if topic in v), None)
                if processor_config and topic_type:
                    # Dynamically recreate argument fields
                    self._processor_changed(proc_name, topic, topic_type,
                                            proc_combo.parent().layout())
                    for arg_name, arg_value in processor_config.items():
                        arg_edit = self.processor_args[topic].get(
                            arg_name, None)
                        if arg_edit:
                            arg_edit.setText(str(arg_value))

        # Set global synchronization settings if present
        for topic, topic_cfg in config.items():
            rcfg = topic_cfg.get("resample_config")
            if rcfg:
                assoc = rcfg.get("association", "no resampling")
                idx = self.assoc_combo.findText(assoc)
                if idx >= 0:
                    self.assoc_combo.setCurrentIndex(idx)
                if "discard_eps" in rcfg:
                    self.eps_edit.setText(str(rcfg["discard_eps"]))
                break

    def select_directory_and_apply(self, edit):
        """
        Prompt the user to select a directory and apply it to all output-directory fields.

        Args:
            edit: QLineEdit widget to update.

        Returns:
            None
        """
        directory = QtWidgets.QFileDialog.getExistingDirectory(
            self, "Select Directory")
        if directory:
            for path_edit in self.all_path_edits:
                path_edit.setText(directory)

    def select_directory(self, edit):
        """
        Prompt the user to select a directory and set it for the given output-directory field.

        Args:
            edit: QLineEdit widget to update.

        Returns:
            None
        """
        directory = QtWidgets.QFileDialog.getExistingDirectory(
            self, "Select Directory")
        if directory:
            edit.setText(directory)