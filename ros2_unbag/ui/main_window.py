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
Main Window Module.

Provides the UnbagApp main application window for the ros2_unbag GUI.
Orchestrates the entire export workflow through a 3-column layout:
- Left: Bag file loading and topic selection
- Middle: Per-topic export configuration
- Right: Global settings and export action

This module also includes worker thread classes for asynchronous operations
to keep the UI responsive during long-running tasks.
"""

import json
from pathlib import Path

from PySide6 import QtCore, QtGui, QtWidgets
from PySide6.QtCore import Q_ARG, Qt

from ros2_unbag.core.bag_reader import BagReader
from ros2_unbag.core.exporter import Exporter
from ros2_unbag.core.routines import ExportRoutine
from ros2_unbag.ui.widgets.topic_list import TopicListWidget
from ros2_unbag.ui.widgets.topic_settings import TopicSettingsWidget
from ros2_unbag.ui.widgets.global_settings import GlobalSettingsWidget
from ros2_unbag.ui.styles import (
    TOP_BAR_STYLE,
    LEFT_CONTAINER_STYLE,
    SCROLL_STYLE,
    GLOBAL_CONTAINER_STYLE,
    LEFT_HEADER_STYLE,
    BG_WHITE,
    PROGRESS_BAR_STYLE,
)

__all__ = ["UnbagApp"]


class WorkerThread(QtCore.QThread):
    """
    Background worker thread for executing long-running tasks without blocking the UI.
    
    Executes a task function in a separate thread and emits signals on completion or error.
    
    Signals:
        finished (object): Emitted when task completes successfully, passes result
        error (Exception): Emitted when task raises an exception, passes the exception
    """
    
    finished = QtCore.Signal(object)
    error = QtCore.Signal(Exception)

    def __init__(self, task_fn, *args):
        """
        Initialize WorkerThread with a task function and arguments.

        Args:
            task_fn: Callable to execute in the thread.
            *args: Arguments to pass to the task function.

        Returns:
            None
        """
        super().__init__()
        self.task_fn = task_fn
        self.args = args

    def run(self):
        """
        Execute the task function with provided args, emit finished signal on success or error signal on exception.

        Args:
            None

        Returns:
            None
        """
        try:
            result = self.task_fn(*self.args)
            self.finished.emit(result)
        except Exception as e:
            self.error.emit(e)


class UnbagApp(QtWidgets.QMainWindow):
    """
    Main application window for ros2_unbag GUI.
    
    Provides a 3-column interface for:
    - Left: Bag file loading and topic selection
    - Middle: Per-topic export settings configuration
    - Right: Global settings and export action
    
    Orchestrates the entire export workflow from bag loading through configuration
    to final export execution.
    """
    
    def __init__(self):
        """
        Initialize the UnbagApp main window and UI components.

        Args:
            None

        Returns:
            None
        """
        super().__init__()
        self.setWindowTitle("ros2 unbag")
        # Start at ~80% of available screen size
        screen = QtGui.QGuiApplication.primaryScreen()
        if screen:
            available = screen.availableGeometry()
            self.resize(int(available.width() * 0.8), int(available.height() * 0.8))
        else:
            self.resize(1200, 800)
        self.setMinimumSize(900, 600)

        self.bag_reader = None
        self.bag_path = None
        self.topics_config = {}  # topic -> config dict
        self.current_exporter = None
        self.default_base_dir = Path.cwd()
        self._export_running = False

        self.init_ui()
        self.show_init_screen()

    def init_ui(self):
        """
        Build the main 3-column UI layout with topic list, settings, and global controls.

        Args:
            None

        Returns:
            None
        """
        # Central Widget
        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)
        root_layout = QtWidgets.QVBoxLayout(central_widget)
        root_layout.setContentsMargins(0, 0, 0, 0)
        root_layout.setSpacing(8)

        # Top Bar
        top_bar = QtWidgets.QWidget()
        top_bar.setObjectName("topBar")
        top_bar.setFixedHeight(60)
        top_bar.setStyleSheet(TOP_BAR_STYLE)
        top_layout = QtWidgets.QHBoxLayout(top_bar)
        top_layout.setContentsMargins(12, 0, 12, 0)
        top_layout.setSpacing(10)
        icon_label = QtWidgets.QLabel()
        icon_path = Path(__file__).resolve().parent / "assets/badge.svg"
        if icon_path.exists():
            icon_pixmap = QtGui.QPixmap(str(icon_path)).scaled(50, 50, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            icon_label.setPixmap(icon_pixmap)
        top_layout.addWidget(icon_label)
        title_label = QtWidgets.QLabel("ros2 unbag")
        title_label.setObjectName("title")
        top_layout.addWidget(title_label)
        top_layout.addStretch()
        self.btn_load_bag = QtWidgets.QPushButton("Load Bag")
        self.btn_load_bag.setObjectName("headerLoadButton")
        self.btn_load_bag.clicked.connect(self.load_bag)
        top_layout.addWidget(self.btn_load_bag)
        root_layout.addWidget(top_bar)

        # Columns container
        columns_container = QtWidgets.QWidget()
        columns_layout = QtWidgets.QHBoxLayout(columns_container)
        columns_layout.setContentsMargins(10, 10, 10, 10)
        columns_layout.setSpacing(0)
        root_layout.addWidget(columns_container)
        splitter = QtWidgets.QSplitter(Qt.Horizontal)
        splitter.setChildrenCollapsible(False)
        splitter.setHandleWidth(6)
        columns_layout.addWidget(splitter)

        # 1. Left Column: Bag & Topics
        left_container = QtWidgets.QWidget()
        left_container.setObjectName("leftContainer")
        left_layout = QtWidgets.QVBoxLayout(left_container)
        left_layout.setContentsMargins(8, 8, 8, 8)
        left_layout.setSpacing(10)
        left_container.setStyleSheet(LEFT_CONTAINER_STYLE)
        left_header = QtWidgets.QLabel("Bag")
        left_header.setStyleSheet(LEFT_HEADER_STYLE)
        left_layout.addWidget(left_header)

        # Bag Loading Area
        bag_group = QtWidgets.QGroupBox("Bag File")
        bag_layout = QtWidgets.QVBoxLayout(bag_group)
        self.lbl_bag_name = QtWidgets.QLabel("No bag loaded")
        self.lbl_bag_name.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)
        self.lbl_bag_name.setWordWrap(True)
        self.btn_load_bag_secondary = QtWidgets.QPushButton("Load Bag")
        self.btn_load_bag_secondary.clicked.connect(self.load_bag)
        bag_row = QtWidgets.QHBoxLayout()
        bag_row.addWidget(self.lbl_bag_name)
        bag_row.addStretch()
        bag_row.addWidget(self.btn_load_bag_secondary)
        bag_layout.addLayout(bag_row)
        left_layout.addWidget(bag_group)

        # Topic List
        self.topic_list = TopicListWidget()
        self.topic_list.topic_selected.connect(self.on_topic_selected)
        self.topic_list.topic_toggled.connect(self.on_topic_toggled)
        left_layout.addWidget(self.topic_list)
        
        # Config Buttons
        cfg_layout = QtWidgets.QHBoxLayout()
        self.btn_load_cfg = QtWidgets.QPushButton("Load Config")
        self.btn_load_cfg.clicked.connect(self.load_config_file)
        self.btn_save_cfg = QtWidgets.QPushButton("Save Config")
        self.btn_save_cfg.clicked.connect(self.save_config_file)
        cfg_layout.addWidget(self.btn_load_cfg)
        cfg_layout.addWidget(self.btn_save_cfg)
        left_layout.addLayout(cfg_layout)

        splitter.addWidget(left_container)

        # 2. Middle Column: Settings
        self.topic_settings = TopicSettingsWidget(Path.cwd())
        self.topic_settings.settings_changed.connect(self.on_settings_changed)
        self.topic_settings.export_toggle_requested.connect(self.on_badge_toggle)

        # Middle container with scroll area; only the scroll contents swap to a loading card
        mid_container = QtWidgets.QWidget()
        mid_layout = QtWidgets.QVBoxLayout(mid_container)
        mid_layout.setContentsMargins(0, 0, 0, 0)
        mid_layout.setSpacing(0)

        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        scroll.setStyleSheet(SCROLL_STYLE)

        self.mid_stack = QtWidgets.QStackedWidget()

        # Loading page: white card with centered gif
        self.loading_page = QtWidgets.QWidget()
        loading_layout = QtWidgets.QVBoxLayout(self.loading_page)
        loading_layout.setContentsMargins(0, 0, 0, 0)
        loading_layout.setSpacing(0)
        loading_layout.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.center_gif = QtWidgets.QLabel()
        self.center_gif.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.center_gif.setStyleSheet(f"background: {BG_WHITE};")
        self.center_gif.setMinimumHeight(240)
        self.loading_percent = QtWidgets.QLabel("")
        self.loading_percent.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.loading_percent.setStyleSheet("font-size: 16px; font-weight: 700; color: #1f2937;")
        gif_path = Path(__file__).resolve().parent / "assets/loading.gif"
        self.center_gif_movie = None
        if gif_path.exists():
            movie = QtGui.QMovie(str(gif_path))
            self.center_gif.setMovie(movie)
            self.center_gif_movie = movie
        loading_layout.addWidget(self.center_gif, 0, Qt.AlignmentFlag.AlignCenter)
        loading_layout.addWidget(self.loading_percent, 0, Qt.AlignmentFlag.AlignCenter)

        # Settings page
        self.settings_page = QtWidgets.QWidget()
        settings_layout = QtWidgets.QVBoxLayout(self.settings_page)
        settings_layout.setContentsMargins(0, 0, 0, 0)
        settings_layout.setSpacing(0)
        settings_layout.addWidget(self.topic_settings)

        self.mid_stack.addWidget(self.loading_page)
        self.mid_stack.addWidget(self.settings_page)
        self.mid_stack.setCurrentWidget(self.settings_page)
        scroll.setWidget(self.mid_stack)
        mid_layout.addWidget(scroll)
        splitter.addWidget(mid_container)

        # 3. Right Column: Global & Summary
        self.global_settings = GlobalSettingsWidget()
        self.global_settings.export_clicked.connect(self.export_data)
        self.global_settings.cancel_clicked.connect(self.cancel_export)
        self.global_settings.base_dir_changed.connect(self.on_base_dir_changed)
        self.global_settings.setFixedWidth(300)
        global_wrapper = QtWidgets.QWidget()
        global_wrapper.setObjectName("globalContainer")
        global_wrapper.setFixedWidth(300)
        global_wrapper.setStyleSheet(GLOBAL_CONTAINER_STYLE)
        global_layout = QtWidgets.QVBoxLayout(global_wrapper)
        global_layout.setContentsMargins(0, 0, 0, 0)
        global_layout.addWidget(self.global_settings)
        splitter.addWidget(global_wrapper)
        splitter.setSizes([350, 650, 300])

        # Status Bar
        self.status_bar = QtWidgets.QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_progress = QtWidgets.QProgressBar()
        self.status_progress.setRange(0, 0)  # indeterminate by default
        self.status_progress.setStyleSheet(PROGRESS_BAR_STYLE)
        self.status_progress.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)
        self.status_progress.setTextVisible(False)
        self.status_progress.setVisible(False)
        self.status_bar.addPermanentWidget(self.status_progress, 1)  # stretch across remaining space
        self._update_status_progress_width()
        self._status_base_msg = "Ready"
        self.status_bar.showMessage("Ready")

    def show_init_screen(self):
        """
        Disable UI controls until a bag file is loaded.

        Args:
            None

        Returns:
            None
        """
        # Disable controls until bag is loaded
        self.topic_list.setEnabled(False)
        # Keep topic settings active so the placeholder/image stays colored
        self.topic_settings.setEnabled(True)
        self.global_settings.setEnabled(False)
        self.btn_load_cfg.setEnabled(False)
        self.btn_save_cfg.setEnabled(False)
        self.global_settings.set_base_dir_enabled(False)

    def _set_export_running(self, running: bool):
        """
        Toggle UI interactivity during export while keeping cancel available.

        Args: 
            running (bool): True if export is running, False otherwise.

        Returns:
            None
        """
        self._export_running = running
        has_bag = self.bag_reader is not None

        self.btn_load_bag.setEnabled(not running)
        self.btn_load_bag_secondary.setEnabled(not running)
        self.topic_list.setEnabled(has_bag and not running)
        self.topic_settings.setEnabled(has_bag and not running)
        self.btn_load_cfg.setEnabled(has_bag and not running)
        self.btn_save_cfg.setEnabled(has_bag and not running)

        self.global_settings.setEnabled(has_bag)
        self.global_settings.set_controls_enabled(not running)
        self.global_settings.set_base_dir_enabled(has_bag and not running)
        self.global_settings.set_export_running(running)

    def load_bag(self):
        """
        Prompt user to select a bag file, reset state, and start background reader thread.

        Args:
            None

        Returns:
            None
        """
        bag_path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Open Bag File", "", "Bag Files (*.db3 *.mcap)")
        if not bag_path:
            return

        self.bag_path = Path(bag_path)
        self.lbl_bag_name.setText(self.bag_path.name)
        self.default_base_dir = self.bag_path.parent
        self.global_settings.set_base_dir(self.default_base_dir)
        
        # Reset state
        self.topics_config = {}
        self.topic_settings.default_folder = self.default_base_dir
        
        # Show loading in status bar
        self._show_status_progress("Loading bag file...", indeterminate=True)

        self.worker = WorkerThread(lambda p: BagReader(p), bag_path)
        self.worker.finished.connect(self.on_bag_loaded)
        self.worker.error.connect(self.handle_bag_error)
        self.worker.start()

    def on_base_dir_changed(self, new_dir):
        """
        Update base directory for all topics when changed from the global settings.
        """
        if not new_dir:
            return
        self.default_base_dir = Path(new_dir)
        self.topic_settings.default_folder = self.default_base_dir

        # Update existing topic configs
        for topic, cfg in self.topics_config.items():
            if isinstance(cfg, dict):
                cfg["path"] = str(self.default_base_dir)

        # Update current topic UI if present
        if self.topic_settings.current_topic:
            self.topic_settings.path_edit.setText(str(self.default_base_dir))

        self.status_bar.showMessage(f"Base directory set to {new_dir}")

    def on_bag_loaded(self, reader):
        """
        Handle successful bag loading: close dialog, populate topic list, enable UI controls.

        Args:
            reader: BagReader instance for the loaded bag.

        Returns:
            None
        """
        self._hide_status_progress(f"Loaded {self.bag_path.name}")
        self.bag_reader = reader
        self.topic_settings.set_bag_path(self.bag_path)
        
        # Populate Topic List
        topics = self.bag_reader.get_topics()
        counts = self.bag_reader.get_message_count()
        self.topic_list.load_topics(topics, counts)
        
        # Initialize default config for all topics
        # We don't pre-populate everything to save memory, but we can if needed.
        # For now, we'll generate config on the fly if missing when selecting.
        
        self.topic_list.setEnabled(True)
        self.topic_settings.setEnabled(True)
        self.global_settings.setEnabled(True)
        self.btn_load_cfg.setEnabled(True)
        self.btn_save_cfg.setEnabled(True)
        self.global_settings.set_base_dir_enabled(True)
        self.global_settings.set_controls_enabled(True)
        self.global_settings.set_export_running(False)
        
        self.status_bar.showMessage(f"Loaded {self.bag_path.name}")
        self.update_summary()

    def resizeEvent(self, event):
        """
        Keep the status bar progress indicator at ~80% of the available width and anchored right.

        Args:
            event: QResizeEvent delivered by Qt when the window is resized.

        Returns:
            None
        """
        super().resizeEvent(event)
        self._update_status_progress_width()

    def _update_status_progress_width(self):
        """
        Adjust the status bar progress width to roughly 80% of the status bar space.

        Ensures a minimum width to keep the progress control usable on very small windows.

        Args:
            None

        Returns:
            None
        """
        if hasattr(self, "status_bar") and hasattr(self, "status_progress"):
            target = max(120, int(self.status_bar.width() * 0.8))
            self.status_progress.setFixedWidth(target)

    def _set_center_animation(self, running: bool):
        """
        Start or stop the inline loading gif in the middle card.

        Shows the loading page and starts the gif when running is True, otherwise
        stops the gif and switches back to the settings page.

        Args:
            running (bool): True to start the loading animation, False to stop it.

        Returns:
            None
        """
        if running:
            self.mid_stack.setCurrentWidget(self.loading_page)
            if self.center_gif_movie:
                self.center_gif_movie.start()
            if hasattr(self, "loading_percent"):
                self.loading_percent.setText("")
        else:
            if self.center_gif_movie:
                self.center_gif_movie.stop()
            if hasattr(self, "loading_percent"):
                self.loading_percent.setText("")
            self.mid_stack.setCurrentWidget(self.settings_page)

    def _show_status_progress(self, message: str, indeterminate: bool = True):
        """
        Display the status bar progress with optional determinate mode.

        Sets the status message, configures the progress bar range/value, makes it visible,
        and enables the center loading animation.

        Args:
            message (str): Message to show in the status bar.
            indeterminate (bool): If True, show an indeterminate (busy) progress bar.
                                 If False, set range 0-100 and reset value to 0.

        Returns:
            None
        """
        self._status_base_msg = message
        self.status_bar.showMessage(message)
        if indeterminate:
            self.status_progress.setRange(0, 0)
        else:
            self.status_progress.setRange(0, 100)
            self.status_progress.setValue(0)
        self.status_progress.setVisible(True)
        self._set_center_animation(True)

    def _hide_status_progress(self, message: str | None = None):
        """
        Hide the status bar progress indicator and optional message.

        Stops the center loading animation and hides the progress bar. If a message
        is provided it is shown in the status bar.

        Args:
            message (str | None): Optional message to display after hiding progress.

        Returns:
            None
        """
        if message:
            self._status_base_msg = message
            self.status_bar.showMessage(message)
        self.status_progress.setVisible(False)
        self._set_center_animation(False)

    def handle_bag_error(self, e):
        """
        Handle errors that occur while loading a bag file.

        Args:
            e: Exception instance raised during bag loading.

        Returns:
            None
        """
        self._hide_status_progress("Failed to load bag")
        QtWidgets.QMessageBox.critical(self, "Error", str(e))

    def on_topic_selected(self, topic):
        """
        Handle topic selection from the list: load topic settings into the middle column.
        
        Finds the topic's message type, creates default config if needed, and displays
        the settings in the TopicSettingsWidget.

        Args:
            topic: Selected topic name string.

        Returns:
            None
        """
        # Get topic type
        topic_type = None
        for t_type, t_list in self.bag_reader.get_topics().items():
            if topic in t_list:
                topic_type = t_type
                break
        
        if not topic_type:
            return

        # Persist current topic settings before switching
        if self.topic_settings.current_topic:
            current_topic = self.topic_settings.current_topic
            self.topics_config.setdefault(current_topic, {})
            self.topics_config[current_topic].update(self.topic_settings.get_config())

        # Get or create config
        if topic not in self.topics_config:
            self.topics_config[topic] = {
                "path": str(self.default_base_dir),
                "subfolder": "%name",
                "naming": "%name",
                "format": ""  # Will default in widget
            }
        
        self.topic_settings.set_topic(topic, topic_type, self.topics_config[topic])
        self.topic_settings.set_export_state(self.topic_list.is_checked(topic))

    def on_settings_changed(self, topic, new_config):
        """
        Handle settings changes from TopicSettingsWidget and update internal config.

        Args:
            topic: Topic name string.
            new_config: Updated configuration dictionary.

        Returns:
            None
        """
        # Merge new config into existing config, preserving any keys not in new_config
        # This allows partial updates without losing other settings
        if topic not in self.topics_config:
            self.topics_config[topic] = {}
        self.topics_config[topic].update(new_config)

    def on_topic_toggled(self, topic, is_checked):
        """
        Handle topic checkbox toggle and update summary display.

        Args:
            topic: Topic name string.
            is_checked: Boolean indicating if topic is now checked.

        Returns:
            None
        """
        self.update_summary()
        if self.topic_settings.current_topic == topic:
            self.topic_settings.set_export_state(is_checked)

    def on_badge_toggle(self, topic):
        """
        Toggle topic export selection when the export badge is clicked in topic settings.
        
        Args:
            topic: Topic name string to toggle.
        """
        if topic not in self.topic_list.topics:
            return
        # Toggle the checkbox state in the topic list
        new_state = not self.topic_list.is_checked(topic)
        self.topic_list.set_checked(topic, new_state)

    def update_summary(self):
        """
        Update the global settings summary with current topic selection counts.

        Args:
            None

        Returns:
            None
        """
        # Count checked items in topic list
        selected_count = 0
        selected_topics = []
        for topic in self.topic_list.topics:
            if self.topic_list.is_checked(topic):
                selected_count += 1
                selected_topics.append(topic)
        
        total_count = len(self.topic_list.topics)
        self.global_settings.update_summary(selected_count, total_count, selected_topics)

    def get_export_config(self):
        """
        Collect and validate export configuration for all selected topics.
        
        Gathers configuration from UI widgets for all checked topics, validates
        global settings (especially master topic selection for resampling), and
        applies default values where needed.

        Args:
            None

        Returns:
            tuple: (final_config dict, global_config dict) containing export settings
                  for selected topics and global configuration.
                  
        Raises:
            ValueError: If resampling is enabled but no master topic is selected,
                       or if other validation fails.
        """
        # Gather config for all SELECTED (checked) topics
        final_config = {}
        errors = []
        selected_topics = [
            topic for topic in self.topic_list.topics
            if self.topic_list.is_checked(topic)
        ]
        if not selected_topics:
            raise ValueError("Select at least one topic to export.")
        
        # First, ensure current settings in middle column are saved
        current_topic = self.topic_settings.current_topic
        if current_topic:
            self.topics_config.setdefault(current_topic, {})
            self.topics_config[current_topic].update(self.topic_settings.get_config())
            self.topic_settings.validate_inputs()

        global_cfg = self.global_settings.get_config()
        
        # Validate global config (master topic)
        if "resample_config" in global_cfg:
            master_topic = global_cfg["resample_config"].get("master_topic")
            if not master_topic or master_topic not in selected_topics:
                raise ValueError("Resampling enabled but no Master Topic selected among exported topics.")

        for topic in self.topic_list.topics:
            if self.topic_list.is_checked(topic):
                # Get config, use defaults if not visited
                cfg = self.topics_config.get(topic, {}).copy()
                
                if not cfg.get("format"):
                    # Find type
                    t_type = next((k for k, v in self.bag_reader.get_topics().items() if topic in v), None)
                    # Default format
                    formats = ExportRoutine.get_formats(t_type)
                    if formats:
                        cfg["format"] = formats[0]
                        # Also set default path/naming if empty
                        if "path" not in cfg:
                            cfg["path"] = str(self.default_base_dir)
                        if "naming" not in cfg:
                            cfg["naming"] = "%name"
                if "subfolder" not in cfg:
                    cfg["subfolder"] = "%name"
                
                cfg["path"] = (cfg.get("path") or "").strip()
                cfg["subfolder"] = (cfg.get("subfolder") or "").strip("/")
                cfg["naming"] = (cfg.get("naming") or "").strip()

                if not cfg["path"]:
                    errors.append(f"{topic}: Output Directory is required.")
                if not cfg["naming"]:
                    errors.append(f"{topic}: Naming scheme is required.")
                
                final_config[topic] = cfg

        if errors:
            raise ValueError("Please fix the following per-topic settings:\n- " + "\n- ".join(errors))

        return final_config, global_cfg

    def export_data(self):
        """
        Initiate the export process: validate config, show progress indicators, and start background export thread.

        Args:
            None

        Returns:
            None
        """
        try:
            config, global_config = self.get_export_config()
        except ValueError as e:
            QtWidgets.QMessageBox.critical(self, "Configuration Error", str(e))
            return

        self.global_settings.hide_feedback()
        self._set_export_running(True)
        self._show_status_progress("Exporting...", indeterminate=False)

        self.worker = WorkerThread(self.run_export, self.bag_reader, config, global_config)
        self.worker.finished.connect(self.on_export_finished)
        self.worker.error.connect(self.handle_export_error)
        self.worker.start()

    def cancel_export(self):
        """
        Request cancellation for the currently running export.

        Args: 
            None

        Returns:
            None
        """
        if not self._export_running:
            return
        self.status_bar.showMessage("Cancelling export...")
        if self.current_exporter is not None:
            self.current_exporter.abort_export()

    def run_export(self, bag_reader, config, global_config):
        """
        Execute the export process in a background thread with progress updates.
        
        Creates an Exporter instance with a progress callback that updates the
        status/progress widgets, then runs the export.

        Args:
            bag_reader: BagReader instance.
            config: Per-topic export configuration dictionary.
            global_config: Global export settings dictionary.

        Returns:
            None
        """
        def progress(current, total):
            if total <= 0:
                value = 0
            else:
                value = int((current / total) * 100)
            QtCore.QMetaObject.invokeMethod(
                self.status_progress, "setValue",
                QtCore.Qt.ConnectionType.QueuedConnection,
                Q_ARG(int, value)
            )
            QtCore.QMetaObject.invokeMethod(
                self.status_bar, "showMessage",
                QtCore.Qt.ConnectionType.QueuedConnection,
                Q_ARG(str, f"{self._status_base_msg} ({value}%)")
            )
            if hasattr(self, "loading_percent"):
                QtCore.QMetaObject.invokeMethod(
                    self.loading_percent, "setText",
                    QtCore.Qt.ConnectionType.QueuedConnection,
                    Q_ARG(str, f"{value}%")
                )
        
        self.current_exporter = Exporter(bag_reader, config, global_config, progress_callback=progress)
        QtCore.QMetaObject.invokeMethod(
            self.global_settings.btn_cancel, "setEnabled",
            QtCore.Qt.ConnectionType.QueuedConnection,
            Q_ARG(bool, True)
        )
        self.current_exporter.run()

    def on_export_finished(self, _):
        """
        Handle successful export completion: close dialog, re-enable UI, show success message.

        Args:
            _: Unused result from worker thread.

        Returns:
            None
        """
        self.current_exporter = None
        self._hide_status_progress("Export complete")
        self._set_export_running(False)
        self.global_settings.show_feedback("Export complete.")

    def handle_export_error(self, e):
        """
        Handle export errors: close dialog, re-enable UI, show error message.

        Args:
            e: Exception instance from the export process.

        Returns:
            None
        """
        self.current_exporter = None
        self._set_export_running(False)

        if "Export aborted by user" in str(e):
            self._hide_status_progress("Export canceled")
            self.global_settings.show_feedback("Export canceled.", feedback_type="cancel")
            return

        self._hide_status_progress("Export failed")
        QtWidgets.QMessageBox.critical(self, "Export Error", str(e))

    def save_config_file(self):
        """
        Prompt for save path, collect all topic and global configs, and write to JSON file.

        Args:
            None

        Returns:
            None
        """
        file_path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Save Config", str(Path.cwd() / "config.json"), "JSON (*.json)")
        if not file_path:
            return
        
        try:
            # Save ALL known config, or just selected?
            # Usually save all config so it can be reloaded.
            # But we also need global config.
            
            # Update current topic first
            current = self.topic_settings.current_topic
            if current:
                self.topics_config[current].update(self.topic_settings.get_config())
                
            full_config = {
                topic: cfg for topic, cfg in self.topics_config.items()
                if isinstance(cfg, dict)
            }
            full_config["__global__"] = self.global_settings.get_config()
            
            with open(file_path, "w", encoding="utf-8") as f:
                json.dump(full_config, f, indent=2)
            
            self.status_bar.showMessage(f"Saved config to {file_path}")
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Error", f"Failed to save: {e}")

    def load_config_file(self):
        """
        Prompt for config file, load JSON, extract global settings, and populate UI with loaded configuration.

        Args:
            None

        Returns:
            None
        """
        file_path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Load Config", str(Path.cwd()), "JSON (*.json)")
        if not file_path:
            return
        if self.bag_reader is None:
            QtWidgets.QMessageBox.warning(self, "Load Bag First", "Please load a bag file before loading a config.")
            return
        
        try:
            with open(file_path, "r", encoding="utf-8") as f:
                config = json.load(f)
            
            if "__global__" in config:
                self.global_settings.set_config(config.pop("__global__"))
            
            self.topics_config = {
                topic: cfg for topic, cfg in config.items()
                if isinstance(cfg, dict)
            }

            # Apply selection state based on loaded config
            missing_topics = []
            for topic in self.topic_list.topics:
                # block signals to avoid redundant updates while we toggle many items
                self.topic_list.set_checked(topic, topic in self.topics_config, block_signals=True)
            for topic in self.topics_config.keys():
                if topic not in self.topic_list.topics:
                    missing_topics.append(topic)
            self.update_summary()
            
            # Refresh current topic if selected
            current = self.topic_settings.current_topic
            if current and current in self.topics_config:
                # Need to re-set topic to refresh UI
                # We need type though.
                t_type = self.topic_settings.current_type
                self.topic_settings.set_topic(current, t_type, self.topics_config[current])
                self.topic_settings.set_export_state(self.topic_list.is_checked(current))
            
            if missing_topics:
                QtWidgets.QMessageBox.warning(
                    self,
                    "Missing Topics",
                    "The following topics from the config are not in the loaded bag:\n"
                    + "\n".join(missing_topics)
                )
            self.status_bar.showMessage(f"Loaded config from {file_path}")
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Error", f"Failed to load: {e}")
