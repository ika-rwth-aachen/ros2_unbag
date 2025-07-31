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

import json
from pathlib import Path

from PySide6 import QtCore, QtGui, QtWidgets
from PySide6.QtCore import Q_ARG, Qt

from ros2_unbag.core.bag_reader import BagReader
from ros2_unbag.core.exporter import Exporter
from ros2_unbag.ui.widgets import ExportOptions, TopicSelector


class WorkerThread(QtCore.QThread):
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


class ExportProgressDialog(QtWidgets.QDialog):
    # Custom dialog with animation and progress bar

    def __init__(self, text, *args, **kwargs):
        """
        Initialize ExportProgressDialog with display text and optional args.

        Args:
            text: Text to display in the dialog.
            *args: Additional positional arguments.
            **kwargs: Additional keyword arguments.

        Returns:
            None
        """
        super().__init__(*args, **kwargs)

        # Display text
        layout = QtWidgets.QVBoxLayout(self)
        text_label = QtWidgets.QLabel(text)
        layout.addWidget(text_label, alignment=Qt.AlignmentFlag.AlignHCenter)

        # Display GIF animation
        gif_label = QtWidgets.QLabel(self)
        base_dir = Path(__file__).resolve().parent
        gif_animation = QtGui.QMovie(str(base_dir / "assets/loading.gif"))
        gif_label.setMovie(gif_animation)
        gif_animation.start()
        layout.addWidget(gif_label, alignment=Qt.AlignmentFlag.AlignHCenter)

        # Initialize progress bar
        self.progress_bar = QtWidgets.QProgressBar(self)
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setSizePolicy(QtWidgets.QSizePolicy.Policy.Expanding,
                                        QtWidgets.QSizePolicy.Policy.Fixed)
        layout.addWidget(self.progress_bar)

    @QtCore.Slot(int)
    def setValue(self, value):
        """
        Update the progress bar to the given integer value.

        Args:
            value: Integer progress value (0-100).

        Returns:
            None
        """
        self.progress_bar.setValue(value)
    
    def closeEvent(self, event):
        """
        Emit finished event and close.

        Args:
            event: QCloseEvent instance.

        Returns:
            None
        """
        self.finished.emit(0)
        super().closeEvent(event)


class UnbagApp(QtWidgets.QWidget):
    # Main application widget for exporting ROS2 bag data

    def __init__(self):
        """
        Initialize UnbagApp UI: set title, size, scroll area, title image, and file selection button.

        Args:
            None

        Returns:
            None
        """
        super().__init__()
        self.setWindowTitle("ros2 unbag")
        self.setGeometry(100, 100, 800, 600)

        # Scrollable area setup
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        self.scroll_content = QtWidgets.QWidget()
        self.layout = QtWidgets.QVBoxLayout(self.scroll_content)
        scroll.setWidget(self.scroll_content)

        main_layout = QtWidgets.QVBoxLayout(self)
        main_layout.addWidget(scroll)
        self.setLayout(main_layout)

        # Show init screen
        self.show_init_screen()

        self.pending_config = None
        self.bag_loaded = False
        self.current_exporter = None

    def load_bag(self):
        """
        Prompt user to select a bag file, disable UI, show loading dialog, and start background reader thread.

        Args:
            None

        Returns:
            None
        """
        # Open file dialog and load bag in background
        bag_path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Open Bag File", "", "Bag Files (*.db3 *.mcap)")
        if not bag_path:
            return
        
        # Reset any previously loaded bag options
        self.bag_reader = None
        self.topic_selector = None
        self.export_options = None
        self.bag_loaded = False

        self.bag_parent_folder = Path(bag_path).parent

        self.setEnabled(False)
        self.wait_dialog = QtWidgets.QProgressDialog(
            "Loading bag file, please wait...", None, 0, 0, self)
        self.wait_dialog.setWindowTitle("Loading")
        self.wait_dialog.setWindowModality(Qt.WindowModality.WindowModal)
        self.wait_dialog.setCancelButton(None)
        self.wait_dialog.resize(400, 100)
        self.wait_dialog.setMinimumDuration(0)
        self.wait_dialog.show()
        QtWidgets.QApplication.processEvents()

        self.worker = WorkerThread(self.load_bag_reader, bag_path)
        self.worker.finished.connect(self.on_bag_loaded)
        self.worker.error.connect(lambda e: QtWidgets.QMessageBox.critical(self, "Error", f"Unexpected error: {e}"))
        QtCore.QTimer.singleShot(100, self.worker.start)

    def load_bag_reader(self, path):
        """
        Attempt to create a BagReader for the given path; return the reader or exception.

        Args:
            path: Path to the ROS2 bag file.

        Returns:
            BagReader instance or Exception.
        """
        try:
            return BagReader(path)
        except Exception as e:
            return e

    def on_bag_loaded(self, result):
        """
        Called when bag loading completes: close dialog, re-enable UI, handle errors or show topic selector.

        Args:
            result: BagReader instance or Exception.

        Returns:
            None
        """
        self.wait_dialog.close()
        self.setEnabled(True)

        if isinstance(result, Exception):
            QtWidgets.QMessageBox.critical(self, "Error",
                                           f"Failed to load bag: {result}")
            return

        self.bag_reader = result
        self.bag_loaded = True
        self.show_topic_selector()

    def _validate_config(self, config):
        """
        Ensure each topic in config has a non-empty output directory; raise ValueError on errors.

        Args:
            config: Dict of per-topic export configuration.

        Returns:
            None

        Raises:
            ValueError: If any topic has an empty output directory.
        """
        # Check if output directories are set for each topic
        errors = []
        for topic, cfg in config.items():
            path = cfg.get('path', '').strip()
            if not path:
                errors.append(f"Empty output directory for topic '{topic}'")
        
        if errors:
            print("\033[91mConfiguration errors found:")
            for error in errors:
                print(f"  - {error}")
            print("Please set output directory paths and try again!\033[0m")
            raise ValueError(f"Invalid export configuration:\n" + "\n".join(errors))

    def show_init_screen(self):
        """
        Clear UI and show the initial file selection screen.

        Args:
            None

        Returns:
            None
        """
        self.clear_layout()

        # Title image
        base_dir = Path(__file__).resolve().parent
        pixmap = QtGui.QPixmap(str(base_dir / "assets/title.png")).scaledToWidth(750, QtCore.Qt.TransformationMode.SmoothTransformation)
        image_label = QtWidgets.QLabel()
        image_label.setPixmap(pixmap)
        image_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.layout.addWidget(image_label)

        # Button to select a bag file
        file_button = QtWidgets.QPushButton("Select ROS2 Bag File (.mcap/.db3)")
        file_button.setFixedHeight(40)
        file_button.clicked.connect(self.load_bag)
        self.layout.addWidget(file_button)

    def show_topic_selector(self):
        """
        Clear UI layout, display TopicSelector in a scroll area, and add Load/Next buttons.

        Args:
            None

        Returns:
            None
        """
        self.clear_layout()

        # Scrollable area for topic selector
        scroll_area = QtWidgets.QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_content = QtWidgets.QWidget()
        scroll_layout = QtWidgets.QVBoxLayout(scroll_content)

        self.topic_selector = TopicSelector(self.bag_reader)
        scroll_layout.addWidget(self.topic_selector)
        scroll_area.setWidget(scroll_content)

        # Add scroll area to main layout
        self.layout.addWidget(scroll_area)

        # Fixed button layout at bottom
        button_layout = QtWidgets.QHBoxLayout()
        
        back_button = QtWidgets.QPushButton("Back")
        back_button.clicked.connect(self.show_init_screen)
        button_layout.addWidget(back_button)

        load_config_button = QtWidgets.QPushButton("Load Config")
        load_config_button.clicked.connect(self.load_config_file)
        button_layout.addWidget(load_config_button)

        next_button = QtWidgets.QPushButton("Next")
        next_button.clicked.connect(self.show_export_settings_page)
        button_layout.addWidget(next_button)

        self.layout.addLayout(button_layout)

    def show_export_settings_page(self, config=None, global_config=None):
        """
        Clear layout and show export options for selected or loaded config, with navigation buttons.

        Args:
            config: Optional dict of per-topic export configuration.
            global_config: Optional dict of global settings.

        Returns:
            None
        """
        if config is not None and isinstance(config, dict):
            selected_topics = list(config.keys())
        else:
            selected_topics = self.topic_selector.get_selected_topics()
        if not selected_topics:
            return

        self.clear_layout()

        # Scrollable area for export options
        scroll_area = QtWidgets.QScrollArea()
        scroll_area.setWidgetResizable(True)

        scroll_content = QtWidgets.QWidget(self)
        scroll_layout = QtWidgets.QVBoxLayout(scroll_content)

        self.export_options = ExportOptions(
            selected_topics,
            self.bag_reader.get_topics(),
            self.bag_parent_folder
        )
        scroll_layout.addWidget(self.export_options)
        scroll_area.setWidget(scroll_content)

        self.layout.addWidget(scroll_area)

        # Fixed button layout at bottom
        button_container = QtWidgets.QWidget(self)
        button_layout = QtWidgets.QHBoxLayout(button_container)

        back_button = QtWidgets.QPushButton("Back", self)
        back_button.clicked.connect(self.show_topic_selector)
        button_layout.addWidget(back_button)

        save_config_button = QtWidgets.QPushButton("Save Config", self)
        save_config_button.clicked.connect(self.save_config_file)
        button_layout.addWidget(save_config_button)

        load_config_button = QtWidgets.QPushButton("Load Config", self)
        load_config_button.clicked.connect(self.load_config_file)
        button_layout.addWidget(load_config_button)

        export_button = QtWidgets.QPushButton("Export", self)
        export_button.clicked.connect(self.export_data)
        button_layout.addWidget(export_button)

        self.layout.addWidget(button_container)

        if config is not None and isinstance(config, dict):
            self.export_options.set_export_config(config, global_config)


    def export_data(self):
        """
        Disable UI, show export progress dialog, validate config, and start background export thread.

        Args:
            None

        Returns:
            None
        """
        # Run export in background with progress dialog
        self.setEnabled(False)
        self.wait_dialog = ExportProgressDialog("Exporting, please wait...", self)
        self.wait_dialog.setWindowTitle("Exporting")
        # Block users from interacting with the main window
        self.wait_dialog.setWindowModality(Qt.WindowModality.WindowModal)
        self.wait_dialog.resize(400, 100)
        self.wait_dialog.setValue(0)
        self.wait_dialog.show()
        self.wait_dialog.finished.connect(self.on_export_aborted)
        QtWidgets.QApplication.processEvents()

        config = None

        try:
            config, global_config = self.export_options.get_export_config()
        except ValueError as e:
            self.wait_dialog.close()
            QtWidgets.QMessageBox.critical(self, "Configuration Error", str(e))
            self.setEnabled(True)
            self.show_export_settings_page()  # show the config UI again
            return
        
        self.last_used_config = config
        self.last_used_global_config = global_config

        self.worker = WorkerThread(self.run_export, self.bag_reader, config, global_config)
        self.worker.finished.connect(self.on_export_finished)
        self.worker.error.connect(self.handle_export_error)
        self.worker.start()


    def run_export(self, bag_reader, config, global_config):
        """
        Validate config, instantiate Exporter with progress callback, and run export process.

        Args:
            bag_reader: BagReader instance.
            config: Dict of per-topic export configuration.
            global_config: Dict of global settings.

        Returns:
            None
        """
        # Run export using Exporter with progress updates
        def progress(current, total):
            value = int((current / total) * 100)
            QtCore.QMetaObject.invokeMethod(
                self.wait_dialog, "setValue",
                QtCore.Qt.ConnectionType.QueuedConnection,
                Q_ARG(int, value)
            )
        
        # If this fails, it will raise an exception that is caught in the worker thread
        self._validate_config(config)
        
        self.current_exporter = Exporter(bag_reader, config, global_config, progress_callback=progress)
        self.current_exporter.run()
        
        return None

    def on_export_finished(self, _):
        """
        Close progress dialog, re-enable UI, notify user of completion, and return to export settings.

        Args:
            _: Unused.

        Returns:
            None
        """
        self.wait_dialog.close()
        self.setEnabled(True)
        QtWidgets.QMessageBox.information(self, "Done", "Export complete.")

        # Return to export settings page with previous config
        self.show_export_settings_page(config=self.last_used_config, global_config=self.last_used_global_config)

    def on_export_aborted(self, _):
        """
        Send out a runtime error, to cleanly kill all workers.

        Args:
            _: Unused.

        Returns:
            None
        """
        if self.current_exporter:
            self.current_exporter.abort_export()

    @QtCore.Slot(Exception)
    def handle_export_error(self, e):
        """
        Terminate export thread on error, show error message, and quit application.

        Args:
            e: Exception instance.

        Returns:
            None
        """
        self.wait_dialog.close()
        self.setEnabled(True)

        QtWidgets.QMessageBox.critical(self, "Export Error", str(e))
        
        # Return to export settings page with previous config
        self.worker.terminate()
        self.show_export_settings_page(config=self.last_used_config, global_config=self.last_used_global_config)

    def clear_layout(self):
        """
        Recursively remove and delete all widgets and sublayouts from the main layout.

        Args:
            None

        Returns:
            None
        """
        while self.layout.count():
            item = self.layout.takeAt(0)
            widget = item.widget()
            if widget is not None:
                widget.deleteLater()
            else:
                sublayout = item.layout()
                if sublayout is not None:
                    while sublayout.count():
                        subitem = sublayout.takeAt(0)
                        subwidget = subitem.widget()
                        if subwidget is not None:
                            subwidget.deleteLater()
                    sublayout.deleteLater()

    def save_config_file(self):
        """
        Prompt for save path, retrieve export config, ensure directory exists, and write JSON config to file.

        Args:
            None

        Returns:
            None
        """
        # Open file dialog to get save path
        file_path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Save Config File", str(Path.cwd() / "config.json"), "Config Files (*.json)")
        if not file_path:
            return
        
        # Load the export options and global config
        try:
            config, global_config = self.export_options.get_export_config()
            config["__global__"] = global_config
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Error", f"Cannot get config: {e}")
            return
        
        # Ensure the directory exists
        config_path = Path(file_path)
        if not config_path.parent.exists():
            try:
                config_path.parent.mkdir(parents=True, exist_ok=True)
            except Exception as e:
                QtWidgets.QMessageBox.critical(self, "Error", f"Failed to create directory: {e}")
                return
        
        # Save the config to the specified file
        try:
            with open(file_path, "w") as f:
                json.dump(config, f, indent=2)
            QtWidgets.QMessageBox.information(self, "Saved", f"Config saved as {config_path}")
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Error", f"Failed to save config: {e}")

    def load_config_file(self):
        """
        Prompt for config file, load JSON, extract global settings, and populate export settings UI.

        Args:
            None

        Returns:
            None
        """
        # Open file dialog to load config
        file_path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Load Config", str(Path.cwd()), "Config Files (*.json)")
        if not file_path:
            return
        try:
            # Parse global config if it exists
            with open(file_path, "r") as f:
                config = json.load(f)
            if "__global__" in config:
                global_config = config.pop("__global__")
            else:
                global_config = {}

            # Configure export options with loaded config
            self.show_export_settings_page(config, global_config)
            QtWidgets.QMessageBox.information(self, "Loaded", f"Config loaded from {file_path}")
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Error", f"Failed to load config: {e}")
