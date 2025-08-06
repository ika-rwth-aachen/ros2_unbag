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

import importlib
import json
import os
import sys

from PySide6 import QtWidgets
from PySide6.QtWidgets import QMessageBox
from ros2cli.command import CommandExtension
from tqdm import tqdm

from ros2_unbag.core.bag_reader import BagReader
from ros2_unbag.core.exporter import Exporter
import ros2_unbag.core.processors
import ros2_unbag.core.routines
from ros2_unbag.ui.main_window import UnbagApp


class ExportCommand(CommandExtension):
    def add_arguments(self, parser, cli_name):
        """
        Add command-line arguments for the export command.

        Args:
            parser: Argument parser object.
            cli_name: Name of the CLI command.

        Returns:
            None
        """
        parser.add_argument("bag", nargs="?", help="Path to ROS2 bag file")
        parser.add_argument(
            "--export", "-e", action="append",
            help="Export spec: /topic:format[:subdir]. Can be repeated.")
        parser.add_argument("--output-dir", "-o", help="Base output directory")
        parser.add_argument(
            "--naming", default="%name_%index",
            help="Naming pattern. Supports %%name, %%index, and strftime (e.g. `%%Y-%%m-%%d_%%H-%%M-%%S`) which uses ROS timestamp.")
        parser.add_argument(
            "--resample",
            help="Optional resampling: /master_topic:association[,discard_eps]")
        parser.add_argument(
            "--processing", "-p", action="append", default=None,
            help="Processing spec: /topic:processor:[args]. Can be repeated.")
        parser.add_argument(
            "--cpu-percentage", type=float, default=80.0,
            help="CPU usage for parallel processing")
        parser.add_argument(
            "--config", type=str,
            help="Path to config JSON (overrides other args)")
        parser.add_argument(
            "--gui", action="store_true",
            help="Launch GUI instead of CLI")
        parser.add_argument(
            "--install-routine", type=str, default=None,
            help="Imports a custom routine from a file. See documentation for details.")
        parser.add_argument(
            "--install-processor", type=str, default=None,
            help="Imports a custom processor from a file. See documentation for details.")
        parser.add_argument(
            "--uninstall-routine", action="store_true",
            help="Removes a routine interactively.")
        parser.add_argument(
            "--uninstall-processor", action="store_true",
            help="Removes a processor interactively.")
        parser.add_argument(
            "--use-routine", type=str, default=None,
            help="Use a routine without installing it. See documentation for details.")
        parser.add_argument(
            "--use-processor", type=str, default=None,
            help="Use a processor without installing it. See documentation for details.")

    def main(self, parser, args):
        """
        Main entry point for the export command. Handles installation, uninstallation, GUI, and CLI modes.

        Args:
            parser: Argument parser object.
            args: Parsed command-line arguments.

        Returns:
            int or None: Return code or None if running GUI.
        """

        # Handle routine or processor installation
        if args.install_routine is not None:
            self.install_routine(args.install_routine)
            return
        if args.install_processor is not None:
            self.install_processor(args.install_processor)
            return
        if args.uninstall_routine:
            self.uninstall_interactive()
            return
        if args.uninstall_processor:
            self.uninstall_interactive(routine=False)
        
        # Handle routine or processor usage
        if args.use_routine is not None:
            self.use_routine_or_processor(args.use_routine)
        if args.use_processor is not None:
            self.use_routine_or_processor(args.use_processor)

        # Start GUI or CLI based on arguments
        if args.gui or (args.bag is None and args.export is None and args.config is None):
            return self._run_gui()
        else:
            return self._run_cli(args)

    def _run_gui(self):
        """
        Launch the GUI application for exporting ROS2 bag data.

        Args:
            None

        Returns:
            int: Exit code from the Qt application.
        """
        def qt_exception_hook(exctype, value, traceback):
            QMessageBox.critical(None, "Unhandled Exception",
                                 f"{exctype.__name__}: {value}")
            sys.__excepthook__(exctype, value, traceback)

        sys.excepthook = qt_exception_hook
        app = QtWidgets.QApplication(sys.argv)
        window = UnbagApp()
        window.show()
        return app.exec()

    def _run_cli(self, args):
        """
        Run the export process in CLI mode using the provided arguments.

        Args:
            args: Parsed command-line arguments.

        Returns:
            int: Exit code (0 for success).
        """
        if not args.bag:
            sys.exit("Error: No bag file provided. Use 'ros2 unbag <bag_path>' or --gui for GUI mode.")
    
        if not os.path.exists(args.bag):
            sys.exit(f"Error: Bag file '{args.bag}' not found.")

        bag_reader = BagReader(args.bag)
        if args.config:
            with open(args.config, "r") as f:
                config = json.load(f)
        else:
            config = self._validate_and_build_config(args, bag_reader)

        global_config = config.pop("__global__", {"cpu_percentage": 80.0})
        Exporter(bag_reader, config, global_config, progress_callback=self.progress).run()
        print("Export complete.")
        return 0

    def progress(self, current, total):
        """
        Update the progress bar with the current progress.

        Args:
            current: Current progress value.
            total: Total value for progress calculation.

        Returns:
            None
        """
        if not hasattr(self, "_pbar"):
            self._pbar = tqdm(total=total)
        self._pbar.n = current
        self._pbar.refresh()
        if current == total:
            self._pbar.close()
            del self._pbar

    def _validate_and_build_config(self, args, bag_reader):
        """
        Validate CLI arguments and build the export configuration dictionary.

        Args:
            args: Parsed command-line arguments.
            bag_reader: BagReader instance for the ROS2 bag.

        Returns:
            dict: Configuration dictionary for export.
        """
        config = {}
        config["__global__"] = {"cpu_percentage": args.cpu_percentage}
        for spec in args.export or []:
            parts = spec.split(":")
            if len(parts) < 2:
                sys.exit(f"Invalid --export: {spec}")
            topic, fmt = parts[0], parts[1]
            subdir = parts[2] if len(parts) > 2 else ""
            if topic not in bag_reader.topic_types:
                sys.exit(f"Topic {topic} not found in bag.")
            config[topic] = {
                "format": fmt,
                "path": args.output_dir or ".",
                "subfolder": subdir.strip("/"),
                "naming": args.naming.strip()
            }

        if args.resample:
            try:
                topic_spec, param_str = args.resample.split(":", 1)
                parts = param_str.split(",")
                association = parts[0]
                discard_eps = float(parts[1]) if len(parts) > 1 else None

                if association not in ("last", "nearest"):
                    sys.exit("association must be 'last' or 'nearest'")
                if association == "nearest" and discard_eps is None:
                    sys.exit("nearest requires discard_eps")
                if topic_spec not in config:
                    sys.exit(f"Resample topic {topic_spec} not in --export")

                config["__global__"]["resample_config"] = {
                    "master_topic": topic_spec,
                    "association": association,
                    "discard_eps": discard_eps
                }
            except Exception:
                sys.exit(f"Invalid --resample: {args.resample}")

        if args.processing:
            for spec in args.processing:
                parts = spec.split(":")
                if len(parts) not in (2, 3):
                    sys.exit(f"Invalid --processing: {spec}")
                topic, processor = parts[0], parts[1]
                if topic not in config:
                    sys.exit(f"Processing topic {topic} not in --export")
                config[topic]['processor'] = processor
                if len(parts) == 3:
                    arg_list = parts[2].split(",")
                    processor_args = {}
                    for arg in arg_list:
                        if "=" not in arg:
                            sys.exit(f"Invalid processor arg: {arg}")
                        k, v = arg.split("=", 1)
                        processor_args[k.strip()] = v.strip()
                    config[topic]['processor_args'] = processor_args

        return config
    
    def install_routine(self, path):
        """
        Install a custom routine from the specified Python file.

        Args:
            path: Path to the Python file to install.

        Returns:
            None
        """
        # Determine destination directory
        routines_dir = os.path.dirname(ros2_unbag.core.routines.__file__)
        dest = os.path.join(routines_dir, os.path.basename(path))

        import_successfull = self.import_file(path, routines_dir)
        if not import_successfull:
            sys.exit(f"Error importing routine from {path}")
        else:
            print(f"Imported routine from {path}")

    def install_processor(self, path):
        """
        Install a custom processor from the specified Python file.

        Args:
            path: Path to the Python file to install.

        Returns:
            None
        """
        # Determine destination directory
        processors_dir = os.path.dirname(ros2_unbag.core.processors.__file__)
        dest = os.path.join(processors_dir, os.path.basename(path))

        import_successfull = self.import_file(path, processors_dir)
        if not import_successfull:
            sys.exit(f"Error importing processor from {path}")
        else:
            print(f"Imported processor from {path}")

    def import_file(self, path, dest_dir):
        """
        Copy a Python file to the destination directory for installation.

        Args:
            path: Path to the source Python file.
            dest_dir: Destination directory for installation.

        Returns:
            bool: True if import succeeded, False otherwise.
        """
        if not os.path.exists(path):
            sys.exit(f"Error: File '{path}' not found.")
        if not path.endswith(".py"):
            sys.exit("Only Python files are supported for installation. See exact format in documentation.")
        if not os.path.isdir(dest_dir):
            sys.exit(f"Error: Destination directory '{dest_dir}' does not exist.")
        dest = os.path.join(dest_dir, os.path.basename(path))

        if os.path.exists(dest):
            sys.exit(f"File already exists: {os.path.basename(path)}")

        with open(path, "r") as src_file, open(dest, "w") as dest_file:
            dest_file.write(src_file.read())
        
        return True
    
    def uninstall_interactive(self, routine=True):
        """
        Interactively uninstall a routine or processor.

        Args:
            routine: If True, uninstall a routine; if False, uninstall a processor.

        Returns:
            None
        """
        if routine:
            dir = os.path.dirname(ros2_unbag.core.routines.__file__)
        else:
            dir = os.path.dirname(ros2_unbag.core.processors.__file__)
        files = [f for f in os.listdir(dir) if f.endswith(".py") and f != "__init__.py" and f != "base.py" and f != "default.py"]

        if not files:
            print("No routines to uninstall.")
            return

        print("Available routines:")
        for i, f in enumerate(files, 1):
            print(f"{i}. {f}")

        choice = input("Enter number to uninstall (or press Enter to cancel): ").strip()
        if not choice:
            print("Cancelled.")
            return

        try:
            idx = int(choice) - 1
            selected_file = files[idx]
            os.remove(os.path.join(dir, selected_file))
            print(f"Uninstalled routine '{selected_file}'")
        except (IndexError, ValueError):
            print("Invalid selection.")
    
    def use_routine_or_processor(self, path):
        """
        Dynamically import and use a routine or processor from the specified Python file.

        Args:
            path: Path to the Python file to import.

        Returns:
            None
        """
        if not os.path.exists(path):
            sys.exit(f"Error: File '{path}' not found.")
        if not path.endswith(".py"):
            sys.exit("Only Python files are supported for use. See exact format in documentation.")

        spec = importlib.util.spec_from_file_location("temp", path)
        module = importlib.util.module_from_spec(spec)
        sys.modules["temp"] = module
        spec.loader.exec_module(module)