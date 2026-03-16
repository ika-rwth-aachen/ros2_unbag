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
import logging
import os
import sys
from typing import Optional, Sequence

from PySide6 import QtWidgets
from PySide6.QtWidgets import QMessageBox
from tqdm import tqdm

from ros2_unbag.core.bag_reader import BagReader
from ros2_unbag.core.exporter import Exporter
from ros2_unbag.core.routines.base import ExportRoutine, ExportMode
from ros2_unbag.core.utils.bag_utils import resolve_bag_path
import ros2_unbag.core.processors
import ros2_unbag.core.routines
from ros2_unbag.ui.main_window import UnbagApp

logger = logging.getLogger(__name__)

# ros2cli is optional
try:
    from ros2cli.command import CommandExtension
except Exception:
    class CommandExtension:
        pass

class ExportCommand(CommandExtension):

    def _configure_logging(self):
        """
        Configure CLI logging once for this process.

        Args:
            None

        Returns:
            None
        """
        logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(message)s')

    def add_arguments(self, parser, cli_name):
        """
        Add command-line arguments for the export command.

        Args:
            parser: Argument parser object.
            cli_name: Name of the CLI command.

        Returns:
            None
        """
        parser.add_argument("bag", nargs="?", help="Path to ROS2 bag file (.db3/.mcap) or split bag folder")
        parser.add_argument(
            "--export", "-e", action="append",
            help="Export spec: /topic:format[:subdir]. Can be repeated.")
        parser.add_argument("--output-dir", "-o", help="Base output directory")
        parser.add_argument(
            "--naming", default=None,
            help="Naming pattern. Supports %%name, %%index, %%timestamp, %%master_timestamp (when resampling), and strftime (e.g. `%%Y-%%m-%%d_%%H-%%M-%%S`) using ROS timestamps. Defaults to %%name for single-file routines and %%name_%%index for multi-file routines.")
        parser.add_argument(
            "--resample",
            help="Optional resampling: /master_topic:association[,discard_eps]")
        parser.add_argument(
            "--processing", "-p", action="append", default=None,
            help="Processing spec: /topic:processor[:arg=value,…]. Repeat to build processor chains; order matters.")
        parser.add_argument(
            "--cpu-percentage", type=float, default=None,
            help="CPU usage for parallel processing")
        parser.add_argument(
            "--config", type=str,
            help="Path to config JSON (used as base; explicitly provided CLI flags override it)")
        parser.add_argument(
            "--gui", action="store_true",
            help="Launch GUI instead of CLI")
        parser.add_argument(
            "--continue-on-error", action="store_true",
            help="Continue export when a processor/routine fails for an item; logs and skips failed items.")
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
        self._configure_logging()

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
            return
        
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

        try:
            bag_path, _ = resolve_bag_path(args.bag)
        except (FileNotFoundError, ValueError) as e:
            sys.exit(f"Error: {e}")

        bag_reader = BagReader(bag_path)
        if args.config:
            with open(args.config, "r") as f:
                base_config = json.load(f)
            config = self._merge_cli_into_config(args, bag_reader, base_config)
        else:
            config = self._build_config_from_cli(args, bag_reader)

        config = self._validate_effective_config(config, bag_reader)

        global_config = config.pop("__global__", {"cpu_percentage": 80.0, "continue_on_error": False})

        exporter = Exporter(bag_reader, config, global_config, progress_callback=self.progress)
        exporter.run()
        if exporter.failed_item_count > 0:
            logger.warning("Export complete with %d skipped item(s).", exporter.failed_item_count)
        else:
            logger.info("Export complete.")
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
        delta = current - self._pbar.n
        if delta > 0:
            self._pbar.update(delta)
        if current >= total:
            self._pbar.close()
            del self._pbar


    def _validate_and_build_config(self, args, bag_reader):
        """
        Backwards-compatible wrapper for legacy internal name.
        """
        return self._build_config_from_cli(args, bag_reader)

    def _build_config_from_cli(self, args, bag_reader):
        """
        Build export configuration from CLI arguments only.

        Args:
            args: Parsed command-line arguments.
            bag_reader: BagReader instance for the ROS2 bag.

        Returns:
            dict: Configuration dictionary for export.
        """
        config = {}
        config["__global__"] = {
            "cpu_percentage": 80.0 if args.cpu_percentage is None else args.cpu_percentage,
            "continue_on_error": bool(args.continue_on_error),
        }
        self._apply_export_overrides(
            config=config,
            bag_reader=bag_reader,
            export_specs=args.export,
            output_dir=args.output_dir,
            naming=args.naming,
            preserve_existing_for_omitted_fields=False,
        )
        self._apply_processing_overrides(
            config=config,
            processing_specs=args.processing,
            reset_per_topic=False,
            missing_topic_error="Processing topic {topic} not in --export",
        )
        self._apply_resample_override(
            global_cfg=config["__global__"],
            config=config,
            resample_spec=args.resample,
            missing_topic_error="Resample topic {topic} not in --export",
        )

        return config

    def _apply_cli_overrides_to_config(self, args, bag_reader, config):
        """
        Backwards-compatible wrapper for legacy internal name.
        """
        return self._merge_cli_into_config(args, bag_reader, config)

    def _merge_cli_into_config(self, args, bag_reader, config):
        """
        Merge config-file settings with explicitly provided CLI flags.

        The config file provides the baseline. CLI options only override fields
        when those options are explicitly provided by the user.

        Args:
            args: Parsed command-line arguments.
            bag_reader: BagReader instance for topic validation.
            config: Configuration loaded from JSON file.

        Returns:
            dict: Effective merged configuration.
        """
        if not isinstance(config, dict):
            sys.exit("Error: Config JSON must contain a dictionary at the top level.")

        config = dict(config)
        global_cfg = config.get("__global__")
        if not isinstance(global_cfg, dict):
            global_cfg = {}
            config["__global__"] = global_cfg

        # 1) Global scalar overrides
        if args.cpu_percentage is not None:
            global_cfg["cpu_percentage"] = args.cpu_percentage
        else:
            global_cfg.setdefault("cpu_percentage", 80.0)
        if args.continue_on_error:
            global_cfg["continue_on_error"] = True
        else:
            global_cfg.setdefault("continue_on_error", False)

        # 2) Export overrides (format/subfolder and topic creation)
        self._apply_export_overrides(
            config=config,
            bag_reader=bag_reader,
            export_specs=args.export,
            output_dir=args.output_dir,
            naming=args.naming,
            preserve_existing_for_omitted_fields=True,
        )

        # 3) Global topic-scoped overrides
        selected_topics = [t for t in config if t != "__global__" and isinstance(config[t], dict)]
        if args.output_dir is not None:
            for topic in selected_topics:
                config[topic]["path"] = args.output_dir
        if args.naming is not None:
            naming = args.naming.strip()
            if not naming:
                sys.exit("Invalid --naming: value cannot be empty.")
            for topic in selected_topics:
                config[topic]["naming"] = naming

        # 4) Processing overrides
        self._apply_processing_overrides(
            config=config,
            processing_specs=args.processing,
            reset_per_topic=True,
            missing_topic_error="Processing topic {topic} not found in merged config.",
        )

        # 5) Resampling override
        self._apply_resample_override(
            global_cfg=global_cfg,
            config=config,
            resample_spec=args.resample,
            missing_topic_error="Resample topic {topic} not found in merged config.",
        )

        return config

    def _validate_effective_config(self, config, bag_reader):
        """
        Validate and normalize the final effective configuration before export.

        Args:
            config: Effective config dictionary (from CLI build or config+CLI merge).
            bag_reader: BagReader instance for topic/type lookups.

        Returns:
            dict: Validated and normalized configuration.
        """
        if not isinstance(config, dict):
            sys.exit("Error: Effective config must be a dictionary.")

        effective = dict(config)
        global_cfg = effective.get("__global__")
        if not isinstance(global_cfg, dict):
            global_cfg = {}
            effective["__global__"] = global_cfg

        if "cpu_percentage" in global_cfg:
            try:
                global_cfg["cpu_percentage"] = float(global_cfg["cpu_percentage"])
            except (TypeError, ValueError):
                sys.exit("Invalid __global__.cpu_percentage: must be a number.")
        else:
            global_cfg["cpu_percentage"] = 80.0
        global_cfg["continue_on_error"] = bool(global_cfg.get("continue_on_error", False))

        selected_topics = [
            topic for topic, cfg in effective.items()
            if topic != "__global__" and isinstance(cfg, dict)
        ]

        for topic in selected_topics:
            cfg = effective[topic]
            fmt = cfg.get("format")
            if not fmt:
                sys.exit(f"Missing required field 'format' for topic '{topic}'.")

            canonical_fmt, mode = self._resolve_format(topic, fmt, bag_reader)
            cfg["format"] = canonical_fmt
            cfg.setdefault("path", ".")
            cfg.setdefault("subfolder", "")
            cfg["subfolder"] = str(cfg.get("subfolder", "")).strip("/")

            naming = cfg.get("naming")
            if naming is None:
                cfg["naming"] = self._default_naming_for_mode(mode)
            else:
                naming = str(naming).strip()
                if not naming:
                    sys.exit(f"Invalid naming for topic '{topic}': value cannot be empty.")
                cfg["naming"] = naming

        rcfg = global_cfg.get("resample_config")
        if rcfg is not None:
            if not isinstance(rcfg, dict):
                sys.exit("Invalid __global__.resample_config: must be a dictionary.")
            master_topic = rcfg.get("master_topic")
            if master_topic not in selected_topics:
                sys.exit(f"Master topic '{master_topic}' not found in effective config.")
            association = rcfg.get("association", "last")
            if association not in ("last", "nearest"):
                sys.exit("association must be 'last' or 'nearest'")
            discard_eps = rcfg.get("discard_eps")
            if association == "nearest" and discard_eps is None:
                sys.exit("nearest requires discard_eps")
            if discard_eps is not None:
                try:
                    discard_eps = float(discard_eps)
                except (TypeError, ValueError):
                    sys.exit("Invalid discard_eps: must be a number.")
            global_cfg["resample_config"] = {
                "master_topic": master_topic,
                "association": association,
                "discard_eps": discard_eps,
            }

        return effective

    def _apply_export_overrides(
        self,
        config,
        bag_reader,
        export_specs,
        output_dir,
        naming,
        preserve_existing_for_omitted_fields,
    ):
        """
        Apply --export based topic configuration updates.

        Args:
            config: Mutable config dictionary.
            bag_reader: BagReader instance for topic/type lookups.
            export_specs: List of --export specs or None.
            output_dir: Optional CLI output directory override.
            naming: Optional CLI naming override.
            preserve_existing_for_omitted_fields: If True, omitted --export subdir keeps existing config value.

        Returns:
            None
        """
        if not export_specs:
            return

        provided_naming = naming.strip() if naming else None
        existing_topic_configs = {
            topic: cfg for topic, cfg in config.items()
            if topic != "__global__" and isinstance(cfg, dict)
        }

        for spec in export_specs:
            topic, requested_fmt, subdir = self._parse_export_spec(spec)
            canonical_fmt, mode = self._resolve_format(topic, requested_fmt, bag_reader)
            cfg = dict(existing_topic_configs.get(topic, {}))

            cfg["format"] = canonical_fmt
            if subdir is not None:
                cfg["subfolder"] = subdir.strip("/")
            elif not preserve_existing_for_omitted_fields or "subfolder" not in cfg:
                cfg["subfolder"] = ""

            if output_dir is not None:
                cfg["path"] = output_dir
            elif "path" not in cfg:
                cfg["path"] = "."

            if provided_naming is not None:
                cfg["naming"] = provided_naming
            elif "naming" not in cfg:
                cfg["naming"] = self._default_naming_for_mode(mode)

            existing_topic_configs[topic] = cfg
            config[topic] = cfg

    def _apply_processing_overrides(self, config, processing_specs, reset_per_topic, missing_topic_error):
        """
        Apply --processing overrides to topics in config.

        Args:
            config: Mutable config dictionary.
            processing_specs: List of --processing specs or None.
            reset_per_topic: Whether to clear existing processors once per referenced topic.
            missing_topic_error: Error message template with {topic}.

        Returns:
            None
        """
        if not processing_specs:
            return

        reset_topics = set()
        for spec in processing_specs:
            topic, processor_entry = self._parse_processing_spec(spec)
            if topic not in config or topic == "__global__":
                sys.exit(missing_topic_error.format(topic=topic))
            if reset_per_topic and topic not in reset_topics:
                config[topic]["processors"] = []
                reset_topics.add(topic)
            config[topic].setdefault("processors", []).append(processor_entry)

    def _apply_resample_override(self, global_cfg, config, resample_spec, missing_topic_error):
        """
        Apply --resample override to global config.

        Args:
            global_cfg: Mutable global config dictionary.
            config: Effective topic config dictionary.
            resample_spec: CLI --resample value or None.
            missing_topic_error: Error message template with {topic}.

        Returns:
            None
        """
        if not resample_spec:
            return

        topic_spec, association, discard_eps = self._parse_resample_spec(resample_spec)
        if topic_spec not in config or topic_spec == "__global__":
            sys.exit(missing_topic_error.format(topic=topic_spec))
        global_cfg["resample_config"] = {
            "master_topic": topic_spec,
            "association": association,
            "discard_eps": discard_eps,
        }

    def _parse_export_spec(self, spec):
        """
        Parse a --export spec.

        Args:
            spec: Raw --export value.

        Returns:
            tuple[str, str, str | None]: topic, requested format, optional subdir.
        """
        parts = spec.split(":")
        if len(parts) < 2:
            sys.exit(f"Invalid --export: {spec}")
        topic = parts[0]
        fmt = parts[1]
        subdir = parts[2] if len(parts) > 2 else None
        return topic, fmt, subdir

    def _resolve_format(self, topic, fmt, bag_reader):
        """
        Resolve CLI format token to canonical format string used in config.

        Args:
            topic: Topic name.
            fmt: Requested format token.
            bag_reader: BagReader instance.

        Returns:
            tuple[str, ExportMode]: canonical format token and resolved export mode.
        """
        if topic not in bag_reader.topic_types:
            sys.exit(f"Topic {topic} not found in bag.")
        topic_type = bag_reader.topic_types[topic]
        resolution = ExportRoutine.resolve(topic_type, fmt)
        if resolution is None:
            sys.exit(f"No export routine found for topic type '{topic_type}' with format '{fmt}'.")
        _, canonical_fmt, mode = resolution
        available_modes = set(ExportRoutine.get_modes_for_format(topic_type, canonical_fmt))
        if mode == ExportMode.SINGLE_FILE and len(available_modes) > 1:
            return f"{canonical_fmt}@single_file", mode
        if mode == ExportMode.MULTI_FILE and len(available_modes) > 1 and "@" in fmt:
            return f"{canonical_fmt}@multi_file", mode
        return canonical_fmt, mode

    def _default_naming_for_mode(self, mode):
        """
        Return default naming pattern for an export mode.

        Args:
            mode: Resolved export mode.

        Returns:
            str: Default naming pattern.
        """
        if mode == ExportMode.SINGLE_FILE:
            return "%name"
        return "%name_%index"

    def _parse_processing_spec(self, spec):
        """
        Parse a --processing spec.

        Args:
            spec: Raw --processing value.

        Returns:
            tuple[str, dict]: topic and processor entry dictionary.
        """
        parts = spec.split(":")
        if len(parts) not in (2, 3):
            sys.exit(f"Invalid --processing: {spec}")
        topic = parts[0]
        processor = parts[1]
        processor_entry = {"name": processor}
        if len(parts) == 3:
            arg_list = parts[2].split(",")
            processor_args = {}
            for arg in arg_list:
                if "=" not in arg:
                    sys.exit(f"Invalid processor arg: {arg}")
                key, value = arg.split("=", 1)
                processor_args[key.strip()] = value.strip()
            processor_entry["args"] = processor_args
        return topic, processor_entry

    def _parse_resample_spec(self, spec):
        """
        Parse and validate a --resample spec.

        Args:
            spec: Raw --resample value.

        Returns:
            tuple[str, str, float | None]: master topic, association, discard epsilon.
        """
        try:
            topic_spec, param_str = spec.split(":", 1)
            parts = param_str.split(",")
            association = parts[0]
            discard_eps = float(parts[1]) if len(parts) > 1 else None
        except ValueError:
            sys.exit(f"Invalid --resample: {spec}")

        if association not in ("last", "nearest"):
            sys.exit("association must be 'last' or 'nearest'")
        if association == "nearest" and discard_eps is None:
            sys.exit("nearest requires discard_eps")
        return topic_spec, association, discard_eps
    

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

        import_successful = self.import_file(path, routines_dir)
        if not import_successful:
            sys.exit(f"Error importing routine from {path}")
        else:
            logger.info("Imported routine from %s", path)


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

        import_successful = self.import_file(path, processors_dir)
        if not import_successful:
            sys.exit(f"Error importing processor from {path}")
        else:
            logger.info("Imported processor from %s", path)


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
        base_dir = os.path.dirname(
            ros2_unbag.core.routines.__file__ if routine else ros2_unbag.core.processors.__file__
        )
        files = [f for f in os.listdir(base_dir) if f.endswith(".py") and f != "__init__.py" and f != "base.py" and f != "default.py"]

        label = "routines" if routine else "processors"
        if not files:
            logger.warning("No %s to uninstall.", label)
            return
        print(f"Available {label}:")
        for i, f in enumerate(files, 1):
            print(f"{i}. {f}")

        choice = input("Enter number to uninstall (or press Enter to cancel): ").strip()
        if not choice:
            logger.warning("Cancelled.")
            return

        try:
            idx = int(choice) - 1
            if idx < 0 or idx >= len(files):
                logger.warning("Invalid selection.")
                return
            selected_file = files[idx]
            os.remove(os.path.join(base_dir, selected_file))
            print(f"Uninstalled {label[:-1]} '{selected_file}'")
        except (IndexError, ValueError):
            logger.warning("Invalid selection.")
    

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


def main(argv: Optional[Sequence[str]] = None) -> int:
    """Standalone CLI entry point for environments without ros2cli.
    Keeps behavior consistent with the ROS 2 verb by wiring argparse
    to the same ExportCommand implementation.
    """
    import argparse

    parser = argparse.ArgumentParser(
        prog="ros2_unbag",
        description=(
            "Export selected topics from ROS 2 bag files to various formats."
        ),
    )
    cmd = ExportCommand()
    cmd.add_arguments(parser, "ros2_unbag")
    args = parser.parse_args(argv)
    rc = cmd.main(parser, args)
    if isinstance(rc, int):
        return rc
    return 0
