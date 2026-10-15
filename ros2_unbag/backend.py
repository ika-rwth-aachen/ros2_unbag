# MIT License

# Copyright (c) 2026 Institute for Automotive Engineering (ika), RWTH Aachen University

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
Backend Helper Module.

Provides Qt-free helper functions shared by the CLI bridge and GUI.
This module centralizes bag inspection, export format discovery, default topic
configuration, and config validation so backend rules remain authoritative and
are not duplicated in UI code.
"""

from __future__ import annotations

import copy
import inspect
from pathlib import Path
from typing import Any

from ros2_unbag.core.bag_reader import BagReader
from ros2_unbag.core.exporter import Exporter
from ros2_unbag.core.processors import Processor
from ros2_unbag.core.routines import ExportMode, ExportRoutine
from ros2_unbag.core.utils.bag_utils import resolve_bag_path


_MODE_TO_NAME = {
    ExportMode.SINGLE_FILE: "single_file",
    ExportMode.MULTI_FILE: "multi_file",
}
_NAME_TO_MODE = {value: key for key, value in _MODE_TO_NAME.items()}


def export_mode_name(mode: ExportMode) -> str:
    """
    Return the serialized mode name for an export mode enum value.

    Args:
        mode: Export mode enum value.

    Returns:
        str: Canonical string name used in bridge payloads.
    """
    return _MODE_TO_NAME[mode]


def export_mode_from_name(name: str | None) -> ExportMode | None:
    """
    Resolve a serialized mode name to an export mode enum value.

    Args:
        name: Serialized mode name or None.

    Returns:
        ExportMode | None: Matching export mode enum value, or None if absent
        or unsupported.
    """
    if not name:
        return None
    return _NAME_TO_MODE.get(name)


def canonicalize_format_selection(topic_type: str, fmt: str, mode_name: str | None = None) -> str:
    """
    Resolve a topic format selection to its canonical stored representation.

    Args:
        topic_type: ROS message type of the selected topic.
        fmt: Requested export format identifier.
        mode_name: Optional serialized export mode override.

    Returns:
        str: Canonical format string, including ``@mode`` suffix when needed.

    Raises:
        ValueError: If the format or requested mode is invalid for the topic.
    """
    resolution = ExportRoutine.resolve(topic_type, fmt)
    if resolution is None:
        raise ValueError(
            f"No export routine found for topic type '{topic_type}' with format '{fmt}'."
        )

    _, canonical_fmt, resolved_mode = resolution
    available_modes = tuple(ExportRoutine.get_modes_for_format(topic_type, canonical_fmt))

    if mode_name is not None:
        requested_mode = export_mode_from_name(mode_name)
        if requested_mode is None:
            raise ValueError(f"Unsupported export mode '{mode_name}'.")
        if requested_mode not in available_modes:
            raise ValueError(
                f"Export mode '{mode_name}' is not available for topic type '{topic_type}' and format '{canonical_fmt}'."
            )
        resolved_mode = requested_mode

    if len(available_modes) > 1:
        return f"{canonical_fmt}@{export_mode_name(resolved_mode)}"
    return canonical_fmt


def list_formats_for_topic_type(topic_type: str) -> list[dict[str, Any]]:
    """
    List available export formats and modes for a topic type.

    Args:
        topic_type: ROS message type name.

    Returns:
        list[dict[str, Any]]: Format descriptors with format names and
        supported mode names.
    """
    formats = []
    for fmt in ExportRoutine.get_formats(topic_type):
        modes = [
            export_mode_name(mode)
            for mode in ExportRoutine.get_modes_for_format(topic_type, fmt)
        ]
        formats.append({"name": fmt, "modes": modes})
    return formats


def list_processors_for_topic_type(topic_type: str) -> list[dict[str, Any]]:
    """
    List available processors and argument metadata for a topic type.

    Args:
        topic_type: ROS message type name.

    Returns:
        list[dict[str, Any]]: Processor descriptors including argument names,
        defaults, annotations, and documentation strings.
    """
    processors = []
    for name in Processor.get_formats(topic_type):
        args = []
        signature = Processor.get_args(topic_type, name) or {}
        for arg_name, (param, doc) in signature.items():
            default = None
            if param.default != inspect.Parameter.empty:
                default = param.default
            annotation = None
            if param.annotation != inspect.Parameter.empty:
                annotation = getattr(param.annotation, "__name__", str(param.annotation))
            args.append(
                {
                    "name": arg_name,
                    "required": param.default == inspect.Parameter.empty,
                    "default": default,
                    "annotation": annotation,
                    "doc": doc,
                }
            )
        processors.append({"name": name, "args": args})
    return processors


def default_topic_config(topic_type: str, base_dir: str | None = None) -> dict[str, Any]:
    """
    Build the default export configuration for a topic type.

    Args:
        topic_type: ROS message type name.
        base_dir: Optional base output directory override.

    Returns:
        dict[str, Any]: Default per-topic export configuration.
    """
    base_path = str(Path(base_dir) if base_dir else Path.cwd())
    formats = ExportRoutine.get_formats(topic_type)
    default_fmt = formats[0] if formats else ""
    naming = "%name_%index"

    if default_fmt:
        resolution = ExportRoutine.resolve(topic_type, default_fmt)
        if resolution is not None:
            _, canonical_fmt, mode = resolution
            default_fmt = canonical_fmt
            naming = "%name" if mode == ExportMode.SINGLE_FILE else "%name_%index"

    return {
        "format": default_fmt,
        "path": base_path,
        "subfolder": "%name",
        "naming": naming,
        "processors": [],
    }


def inspect_bag(bag_path: str, base_dir: str | None = None) -> dict[str, Any]:
    """
    Inspect a bag and return GUI-facing topic and format metadata.

    Args:
        bag_path: Path to a bag file or split bag directory.
        base_dir: Optional base directory used for default topic configs.

    Returns:
        dict[str, Any]: Bag metadata including topic list, message counts,
        available formats, processors, and default per-topic configs.
    """
    resolved_path, storage_id = resolve_bag_path(bag_path)
    bag_reader = BagReader(resolved_path)
    topics_by_type = bag_reader.get_topics()
    message_counts = bag_reader.get_message_count()

    topics = []
    for topic_name in sorted(bag_reader.topic_types):
        topic_type = bag_reader.topic_types[topic_name]
        topics.append(
            {
                "name": topic_name,
                "type": topic_type,
                "count": message_counts.get(topic_name, 0),
                "formats": list_formats_for_topic_type(topic_type),
                "processors": list_processors_for_topic_type(topic_type),
                "default_config": default_topic_config(topic_type, base_dir),
            }
        )

    return {
        "bag_path": resolved_path,
        "storage_id": storage_id,
        "topics_by_type": topics_by_type,
        "message_counts": message_counts,
        "topics": topics,
    }


def validate_export_config(
    bag_path: str,
    topic_configs: dict[str, Any],
    global_config: dict[str, Any] | None,
    selected_topics: list[str] | tuple[str, ...] | None = None,
    base_dir: str | None = None,
) -> tuple[dict[str, Any], dict[str, Any]]:
    """
    Normalize and validate topic/global export configuration.

    Args:
        bag_path: Path to a bag file or split bag directory.
        topic_configs: Raw per-topic configuration payload.
        global_config: Raw global configuration payload.
        selected_topics: Topics selected for export.
        base_dir: Optional base output directory override.

    Returns:
        tuple[dict[str, Any], dict[str, Any]]: Normalized per-topic and global
        export configuration dictionaries.

    Raises:
        ValueError: If topic selection, export formats, paths, naming, or
        resampling configuration are invalid.
    """
    resolved_path, _ = resolve_bag_path(bag_path)
    bag_reader = BagReader(resolved_path)

    global_cfg = copy.deepcopy(global_config or {})
    global_cfg.setdefault("cpu_percentage", 80.0)

    selected = list(selected_topics or [])
    if not selected:
        raise ValueError("Select at least one topic to export.")

    final_config: dict[str, Any] = {}
    errors = []
    base_dir_value = str(Path(base_dir) if base_dir else Path.cwd())

    for topic in selected:
        if topic not in bag_reader.topic_types:
            raw_cfg = copy.deepcopy(topic_configs.get(topic, {}) or {})
            if raw_cfg.get("optional", False) is True:
                final_config[topic] = raw_cfg
                continue
            raise ValueError(f"Topic '{topic}' not found in bag.")

        topic_type = bag_reader.topic_types[topic]
        merged = default_topic_config(topic_type, base_dir_value)
        raw_cfg = copy.deepcopy(topic_configs.get(topic, {}) or {})
        mode_name = raw_cfg.pop("mode", None)
        merged.update(raw_cfg)

        fmt = (merged.get("format") or "").strip()
        if not fmt:
            fmt = default_topic_config(topic_type, base_dir_value)["format"]
        if not fmt:
            raise ValueError(f"No export format available for topic '{topic}'.")
        merged["format"] = canonicalize_format_selection(topic_type, fmt, mode_name)

        merged["path"] = str((merged.get("path") or "").strip())
        merged["subfolder"] = str((merged.get("subfolder") or "").strip("/"))
        merged["naming"] = str((merged.get("naming") or "").strip())

        if not merged["path"]:
            errors.append(f"{topic}: Output Directory is required.")
        if not merged["naming"]:
            errors.append(f"{topic}: Naming scheme is required.")

        processors = merged.get("processors")
        if processors is None:
            merged["processors"] = []

        final_config[topic] = merged

    if "resample_config" in global_cfg:
        master_topic = (global_cfg["resample_config"] or {}).get("master_topic")
        if not master_topic or master_topic not in selected:
            raise ValueError(
                "Resampling enabled but no Master Topic selected among exported topics."
            )

    if errors:
        raise ValueError("Please fix the following per-topic settings:\n- " + "\n- ".join(errors))

    # Reuse the exporter constructor as the single source of truth for format and processor validation.
    Exporter(bag_reader, copy.deepcopy(final_config), copy.deepcopy(global_cfg))
    return final_config, global_cfg
