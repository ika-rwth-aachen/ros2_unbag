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

from collections import OrderedDict
import csv
import json
from pathlib import Path

from rosidl_runtime_py import message_to_ordereddict
import yaml
from yaml import CSafeDumper

from ros2_unbag.core.routines.base import ExportRoutine, ExportMode, ExportMetadata
from ros2_unbag.core.utils.file_utils import get_time_from_msg


class _RosYamlDumper(CSafeDumper):
    """Serialize ROS field mappings with LibYAML while preserving field order."""


def _represent_ordered_mapping(dumper, data):
    """
    Represent ordered ROS fields as a standard YAML mapping, without sorting.

    Args:
        dumper: YAML dumper used to represent the mapping.
        data: Ordered dictionary containing ROS message fields.

    Returns:
        yaml.nodes.MappingNode: YAML mapping with the original field order.
    """
    return dumper.represent_dict(data.items())


def _represent_yaml_string(dumper, data):
    """
    Quote Unicode line separators so YAML parsing preserves their values.

    Args:
        dumper: YAML dumper used to represent the string.
        data: String value to serialize.

    Returns:
        yaml.nodes.ScalarNode: YAML string scalar with Unicode separators escaped.
    """
    style = '"' if any(separator in data for separator in ("\x85", "\u2028", "\u2029")) else None
    return dumper.represent_scalar("tag:yaml.org,2002:str", data, style=style)


_RosYamlDumper.add_representer(OrderedDict, _represent_ordered_mapping)
_RosYamlDumper.add_representer(str, _represent_yaml_string)


@ExportRoutine.set_catch_all(["text/json", "text/yaml", "table/csv"], mode=ExportMode.MULTI_FILE)
def export_generic_multi_file(msg, path: Path, fmt: str, metadata: ExportMetadata):
    """
    Generic export handler supporting JSON, YAML, and CSV formats. 
    Serialize the message, determine file extension, and save to the given path.

    Args:
        msg: ROS message instance to export.
        path: Output file path (without extension).
        fmt: Export format string ("text/yaml", "text/json", "table/csv").
        metadata: Export metadata including message index and max index.

    Returns:
        None
    """
    timestamp = get_time_from_msg(msg, return_datetime=True)

    if fmt == "text/json":
        payload = _serialize_message_with_timestamp(msg, "json", timestamp)
        file_ending = ".json"
    elif fmt == "text/yaml":
        payload = _serialize_message_with_timestamp(msg, "yaml", timestamp)
        file_ending = ".yaml"
    elif fmt == "table/csv":
        payload = _serialize_message_with_timestamp(msg, "csv", timestamp)
        file_ending = ".csv"

    # Save the serialized message to a file
    with open(path.with_suffix(file_ending), "w") as f:
        _write_line(f, payload, fmt, True, True)


@ExportRoutine.set_catch_all(["text/json", "text/yaml", "table/csv"], mode=ExportMode.SINGLE_FILE)
def export_generic_single_file(msg, path: Path, fmt: str, metadata: ExportMetadata):
    """
    Generic export handler supporting JSON, YAML, and CSV formats.
    Serialize the message, determine file extension, and append to the given path with file locking (precaution).

    Args:
        msg: ROS message instance to export.
        path: Output file path (without extension).
        fmt: Export format string ("text/yaml", "text/json", "table/csv").
        metadata: Export metadata including message index and max index.

    Returns:
        None
    """
    timestamp = get_time_from_msg(msg, return_datetime=True)

    if fmt == "text/json":
        payload = _serialize_message_with_timestamp(msg, "json", timestamp)
        file_ending = ".json"
    elif fmt == "text/yaml":
        payload = _serialize_message_with_timestamp(msg, "yaml", timestamp)
        file_ending = ".yaml"
    elif fmt == "table/csv":
        payload = _serialize_message_with_timestamp(msg, "csv", timestamp)
        file_ending = ".csv"

    # Determine if this is the first or last message for the file
    is_first = metadata.index == 0
    is_last = metadata.index == metadata.max_index

    # Save the serialized message to a file - if the filename is constant, messages will be appended
    with open(path.with_suffix(file_ending), "a+") as f:
        if is_first:
            # clear the file if this is the first message
            f.seek(0)
            f.truncate()
        # Write payload line to the file
        _write_line(f, payload, fmt, is_first, is_last)


def _serialize_message_with_timestamp(msg, fmt, timestamp):
    """
    Serialize a ROS message to the specified format.

    Args:
        msg: ROS message instance to serialize.
        fmt: Export format string ("yaml", "json", "csv").
        timestamp: Timestamp to include in the serialized output.

    Returns:
        str: Serialized message as a string.
    """
    if fmt == "json":
        message_dict = message_to_ordereddict(msg)
        serialized_line = json.dumps(message_dict, default=str)
        serialized_line_with_timestamp = f'"{timestamp.isoformat()}": {serialized_line}'
        return serialized_line_with_timestamp
    elif fmt == "yaml":
        # LibYAML's line-width setting is a signed 32-bit integer.
        yaml_content = yaml.dump(
            message_to_ordereddict(msg), Dumper=_RosYamlDumper,
            allow_unicode=True, width=2**31 - 1, default_flow_style=False,
        )
        # Split only emitted LF lines: splitlines() also consumes Unicode line
        # separators inside scalars. Remove just the document's final newline.
        indented_yaml_content = "\n".join(f"  {line}" for line in yaml_content.removesuffix("\n").split("\n"))
        serialized_line_with_timestamp = f"{timestamp}:\n{indented_yaml_content}"
        return serialized_line_with_timestamp
    elif fmt == "csv":
        flat_data = _flatten(message_to_ordereddict(msg))
        header = ["timestamp", *flat_data.keys()]
        values = [str(timestamp), *flat_data.values()]
        return [header, values]


def _write_line(file, line, filetype, is_first, is_last):
    """
    Write a serialized message line to the file.
    For JSON/YAML, write the string; for CSV, ensure header and write the row.

    Args:
        file: File object to write to.
        line: String for JSON/YAML, or [header, values] list for CSV.
        filetype: Export format string.
        is_first: Boolean indicating if this is the first message for the file.
        is_last: Boolean indicating if this is the last message for the file.

    Returns:
        None
    """

    # Simple writing for yaml
    if "text/yaml" in filetype:
        file.write(line)
        file.write("\n")

    # Writing for json - include parentheses before first line and after last line
    elif "text/json" in filetype:
        if is_first:
            file.write("{\n")
        file.write(line)
        if is_last:
            file.write("\n}\n")
        else:
            file.write(",\n")

    # Writing for csv - include header only for the first line
    if "table/csv" in filetype:
        if is_first:
            _add_csv_header(file, line[0])
        writer = csv.writer(file)
        writer.writerow(line[1])   

    file.flush()


def _add_csv_header(file, header):
    """
    Ensure the CSV file starts with the correct header.

    Args:
        file: File object to write to.
        header: List of column names for the CSV header.

    Returns:
        None
    """
    file.seek(0)
    file.truncate()
    writer = csv.writer(file)
    writer.writerow(header)


def _flatten(d, parent_key='', sep='.'):
    """
    Flatten a nested dict into a single-level dict with compound keys separated by sep.

    Args:
        d: Dictionary to flatten.
        parent_key: Prefix for keys (used in recursion).
        sep: Separator string for compound keys.

    Returns:
        dict: Flattened dictionary.
    """
    items = []
    for k, v in d.items():
        new_key = f"{parent_key}{sep}{k}" if parent_key else k
        if isinstance(v, dict):
            items.extend(_flatten(v, new_key, sep=sep).items())
        else:
            items.append((new_key, v))
    return dict(items)
