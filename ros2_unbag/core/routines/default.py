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

import csv
from datetime import datetime
import fcntl
import json
import textwrap

from rosidl_runtime_py import message_to_ordereddict, message_to_yaml

from ros2_unbag.core.routines.base import ExportRoutine


@ExportRoutine.set_catch_all(["text/yaml", "text/json", "text/csv"])
def export_generic(msg, path, fmt="text/yaml", is_first=True):
    """
    Generic export handler supporting JSON, YAML, and CSV formats.
    Serialize the message, determine file extension, and append to the given path with file locking.

    Args:
        msg: ROS message instance to export.
        path: Output file path (without extension).
        fmt: Export format string ("text/yaml", "text/json", "text/csv").
        is_first: Boolean indicating if this is the first message for the file.

    Returns:
        None
    """
    
    # Build timestamp
    try:
        timestamp = datetime.fromtimestamp(msg.header.stamp.sec +
                                            msg.header.stamp.nanosec * 1e-9)
    except AttributeError:
        # Fallback timestamp (receive time)
        timestamp = datetime.fromtimestamp(msg.stamp.sec +
                                            msg.stamp.nanosec * 1e-9)
        
    if fmt == "text/json":
        serialized = message_to_ordereddict(msg)
        serialized_with_timestamp = {str(timestamp): serialized}
        serialized_line = json.dumps(serialized_with_timestamp, default=str) + "\n"
        file_ending = ".json"
    elif fmt == "text/yaml":
        yaml_content = message_to_yaml(msg)
        indented_yaml = textwrap.indent(yaml_content, prefix="  ")
        serialized_line = f"{timestamp}:\n{indented_yaml}\n"
        file_ending = ".yaml"
    elif fmt in ["text/csv", "table/csv"]:
        flat_data = flatten(message_to_ordereddict(msg))
        header = ["timestamp", *flat_data.keys()]
        values = [str(timestamp), *flat_data.values()]
        file_ending = ".csv"

    # Save the serialized message to a file - if the filename is constant, messages will be appended
    with open(path + file_ending, "a+") as f:
        while True:
            try:
                fcntl.flock(f, fcntl.LOCK_EX)
                if is_first:
                    # clear the file if this is the first message
                    f.seek(0)
                    f.truncate()
                # Write the serialized line to the file
                write_line(f, serialized_line if fmt != "text/csv" else [header, values], fmt, is_first)
                fcntl.flock(f, fcntl.LOCK_UN)
                break
            except BlockingIOError:
                continue    #retry if the file is locked by another process

def write_line(file, line, filetype, is_first):
    """
    Write a serialized message line to the file.
    For JSON/YAML, write the string; for CSV, ensure header and write the row.

    Args:
        file: File object to write to.
        line: String for JSON/YAML, or [header, values] list for CSV.
        filetype: Export format string.
        is_first: Boolean indicating if this is the first message for the file.

    Returns:
        None
    """

    # Simple writing for json and yaml
    if filetype == "text/json" or filetype == "text/yaml":
        file.write(line)

    # Special handling for CSV
    if filetype == "text/csv":
        if is_first:
            add_csv_header(file, line[0])
        writer = csv.writer(file)
        writer.writerow(line[1])   

    file.flush()

def add_csv_header(file, header):
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

def flatten(d, parent_key='', sep='.'):
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
            items.extend(flatten(v, new_key, sep=sep).items())
        else:
            items.append((new_key, v))
    return dict(items)