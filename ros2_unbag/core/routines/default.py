import fcntl
import json
import csv
from rosidl_runtime_py import message_to_ordereddict, message_to_yaml

from ros2_unbag.core.routines.base import ExportRoutine


@ExportRoutine.set_catch_all(["text/json", "text/yaml", "text/csv"])
def export_generic(msg, path, fmt="text/json"):
    # Generic export handler for any message type, supports JSON and YAML

    if fmt == "text/json":
        serialized = message_to_ordereddict(msg)
        serialized_line = json.dumps(serialized, default=str) + "\n"
        file_ending = ".json"
    elif fmt == "text/yaml":
        serialized_line = message_to_yaml(msg) + "---\n"
        file_ending = ".yaml"
    elif fmt in ["text/csv", "table/csv"]:
        flat_data = flatten(message_to_ordereddict(msg))
        values = list(flat_data.values())
        header = list(flat_data.keys())
        file_ending = ".csv"

    # Save the serialized message to a file - if the filename is constant, messages will be appended
    with open(path + file_ending, "a+") as f:
        while True:
            try:
                fcntl.flock(f, fcntl.LOCK_EX)
                write_line(f, serialized_line if fmt != "text/csv" else [header, values], fmt)
                fcntl.flock(f, fcntl.LOCK_UN)
                break
            except BlockingIOError:
                continue    #retry if the file is locked by another process

def write_line(file, line, filetype):
    # Simple writing for json and yaml
    if filetype == "text/json" or filetype == "text/yaml":
        file.write(line)

    # Special handling for CSV
    if filetype == "text/csv":
        add_csv_header(file, line[0])
        writer = csv.writer(file)
        writer.writerow(line[1])   

    file.flush()

def add_csv_header(file, header):
    file.seek(0)
    reader = csv.reader(file)
    first_row = next(reader, None)
    if first_row != header:
        file.seek(0)
        content = file.read()
        file.seek(0)
        file.truncate()
        writer = csv.writer(file)
        writer.writerow(header)
        if content:
            file.write(content)
    file.seek(0, 2)

def flatten(d, parent_key='', sep='.'):
    items = []
    for k, v in d.items():
        new_key = f"{parent_key}{sep}{k}" if parent_key else k
        if isinstance(v, dict):
            items.extend(flatten(v, new_key, sep=sep).items())
        else:
            items.append((new_key, v))
    return dict(items)