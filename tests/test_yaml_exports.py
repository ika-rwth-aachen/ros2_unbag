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

"""YAML round-trip checks using real ROS messages and the C-backed dumper."""

from collections import OrderedDict
import datetime
import io
import json
import math
import random

import pytest
import yaml

pytest.importorskip("rosidl_runtime_py")
pytest.importorskip("std_msgs.msg")

from rosidl_runtime_py import message_to_ordereddict, message_to_yaml
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, String

from ros2_unbag.core.routines import default
from ros2_unbag.core.routines.base import ExportMetadata


TIMESTAMP = datetime.datetime(2026, 10, 22, 12)


def serialize_yaml(msg):
    """
    Serialize one message through the current timestamp and YAML writer path.

    Args:
        msg: ROS message to serialize.

    Returns:
        str: Complete timestamped YAML document, including its final newline.
    """
    output = io.StringIO()
    payload = default._serialize_message_with_timestamp(msg, "yaml", TIMESTAMP)
    default._write_line(output, payload, "text/yaml", True, True)
    return output.getvalue()


def legacy_yaml(msg):
    """
    Reproduce the YAML output emitted before the C-backed dumper change.

    Args:
        msg: ROS message to serialize with the original helper.

    Returns:
        str: Original timestamped YAML document for byte comparisons.
    """
    content = message_to_yaml(msg)
    return f"{TIMESTAMP}:\n" + "\n".join(f"  {line}" for line in content.splitlines()) + "\n"


@pytest.mark.parametrize("value", [
    "", "plain", "yes", "null", "true", "1", ".nan", "2026-10-22", "a: b",
    "?", "-", "#comment", "'single'", '"double"', " leading", "trailing ",
    "a\tb", "a\rb", "a\x00b", "a\x0bb", "a\x0cb", "a\x1cb", "a\x1db", "a\x1eb",
    "one\ntwo", "one\ntwo\n", "one\ntwo\n\n", "one \ntwo", "\n", "\n\n",
    "éö漢字", "emoji: 😀", "a\u0085b", "a\u2028b", "a\u2029b", "a\ufeffb",
    "a\x7fb", "a\x1bb", "a" * 10000,
])
def test_yaml_strings_roundtrip(value):
    """Preserve string values despite YAML syntax and Unicode edge cases."""
    output = serialize_yaml(String(data=value))
    assert yaml.safe_load(output)[TIMESTAMP]["data"] == value


def test_yaml_generated_strings_roundtrip():
    """Round-trip deterministic mixtures of quoting and line separators."""
    rng = random.Random(42)
    alphabet = "abc 09\n\r\t:#?-'\"[]{}é漢😀\u0085\u2028\u2029"
    for _ in range(500):
        value = "".join(rng.choices(alphabet, k=rng.randrange(1, 60)))
        assert yaml.safe_load(serialize_yaml(String(data=value)))[TIMESTAMP]["data"] == value


def test_yaml_nested_arrays_and_numeric_values():
    """Preserve nested labels, signed zero, infinities, and NaN values."""
    msg = Float64MultiArray()
    msg.layout.dim = [MultiArrayDimension(label="row\u0085label", size=2, stride=6),
                      MultiArrayDimension(label="column\u2028label", size=3, stride=3)]
    msg.data = [0.0, -0.0, float("nan"), float("inf"), -float("inf"), 1e-300]
    parsed = yaml.safe_load(serialize_yaml(msg))[TIMESTAMP]
    labels = [dim["label"] for dim in parsed["layout"]["dim"]]
    assert labels == [dim.label for dim in msg.layout.dim]
    values = parsed["data"]
    assert values[0] == 0.0 and math.copysign(1, values[0]) == 1
    assert values[1] == 0.0 and math.copysign(1, values[1]) == -1
    assert math.isnan(values[2])
    assert values[3:] == [float("inf"), -float("inf"), 1e-300]


@pytest.mark.parametrize("msg", [Imu(), Float64MultiArray(data=[i / 7.0 for i in range(10000)])])
def test_yaml_standard_numeric_output_remains_byte_identical(msg):
    """Retain the original bytes for representative numeric ROS messages."""
    assert serialize_yaml(msg).encode() == legacy_yaml(msg).encode()


def test_yaml_field_order_and_isolated_representers():
    """Preserve field order without changing PyYAML's global representers."""
    before = yaml.CSafeDumper.yaml_representers.copy()
    data = OrderedDict([("z", 1), ("a", OrderedDict([("y", "a\u0085b"), ("b", 2)]))])
    dumped = yaml.dump(data, Dumper=default._RosYamlDumper, allow_unicode=True)
    assert list(yaml.safe_load(dumped)) == ["z", "a"]
    assert list(yaml.safe_load(dumped)["a"]) == ["y", "b"]
    assert yaml.safe_load(dumped)["a"]["y"] == "a\u0085b"
    assert yaml.CSafeDumper.yaml_representers == before
    assert issubclass(default._RosYamlDumper, yaml.CSafeDumper)


def test_yaml_multifile_roundtrip(monkeypatch, tmp_path):
    """Write one complete YAML file with Unicode and trailing newlines intact."""
    monkeypatch.setattr(default, "get_time_from_msg", lambda *args, **kwargs: TIMESTAMP)
    msg = String(data="one\u0085two\u2028three\u2029four\n\n")
    default.export_generic_multi_file(msg, tmp_path / "frame", "text/yaml", None)
    content = (tmp_path / "frame.yaml").read_text()
    assert yaml.safe_load(content)[TIMESTAMP]["data"] == msg.data
    assert content.endswith("\n")


def test_yaml_singlefile_multiple_messages(monkeypatch, tmp_path):
    """Append independently readable timestamped messages to one YAML file."""
    timestamps = [TIMESTAMP + datetime.timedelta(seconds=i) for i in range(3)]
    messages = [
        String(data="one\u0085two"),
        String(data="three\u2028four"),
        String(data="five\n\n"),
    ]
    pending = iter(timestamps)
    monkeypatch.setattr(default, "get_time_from_msg", lambda *args, **kwargs: next(pending))
    for index, msg in enumerate(messages):
        default.export_generic_single_file(
            msg, tmp_path / "frames", "text/yaml",
            ExportMetadata(index=index, max_index=2),
        )
    parsed = yaml.safe_load((tmp_path / "frames.yaml").read_text())
    assert [parsed[timestamp]["data"] for timestamp in timestamps] == [msg.data for msg in messages]


def test_json_and_csv_serialization_unchanged():
    """Keep JSON and CSV serialization independent of the YAML dumper change."""
    msg = Imu()
    data = message_to_ordereddict(msg)
    expected_json = f'"{TIMESTAMP.isoformat()}": ' + json.dumps(data, default=str)
    assert default._serialize_message_with_timestamp(msg, "json", TIMESTAMP) == expected_json
    flat = default._flatten(data)
    assert default._serialize_message_with_timestamp(msg, "csv", TIMESTAMP) == [
        ["timestamp", *flat.keys()], [str(TIMESTAMP), *flat.values()]]
