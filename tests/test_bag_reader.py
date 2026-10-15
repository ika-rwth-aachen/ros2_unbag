# MIT License
#
# Copyright (c) 2026 Institute for Automotive Engineering (ika), RWTH Aachen University
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest


@pytest.fixture
def reader_module(monkeypatch):
    # Load the reader with isolated ROS bindings so these tests also run without ROS.
    bindings = SimpleNamespace(
        ConverterOptions=MagicMock(), StorageOptions=MagicMock(), StorageFilter=MagicMock(),
        SequentialReader=MagicMock(), SequentialCompressionReader=MagicMock(),
    )
    for name, module in {
        "rosbag2_py": bindings,
        "tf2_msgs.msg": SimpleNamespace(TFMessage=type("TFMessage", (), {})),
        "rclpy.serialization": SimpleNamespace(deserialize_message=MagicMock(return_value="decoded")),
        "rosidl_runtime_py.utilities": SimpleNamespace(get_message=MagicMock(return_value="message_type")),
    }.items():
        monkeypatch.setitem(sys.modules, name, module)
    path = Path(__file__).resolve().parents[1] / "ros2_unbag/core/bag_reader.py"
    spec = importlib.util.spec_from_file_location("compression_test_reader", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def bag_directory(tmp_path):
    def create(mode="", compression_format=""):
        segment = "bag_0.db3" + (f".{compression_format}" if mode.lower() == "file" else "")
        (tmp_path / "metadata.yaml").write_text(
            "rosbag2_bagfile_information:\n"
            "  storage_identifier: sqlite3\n"
            f"  compression_mode: '{mode}'\n"
            f"  compression_format: '{compression_format}'\n"
            f"  relative_file_paths: [{segment}]\n"
        )
        (tmp_path / segment).touch()
        return str(tmp_path)
    return create


@pytest.mark.parametrize("mode, codec", [("", ""), ("message", "zstd"), ("file", "zstd"), ("MESSAGE", "zstd"), ("FILE", "zstd"), ("message", "custom_codec")])
def test_reader_selection_and_frequency(reader_module, bag_directory, mode, codec):
    compressed = mode.lower() in ("message", "file")
    factory = reader_module.SequentialCompressionReader if compressed else reader_module.SequentialReader
    unused_factory = reader_module.SequentialReader if compressed else reader_module.SequentialCompressionReader
    native_reader = factory.return_value
    native_reader.get_all_topics_and_types.return_value = [SimpleNamespace(name="/test", type="test/Msg")]

    reader = reader_module.BagReader(bag_directory(mode, codec))
    assert reader.reader is native_reader
    unused_factory.assert_not_called()

    native_reader.has_next.side_effect = [True, True, False]
    native_reader.read_next.side_effect = [("/test", b"cdr", 0), ("/test", b"cdr", 1_000_000_000)]
    assert reader.get_topics_with_frequency() == [{"name": "/test", "type": "test/Msg", "frequency": 2.0}]
    assert factory.call_count == 2
    assert native_reader.open.call_count == 2


@pytest.mark.parametrize("method", ["read_next_message", "read_messages"])
def test_compressed_reader_output_reaches_deserialization(reader_module, bag_directory, method):
    native_reader = reader_module.SequentialCompressionReader.return_value
    native_reader.get_all_topics_and_types.return_value = [SimpleNamespace(name="/test", type="test/Msg")]
    native_reader.has_next.side_effect = [True, False]
    native_reader.read_next.return_value = ("/test", b"decompressed_cdr", 123)
    reader = reader_module.BagReader(bag_directory("message", "zstd"))

    result = reader.read_next_message() if method == "read_next_message" else next(reader.read_messages(["/test"]))

    assert result == ("/test", "decoded", 123)
    reader_module.deserialize_message.assert_called_once_with(b"decompressed_cdr", "message_type")


def test_standalone_file_uses_plain_reader(reader_module, tmp_path):
    bag = tmp_path / "bag.mcap"
    bag.touch()

    reader_module.BagReader(str(bag))

    reader_module.SequentialReader.assert_called_once()
    reader_module.SequentialCompressionReader.assert_not_called()


def test_missing_decompressor_error_preserves_codec_and_cause(reader_module, bag_directory):
    reader_module.SequentialCompressionReader.return_value.open.side_effect = RuntimeError("Couldn't initialize decompressor")

    with pytest.raises(RuntimeError, match="message-compressed bag \\(zstd\\).*Couldn't initialize decompressor"):
        reader_module.BagReader(bag_directory("message", "zstd"))
    reader_module.SequentialReader.assert_not_called()
