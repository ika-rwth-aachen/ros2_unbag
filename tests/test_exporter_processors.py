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

import pytest

from ros2_unbag.core.exporter import Exporter
from ros2_unbag.core.processors.base import Processor
from ros2_unbag.core.routines.base import ExportMode, ExportRoutine


@ExportRoutine("custom_msgs/msg/Test", ["test_fmt"], ExportMode.MULTI_FILE)
def dummy_routine(msg, path, fmt, metadata):
    return msg


@Processor("custom_msgs/msg/Test", ["proc_a"])
def proc_a(msg):
    msg["chain"].append("a")
    return msg


@Processor("custom_msgs/msg/Test", ["proc_b"])
def proc_b(msg, multiplier):
    msg["chain"].append(f"b:{multiplier}")
    return msg


class DummyBagReader:
    def __init__(self):
        self.topic_types = {"/test": "custom_msgs/msg/Test"}


def build_exporter(topic_config):
    bag_reader = DummyBagReader()
    global_config = {"cpu_percentage": 100}
    return Exporter(bag_reader, {"/test": topic_config}, global_config)


def test_exporter_builds_ordered_processor_chain():
    topic_cfg = {
        "format": "test_fmt",
        "path": ".",
        "subfolder": "",
        "naming": "%name_%index",
        "processors": [
            {"name": "proc_a"},
            {"name": "proc_b", "args": {"multiplier": "2"}},
        ],
    }
    exporter = build_exporter(topic_cfg)

    chain = exporter.topic_processors["/test"]
    assert len(chain) == 2

    names = []
    for handler, args in chain:
        names.append(handler.__name__)
    assert names == ["proc_a", "proc_b"]

    canonical = topic_cfg["processors"]
    assert canonical == [
        {"name": "proc_a", "args": {}},
        {"name": "proc_b", "args": {"multiplier": "2"}},
    ]
    assert "processor" not in topic_cfg
    assert "processor_args" not in topic_cfg


def test_exporter_supports_legacy_processor_fields():
    topic_cfg = {
        "format": "test_fmt",
        "path": ".",
        "subfolder": "",
        "naming": "%name_%index",
        "processor": "proc_b",
        "processor_args": {"multiplier": "4"},
    }
    exporter = build_exporter(topic_cfg)
    chain = exporter.topic_processors["/test"]
    assert len(chain) == 1
    handler, args = chain[0]
    assert handler.__name__ == "proc_b"
    assert args == {"multiplier": "4"}


def test_exporter_applies_processor_chain_ordered():
    topic_cfg = {
        "format": "test_fmt",
        "path": ".",
        "subfolder": "",
        "naming": "%name_%index",
        "processors": [
            {"name": "proc_a"},
            {"name": "proc_b", "args": {"multiplier": "3"}},
        ],
    }
    exporter = build_exporter(topic_cfg)
    chain = exporter.topic_processors["/test"]
    message = {"chain": []}
    for handler, args in chain:
        message = handler(msg=message, **args)
    assert message["chain"] == ["a", "b:3"]


def test_exporter_raises_for_missing_required_args():
    topic_cfg = {
        "format": "test_fmt",
        "path": ".",
        "subfolder": "",
        "naming": "%name_%index",
        "processors": [
            {"name": "proc_b"},
        ],
    }
    with pytest.raises(ValueError, match="Missing required arguments"):
        build_exporter(topic_cfg)


@pytest.fixture
def optional_topic_config():
    return {
        "/test": {
            "format": "test_fmt",
            "path": ".",
            "subfolder": "",
            "naming": "%name_%index",
            "optional": True,
        },
        "/joint_states": {"optional": True},
    }


def test_exporter_skips_missing_optional_topic(optional_topic_config, caplog):
    exporter = Exporter(DummyBagReader(), optional_topic_config, {})

    assert list(exporter.config) == ["/test"]
    assert "/test" in exporter.topic_handlers
    assert "/joint_states" not in exporter.index_map
    assert "/joint_states" in optional_topic_config
    assert caplog.messages == ["Optional topic '/joint_states' not found in bag; skipping."]


@pytest.mark.parametrize("optional_setting", [{}, {"optional": False}])
def test_exporter_rejects_missing_required_topic(optional_topic_config, optional_setting):
    optional_topic_config["/joint_states"] = optional_setting

    with pytest.raises(ValueError, match="Topic '/joint_states' not found in bag"):
        Exporter(DummyBagReader(), optional_topic_config, {})


def test_exporter_rejects_missing_optional_master(optional_topic_config):
    global_config = {"resample_config": {"master_topic": "/joint_states"}}

    with pytest.raises(ValueError, match="Resampling master topic '/joint_states' not found"):
        Exporter(DummyBagReader(), optional_topic_config, global_config)


def test_exporter_rejects_no_remaining_topics(optional_topic_config):
    del optional_topic_config["/test"]

    with pytest.raises(ValueError, match="nothing to export"):
        Exporter(DummyBagReader(), optional_topic_config, {})


def test_backend_accepts_missing_optional_topic(monkeypatch, optional_topic_config, caplog):
    from ros2_unbag import backend

    monkeypatch.setattr(backend, "resolve_bag_path", lambda path: (path, "sqlite3"))
    monkeypatch.setattr(backend, "BagReader", lambda path: DummyBagReader())

    config, _ = backend.validate_export_config(
        "bag", optional_topic_config, {}, selected_topics=list(optional_topic_config)
    )

    assert config["/joint_states"]["optional"] is True
    assert config["/test"]["optional"] is True
    assert "Optional topic '/joint_states' not found in bag; skipping." in caplog.messages
