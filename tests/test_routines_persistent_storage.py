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

from collections import defaultdict
from pathlib import Path

from ros2_unbag.core.routines.base import ExportRoutine, ExportMode, ExportMetadata


def setup_function(_):
    ExportRoutine.registry = defaultdict(list)
    ExportRoutine.catch_all_registry = defaultdict(list)


def test_persistent_storage_is_isolated_per_topic(tmp_path: Path):
    calls = []

    @ExportRoutine.set_catch_all(["text/custom"], mode=ExportMode.SINGLE_FILE)
    def do_export(msg, path: Path, fmt: str, metadata: ExportMetadata):
        # Use the wrapper's persistent storage to maintain a per-topic counter
        store = do_export.persistent_storage
        store["count"] = store.get("count", 0) + 1
        calls.append((fmt, metadata.index, store["count"]))
        p = Path(str(path) + ".out")
        p.write_text(str(store["count"]))

    handler = ExportRoutine.get_handler("any/type", "text/custom")
    assert callable(handler)

    # Two topics, multiple invocations; counters must be independent
    md0 = ExportMetadata(index=0, max_index=3)
    handler(msg=object(), path=tmp_path / "a0", fmt="text/custom", metadata=md0, topic="/A")
    md1 = ExportMetadata(index=1, max_index=3)
    handler(msg=object(), path=tmp_path / "a1", fmt="text/custom", metadata=md1, topic="/A")

    md2 = ExportMetadata(index=2, max_index=3)
    handler(msg=object(), path=tmp_path / "b0", fmt="text/custom", metadata=md2, topic="/B")
    md3 = ExportMetadata(index=3, max_index=3)
    handler(msg=object(), path=tmp_path / "b1", fmt="text/custom", metadata=md3, topic="/B")

    # For /A: counts 1,2; for /B: counts 1,2
    assert calls == [
        ("text/custom", 0, 1),
        ("text/custom", 1, 2),
        ("text/custom", 2, 1),
        ("text/custom", 3, 2),
    ]

    assert (tmp_path / "a0.out").read_text() == "1"
    assert (tmp_path / "a1.out").read_text() == "2"
    assert (tmp_path / "b0.out").read_text() == "1"
    assert (tmp_path / "b1.out").read_text() == "2"

