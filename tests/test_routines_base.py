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
    # Reset registries before each test
    ExportRoutine.registry = defaultdict(list)
    ExportRoutine.catch_all_registry = defaultdict(list)


def test_catch_all_registration_and_queries(tmp_path: Path):
    calls = []

    @ExportRoutine.set_catch_all(["text/custom@multi_file"], mode=ExportMode.MULTI_FILE)
    def do_export(msg, path: Path, fmt: str, metadata: ExportMetadata):
        # Record a call and write a file to prove invocation
        calls.append((fmt, metadata.index))
        p = Path(str(path) + ".out")
        p.write_text("ok")

    # Formats include catch-all
    assert "text/custom@multi_file" in ExportRoutine.get_formats("any/msg")

    # Handler lookup falls back to catch-all
    handler = ExportRoutine.get_handler("any/msg", "text/custom@multi_file")
    assert callable(handler)

    # Mode is from catch-all
    assert ExportRoutine.get_mode("any/msg", "text/custom@multi_file") == ExportMode.MULTI_FILE

    # Invoke through handler (with topic to test persistent storage isolation)
    md1 = ExportMetadata(index=0, max_index=0)
    handler(msg=object(), path=tmp_path / "file1", fmt="text/custom@multi_file", metadata=md1, topic="/a")
    md2 = ExportMetadata(index=1, max_index=1)
    handler(msg=object(), path=tmp_path / "file2", fmt="text/custom@multi_file", metadata=md2, topic="/b")

    # Calls recorded
    assert calls == [("text/custom@multi_file", 0), ("text/custom@multi_file", 1)]

    # Files written
    assert (tmp_path / "file1.out").exists()
    assert (tmp_path / "file2.out").exists()

