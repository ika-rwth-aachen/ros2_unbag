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

