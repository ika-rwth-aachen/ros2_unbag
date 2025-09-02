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

import pytest

pytest.importorskip("cv2")
import numpy as np

from pathlib import Path

from ros2_unbag.core.utils import video_utils as V


class DummyWriter:
    def __init__(self):
        self._open = True
        self.written = 0

    def isOpened(self):
        return self._open

    def write(self, frame):
        assert frame is not None
        self.written += 1

    def release(self):
        self._open = False


def test_ensure_bgr_converts_grayscale():
    import cv2
    gray = np.zeros((2, 3), dtype=np.uint8)
    bgr = V.ensure_bgr(gray)
    assert bgr.shape == (2, 3, 3)
    # Already BGR remains unchanged
    bgr2 = V.ensure_bgr(bgr)
    assert bgr2.shape == (2, 3, 3)


def test_open_writer_and_write_flow(monkeypatch, tmp_path: Path):
    # Stub underlying writer creation
    def fake_open_writer(path, fmt, size_hw, fps):
        assert fmt in V.FOURCC_MAP
        assert (path.with_suffix(V.EXT_MAP[fmt])).suffix in (".mp4", ".avi")
        return DummyWriter()

    monkeypatch.setattr(V, "_open_writer", fake_open_writer)

    ps = {}
    img = np.zeros((4, 5, 3), dtype=np.uint8)

    # First frame buffers and sets first_ts_ns
    V.write_video_frame(ps, img, ts_ns=0, path=tmp_path / "out", fmt="video/mp4")
    assert "first_ts_ns" in ps and "writer" not in ps

    # Second frame creates writer and flushes buffer
    V.write_video_frame(ps, img, ts_ns=1_000_000_000, path=tmp_path / "out", fmt="video/mp4")
    assert ps["writer"].written >= 2

    # Finalize releases writer and clears state
    V.finalize_video(ps, path=tmp_path / "out", fmt="video/mp4")
    assert "writer" not in ps
    assert "buffer" not in ps


def test_frame_size_mismatch_raises(monkeypatch, tmp_path: Path):
    def fake_open_writer(path, fmt, size_hw, fps):
        return DummyWriter()
    monkeypatch.setattr(V, "_open_writer", fake_open_writer)

    ps = {}
    img1 = np.zeros((2, 2, 3), dtype=np.uint8)
    img2 = np.zeros((3, 2, 3), dtype=np.uint8)

    V.write_video_frame(ps, img1, ts_ns=0, path=tmp_path / "v", fmt="video/avi")
    with pytest.raises(ValueError):
        V.write_video_frame(ps, img2, ts_ns=10, path=tmp_path / "v", fmt="video/avi")


def test_finalize_single_frame_creates_writer(monkeypatch, tmp_path: Path):
    created = {"count": 0}

    def fake_open_writer(path, fmt, size_hw, fps):
        created["count"] += 1
        return DummyWriter()

    monkeypatch.setattr(V, "_open_writer", fake_open_writer)

    ps = {"buffer": [np.zeros((2, 2, 3), dtype=np.uint8)], "frame_size": (2, 2)}
    V.finalize_video(ps, path=tmp_path / "f", fmt="video/mp4")
    assert created["count"] == 1
    assert "writer" not in ps
    assert ps.get("buffer") is None or len(ps.get("buffer", [])) == 0

