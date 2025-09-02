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

import cv2
from pathlib import Path

# Mapping from mimetype to OpenCV FOURCC and file extension
FOURCC_MAP = {
    "video/mp4": cv2.VideoWriter_fourcc(*"mp4v"),
    "video/avi": cv2.VideoWriter_fourcc(*"XVID"),
}

EXT_MAP = {
    "video/mp4": ".mp4",
    "video/avi": ".avi",
}


def ensure_bgr(img):
    if img is None:
        raise ValueError("Decoded image is None")
    # Convert single channel to 3-channel BGR
    if len(img.shape) == 2 or (img.ndim == 3 and img.shape[2] == 1):
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    return img


def _open_writer(path: Path, fmt: str, size_hw, fps: float):
    if fmt not in FOURCC_MAP:
        raise ValueError(f"Unsupported export format: {fmt}")
    h, w = size_hw
    writer = cv2.VideoWriter(
        str(path.with_suffix(EXT_MAP[fmt])), FOURCC_MAP[fmt], float(fps), (w, h)
    )
    if not writer.isOpened():
        raise RuntimeError(
            f"Failed to open video writer for '{path.with_suffix(EXT_MAP[fmt])}'. "
            f"Codec may be unavailable (fmt='{fmt}'). Ensure OpenCV is built with FFmpeg/GStreamer, or try 'video/avi'."
        )
    return writer


def write_video_frame(ps: dict, img, ts_ns: int, path: Path, fmt: str):
    """Write a frame to a per-topic writer with auto FPS estimation and buffering.

    ps is a per-topic persistent storage dict supplied by the routine wrapper.
    """
    size_hw = img.shape[:2]

    if "frame_size" not in ps:
        ps["frame_size"] = size_hw
    if size_hw != ps["frame_size"]:
        raise ValueError("All images must have the same dimensions")

    buf = ps.setdefault("buffer", [])

    if "writer" not in ps:
        if "first_ts_ns" not in ps:
            ps["first_ts_ns"] = ts_ns
            buf.append(img)
            return

        dt_ns = ts_ns - ps["first_ts_ns"]
        fps = 30.0
        if dt_ns > 0:
            fps = max(1.0, min(240.0, 1e9 / dt_ns))

        ps["writer"] = _open_writer(path, fmt, size_hw, fps)
        for f in buf:
            ps["writer"].write(f)
        buf.clear()

    ps["writer"].write(img)


def finalize_video(ps: dict, path: Path, fmt: str):
    """Finalize the video file, handling single-frame case by opening at fallback FPS."""
    if "writer" not in ps and ps.get("buffer"):
        size_hw = ps["frame_size"]
        ps["writer"] = _open_writer(path, fmt, size_hw, 30.0)
        for f in ps["buffer"]:
            ps["writer"].write(f)
        ps["buffer"].clear()

    if "writer" in ps:
        ps["writer"].release()
        del ps["writer"]
    ps.pop("first_ts_ns", None)
    ps.pop("buffer", None)
