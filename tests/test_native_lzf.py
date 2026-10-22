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

"""Compare the native extension with the original Python reference, without needing ROS."""

import importlib.util
from pathlib import Path
import random
import shutil
import struct
import subprocess
import sys
import sysconfig
import types

import pytest


ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def native(tmp_path_factory):
    """Build the native compressor in an isolated temporary directory."""
    compiler = shutil.which("c++")
    if compiler is None or not (Path(sysconfig.get_path("include")) / "Python.h").is_file():
        pytest.skip("Building the native extension requires c++ and Python development headers")
    library = tmp_path_factory.mktemp("native-lzf") / "_native.so"
    subprocess.run([
        compiler, "-std=c++17", "-O3", "-Wall", "-Wextra", "-Wpedantic", "-shared", "-fPIC",
        "-I" + sysconfig.get_path("include"), str(ROOT / "src/native/lzf.cpp"), "-o", str(library),
    ], check=True)
    spec = importlib.util.spec_from_file_location("_native", library)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def load_utils(monkeypatch):
    """Load point cloud utilities with controlled ROS and native dependencies."""
    class PointField:
        INT8, UINT8, INT16, UINT16, INT32, UINT32, FLOAT32, FLOAT64 = range(1, 9)

    messages = types.ModuleType("sensor_msgs.msg")
    messages.PointField, messages.PointCloud2 = PointField, types.SimpleNamespace
    package = types.ModuleType("sensor_msgs")
    package.msg = messages
    monkeypatch.setitem(sys.modules, "sensor_msgs", package)
    monkeypatch.setitem(sys.modules, "sensor_msgs.msg", messages)

    def load(extension):
        # None simulates a missing required native extension.
        monkeypatch.setitem(sys.modules, "ros2_unbag._native", extension)
        spec = importlib.util.spec_from_file_location(
            "_lzf_utils_test", ROOT / "ros2_unbag/core/utils/pointcloud_utils.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    return load


# Keep the original algorithm here as a test oracle, never as a runtime fallback.
def python_lzf_compress(data: bytes) -> bytes:
    """
    Compress data using the LZF format expected by binary-compressed PCD files.

    Args:
        data: Uncompressed byte payload.

    Returns:
        bytes: LZF-compressed payload.
    """
    if not data:
        return b""

    hlog = 14
    hsize = 1 << hlog
    max_lit = 1 << 5
    max_off = 1 << 13
    max_ref = (1 << 8) + (1 << 3)
    table = [-1] * hsize
    output = bytearray()
    literals = bytearray()
    i = 0
    data_len = len(data)

    def flush_literals() -> None:
        while literals:
            chunk = literals[:max_lit]
            del literals[:max_lit]
            output.append(len(chunk) - 1)
            output.extend(chunk)

    while i < data_len:
        if i < data_len - 2:
            hval = (data[i] << 16) | (data[i + 1] << 8) | data[i + 2]
            slot = ((hval * 57321) >> 9) & (hsize - 1)
            ref = table[slot]
            table[slot] = i
            off = i - ref - 1

            if (
                ref >= 0
                and off < max_off
                and data[ref:ref + 3] == data[i:i + 3]
            ):
                flush_literals()
                match_len = 3
                max_match = min(data_len - i, max_ref)
                while match_len < max_match and data[ref + match_len] == data[i + match_len]:
                    match_len += 1

                length_code = match_len - 2
                if length_code < 7:
                    output.append((length_code << 5) | (off >> 8))
                else:
                    output.append((7 << 5) | (off >> 8))
                    output.append(length_code - 7)
                output.append(off & 0xFF)

                i += match_len
                continue

        literals.append(data[i])
        if len(literals) == max_lit:
            flush_literals()
        i += 1

    flush_literals()
    return bytes(output)


def decompress(data):
    """
    Decode an LZF stream independently of either compressor implementation.

    Args:
        data: Compressed LZF payload.

    Returns:
        bytes: Decompressed payload.
    """
    output = bytearray()
    index = 0
    while index < len(data):
        control = data[index]
        index += 1
        if control < 32:
            length = control + 1
            output.extend(data[index:index + length])
            index += length
        else:
            length = control >> 5
            reference = len(output) - ((control & 31) << 8) - 1
            if length == 7:
                length += data[index]
                index += 1
            reference -= data[index]
            index += 1
            for offset in range(length + 2):
                output.append(output[reference + offset])
    return bytes(output)


def test_native_matches_reference_and_roundtrips(native, load_utils):
    """Match the original byte stream and round-trip literals and references."""
    accelerated = load_utils(native)
    assert accelerated._lzf_compress is native.lzf_compress
    rng = random.Random(42)
    lengths = list(range(100)) + [255, 256, 263, 264, 265, 8191, 8192, 8193, 65536]
    cases = [b"a" * n for n in lengths] + [rng.randbytes(n) for n in lengths]
    cases += [bytes(rng.randrange(4) for _ in range(n)) for n in lengths]
    cases += [bytes(range(256)) * 100, b"abc" * 10000]
    # Repeated blocks near the maximum reference distance.
    for distance in (8191, 8192, 8193):
        prefix = rng.randbytes(distance)
        cases.append(prefix + prefix[:300])
    for data in cases:
        expected = python_lzf_compress(data)
        actual = accelerated._lzf_compress(data)
        assert actual == expected
        assert decompress(actual) == data


@pytest.mark.parametrize("bigendian", [False, True])
def test_full_compressed_pcd_file_is_identical(native, load_utils, tmp_path, bigendian):
    """Match complete PCD files for padded clouds of either byte order."""
    reference, accelerated = load_utils(native), load_utils(native)
    reference._lzf_compress = python_lzf_compress
    field = reference.PointField
    msg = types.SimpleNamespace(width=2, height=2, point_step=16, row_step=37,
                                is_bigendian=bigendian, fields=[])
    msg.fields = [types.SimpleNamespace(name=name, offset=offset, datatype=datatype, count=1)
                  for name, offset, datatype in [
                      ("x", 0, field.FLOAT32), ("y", 4, field.FLOAT32),
                      ("z", 8, field.FLOAT32), ("intensity", 12, field.UINT16)]]
    data = bytearray([0xCD]) * (msg.height * msg.row_step)
    for i in range(4):
        struct.pack_into((">" if bigendian else "<") + "fffH", data,
                         (i // 2) * msg.row_step + (i % 2) * msg.point_step,
                         i + 0.25, -i - 0.5, i * 2.5, 100 + i)
    msg.data = bytes(data)
    expected, actual = tmp_path / "python.pcd", tmp_path / "native.pcd"
    reference.write_pointcloud_pcd(msg, expected, "binary_compressed")
    accelerated.write_pointcloud_pcd(msg, actual, "binary_compressed")
    assert actual.read_bytes() == expected.read_bytes()


@pytest.mark.parametrize("data", [None, "text", bytearray(b"mutable"), memoryview(b"view"), [1, 2]])
def test_native_rejects_non_bytes(native, data):
    """Reject inputs that are unsafe to access while the GIL is released."""
    with pytest.raises(TypeError):
        native.lzf_compress(data)


def test_missing_extension_fails_clearly(load_utils):
    """Require the native extension instead of silently falling back to Python."""
    with pytest.raises(ImportError):
        load_utils(None)
