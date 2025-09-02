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

from datetime import datetime

import pytest

from ros2_unbag.core.utils.file_utils import (
    get_time_from_msg,
    substitute_placeholders,
    is_strftime_in_template,
)


class _Stamp:
    def __init__(self, sec, nanosec):
        self.sec = sec
        self.nanosec = nanosec


class _Header:
    def __init__(self, sec, nanosec):
        self.stamp = _Stamp(sec, nanosec)


class _Msg:
    def __init__(self, header=None, stamp=None):
        if header is not None:
            self.header = header
        if stamp is not None:
            self.stamp = stamp


def test_get_time_from_msg_with_header_stamp_datetime():
    msg = _Msg(header=_Header(1700000000, 250))
    dt = get_time_from_msg(msg, return_datetime=True)
    assert isinstance(dt, datetime)
    # 1700000000.000000250 -> seconds and microseconds order check
    assert abs(dt.timestamp() - (1700000000 + 250e-9)) < 1e-6


def test_get_time_from_msg_with_stamp_int_ns():
    msg = _Msg(stamp=_Stamp(1700000001, 123456789))
    ns = get_time_from_msg(msg, return_datetime=False)
    assert isinstance(ns, int)
    assert ns == 1700000001 * 1_000_000_000 + 123_456_789


def test_substitute_placeholders_basic():
    template = "%name_%index_%timestamp"
    repl = {"name": "camera", "index": "0007", "timestamp": "123"}
    assert substitute_placeholders(template, repl) == "camera_0007_123"


def test_substitute_placeholders_no_percent_returns_input():
    s = "no_placeholders_here"
    assert substitute_placeholders(s, {"name": "x", "index": "y", "timestamp": "z"}) == s


@pytest.mark.parametrize(
    "template,expected",
    [
        ("%Y-%m-%d_%H-%M-%S", True),
        ("prefix_%name_suffix", False),
        ("mix_%Y-%m-%d_%index", True),
        ("nothing", False),
    ],
)
def test_is_strftime_in_template(template, expected):
    assert is_strftime_in_template(template) is expected


def test_get_time_from_msg_fallback_types():
    class _NoStamp:
        pass

    # When no stamp is present, return types remain consistent
    dt = get_time_from_msg(_NoStamp(), return_datetime=True)
    ns = get_time_from_msg(_NoStamp(), return_datetime=False)
    assert hasattr(dt, 'timestamp')
    assert isinstance(ns, int)
