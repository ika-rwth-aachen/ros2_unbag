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

import inspect

from ros2_unbag.core.processors.base import Processor


def setup_function(_):
    # Reset registry before each test to avoid cross-test pollution
    Processor.registry = defaultdict(list)


def test_processor_registration_and_queries():
    @Processor(["pkg/MsgA", "pkg/MsgB"], ["fmt1", "fmt2"])
    def handler(msg, required_arg, opt_arg: int = 5):
        """
        Example handler.

        Args:
            msg: Message object.
            required_arg (int): A required argument.
            opt_arg (int): Optional argument.
        """
        return (msg, required_arg, opt_arg)

    # Registry holds per-message-type entries
    assert set(Processor.registry.keys()) == {"pkg/MsgA", "pkg/MsgB"}

    # Formats flatten across routines
    assert sorted(Processor.get_formats("pkg/MsgA")) == ["fmt1", "fmt2"]

    # Handler lookup
    func = Processor.get_handler("pkg/MsgA", "fmt1")
    assert callable(func)

    # Args metadata excludes 'msg'
    args = Processor.get_args("pkg/MsgA", "fmt1")
    assert set(args.keys()) == {"required_arg", "opt_arg"}
    assert isinstance(args["required_arg"][0], inspect.Parameter)

    # Required args
    req = Processor.get_required_args("pkg/MsgA", "fmt1")
    assert req == ["required_arg"]

