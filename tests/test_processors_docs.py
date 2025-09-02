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

from ros2_unbag.core.processors.base import Processor


def setup_function(_):
    Processor.registry = defaultdict(list)


def test_processor_doc_arg_extraction():
    @Processor(["pkg/Msg"], ["fmt"]) 
    def handler(msg, alpha, beta: int = 2):
        """
        Example with documented args.

        Args:
            msg: Message object (ignored by get_args).
            alpha (int): first param.
            beta (int): second param with default.
        """
        return (msg, alpha, beta)

    args = Processor.get_args("pkg/Msg", "fmt")
    assert set(args.keys()) == {"alpha", "beta"}
    # Docstrings parsed
    assert args["alpha"][1].startswith("first param")
    assert args["beta"][1].startswith("second param")

    required = Processor.get_required_args("pkg/Msg", "fmt")
    assert required == ["alpha"]

