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

