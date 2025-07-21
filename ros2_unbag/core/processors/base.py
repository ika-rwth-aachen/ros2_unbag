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


class Processor:
    # Registry for processing steps by message type and format
    registry = defaultdict(list)

    def __init__(self, msg_types, formats):
        """
        Register processing steps for the specified message types and formats.

        Args:
            msg_types: Message type string or list of message types.
            formats: List of supported processor formats.

        Returns:
            None
        """
        self.msg_types = msg_types if isinstance(msg_types,
                                                 list) else [msg_types]
        self.formats = formats
        self.__class__.register(self)

    def __call__(self, func):
        """
        Decorate a function to assign it as this processor’s handler.

        Args:
            func: Function to be used as the processor handler.

        Returns:
            Processor: The processor instance itself.
        """
        self.func = func
        return self

    @classmethod
    def register(cls, routine):
        """
        Add a processor routine to the registry under each of its message types.

        Args:
            routine: Processor instance to register.

        Returns:
            None
        """
        for msg_type in routine.msg_types:
            cls.registry[msg_type].append(routine)

    @classmethod
    def get_formats(cls, msg_type):
        """
        Return all supported formats for a given message type.

        Args:
            msg_type: Message type string.

        Returns:
            list: List of supported format strings.
        """
        if msg_type in cls.registry:
            return [fmt for r in cls.registry[msg_type] for fmt in r.formats]
        return []

    @classmethod
    def get_handler(cls, msg_type, fmt):
        """
        Retrieve the processing handler function for a message type and format.

        Args:
            msg_type: Message type string.
            fmt: Processor format string.

        Returns:
            function or None: Processor handler function or None if not found.
        """
        for r in cls.registry.get(msg_type, []):
            if fmt in r.formats:
                return r.func
        return None

    @classmethod
    def get_args(cls, msg_type, fmt):
        """
        Return a dict of argument names and parameters (excluding 'msg') for the handler.

        Args:
            msg_type: Message type string.
            fmt: Processor format string.

        Returns:
            dict or None: Mapping of argument names to inspect.Parameter objects, or None if not found.
        """
        # Get the argument names for the processing function
        handler = cls.get_handler(msg_type, fmt)
        if handler:
            signature = inspect.signature(handler)
            # Exclude 'msg' parameter (always passed automatically)
            return {
                name: param
                for name, param in signature.parameters.items()
                if name != 'msg'
            }
        return None

    @classmethod
    def get_required_args(cls, msg_type, fmt):
        """
        Return the list of required (non-default) argument names for the handler.

        Args:
            msg_type: Message type string.
            fmt: Processor format string.

        Returns:
            list: List of required argument names.
        """
        # Get the required argument names for the processing function
        args = cls.get_args(msg_type, fmt)
        if args:
            return [
                name for name, param in args.items()
                if param.default == inspect.Parameter.empty
            ]
        return []
