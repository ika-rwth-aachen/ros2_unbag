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
        Return a dict mapping argument names to a tuple: (inspect.Parameter, docstring_description).

        Args:
            msg_type: Message type string.
            fmt: Processor format string.

        Returns:
            dict or None: Mapping of argument names to (Parameter, docstring description), or None.
        """
        handler = cls.get_handler(msg_type, fmt)
        if not handler:
            return None

        signature = inspect.signature(handler)
        docstring = inspect.getdoc(handler)
        param_docs = cls._extract_param_docs(docstring)

        return {
            name: (param, param_docs.get(name, ""))
            for name, param in signature.parameters.items()
            if name != 'msg'
        }

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
                name for name, (param, _) in args.items()
                if isinstance(param, inspect.Parameter) and param.default == inspect.Parameter.empty
            ]
        return []
    
    @staticmethod
    def _extract_param_docs(docstring):
        """
        Extract parameter descriptions from a Google-style docstring.

        Args:
            docstring: The full docstring of the processor function.

        Returns:
            dict: Mapping of parameter name to description string.
        """
        import re

        if not docstring:
            return {}

        param_docs = {}
        lines = docstring.splitlines()
        in_args = False
        for line in lines:
            line = line.strip()
            if line.startswith("Args:"):
                in_args = True
                continue
            if in_args:
                if re.match(r"^\w+\s*\(.*\):", line):  # param with type
                    key = line.split(":", 1)[0].split("(")[0].strip()
                    desc = line.split(":", 1)[1].strip()
                    param_docs[key] = desc
                elif re.match(r"^\w+\s*:", line):  # param without type
                    key = line.split(":", 1)[0].strip()
                    desc = line.split(":", 1)[1].strip()
                    param_docs[key] = desc
                elif line == "":
                    break  # end of block
        return param_docs
