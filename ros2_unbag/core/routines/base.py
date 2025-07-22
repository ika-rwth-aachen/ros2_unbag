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
from enum import Enum, auto


class ExportMode(Enum):
    SINGLE_FILE = auto()
    MULTI_FILE = auto()
    SINGLE_OR_MULTI_FILE = auto()
    
    
class ExportRoutine:
    # Registry for export routines by message type and format
    registry = defaultdict(list)
    catch_all = None  # fallback routine if no specific match is found

    def __init__(self, msg_types, formats, mode=ExportMode.SINGLE_OR_MULTI_FILE):
        """
        Register an export routine for the specified message types and formats.

        Args:
            msg_types: Message type string or list of message types.
            formats: List of supported export formats.

        Returns:
            None
        """
        self.msg_types = msg_types if isinstance(msg_types,
                                                 list) else [msg_types]
        self.formats = formats
        self.mode = mode
        self.__class__.register(self)

    def __call__(self, func):
        """
        Decorate a function to assign it as this routine's export handler.

        Args:
            func: Function to be used as the export handler.

        Returns:
            ExportRoutine: The routine instance itself.
        """
        self.func = func
        return self

    @classmethod
    def register(cls, routine):
        """
        Add a routine to the registry under each of its message types.

        Args:
            routine: ExportRoutine instance to register.

        Returns:
            None
        """
        for msg_type in routine.msg_types:
            cls.registry[msg_type].append(routine)

    @classmethod
    def get_formats(cls, msg_type):
        """
        Return all supported formats for a given message type, including catch-all formats.

        Args:
            msg_type: Message type string.

        Returns:
            list: List of supported format strings.
        """
        supported_formats = []
        if msg_type in cls.registry:
            supported_formats.extend(fmt for r in cls.registry[msg_type] for fmt in r.formats)
        if cls.catch_all:
            supported_formats.extend(cls.catch_all.formats)
        return supported_formats

    @classmethod
    def get_handler(cls, msg_type, fmt):
        """
        Retrieve the export handler function for a message type and format, falling back to catch-all if needed.

        Args:
            msg_type: Message type string.
            fmt: Export format string.

        Returns:
            function or None: Export handler function or None if not found.
        """
        for r in cls.registry.get(msg_type, []):
            if fmt in r.formats:
                return r.func
        if cls.catch_all and fmt in cls.catch_all.formats:
            return cls.catch_all.func
        return None
    
    @classmethod
    def get_mode(cls, msg_type, fmt):
        """
        Get the export mode for a specific message type and format.

        Args:
            msg_type: Message type string.
            fmt: Export format string.

        Returns:
            ExportMode: The export mode for the given message type and format.
        """
        for r in cls.registry.get(msg_type, []):
            if fmt in r.formats:
                return r.mode
        if cls.catch_all and fmt in cls.catch_all.formats:
            return cls.catch_all.mode
        return ExportMode.SINGLE_OR_MULTI_FILE

    @classmethod
    def set_catch_all(cls, formats):
        """
        Decorator to register a fallback export routine for any message type with specified formats.

        Args:
            formats: List of supported export formats.

        Returns:
            function: Decorator function.
        """
        def decorator(func):
            cls.catch_all = ExportRoutine(msg_types=[], formats=formats, mode=ExportMode.SINGLE_OR_MULTI_FILE)
            cls.catch_all.func = func
            return func

        return decorator
