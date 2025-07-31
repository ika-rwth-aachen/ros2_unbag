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
from dataclasses import dataclass
from enum import Enum, auto


class ExportMode(Enum):
    SINGLE_FILE = auto()
    MULTI_FILE = auto()

@dataclass(frozen=True)
class ExportMetadata:
    index: int                     # The index of the message in the topic
    max_index: int                 # The maximum index of the message in the topic  
    
class ExportRoutine:
    # Registry for export routines by message type and format
    registry = defaultdict(list)
    catch_all_registry = defaultdict(list)

    def __init__(self, msg_types, formats, mode):
        """
        Register an export routine for the specified message types and formats.

        Args:
            msg_types: Message type string or list of message types.
            formats: List of supported export formats.
            mode: ExportMode indicating whether to use single or multi-file export.

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
        storage = defaultdict(dict)  # Define a persistent storage for each topic

        def wrapper(msg, path, fmt, metadata, topic=None):
            wrapper.persistent_storage = storage[topic] if topic else {}
            return func(msg, path, fmt, metadata)

        wrapper.persistent_storage = {}  # Initialize persistent storage
        self.func = wrapper
        return wrapper


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
        supported_formats.extend(cls.catch_all_registry.keys())
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
        for r in cls.catch_all_registry.get(fmt, []):
            return r.func
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
        for r in cls.catch_all_registry.get(fmt, []):
            return r.mode
        return None

    @classmethod
    def set_catch_all(cls, formats, mode):
        """
        Decorator to register a fallback export routine for any message type with specified formats.

        Args:
            formats: List of supported export formats.

        Returns:
            function: Decorator function.
        """
        def decorator(func):
            routine = ExportRoutine(msg_types=[], formats=formats, mode=mode)
            wrapped_func = routine(func)
            for fmt in formats:
                cls.catch_all_registry[fmt].append(routine)
            return wrapped_func
        return decorator
