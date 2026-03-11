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
import inspect
from typing import Dict, Iterable, Optional, Tuple


class ExportMode(Enum):
    SINGLE_FILE = auto()
    MULTI_FILE = auto()

@dataclass(frozen=True)
class ExportMetadata:
    index: int                     # The index of the message in the topic
    max_index: int                 # The maximum index of the message in the topic  
    
class ExportRoutine:
    # Registry for export routines by message type and format
    registry = defaultdict(lambda: defaultdict(dict))
    catch_all_registry = defaultdict(dict)
    _MODE_SUFFIX = {
        ExportMode.SINGLE_FILE: "single_file",
        ExportMode.MULTI_FILE: "multi_file",
    }

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
        self.formats = [self._normalize_registered_format(fmt, mode) for fmt in formats]
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

        def wrapper(msg, path, fmt, metadata, topic=None, routine_args=None, **kwargs):
            wrapper.persistent_storage = storage[topic]
            canonical_fmt, _ = ExportRoutine._split_format(fmt)
            merged = {**(routine_args or {}), **kwargs}
            return func(msg, path, canonical_fmt, metadata, **merged)

        wrapper.persistent_storage = {}  # Initialize persistent storage
        self.func = wrapper
        self._original_func = func  # Keep reference for signature introspection
        return wrapper


    @classmethod
    def reset_registry(cls):
        """
        Reset the routine registry and catch-all registry. Primarily intended for testing.

        Args:
            None
    
        Returns:
            None
        """
        cls.registry = defaultdict(lambda: defaultdict(dict))
        cls.catch_all_registry = defaultdict(dict)

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
            if msg_type is None:
                continue
            for base_fmt in routine.formats:
                cls.registry[msg_type][base_fmt][routine.mode] = routine

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
        seen = set()

        specific = cls.registry.get(msg_type, {})
        for base_fmt in specific:
            if base_fmt not in seen:
                supported_formats.append(base_fmt)
                seen.add(base_fmt)

        for base_fmt in cls.catch_all_registry:
            if base_fmt not in seen and cls._get_modes_for(msg_type, base_fmt):
                supported_formats.append(base_fmt)
                seen.add(base_fmt)

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
        resolved = cls.resolve(msg_type, fmt)
        if not resolved:
            return None
        routine, _, _ = resolved
        return routine.func

    @classmethod
    def get_args(cls, msg_type, fmt):
        """
        Return the extra keyword arguments (beyond the 4 fixed positional args) declared by the
        registered routine for the given message type and format.

        Returns a dict mapping parameter name to a (inspect.Parameter, doc_str) tuple,
        mirroring the interface of Processor.get_args().

        Args:
            msg_type: Message type string.
            fmt: Export format string.

        Returns:
            dict: Mapping of parameter name to (inspect.Parameter, description string).
                  Empty dict if no routine is found or no extra args are declared.
        """
        resolved = cls.resolve(msg_type, fmt)
        if not resolved:
            return {}
        routine, _, _ = resolved
        if not hasattr(routine, "_original_func"):
            return {}
        func = routine._original_func
        sig = inspect.signature(func)
        doc = inspect.getdoc(func)
        param_docs = cls._extract_param_docs(doc)
        # Skip the 4 fixed positional args of every routine
        _SKIP = {"msg", "path", "fmt", "metadata"}
        return {
            name: (param, param_docs.get(name, ""))
            for name, param in sig.parameters.items()
            if name not in _SKIP
        }

    @staticmethod
    def _extract_param_docs(docstring):
        """
        Extract parameter descriptions from a Google-style docstring.

        Args:
            docstring: The full docstring of the routine function.

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
        resolved = cls.resolve(msg_type, fmt)
        if not resolved:
            return None
        _, _, mode = resolved
        return mode

    @classmethod
    def resolve(cls, msg_type, fmt: str) -> Optional[Tuple["ExportRoutine", str, ExportMode]]:
        """
        Resolve a format string to the registered routine, returning the routine, canonical format name and mode.

        Args:
            msg_type: Message type string.
            fmt: User supplied format string (with or without @ modifier).

        Returns:
            tuple or None: (ExportRoutine, canonical_format, ExportMode) or None if not found.
        """
        base_fmt, explicit_mode = cls._split_format(fmt)
        candidates = cls._get_modes_for(msg_type, base_fmt)
        if not candidates:
            return None

        if explicit_mode:
            routine = candidates.get(explicit_mode)
            if not routine:
                return None
            return routine, base_fmt, explicit_mode

        selected_mode = (ExportMode.MULTI_FILE
                         if ExportMode.MULTI_FILE in candidates
                         else next(iter(candidates)))
        routine = candidates[selected_mode]
        return routine, base_fmt, selected_mode

    @classmethod
    def get_modes_for_format(cls, msg_type, fmt: str) -> Iterable[ExportMode]:
        """
        Return the available modes for a given message type and format string.

        Args:
            msg_type: Message type string.
            fmt: Export format string.

        Returns:
            Iterable[ExportMode]: List of available export modes.
        """
        base_fmt, _ = cls._split_format(fmt)
        return cls._get_modes_for(msg_type, base_fmt).keys()

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
            for base_fmt in routine.formats:
                cls.catch_all_registry[base_fmt][mode] = routine
            return wrapped_func
        return decorator

    @classmethod
    def _get_modes_for(cls, msg_type: str, base_fmt: str) -> Dict[ExportMode, "ExportRoutine"]:
        """
        Retrieve all available export modes for a given message type and base format, including catch-all.

        Args:
            msg_type: Message type string.
            base_fmt: Base export format string (without @ modifier).

        Returns:
            Dict[ExportMode, ExportRoutine]: Dictionary mapping ExportMode to ExportRoutine.
        """
        modes: Dict[ExportMode, ExportRoutine] = {}
        specific = cls.registry.get(msg_type, {})
        if base_fmt in specific:
            modes.update(specific[base_fmt])

        catch_all = cls.catch_all_registry.get(base_fmt, {})
        for mode, routine in catch_all.items():
            modes.setdefault(mode, routine)
        return modes

    @classmethod
    def _normalize_registered_format(cls, fmt: str, mode: ExportMode) -> str:
        """
        Normalize a registered format string by ensuring it matches the routine's mode.
        
        Args:
            fmt: Export format string (possibly with @ modifier).
            mode: ExportMode of the routine.
        
        Returns:
            str: Canonical base format string without @ modifier.
        
        Raises:
            ValueError: If the format string's mode conflicts with the routine's mode.
        """
        base_fmt, embedded_mode = cls._split_format(fmt, strict=True)
        if embedded_mode and embedded_mode != mode:
            raise ValueError(
                f"Format '{fmt}' declares @{cls._MODE_SUFFIX.get(embedded_mode)} but routine is registered as {mode}."
            )
        return base_fmt

    @classmethod
    def _split_format(cls, fmt: str, strict: bool = False) -> Tuple[str, Optional[ExportMode]]:
        """
        Split a format string into its base format and optional mode suffix.
        
        Args:
            fmt: Export format string (possibly with @ modifier).
            strict: If True, raise an error for unknown suffixes.
        
        Returns:
            tuple: (base_format, ExportMode or None)
        
        Raises:
            ValueError: If strict is True and the suffix is unknown.
        """
        fmt = fmt.strip()
        if "@" not in fmt:
            return fmt, None
        base, suffix = fmt.rsplit("@", 1)
        mode = cls._mode_from_suffix(suffix)
        if mode is None:
            if strict:
                raise ValueError(f"Unknown export mode suffix '@{suffix}' in format '{fmt}'.")
            return fmt, None
        return base, mode

    @classmethod
    def _mode_from_suffix(cls, suffix: str) -> Optional[ExportMode]:
        """
        Convert a suffix string to its corresponding ExportMode.

        Args:
            suffix: Suffix string (e.g., "single_file", "multi_file").
            
        Returns:
            ExportMode or None: Corresponding ExportMode or None if not found.
        """
        suffix = suffix.strip().lower()
        for mode, token in cls._MODE_SUFFIX.items():
            if suffix == token:
                return mode
        return None
