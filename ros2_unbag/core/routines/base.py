from collections import defaultdict


class ExportRoutine:
    # Registry for export routines by message type and format
    registry = defaultdict(list)
    catch_all = None  # fallback routine if no specific match is found

    def __init__(self, msg_types, formats):
        # Register export routine for given message types and formats
        self.msg_types = msg_types if isinstance(msg_types,
                                                 list) else [msg_types]
        self.formats = formats
        self.__class__.register(self)

    def __call__(self, func):
        # Store the function to be called for export
        self.func = func
        return self

    @classmethod
    def register(cls, routine):
        # Add routine to registry for each message type
        for msg_type in routine.msg_types:
            cls.registry[msg_type].append(routine)

    @classmethod
    def get_formats(cls, msg_type):
        # Return list of supported formats for a message type
        supported_formats = []
        if msg_type in cls.registry:
            supported_formats.extend(fmt for r in cls.registry[msg_type] for fmt in r.formats)
        if cls.catch_all:
            supported_formats.extend(cls.catch_all.formats)
        return supported_formats

    @classmethod
    def get_handler(cls, msg_type, fmt):
        # Get the appropriate export function for a type and format
        for r in cls.registry.get(msg_type, []):
            if fmt in r.formats:
                return r.func
        if cls.catch_all and fmt in cls.catch_all.formats:
            return cls.catch_all.func
        return None

    @classmethod
    def set_catch_all(cls, formats):
        # Register a fallback routine for all message types

        def decorator(func):
            cls.catch_all = ExportRoutine(msg_types=[], formats=formats)
            cls.catch_all.func = func
            return func

        return decorator
