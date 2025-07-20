from collections import defaultdict


class ExportRoutine:
    # Registry for export routines by message type and format
    registry = defaultdict(list)
    catch_all = None  # fallback routine if no specific match is found

    def __init__(self, msg_types, formats):
        """
        Register an export routine for the specified message types and formats.
        """
        self.msg_types = msg_types if isinstance(msg_types,
                                                 list) else [msg_types]
        self.formats = formats
        self.__class__.register(self)

    def __call__(self, func):
        """
        Decorate a function to assign it as this routine's export handler.
        """
        self.func = func
        return self

    @classmethod
    def register(cls, routine):
        """
        Add a routine to the registry under each of its message types.
        """
        for msg_type in routine.msg_types:
            cls.registry[msg_type].append(routine)

    @classmethod
    def get_formats(cls, msg_type):
        """
        Return all supported formats for a given message type, including catch-all formats.
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
        """
        for r in cls.registry.get(msg_type, []):
            if fmt in r.formats:
                return r.func
        if cls.catch_all and fmt in cls.catch_all.formats:
            return cls.catch_all.func
        return None

    @classmethod
    def set_catch_all(cls, formats):
        """
        Decorator to register a fallback export routine for any message type with specified formats.
        """
        def decorator(func):
            cls.catch_all = ExportRoutine(msg_types=[], formats=formats)
            cls.catch_all.func = func
            return func

        return decorator
