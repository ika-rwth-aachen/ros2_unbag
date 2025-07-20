from collections import defaultdict
import inspect


class Processor:
    # Registry for processing steps by message type and format
    registry = defaultdict(list)

    def __init__(self, msg_types, formats):
        # Register processing steps for given message types and formats
        self.msg_types = msg_types if isinstance(msg_types,
                                                 list) else [msg_types]
        self.formats = formats
        self.__class__.register(self)

    def __call__(self, func):
        # Store the function to be called for processing
        self.func = func
        return self

    @classmethod
    def register(cls, routine):
        # Add processing step to registry for each message type
        for msg_type in routine.msg_types:
            cls.registry[msg_type].append(routine)

    @classmethod
    def get_formats(cls, msg_type):
        # Return list of supported formats for a message type
        if msg_type in cls.registry:
            return [fmt for r in cls.registry[msg_type] for fmt in r.formats]
        return []

    @classmethod
    def get_handler(cls, msg_type, fmt):
        # Get the appropriate processing function for a type and format
        for r in cls.registry.get(msg_type, []):
            if fmt in r.formats:
                return r.func
        return None

    @classmethod
    def get_args(cls, msg_type, fmt):
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
        # Get the required argument names for the processing function
        args = cls.get_args(msg_type, fmt)
        if args:
            return [
                name for name, param in args.items()
                if param.default == inspect.Parameter.empty
            ]
        return []
