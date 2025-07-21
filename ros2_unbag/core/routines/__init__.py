import importlib
import pkgutil

from .base import ExportRoutine

def load_all_routines():
    """
    Dynamically import all modules in the current package to register ExportRoutine handlers.

    Args:
        None

    Returns:
        None
    """
    package = __name__
    for _, module_name, _ in pkgutil.iter_modules(__path__):
        importlib.import_module(f"{package}.{module_name}")


# Load all modules when the package is imported
load_all_routines()
