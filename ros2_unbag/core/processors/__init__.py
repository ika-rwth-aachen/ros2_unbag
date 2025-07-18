import importlib
import pkgutil

from .base import Processor


def load_all_processors():
    # Dynamically import all modules in the current package
    package = __name__
    for _, module_name, _ in pkgutil.iter_modules(__path__):
        importlib.import_module(f"{package}.{module_name}")


# Load all modules when the package is imported
load_all_processors()
