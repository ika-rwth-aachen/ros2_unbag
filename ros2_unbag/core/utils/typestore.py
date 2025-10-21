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

"""Helpers for accessing a shared rosbags typestore."""

from functools import lru_cache
from typing import Any

from rosbags.typesys import Stores, get_typestore


@lru_cache(maxsize=1)
def get_default_typestore():
    """
    Return the lazily-instantiated ROS 2 Jazzy typestore used across the project.

    Args:
        None

    Returns:
        rosbags.typesys.Typestore: The ROS 2 Jazzy typestore.
    """
    return get_typestore(Stores.ROS2_JAZZY)


def get_message_class(ros_type: str) -> Any:
    """
    Resolve and return the dataclass implementing ``ros_type`` from the default typestore.
    
    Args:
        ros_type (str): The fully-qualified ROS message type name, e.g., "std_msgs/msg/String".

    Returns:
        Any: The dataclass implementing the specified ROS message type.

    Raises:
        KeyError: If the specified message type is not registered in the default typestore.
    """
    typestore = get_default_typestore()
    try:
        return typestore.types[ros_type]
    except KeyError as exc:
        raise KeyError(
            f"Message type '{ros_type}' is not registered in the default typestore."
        ) from exc
