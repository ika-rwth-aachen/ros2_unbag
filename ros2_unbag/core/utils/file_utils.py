from datetime import datetime
import logging
import re

logger = logging.getLogger(__name__)
_MISSING_TIMESTAMP_WARNING_TYPES = set()

def get_time_from_msg(msg, return_datetime=True):
    """
    Extract the timestamp from a ROS2 message and return it as a datetime object.
    Raises ValueError if the timestamp is invalid.

    Args:
        msg: ROS2 message instance.

    Returns:
        datetime: The extracted timestamp as a datetime object or an integer timestamp.
    """
    try:
        sec = msg.header.stamp.sec
        nanosec = msg.header.stamp.nanosec
    except AttributeError:
        try:
            sec = msg.stamp.sec
            nanosec = msg.stamp.nanosec
        except AttributeError:
            msg_type = type(msg).__name__
            if msg_type not in _MISSING_TIMESTAMP_WARNING_TYPES:
                _MISSING_TIMESTAMP_WARNING_TYPES.add(msg_type)
                logger.warning(
                    "Message type '%s' has no valid timestamp; falling back to datetime.now(). "
                    "This may lead to incorrect behavior.",
                    msg_type,
                )
            now = datetime.now()
            if return_datetime:
                return now
            return int(now.timestamp() * 1_000_000_000)

    if not return_datetime:
        return int(sec) * 1_000_000_000 + int(nanosec)
    return datetime.fromtimestamp(sec + nanosec * 1e-9)


_PLACEHOLDER_RE = re.compile(r"%(name|index|timestamp|master_timestamp)")
def substitute_placeholders(template_string: str, replacements: dict) -> str:
    """
    Replace %name, %index, %timestamp, %master_timestamp in a template string.

    Args:
        template_string (str): The string containing placeholders.
        replacements (dict): A dictionary with keys 'name', 'index', 'timestamp', 'master_timestamp'
                             and their corresponding replacement values.
    
    Returns:
        str: The string with placeholders replaced by their corresponding values.
    """
    if not template_string or "%" not in template_string:
        return template_string
    return _PLACEHOLDER_RE.sub(lambda m: replacements[m.group(1)], template_string)

_STRFTIME_RE = re.compile(r"%(?!name|index|timestamp|master_timestamp)[A-Za-z]")
def is_strftime_in_template(template_string: str) -> bool:
    """
    Check if the template string contains strftime format specifiers.

    Args:
        template_string (str): The string to check for strftime specifiers.

    Returns:
        bool: True if the string contains strftime specifiers, False otherwise.
    """
    return bool(_STRFTIME_RE.search(template_string))
