from datetime import datetime
import logging

logger = logging.getLogger(__name__)

def get_time_from_msg(msg, return_datetime=True):
    """
    Extract the timestamp from a ROS2 message and return it as a datetime object.
    Raises ValueError if the timestamp is invalid.

    Args:
        msg: ROS2 message instance.

    Returns:
        datetime: The extracted timestamp as a datetime object.
    """
    try:
        sec = msg.header.stamp.sec
        nanosec = msg.header.stamp.nanosec
    except AttributeError:
        try:
            sec = msg.stamp.sec
            nanosec = msg.stamp.nanosec
        except AttributeError:
            logger.warning("Message has no valid timestamp; falling back to datetime.now() - This may lead to incorrect behavior.")
            return datetime.now() if return_datetime else datetime.now().timestamp()

    return datetime.fromtimestamp(sec + nanosec * 1e-9) if return_datetime else sec + nanosec * 1e-9