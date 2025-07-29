# Import any necessary libraries for the processor
# ...

# Import the processor decorator
from ros2_unbag.core.processors.base import Processor

# Define the processor class with the appropriate message types and give it a name
@Processor(["std_msgs/msg/String"], ["your_processor_name"]) 
def your_processor_name(msg, your_parameter: str = "default", your_parameter_2: str = "template"):
    """
    Short description of what the processor does.

    Args:
        msg: The ROS message you want to process.
        your_parameter: Describe the parameter. This will be shown in the UI.
        your_parameter_2: You can add more parameters as needed.

    Returns:
        The return always needs to match the incoming message type.
    """

    # Validate and convert parameter
    try:
        your_parameter = str(your_parameter)
        your_parameter_2 = str(your_parameter_2)
    except ValueError:
        raise ValueError(f"One of the parameters is not valid: {your_parameter}, {your_parameter_2}")

    # Decode ROS message if necessary
    string_msg = msg.data  # Assuming msg is a String message

    # --- Apply your processing here ---
    processed_msg = string_msg.replace(your_parameter, your_parameter_2)

    # Re-encode the image
    msg.data = processed_msg

    return msg
