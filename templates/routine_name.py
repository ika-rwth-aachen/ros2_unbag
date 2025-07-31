# Import any necessary libraries for the routine
# ...

from pathlib import Path

# Import the routine decorator and dependencies
from ros2_unbag.core.routines.base import ExportRoutine, ExportMode, ExportMetadata

## MULTI-FILE EXPORT ROUTINE TEMPLATE
# Define the export routine class with the appropriate message types and give it a name. The mode determines if the routine will run parallel or sequential.
@ExportRoutine("std_msgs/msg/String", ["text/plain"], mode=ExportMode.MULTI_FILE)
def your_export_routine(msg, path: Path, fmt: str, metadata: ExportMetadata):
    """
    Short description of what the export routine does.

    Args:
        msg: The ROS message to export.
        path: Output file path (without extension).
        fmt: Desired export format (e.g. "text/plain").
        metadata: Additional metadata (e.g. index, max index).

    Returns:
        None
    """

    # Validate or parse parameters if needed
    if fmt != "text/plain":
        raise ValueError(f"Unsupported format: {fmt}")

    # --- Apply export logic here ---
    text = str(msg.data)

    with open(path.with_suffix(".txt"), "w", encoding="utf-8") as f:
        f.write(text)


## SINGLE-FILE EXPORT ROUTINE TEMPLATE
# Define the export routine class with the appropriate message types and give it a name. The mode determines if the routine will run parallel or sequential.
@ExportRoutine("std_msgs/msg/String", ["text/plain"], mode=ExportMode.SINGLE_FILE)
def your_export_routine(msg, path: Path, fmt: str, metadata: ExportMetadata):
    """
    Short description of what the export routine does.

    Args:
        msg: The ROS message to export.
        path: Output file path (without extension).
        fmt: Desired export format (e.g. "text/plain").
        metadata: Additional metadata (e.g. index, max index).

    Returns:
        None
    """

    # Validate or parse parameters if needed
    if fmt != "text/plain":
        raise ValueError(f"Unsupported format: {fmt}")
    
    # If you need persistent storage, you can use the persistent_storage attribute of the routine
    # this is a global dictionary that can be used to store state across calls
    ps = your_export_routine.persistent_storage

    # --- Apply export logic here ---
    text = str(msg.data)

    # Write to a single file, appending if not the first message
    # This allows for sequential writing of messages to the same file
    mode = "w" if metadata.index == 0 else "a"
    with open(path.with_suffix(".txt"), mode, encoding="utf-8") as f:
        f.write(text)
        f.write("\n")
        if metadata.index == metadata.max_index:                        # metadata object contains the index of the current message and the maximum index
            f.write("End of File\n")