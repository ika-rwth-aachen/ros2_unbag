<img src="ros2_unbag/ui/title.png" style="height:130px; object-fit: cover; object-position: top; clip-path: inset(25% 0 0 0); float: right; margin-top: 20px;">

# ros2 unbag

ros2 unbag is a plugin directly build in to the ROS 2 CLI to export selected topics from a ROS2 bag file (`.db3` or `.mcap`) into custom formats using pluggable export routines.

- [Installation](#installation)
   - [Prerequisites](#prerequisites)
   - [Install via pip (from source)](#install-via-pip-from-source)
   - [Install in a ROS 2 workspace](#install-in-a-ros-2-workspace)
- [Usage](#usage)
   - [GUI](#gui)
   - [CLI](#cli)
- [Routines](#routines)
- [Processors](#processors)
- [Resampling](#resampling)
   - [Last](#last)
   - [Nearest](#nearest)
- [CPU Utilization](#cpu-utilization)
- [Configs](#configs)

## Installation 

### Prerequisites

Make sure you have a working ROS 2 installation (e.g., Humble, Iron, Jazzy) and that your environment is sourced:

```bash
source /opt/ros/<distro>/setup.bash
````

Replace `<distro>` with your ROS 2 distribution.

### Install via pip (from source)

Clone the repository:

```bash
git clone https://gitlab.ika.rwth-aachen.de/fb-fi/data/r2d2.git
cd ros2_unbag
```

Install the package:

```bash
pip install .
```

### Install in a ROS 2 workspace

Clone into your ROS 2 workspace:

```bash
cd ~/ros2_ws/src
git clone https://gitlab.ika.rwth-aachen.de/fb-fi/data/r2d2.git
cd ..
colcon build --packages-select ros2_unbag
source install/setup.bash
```

The command will be available as:

```bash
ros2 unbag
```

## Usage

You can use the tool either via a graphical user interface (GUI) or a command-line interface (CLI).

### GUI

Run the command below and then follow the screen instructions.

```bash
ros2 unbag
```

### CLI

Run the CLI tool by calling below command.

```bash
ros2 unbag <path_to_rosbag> 
    --output-dir <directory> 
    --export </topic:format[:subdir]> 
    --naming <naming_pattern> 
    --resample </master_topic:resample_type[,discard_eps]> 
    --processing </topic:processor:[arg1_name=arg1_value,arg2_name=arg2_value;...]> 
    --config <config_file>
```

    The naming pattern supports `%name`, `%index`, and datetime placeholders such as `%d`, `%m`, `%Y`, etc.
    When using the build-in export formats [text/csv], [text/json] or [text/yaml], everything gets exported into one file, if you select a name, that does not change during execution (i.e does not include %index or any datetime placeholders).

    If you specify the `--config` option (e.g., `--config configs/my_config.json`), the tool will load all export settings from the given JSON configuration file. In this case, all other command-line options except `<path_to_rosbag>` are ignored, and the export process is fully controlled by the config file. The `<path_to_rosbag>` is always required.

    Example: `./main_cli.py rosbag2/rosbag2.db3 
    --output-dir /docker-ros/ws/test/ --export /altos_radar/altosRadar:pointcloud/xyz:radar --resample /altos_radar/altosRadar:last,0.2`

## Routines 

Routines define the way how messages are exported from the ROS 2 bag file to the desired output format. The tool comes with a set of predefined routines for common message types and formats, such as `sensor_msgs/msg/PointCloud2` to `pointcloud/xyz`, `sensor_msgs/msg/Image` to `image/jpeg`, and many more.

Your message type or output format is not supported by default? No problem! You can add your own export routines to handle custom message types or output formats.

Routines are defined like this: 

```python
from ros2_unbag.core.routines.base import ExportRoutine            # import the base class
# you can also import other packages here - e.g., numpy, cv2, etc.

@ExportRoutine("sensor_msgs/msg/PointCloud2", ["pointcloud/xyz"])       # define the message type and output format, each of these can be a list of formats
def export_pointcloud_xyz(msg, path, fmt="pointcloud/xyz"):             # define the export function
    # the name of the function does not matter
    # the parameters do need to be defined like this
        # msg: the message to export
        # path: the path to the output folder (without extension)
        # fmt: the format to export to - can be any of the formats defined in the decorator
    with open(path + ".xyz", 'w') as f:                                 # define your custom logic to export the message
        for i in range(0, len(msg.data), msg.point_step):
            x, y, z = struct.unpack_from("fff", msg.data, offset=i)
            f.write(f"{x} {y} {z}\n")
```

You can import your own routines permanently by calling 
```bash 
ros2 unbag --install-routine <path_to_your_routine_file>
```

or use them only temporarily by specifying the `--use-routine` option when starting the program. This works in both the GUI and CLI versions.

```bash
ros2 unbag --use-routine <path_to_your_routine_file>
```

## Processors

Processors are used to modify messages before they are exported. They can be applied to specific topics and allow you to perform operations such as filtering, transforming, or enriching the data.

You can define your own processors like this:

```python
from ros2_unbag.core.processors.base import Processor              # import the base class
# you can also import other packages here - e.g., numpy, cv2, etc.

@Processor("sensor_msgs/msg/CompressedImage", ["recolor"])          # define the message type and the processor name
def recolor_compressed_image(msg, color_map):                       # define the processor function 
    # the name of the function does not matter
    # the first parameter must be the message to process
    # any other parameters can be set by the user during runtime
    """Recolor a compressed image using a cv2 color map
    """
    try:
        color_map = int(color_map)
    except ValueError:                                              # exceptions will be forwarded to the gui
        raise ValueError(
            f"Invalid color map value: {color_map}. Must be an integer.")

    img_array = np.frombuffer(msg.data, np.uint8)
    img = cv2.imdecode(
        img_array,
        cv2.IMREAD_GRAYSCALE)

    recolored = cv2.applyColorMap(img, color_map)

    ext = '.jpg' if 'jpeg' in msg.format.lower() else '.png'
    success, encoded = cv2.imencode(ext, recolored)

    if not success:
        raise RuntimeError("Failed to encode recolored image")

    msg.data = encoded.tobytes()
    return msg                                                      # return the modified message

```

You can import your own processors by calling 
```bash
ros2 unbag --install-processor <path_to_your_processor_file>
```

or use them only temporarily by specifying the `--use-processor` option when starting the program. This works in both the GUI and CLI versions.

```bash
ros2 unbag --use-processor <path_to_your_processor_file>
```

## Resampling
In many cases, you may want to resample messages in the frequency of a master topic. This allows you to assemble a "frame" of data that is temporally aligned with a specific topic, such as a camera or LIDAR sensor. The resampling process will ensure that the messages from other topics are exported in sync with the master topic's timestamps.

ros2 unbag supports resampling of messages based on a master topic. You can specify the master topic and the resampling type (e.g., `last` or `nearest`) along with an optional discard epsilon value.

### Last
The `last` resampling type will listen for the master topic. As soon as a message of the master topic is received, a frame will be assembled, containing the last last message of any other selected topics. With an optional `discard_eps` value, you can specify a maximum time difference between the master topic message and the other topics' messages. If no message is found within the `discard_eps` value, the whole frame is discarded.

### Nearest
The `nearest` resampling type will listen for the master topic and export it along with the (temporally) nearest message of the other topics that were published in the time range of the master topic message. This resampling strategy is only usable with an `discard_eps` value, which defines the maximum time difference between the master topic message and the other topics' messages. If no message is found within the `discard_eps` value, the whole frame is discarded.

## CPU utilization
ros2 unbag uses multi-processing to export messages in parallel. The number of processes is determined by the number of CPU cores available on your system. You can control the number of processes by setting the `--cpu-percentage` option when running the CLI tool. The default value is 80%, which means that the tool will use 80% of the available CPU cores for processing. You can adjust this value to control the CPU utilization during the export process.

## Configs
When using ROS2 Unbag, you can define your export settings in a JSON configuration file. This works in the GUI, as well as in the CLI version. It allows you to easily reuse your export settings without having to specify them on the command line every time.

Tip: Use the GUI to create your export settings and then save them via the "Save Config" button. This will create a JSON file with all your export settings, which you can then use in the CLI version.