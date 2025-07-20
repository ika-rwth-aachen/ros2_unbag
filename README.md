<img src="ros2_unbag/ui/title.png" height=130 align="right">

# *ros2 unbag* - fast ROS 2 bag export for any format

<p align="center">
  <img src="https://img.shields.io/github/license/ika-rwth-aachen/ros2_unbag"/>
  <a href="https://github.com/ika-rwth-aachen/ros2_unbag/actions/workflows/build_docker.yml"><img src="https://github.com/ika-rwth-aachen/ros2_unbag/actions/workflows/build_docker.yml/badge.svg"/></a>
</p>

*ros2 unbag* is a ROS 2 CLI plugin with optional GUI for extracting selected topics from `.db3` or `.mcap` bag files into formats like CSV, JSON, PCD, images, and more.

It comes with export routines for [common message types](#export-routines) (sensor data, point clouds, images). You need a special file format or message type? Add your own export plugin for any ROS 2 message or format, and chain custom processors to filter, transform or enrich messages (e.g. drop fields, compute derived values, remap frames).

Optional resampling synchronizes your data streams around a chosen master topic—aligning each other topic either to its last‑known sample (“last”) or to the temporally closest sample (“nearest”)—so you get a consistent sample count in your exports.

For high‑throughput workflows, *ros2 unbag* can spawn multiple worker processes and lets you tune CPU usage. Your topic selections, processor chains, export parameters and resampling mode (last or nearest) can be saved to and loaded from a JSON configuration, ensuring reproducibility across runs.

Use it as `ros2 unbag <args>` or in the GUI for a flexible, extensible way to turn bag files into the data you need.

## Table of Contents

- [Features](#features)  
- [Installation](#installation)  
  - [Prerequisites](#prerequisites)  
  - [From PyPI (via pip)](#from-pypi-via-pip)  
  - [From Source](#from-source-via-pip)  
  - [In a ROS 2 Workspace](#in-a-ros-2-workspace-via-colcon)  
  - [Docker](#docker)  
- [Quick Start](#quick-start)  
  - [GUI Mode](#gui-mode)  
  - [CLI Mode](#cli-mode)  
- [Config File](#config-file)  
- [Export Routines](#export-routines)  
- [Processors](#processors)  
- [Resampling](#resampling)  
  - [last](#last)  
  - [nearest](#nearest)  
- [CPU Utilization](#cpu-utilization)  

## Features

- **Integrated ROS 2 CLI plugin**: `ros2 unbag <args>`  
- **GUI interface** for interactive export  
- **Pluggable export routines** enable export of any message to any type  
- **Custom processors** to filter, transform or enrich messages  
- **Time‐aligned resampling** (`last` | `nearest`)  
- **Multi‐process** export with adjustable CPU usage  
- **JSON config** saving/loading for repeatable workflows  

## Installation 

### Prerequisites

Make sure you have a working ROS 2 installation (e.g., Humble, Iron, Jazzy, or newer) and that your environment is sourced:

```bash
source /opt/ros/<distro>/setup.bash
```

Replace `<distro>` with your ROS 2 distribution.

### From PyPI (via pip)

```bash
pip install ros2_unbag
```

### From source (via pip)

```bash
git clone https://github.com/ika-rwth-aachen/ros2_unbag.git
cd ros2_unbag
pip install .
```

### In a ROS 2 workspace (via colcon)

```bash
cd ~/ros2_ws/src
git clone https://github.com/ika-rwth-aachen/ros2_unbag.git
cd ..
colcon build --packages-select ros2_unbag
source install/setup.bash
```

### Docker 

You can skip local installs by running our ready‑to‑go Docker image:

```bash
docker pull ghcr.io/ika-rwth-aachen/ros2_unbag:latest
```

This image comes with ROS 2 Jazzy and *ros2 unbag* preinstalled. To launch it:

1. Clone or download the `docker/docker-compose.yml` in this repo.
2. Run:

   ```bash
   docker-compose -f docker/docker-compose.yml up
   ```
3. If you need the GUI, first enable X11 forwarding on your host:

   ```bash
   xhost +local:
   ```

   Then start the container as above—the GUI will appear on your desktop.


## Quick Start

You can use the tool either via a graphical user interface (GUI) or a command-line interface (CLI).

### GUI Mode

Launch the interactive interface:

```bash
ros2 unbag
```

Then follow the on‑screen prompts to pick your bag file, select topics, and choose export settings.


### CLI Mode

Run the CLI tool by calling *ros2 unbag* with a path to a rosbag and an export config, consisting of one or more topic:format:[subdirectory] combinations:

```bash
ros2 unbag <path_to_rosbag> --export </topic:format[:subdir]>…
```

Alternatively you can load a config file. In this case you do not need any `--export` flag:
```bash
ros2 unbag <path_to_rosbag> --config <config.json>
```
the structure of config files is described in [here](#config-file).

In addition to these required flags, there are some optional flags. See the table below, for all possible flags:
| Flag                        | Value/Format                        | Description                                                                                               | Usage                              | Default        |   |
| --------------------------- | ----------------------------------- | --------------------------------------------------------------------------------------------------------- | ---------------------------------- | -------------- | - |
| **`bag`**                   | `<path>`                            | Path to ROS 2 bag file (`.db3` or `.mcap`).                                                               | CLI mode (required)                | –              |   |
| **`-e, --export`**          | `/topic:format[:subdir]`            | Topic → format export spec. Repeatable.                                                                   | CLI mode (required or `--config`)  | –              |   |
| **`-o, --output-dir`**      | `<directory>`                       | Base directory for all exports.                                                                           | Optional                           | `.`            |   |
| **`--naming`**              | `<pattern>`                         | Filename pattern. Supports `%name`, `%index`, `%Y`, `%m`, `%d`, `%ros_timestamp`, etc.                    | Optional                           | `%name_%index` |   |
| **`--resample`**            | `/master:association[,discard_eps]` | Time‑align to master topic. `association` = `last` or `nearest`; `nearest` needs a numeric `discard_eps`. | Optional                           | –              |   |
| **`-p, --processing`**      | `/topic:processor[:arg1=val1,…]`    | Pre‑export processor spec. Repeatable.                                                                    | Optional                           | –              |   |
| **`--cpu-percentage`**      | `<float>`                           | % of cores for parallel export (0–100). Use `0` for single‑threaded.                                      | Optional                           | `80.0`         |   |
| **`--config`**              | `<config.json>`                     | JSON config file path. Overrides all other args (except `bag`).                                           | Optional                           | –              |   |
| **`--gui`**                 | (flag)                              | Launch Qt GUI. If no `bag`/`--export`/`--config`, GUI is auto‑started.                                    | Optional                           | `false`        |   |
| **`--use-routine`**         | `<file.py>`                         | Load a routine for this run only (no install).                                                            | Optional                           | –              |   |
| **`--use-processor`**       | `<file.py>`                         | Load a processor for this run only (no install).                                                          | Optional                           | –              |   |
| **`--install-routine`**     | `<file.py>`                         | Copy & register custom export routine.                                                                    | Standalone                         | –              |   |
| **`--install-processor`**   | `<file.py>`                         | Copy & register custom processor.                                                                         | Standalone                         | –              |   |
| **`--uninstall-routine`**   | (flag)                              | Interactive removal of an installed routine.                                                              | Standalone                         | -              |   |
| **`--uninstall-processor`** | (flag)                              | Interactive removal of an installed processor.                                                            | Standalone                         | -              |   |
| **`--help`**                | (flag)                              | Show usage information and exit.                                                                          | Standalone                         | -              |   |

⚠️ For `[text/csv]`, `[text/json]` or `[text/yaml]` exports, any changing name pattern (e.g. `%index` or date/time placeholders) will produce a separate file per message. To bundle all messages into one file, use a fixed filename (omit `%index` and any timestamp placeholders).

⚠️ If you specify the `--config` option (e.g., `--config configs/my_config.json`), the tool will load all export settings from the given JSON configuration file. In this case, all other command-line options except `<path_to_rosbag>` are ignored, and the export process is fully controlled by the config file. The `<path_to_rosbag>` is always required in CLI use.

Example: 
```bash
ros2 unbag rosbag/rosbag.mcap 
    --output-dir /docker-ros/ws/example/ --export /lidar/point_cloud:pointcloud/pcd:lidar --resample /lidar/point_cloud:last,0.2
```

## Config File
When using ros2 unbag, you can define your export settings in a JSON configuration file. This works in the GUI, as well as in the CLI version. It allows you to easily reuse your export settings without having to specify them on the command line every time.

💡 Tip: Use the GUI to create your export settings and then save them via the "Save Config" button. This will create a JSON file with all your export settings, which you can then use in the CLI version.

```jsonc
{
  "bag_path": "rosbag/data.mcap",
  "output_dir": "./out",
  "exports": [
    { "topic": "/cam/image_raw", "format": "image/png", "subdir": "cam" },
    { "topic": "/imu", "format": "text/csv" }
  ],
  "resample": [
    { "master": "/cam/image_raw", "type": "nearest", "discard_eps": 0.05 }
  ],
  "processing": [
    { "topic": "/cam/image_raw", "processor": "recolor", "args": { "color_map": 2 } }
  ],
  "naming": "%Y-%m-%d_%H-%M-%S_%name_%index",
  "cpu_percentage": 50
}
```

## Export Routines 

Export routines define the way how messages are exported from the ros2 bag file to the desired output format. The tool comes with a set of predefined routines for common message types and formats, such as:

| Identifier(s)                                       | Topic(s)                                                         | Description                                                                                                                                                                                          |
| --------------------------------------------------- | ---------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **\[image/png]**, **\[image/jpeg]**                 | • `sensor_msgs/msg/Image`<br>• `sensor_msgs/msg/CompressedImage` | Exports images via openCV to JPEG or PNG.                                                    |
| **\[pointcloud/pkl]**                               | `sensor_msgs/msg/PointCloud2`                                    | Serializes the entire `PointCloud2` message object using Python’s `pickle`, producing a `.pkl` file.                                                                                                 |
| **\[pointcloud/xyz]**                               | `sensor_msgs/msg/PointCloud2`                                    | Unpacks each point’s x, y, z floats from the binary buffer and writes one `x y z` line per point into a plain `.xyz` text file.                                                                      |
| **\[pointcloud/pcd]**                               | `sensor_msgs/msg/PointCloud2`                                    | Constructs a PCD v0.7 file and writes binary point data in PCD format to a `.pcd` file.                                                                          |
| **\[text/json]**, **\[text/yaml]**, **\[text/csv]** | *(any message type)*                                 | Generic serializer for any message type:<br>• **JSON**: one object per line (`.json`)<br>• **YAML**: full YAML doc per message (`.yaml`)<br>• **CSV**: flatten fields, write header + rows (`.csv`). |

Your message type or output format is not supported by default? No problem! You can add your own export routines to handle custom message types or output formats.

Routines are defined like this: 

```python
from ros2_unbag.core.routines.base import ExportRoutine                  # import the base class
# you can also import other packages here - e.g., numpy, cv2, etc.

@ExportRoutine("sensor_msgs/msg/PointCloud2", ["pointcloud/xyz"])        # define the message type and output format, each of these can be a list of formats
def export_pointcloud_xyz(msg, path, fmt="pointcloud/xyz"):              # define the export function
    # the name of the function does not matter
    # the parameters do need to be defined like this
        # msg: the message to export
        # path: the path to the output folder (without extension)
        # fmt: the format to export to - can be any of the formats defined in the decorator
    with open(path + ".xyz", 'w') as f:                                  # define your custom logic to export the message
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

If you installed a routine and do not want it anymore, you can delete it by calling
```bash
ros2 unbag --uninstall-routine
```
You’ll be prompted to pick which routine to uninstall.

## Processors

Processors are used to modify messages before they are exported. They can be applied to specific topics and allow you to perform operations such as filtering, transforming, or enriching the data.

You can define your own processors like this:

```python
from ros2_unbag.core.processors.base import Processor               # import the base class
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

If you installed a processor and do not want it anymore, you can delete it by calling
```bash
ros2 unbag --uninstall-processor
```
You’ll be prompted to pick which processor to uninstall.

## Resampling
In many cases, you may want to resample messages in the frequency of a master topic. This allows you to assemble a "frame" of data that is temporally aligned with a specific topic, such as a camera or LIDAR sensor. The resampling process will ensure that the messages from other topics are exported in sync with the master topic's timestamps.

ros2 unbag supports resampling of messages based on a master topic. You can specify the master topic and the resampling type (e.g., `last` or `nearest`) along with an optional discard epsilon value.

### Last
The `last` resampling type will listen for the master topic. As soon as a message of the master topic is received, a frame will be assembled, containing the last last message of any other selected topics. With an optional `discard_eps` value, you can specify a maximum time difference between the master topic message and the other topics' messages. If no message is found within the `discard_eps` value, the whole frame is discarded.

### Nearest
The `nearest` resampling type will listen for the master topic and export it along with the (temporally) nearest message of the other topics that were published in the time range of the master topic message. This resampling strategy is only usable with an `discard_eps` value, which defines the maximum time difference between the master topic message and the other topics' messages. If no message is found within the `discard_eps` value, the whole frame is discarded.

## CPU utilization
ros2 unbag uses multi-processing to export messages in parallel. The number of processes is determined by the number of CPU cores available on your system. You can control the number of processes by setting the `--cpu-percentage` option when running the CLI tool. The default value is 80%, which means that the tool will use 80% of the available CPU cores for processing. You can adjust this value to control the CPU utilization during the export process.

⚠️ Note: Parallel exports can interleave messages in a single output file. For strict, in‑order output, run with --cpu-percentage 0 to force single‑threaded processing.