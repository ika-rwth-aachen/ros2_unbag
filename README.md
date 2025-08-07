<img src="ros2_unbag/ui/assets/badge.svg" height=130 align="right">

# *ros2 unbag* - fast ROS 2 bag export for any format

<p align="center">
  <img src="https://img.shields.io/github/license/ika-rwth-aachen/ros2_unbag"/>
  <a href="https://github.com/ika-rwth-aachen/ros2_unbag/actions/workflows/build_docker.yml"><img src="https://github.com/ika-rwth-aachen/ros2_unbag/actions/workflows/build_docker.yml/badge.svg"/></a>
  <a href="https://pypi.org/project/ros2-unbag/"><img src="https://img.shields.io/pypi/v/ros2-unbag?label=PyPI"/></a>
</p>

*ros2 unbag* is a ROS 2 CLI plugin with optional GUI for extracting selected topics from `.db3` or `.mcap` bag files into formats like CSV, JSON, PCD, images, and more.

It comes with export routines for [all message types](#export-routines) (sensor data, point clouds, images). You need a special file format or message type? Add your [own export plugin](#custom-export-routines) for any ROS 2 message or format, and chain custom processors to filter, transform or enrich messages (e.g. drop fields, compute derived values, remap frames).

Optional resampling synchronizes your data streams around a chosen master topic—aligning each other topic either to its last‑known sample (“last”) or to the temporally closest sample (“nearest”)—so you get a consistent sample count in your exports.

For high‑throughput workflows, *ros2 unbag* can spawn multiple worker processes and lets you tune CPU usage. Your topic selections, processor chains, export parameters and resampling mode (last or nearest) can be saved to and loaded from a JSON configuration, ensuring reproducibility across runs.

Use it as `ros2 unbag <args>` or in the GUI for a flexible, extensible way to turn bag files into the data you need.

## Table of Contents

- [Features](#features)  
- [Installation](#installation)  
  - [Prerequisites](#prerequisites)  
  - [From PyPI (via pip)](#from-pypi-via-pip)  
  - [From Source](#from-source)  
  - [Docker](#docker)  
- [Quick Start](#quick-start)  
  - [GUI Mode](#gui-mode)  
  - [CLI Mode](#cli-mode)  
- [Config File](#config-file)  
- [Export Routines](#export-routines)
  - [Custom Export Routines](#custom-export-routines)
- [Processors](#processors)  
- [Resampling](#resampling)  
  - [last](#last)  
  - [nearest](#nearest)  
- [CPU Utilization](#cpu-utilization)  
- [Acknowledgements](#acknowledgements)

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

Install the required apt dependencies:

```bash
sudo apt update
sudo apt install libxcb-cursor0 libxcb-shape0 libxcb-icccm4 libxcb-keysyms1 libxkbcommon-x11-0
```

### From PyPI (via pip)

```bash
pip install ros2-unbag
```

### From source

```bash
git clone https://github.com/ika-rwth-aachen/ros2_unbag.git
cd ros2_unbag
pip install .
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
3. If you need the GUI, first enable X11 forwarding on your host (at your own risk!):

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
| Flag                        | Value/Format                             | Description                                                                                               | Usage                              | Default        |
| --------------------------- | ---------------------------------------- | --------------------------------------------------------------------------------------------------------- | ---------------------------------- | -------------- |
| **`bag`**                   | `<path>`                                 | Path to ROS 2 bag file (`.db3` or `.mcap`).                                                               | CLI mode (required)                | –              |
| **`-e, --export`**          | `/topic:format[:subdir]`                 | Topic → format export spec. Repeatable.                                                                   | CLI mode (required or `--config`)  | –              |
| **`-o, --output-dir`**      | `<directory>`                            | Base directory for all exports.                                                                           | Optional                           | `.`            |
| **`--naming`**              | `<pattern>`                              | Filename pattern. Supports `%name`, `%index` and strftime (e.g. `%Y-%m-%d_%H-%M-%S`) - uses ROS timestamp | Optional                           | `%name_%index` |
| **`--resample`**            | `/master:association[,discard_eps]`.     | Time‑align to master topic. `association` = `last` or `nearest`; `nearest` needs a numeric `discard_eps`. | Optional                           | –              |
| **`-p, --processing`**      | `/topic:processor[:arg1=val1,…]`         | Pre‑export processor spec. Repeatable.                                                                    | Optional                           | –              |
| **`--cpu-percentage`**      | `<float>`                                | % of cores for parallel export (0–100). Use `0` for single‑threaded.                                      | Optional                           | `80.0`         |
| **`--config`**              | `<config.json>`                          | JSON config file path. Overrides all other args (except `bag`).                                           | Optional                           | –              |
| **`--gui`**                 | (flag)                                   | Launch Qt GUI. If no `bag`/`--export`/`--config`, GUI is auto‑started.                                    | Optional                           | `false`        |
| **`--use-routine`**         | `<file.py>`                              | Load a routine for this run only (no install).                                                            | Optional                           | –              |
| **`--use-processor`**       | `<file.py>`                              | Load a processor for this run only (no install).                                                          | Optional                           | –              |
| **`--install-routine`**     | `<file.py>`                              | Copy & register custom export routine.                                                                    | Standalone                         | –              |
| **`--install-processor`**   | `<file.py>`                              | Copy & register custom processor.                                                                         | Standalone                         | –              |
| **`--uninstall-routine`**   | (flag)                                   | Interactive removal of an installed routine.                                                              | Standalone                         | -              |
| **`--uninstall-processor`** | (flag)                                   | Interactive removal of an installed processor.                                                            | Standalone                         | -              |
| **`--help`**                | (flag)                                   | Show usage information and exit.                                                                          | Standalone                         | -              |

Example: 
```bash
ros2 unbag rosbag/rosbag.mcap 
    --output-dir /docker-ros/ws/example/ --export /lidar/point_cloud:pointcloud/pcd:lidar --export /radar/point_cloud:pointcloud/pcd:radar --resample /lidar/point_cloud:last,0.2
```

⚠️ If you specify the `--config` option (e.g., `--config configs/my_config.json`), the tool will load all export settings from the given JSON configuration file. In this case, all other command-line options except `<path_to_rosbag>` are ignored, and the export process is fully controlled by the config file. The `<path_to_rosbag>` is always required in CLI use.

## Config File
When using ros2 unbag, you can define your export settings in a JSON configuration file. This works in the GUI, as well as in the CLI version. It allows you to easily reuse your export settings without having to specify them on the command line every time.

💡 Tip: Use the GUI to create your export settings and then save them via the "Save Config" button. This will create a JSON file with all your export settings, which you can then use in the CLI version.

```jsonc
{
  "/imu/pos": {
    "format": "text/json@single_file",
    "path": "/docker-ros/data/rosbag2_2025_08_19-12_34_56",
    "subfolder": "%name",
    "naming": "%name"
  },
  "/drivers/lidar_fl/nearir_image": {
    "format": "image/png",
    "path": "/docker-ros/data/rosbag2_2025_08_19-12_34_56",
    "subfolder": "%name",
    "naming": "%name_%index"
  },
  "/drivers/lidar_fl/pointcloud": {
    "format": "pointcloud/pcd",
    "path": "/docker-ros/data/rosbag2_2025_08_19-12_34_56",
    "subfolder": "%name",
    "naming": "%name_%index",
    "processor": "transform_from_yaml",
    "processor_args": {
      "custom_frame_path": "test.yml"
    }
  },
  "__global__": {
    "cpu_percentage": 85.0,
    "resample_config": {
      "master_topic": "/drivers/lidar_fl/pointcloud",
      "association": "nearest",
      "discard_eps": 0.5
      }
  }
}
```

## Export Routines 

Export routines define the way how messages are exported from the ros2 bag file to the desired output format. The tool comes with a set of predefined routines for **all** message types and formats, such as:

| Identifier(s)       | Topic(s)                                                      | Description                                                                                                                                |
| ------------------- | ------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------ |
| **image/png**                  | `sensor_msgs/msg/Image`<br> `sensor_msgs/msg/CompressedImage` | Exports images via openCV to PNG.                                                                                               |  
| **image/jpeg**                 | `sensor_msgs/msg/Image`<br> `sensor_msgs/msg/CompressedImage` | Exports images via openCV to JPEG.                                                                                              |
| **video/mp4**                  | `sensor_msgs/msg/Image`<br> `sensor_msgs/msg/CompressedImage` | Exports image sequences via openCV to MP4.                                                                                      |
| **video/avi**                  | `sensor_msgs/msg/Image`<br> `sensor_msgs/msg/CompressedImage` | Exports image sequences via openCV to AVI.                                                                                      |
| **pointcloud/pkl**             | `sensor_msgs/msg/PointCloud2`                                 | Serializes the entire `PointCloud2` message object using Python’s `pickle`, producing a `.pkl` file.                            |
| **pointcloud/xyz**             | `sensor_msgs/msg/PointCloud2`                                 | Unpacks each point’s x, y, z floats from the binary buffer and writes one `x y z` line per point into a plain `.xyz` text file. |
| **pointcloud/pcd**             | `sensor_msgs/msg/PointCloud2`                                 | Constructs a PCD v0.7 file and writes binary point data* in PCD format to a `.pcd` file.                                        |
| **pointcloud/pcd_compressed**  | `sensor_msgs/msg/PointCloud2`                                 | Constructs a PCD v0.7 file and writes compressed binary point data* in PCD format to a `.pcd` file.                             |
| **pointcloud/pcd_ascii**       | `sensor_msgs/msg/PointCloud2`                                 | Constructs a PCD v0.7 file and writes ASCII point data* in PCD format to a `.pcd` file.                                         |

***Note:** Point data in PCD files is written with all fields, that are present in the `PointCloud2` message. Some programs do not support arbitrary fields in PCD files. If you need to export only specific fields, you can use the `remove_fields` processor to drop unwanted fields before exporting. See the [Processors](#processors) section for more information.*

In addition to these specialized routines, there are also generic routines for exporting any message type to common formats. These are available as `@single_file` and `@multi_file` variants, which determine whether all messages are written to a single file or each message is written to its own file:

| Identifier    | Topic(s)             | `@single_file` Description                                                      | `@multi_file` Description                                                         |
| ------------- | -------------------- | ------------------------------------------------------------------------------- | --------------------------------------------------------------------------------- |
| **table/csv** | *any message type*   | Flattens fields, writes header + one row per message into a single `.csv` file. | Flattens fields, writes header + one message per file into separate `.csv` files. |
| **text/json** | *any message type*   | All messages in one `.json` file as a list of objects.                          | One `.json` file per message.                                                     |
| **text/yaml** | *any message type*   | One `.yaml` document containing all messages in a single `.yaml` file.          | One `.yaml` document per message.                                                 |

You can call these as `table/csv@single_file` or `table/csv@multi_file`.

### Custom Export Routines
Your message type or output format is not supported by default? No problem! You can add your own export routines to handle custom message types or output formats.

Routines are defined like this: 

```python
from pathlib import Path                                                          # import Path from pathlib for file path handling
from ros2_unbag.core.routines.base import ExportRoutine                           # import the base class
# you can also import other packages here - e.g., numpy, cv2, etc.

@ExportRoutine("sensor_msgs/msg/PointCloud2", ["pointcloud/xyz"], mode=ExportMode.MULTI_FILE)
def export_pointcloud_xyz(msg, path: Path, fmt: str, metadata: ExportMetadata):   # define the export routine function, the name of the function does not matter
    """
    Export PointCloud2 message as an XYZ text file by unpacking x, y, z floats from each point and writing lines.

    Args:
        msg: PointCloud2 message instance.
        path: Output file path (without extension).
        fmt: Export format string (default "pointcloud/xyz").
        metadata: Export metadata including message index and max index.

    Returns:
        None
    """
    with open(path + ".xyz", 'w') as f:                                            # define your custom logic to export the message
        for i in range(0, len(msg.data), msg.point_step):
            x, y, z = struct.unpack_from("fff", msg.data, offset=i)
            f.write(f"{x} {y} {z}\n")
```

A template for this, including single file handling is available in the `templates` directory of the repository. You can copy it and modify it to suit your needs.

The message type, format and mode are defined in the decorator. The `ExportRoutine` decorator registers the function as an export routine for the specified message type and format. It has the following attributes:

- `msg_types`: The message types that this routine can handle. (Can be a single type or a list of types.) Note that the message type must be installed in the system, i.e., it must be available in the ROS 2 environment.
- `formats`: The output formats that this routine supports. (Can be a single format or a list of formats.)
- `mode`: Specifies the export mode — SINGLE_FILE or MULTI_FILE. This determines whether the routine is designed for exporting data into a single file or multiple files. While this setting affects parallelization and naming conventions, you must implement the logic for single file exports yourself if you choose SINGLE_FILE mode (e.g., appending data to the same file during each function call).

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

⚠️ Never use or install new routines that you did not write yourself or that you do not trust. The code gets ingested and executed in the context of the *ros2 unbag* process, which means it can access all data and resources available to the process. This includes reading and writing files, accessing network resources, and more. Always review the code of any routine you use or install.

## Processors

Processors are used to modify messages before they are exported. They can be applied to specific topics and allow you to perform operations such as filtering, transforming, or enriching the data.

The following processors are available by default:
| Identifier(s)          | Topic(s)                                                            |    Arguments                                                   | Description                                |
| ---------------------- | ------------------------------------------------------------------- | -------------------------------------------------------------- | ------------------------------------------ |
| **field_mapping**      | `sensor_msgs/msg/PointCloud2`                                       | `field_mapping` String in the form `old_field:new_field, ...`  | Remaps fields in a PointCloud2 message.    |
| **remove_fields**      | `sensor_msgs/msg/PointCloud2`                                       | `fields_to_remove` List of field names to remove `field1, ...` | Removes specified fields from PointCloud2. |
| **transform_from_yaml**| `sensor_msgs/msg/PointCloud2`                                       | `custom_frame_path` Path to a YAML file with custom frame data | Transforms PointCloud2 to a custom frame.  |
| **apply_color_map**    | `sensor_msgs/msg/Image` <br> `sensor_msgs/msg/CompressedImage`      | `color_map` Integer specifying cv2 colormap index*.            | Applies a color map to an image.           |

*Note: The `color_map` argument is an integer that specifies the OpenCV colormap index. You can find a list of available colormaps in the [OpenCV documentation](https://docs.opencv.org/4.x/d3/d50/group__imgproc__colormap.html).*

### Custom Processors

You can define your own processors like this:

```python
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
```

A template for this is available in the `templates` directory of the repository. You can copy it and modify it to suit your needs.

The message type and processor name are defined in the decorator. The `Processor` decorator registers the function as a processor for the specified message type and name. It has the following attributes:

- `msg_types`: The message types that this processor can handle. (Can be a single type or a list of types.) Note that the message type must be installed in the system, i.e., it must be available in the ROS 2 environment.
- `name`: The name of the processor, which is used to identify it in the system.

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

⚠️ Never use or install new processes that you did not write yourself or that you do not trust. The code gets ingested and executed in the context of the *ros2 unbag* process, which means it can access all data and resources available to the process. This includes reading and writing files, accessing network resources, and more. Always review the code of any routine you use or install.

## Resampling
In many cases, you may want to resample messages in the frequency of a master topic. This allows you to assemble a "frame" of data that is temporally aligned with a specific topic, such as a camera or LIDAR sensor. The resampling process will ensure that the messages from other topics are exported in sync with the master topic's timestamps.

ros2 unbag supports resampling of messages based on a master topic. You can specify the master topic and the resampling type (e.g., `last` or `nearest`) along with an optional discard epsilon value.

### Last
The `last` resampling type will listen for the master topic. As soon as a message of the master topic is received, a frame will be assembled, containing the last message of any other selected topics. With an optional `discard_eps` value, you can specify a maximum time difference between the master topic message and the other topics' messages. If no message is found within the `discard_eps` value, the whole frame is discarded.

### Nearest
The `nearest` resampling type will listen for the master topic and export it along with the (temporally) nearest message of the other topics that were published in the time range of the master topic message. This resampling strategy is only usable with an `discard_eps` value, which defines the maximum time difference between the master topic message and the other topics' messages. If no message is found within the `discard_eps` value, the whole frame is discarded.

## CPU utilization
*ros2 unbag* uses multi-processing to export messages in parallel. By default, full parallelization is applied only when exporting to multiple files. For single-file outputs, it uses one process per file to ensure deterministic ordering, which still utilizes multi-processing but with limited concurrency. You can control the number of processes by setting the --cpu-percentage option. The default value is 80%, meaning the tool will use 80% of available CPU cores for processing. Adjust this value to control CPU utilization during export.

## Acknowledgements
This research is accomplished within the following research projects:

| Project | Funding Source |      | 
|---------|----------------|:----:|
| <a href="https://www.ika.rwth-aachen.de/de/kompetenzen/projekte/automatisiertes-fahren/4-cad.html"><img src="https://www.ika.rwth-aachen.de/images/projekte/4cad/4cad-logo.svg" alt="4-CAD" height="40"/></a> | Funded by the Deutsche Forschungsgemeinschaft (DFG, German Research Foundation) DFG Proj. Nr. 503852364 | <p align="center"><img src="https://www.ika.rwth-aachen.de/images/foerderer/dfg.svg" height="50"/></p> |
| <a href="https://iexoddus-project.eu/"><img src="https://www.ika.rwth-aachen.de/images/projekte/iexoddus/iEXODDUS%20Logo%20color.svg" alt="iEXXODUS" height="40"/></a> | Funded by the European Union’s Horizon Europe Research and Innovation Programme under Grant Agreement No 101146091 | <p align="center"><img src="https://www.ika.rwth-aachen.de/images/foerderer/eu.svg" height="50"/></p> |
| <a href="https://synergies-ccam.eu/"><img src="https://www.ika.rwth-aachen.de/images/projekte/synergies/SYNERGIES_Logo%201.png" alt="SYNERGIES" height="40"/></a> | Funded by the European Union’s Horizon Europe Research and Innovation Programme under Grant Agreement No 101146542 | <p align="center"><img src="https://www.ika.rwth-aachen.de/images/foerderer/eu.svg" height="50"/></p> |

## Notice 

> [!IMPORTANT]  
> This repository is open-sourced and maintained by the [**Institute for Automotive Engineering (ika) at RWTH Aachen University**](https://www.ika.rwth-aachen.de/).  
> We cover a wide variety of research topics within our [*Vehicle Intelligence & Automated Driving*](https://www.ika.rwth-aachen.de/en/competences/fields-of-research/vehicle-intelligence-automated-driving.html) domain.  
> If you would like to learn more about how we can support your automated driving or robotics efforts, feel free to reach out to us!  
> :email: ***opensource@ika.rwth-aachen.de***