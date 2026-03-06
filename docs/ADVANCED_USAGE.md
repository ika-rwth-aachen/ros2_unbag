# Advanced Usage

This guide covers advanced features of *ros2 unbag* including configuration files, resampling strategies, and CPU utilization tuning.

## Table of Contents

- [Configuration Files](#configuration-files)
  - [Creating Configuration Files](#creating-configuration-files)
  - [Configuration File Structure](#configuration-file-structure)
  - [Using Configuration Files](#using-configuration-files)
- [Resampling](#resampling)
  - [Last Strategy](#last-strategy)
  - [Nearest Strategy](#nearest-strategy)
- [CPU Utilization](#cpu-utilization)

## Configuration Files

When using *ros2 unbag*, you can define your export settings in a JSON configuration file. This works in both the GUI and CLI versions, allowing you to easily reuse your export settings without having to specify them on the command line every time.

### Creating Configuration Files

💡 **Pro Tip**: The easiest way to create a configuration file is through the GUI! Simply configure your export settings visually, then click the **"Save Config"** button. This generates a JSON file with all your settings, which you can then use in the CLI for automated workflows. This GUI-to-CLI workflow is perfect for developing and testing configurations interactively before deploying them in production scripts.

### Configuration File Structure

Here's an example configuration file:

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
    "processors": [
      {"name": "transform_from_yaml", "args": {"custom_frame_path": "test.yml"}}
    ]
  },
  "__global__": {
    "cpu_percentage": 85.0,
    "continue_on_error": false,
    "resample_config": {
      "master_topic": "/drivers/lidar_fl/pointcloud",
      "association": "nearest",
      "discard_eps": 0.5
      }
  }
}
```

#### Configuration Fields

**Per-topic settings:**
- `format`: Export format identifier (see [Supported Formats](./EXPORT_ROUTINES.md#specialized-routines))
- `path`: Output directory path
- `subfolder`: Subdirectory pattern (supports `%name` placeholder)
- `naming`: Filename pattern (supports `%name`, `%index`, `%timestamp`, `%master_timestamp`, and strftime formats)
- `processors`: Optional array of processor configurations with `name` and `args`

**Global settings (`__global__`):**
- `cpu_percentage`: Percentage of CPU cores to use (0-100)
- `continue_on_error`: If `true`, item-level processor/routine errors are logged and skipped; if `false`, export aborts on first such error
- `resample_config`: Resampling configuration (see [Resampling](#resampling) section)

### Using Configuration Files

Load a configuration file in the CLI:

```bash
ros2 unbag <path_to_rosbag> --config <config.json>
```

> [!IMPORTANT]
> If you specify the `--config` option, the tool will load all export settings from the given JSON configuration file as the baseline. Any CLI flags you explicitly provide then override matching fields. The `<path_to_rosbag>` positional argument is still required in CLI mode.

## Resampling

In many cases, you may want to resample messages at the frequency of a master topic. This allows you to assemble a "frame" of data that is temporally aligned with a specific topic, such as a camera or LIDAR sensor. The resampling process ensures that messages from other topics are exported in sync with the master topic's timestamps.

*ros2 unbag* supports two resampling strategies: `last` and `nearest`.

### Last Strategy

The `last` resampling strategy listens for the master topic. As soon as a message from the master topic is received, a frame is assembled containing the last message from any other selected topics.

**CLI Usage:**

```bash
ros2 unbag <bag> --export <topics> --resample /master_topic:last
```

**Optional discard epsilon:**

With an optional `discard_eps` value, you can specify a maximum time difference between the master topic message and the other topics' messages. If no message is found within the `discard_eps` value, the whole frame is discarded.

```bash
ros2 unbag <bag> --export <topics> --resample /master_topic:last,0.2
```

**Example:**

```bash
ros2 unbag rosbag.mcap \
    --export /lidar/point_cloud:pointcloud/pcd \
    --export /camera/image:image/png \
    --resample /lidar/point_cloud:last,0.2
```

This exports point clouds and camera images, ensuring each camera image is the last one published before each point cloud, within 0.2 seconds.

### Nearest Strategy

The `nearest` resampling strategy listens for the master topic and exports it along with the temporally nearest message from the other topics that were published in the time range of the master topic message.

**CLI Usage:**

This resampling strategy requires a `discard_eps` value, which defines the maximum time difference between the master topic message and the other topics' messages. If no message is found within the `discard_eps` value, the whole frame is discarded.

```bash
ros2 unbag <bag> --export <topics> --resample /master_topic:nearest,<discard_eps>
```

**Example:**

```bash
ros2 unbag rosbag.mcap \
    --export /lidar/point_cloud:pointcloud/pcd \
    --export /camera/image:image/png \
    --resample /lidar/point_cloud:nearest,0.5
```

This exports point clouds and camera images, ensuring each camera image is the temporally closest one to each point cloud, within 0.5 seconds.

## CPU Utilization

*ros2 unbag* uses multi-processing to export messages in parallel, significantly improving performance for large bag files.

### Default Behavior

- **Multi-file exports**: Full parallelization is applied by default
- **Single-file exports**: One process per file to ensure deterministic ordering (still utilizes multi-processing but with limited concurrency)

### Controlling CPU Usage

You can control the number of processes by setting the `--cpu-percentage` option:

```bash
ros2 unbag <bag> --export <topics> --cpu-percentage 50
```

**Values:**
- `0`: Single-threaded execution (no parallelization)
- `1-100`: Percentage of available CPU cores to use
- Default: `80` (uses 80% of available cores)

### Examples

**Use all available cores:**
```bash
ros2 unbag rosbag.mcap --export /topic:format --cpu-percentage 100
```

**Use single-threaded mode:**
```bash
ros2 unbag rosbag.mcap --export /topic:format --cpu-percentage 0
```

**Use half of available cores:**
```bash
ros2 unbag rosbag.mcap --export /topic:format --cpu-percentage 50
```

💡 **Tip**: Adjust the CPU percentage based on your system's workload. Lower values are useful when running other intensive tasks simultaneously, while higher values maximize export speed when the system is otherwise idle.
