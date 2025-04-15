# Rust Livox HAP Driver

## Author: YinMo19

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

A Rust implementation of the Livox HAP LiDAR driver for ROS 2.

## Features

- **UDP Communication**: Handles Livox HAP's UDP protocol for point cloud data
- **Point Cloud Publishing**: Publishes `sensor_msgs/PointCloud2` messages
- **Parameter Configuration**: Supports runtime configuration via ROS parameters
- **Multi-threaded**: Uses Tokio for async I/O operations
- **Protocol Support**: Implements Livox's PCD1 data format

## Prerequisites

- ROS 2 Humble or newer
- Rust toolchain (stable)
- `rust-ros2` packages
- Livox HAP LiDAR device

## Installation

1. Clone this repository into your ROS workspace and build:

    ```bash
    # Install Rust, e.g. as described in https://rustup.rs/
    # Install ROS 2 as described in https://docs.ros.org/en/humble/Installation.html
    # Assuming you installed the minimal version of ROS 2, you need these additional packages:
    sudo apt install -y git libclang-dev python3-pip python3-vcstool # libclang-dev is required by bindgen
    # Install these plugins for cargo and colcon:
    pip install git+https://github.com/colcon/colcon-cargo.git
    pip install git+https://github.com/colcon/colcon-ros-cargo.git

    git clone https://github.com/ros2-rust/ros2_rust.git src/ros2_rust
    vcs import src < src/ros2_rust/ros2_rust_humble.repos
    . /opt/ros/humble/setup.sh

    # if example is not needed, you can remove it to speed up compilation
    rm -r src/ros2_rust/examples
    git clone https://github.com/YinMo19/Livox-HAP-Driver.git src/livox_hap_driver

    colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --symlink-install

    # or open all optimizations
    colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --cargo-args --release
    ```

2. Source the workspace:
    ```bash
    source install/setup.bash
    ```

## Configuration

Create a configuration YAML file (e.g., `config/livox_minimal.yaml`) with your device settings:

```yaml
dest_ip: "192.168.1.101" # Livox HAP device IP
udp_port: 57000 # Local UDP port
batch_dot_num: 9600 # Points per batch
lidar_line: 6 # Number of LiDAR lines
timeout_ms: 1000 # UDP receive timeout
```

## Usage

### Launch File

Launch file (`launch/livox_driver.launch.py`) to read the config.xml.

### Command Line

```bash
source install/setup.(z/ba/)sh
ros2 run livox_hap_driver livox_hap_driver_node --ros-args -p dest_ip:="192.168.1.102" -p local_udp_port:=57000
```

## Parameters

| Parameter        | Type   | Default           | Description                         |
| ---------------- | ------ | ----------------- | ----------------------------------- |
| `dest_ip`        | string | "192.168.1.101"   | Livox HAP device IP address         |
| `dest_udp_port`  | int    | 57000             | dest UDP port for communication     |
| `local_ip`       | int    | "192.168.1.50"    | Local ip IP address                 |
| `local_udp_port` | int    | 57000             | Local UDP port for communication    |
| `batch_dot_num`  | int    | 9600              | Number of points per batch          |
| `lidar_line`     | int    | 6                 | Number of LiDAR lines (1-128)       |
| `timeout_ms`     | int    | 1000              | UDP receive timeout in milliseconds |
| `frame_id`       | string | "lidar_hap_frame" | TF frame ID for point cloud         |


namespace can be set in it. But the command line param is
```sh
ros2 run xxx xxx --ros-args <xxxx....zzzz> -r __ns:="/xxxx/yyyyy"
```

## Topics

### Published

- `~/pc_raw` (`sensor_msgs/PointCloud2`) - Raw point cloud data

## Protocol Details

The driver implements:

- Livox control protocol (for device initialization)
- PCD1 data format (point cloud data)
- CRC32 validation for data integrity

## Troubleshooting

1. **No data received**:

    - Verify Livox HAP is powered on and connected
    - Check IP address configuration
    - Ensure firewall allows UDP traffic on specified ports

2. **CRC errors**:

    - Check for network interference
    - Verify Livox firmware version compatibility

3. **ROS connection issues**:
    - Ensure ROS 2 environment is properly sourced
    - Check topic namespaces match

## License

Apache License 2.0

## Author

YinMo19 - me@yinmo19.top
