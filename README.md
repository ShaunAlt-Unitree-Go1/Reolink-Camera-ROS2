# Reolink-Camera-ROS2
Wirelessly stream data from the Reolink RTC-840WA camera and publish it in ROS2.

## Table of Contents
- [Contributors](#contributors)
- [Supports](#supports)
- [Hardware](#hardware)
- [Setup](#setup)
- [Usage](#usage)

## Contributors
Created by: Shaun Altmann (saltmann@deakin.edu.au).

## Supports
The following ROS2 versions have been tested and are supported by this package.
| ROS2 Version | Ubuntu Version | Single Camera | Multiple Camera |
| :---: | :---: | :---: | :---: |
| [Iron](https://docs.ros.org/en/iron/Installation.html) | [22.04](https://cdimage.ubuntu.com/releases/jammy/release/) | ✔️ | ✔️ |
| [Humble](https://docs.ros.org/en/humble/Installation.html) | [22.04](https://cdimage.ubuntu.com/releases/jammy/release/) | ✔️ | ✔️ |
| [Galactic](https://docs.ros.org/en/galactic/Installation.html) | [20.04](https://cdimage.ubuntu.com/releases/focal/release/) | ✔️ | ✔️ |
| [Foxy](https://docs.ros.org/en/foxy/Installation.html) | [20.04](https://cdimage.ubuntu.com/releases/focal/release/) | ✔️ | ✔️ |

## Hardware
This project implements the following hardware:
- 1x Router ([TP-Link Archer AX55 Pro](https://www.tp-link.com/au/home-networking/wifi-router/archer-ax55-pro/)).
- 1+ Cameras ([Reolink RLC-840WA](https://reolink.com/au/product/rlc-840wa/)).

## Setup
Setup the router using the [Router Setup Guide](docs/setup-router.md).

Setup all of the cameras using the [Camera Setup Guide](docs/setup-camera.md).

## Usage
This section presumes that you have already setup your device, router, and cameras, and have installed your desired ROS2 distribution (`<ros-distro>`) on your device.
1. Create a new ROS2 workspace.
    ``` bash
    source /opt/ros/<ros-distro>/setup.sh
    mkdir -p ~/ros2_camera_stream_ws/src
    cd ~/ros2_camera_stream_ws/src
    ```
2. Clone this GitHub repository.
    ``` bash
    git clone https://github.com/ShaunAlt-Unitree-Go1/Reolink-Camera-ROS2.git
    ```
3. Install ROS2 dependencies.
    ``` bash
    cd ~/ros2_camera_stream_ws
    rosdep install --from-paths src --ignore-src -y
    ```
4. Build the workspace.
    ``` bash
    colcon build
    source install/setup.sh
    ```
5. Run the ROS2 Camera Reader + Streamer.
    - To just run a single reader, use the following:
        ``` bash
        ros2 launch camera_ros2 launch_camera_read_stream.py \
            namespace:='<Camera-Name (e.g. "Camera001")>' \
            camera_uid:='<Camera-Username (e.g. "admin")>' \
            camera_pwd:='<Camera-Password (e.g. "Camera001!")' \
            camera_ip:='<Camera-IP (e.g. "192.168.0.10")'
        ```
    - To add a streamer to this, add the following argument to the above command: `create_streamer:="True"`.
    - You can use this process to read/stream multiple cameras at once by opening a new terminal tab/window, and running the exact same command with the values for the new camera.
    - The image data from the cameras will be published on the `/<namespace>/camera_image` ROS2 topic.
