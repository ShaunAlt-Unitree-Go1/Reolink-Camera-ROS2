# Reolink-Camera-ROS2
Wirelessly stream data from the Reolink RTC-840WA camera and publish it in ROS2.

## Table of Contents
- [Contributors](#contributors)
- [Hardware](#hardware)
- [Setup](#setup)
- [Usage](#usage)

## Contributors
Created by: Shaun Altmann (saltmann@deakin.edu.au).

## Supports
The following ROS2 versions have been tested and are supported by this package.
<!-- `&#9745;` = Tick in Box, `&#9746;` = Cross in Box -->
| ROS2 Version | Ubuntu Version | Single Camera | Multiple Camera |
| :---: | :---: | :---: | :---: |
| [Jazzy](https://docs.ros.org/en/jazzy/Installation.html) | [24.04](https://cdimage.ubuntu.com/releases/noble/release/) | &#9746; | &#9746; |
| [Iron](https://docs.ros.org/en/iron/Installation.html) | [22.04](https://cdimage.ubuntu.com/releases/jammy/release/) | &#9745; | &#9745; |
| [Humble](https://docs.ros.org/en/humble/Installation.html) | [22.04](https://cdimage.ubuntu.com/releases/jammy/release/) | &#9746; | &#9746; |
| [Galactic](https://docs.ros.org/en/galactic/Installation.html) | [20.04](https://cdimage.ubuntu.com/releases/focal/release/) | &#9746; | &#9746; |
| [Foxy](https://docs.ros.org/en/foxy/Installation.html) | [20.04](https://cdimage.ubuntu.com/releases/focal/release/) | &#9746; | &#9746; |
| [Eloquent](https://docs.ros.org/en/eloquent/Installation.html) | [18.04](https://cdimage.ubuntu.com/releases/bionic/release/) | &#9746; | &#9746; |
| [Dashing](https://docs.ros.org/en/dashing/Installation.html) | [18.04](https://cdimage.ubuntu.com/releases/bionic/release/) | &#9746; | &#9746; |


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
5. Run the ROS2 script you require.
    - Single Camera Streamer:
        ``` bash
        ros2 run camera_ros2 camera_reader \
            -p camera_uid:=<Camera-Username>
            -p camera_pwd:=<Camera-Password>
            -p camera_ip:=<Camera-IP>
        ```
        You can run the Single Camera Streamer with additional arguments, the documentation for which you can find in the [camera_streamer.py](camera_ros2/camera_ros2/camera_reader.py) source code.
    - Multiple Cameras Streamer:
        - This is identical to the Single Camera Streamer, except that now you will add a namespace to each camera you create.
        - Terminal 1:
            ``` bash
            ros2 run camera_ros2 camera_reader \
                -r __ns:=/<Camera-1-Namespace>
                -p camera_uid:=<Camera-1-Username>
                -p camera_pwd:=<Camera-1-Password>
                -p camera_ip:=<Camera-1-IP>
            ```
        - Terminal 2:
            ``` bash
            ros2 run camera_ros2 camera_reader \
                -r __ns:=/<Camera-2-Namespace>
                -p camera_uid:=<Camera-2-Username>
                -p camera_pwd:=<Camera-2-Password>
                -p camera_ip:=<Camera-2-IP>
            ```
        - Extra cameras can be connected to in additional terminals by following the same format.
