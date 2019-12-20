# Cepton ROS

## Overview

This package provides ROS support for Cepton LiDAR. It is meant as reference code, and not actively developed.

## Installation

First, install [CeptonViewer](https://ceptontech.github.io/cepton_sdk_redist/cepton_viewer.html).

If you have not done so already, install ROS, and [create a catkin workspace](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment).

Change to the catkin workspace directory.

Clone the repository.

```sh
git clone https://github.com/ceptontech/cepton_ros.git src/cepton_ros
```

Run catkin make.

```sh
catkin_make
```

Source the catkin setup script.

```sh
source devel/setup.bash
```

## Getting started

Launch the ROS demo

```sh
roslaunch cepton_ros demo.launch
```

A rviz window should popup showing a sample point cloud.

To launch the driver standalone, you need to first launch the nodelet manager

```sh
roslaunch cepton_ros manager.launch
```

Then, you can launch the driver

```sh

roslaunch cepton_ros driver.launch
```

You can print a help menu for the driver launcher

```sh
roslaunch --ros-args cepton_ros driver.launch
```

### Using multiple sensors

If the `settings_dir` parameter is passed, the driver will load sensor transforms (use `CeptonViewer` to generate the settings directory)

- Publish transforms in the `cepton_transforms.json` file.

```sh
roslaunch cepton_ros driver.launch settings_dir:=<path_to_settings_directory>
```

## Capture Replay

A PCAP capture file can be converted to a bag file as follows:

```sh
roslaunch cepton_ros convert_capture.launch capture_path:=<path_to_capture.pcap> output_path:=<path_to_output.bag>
```

A PCAP capture file can also passed directly to the launch files. For example:

```sh
roslaunch cepton_ros demo.launch capture_path:=<path_to_capture.pcap>
```

## Troubleshooting

First, try viewing the sensor in CeptonViewer to determine if the issue is ROS or the sensor/network.

## Reference

### Driver nodelet

The driver nodelet is a thin wrapper around the Cepton SDK. The point type definitions can be found in `include/cepton_ros/point.hpp`.
