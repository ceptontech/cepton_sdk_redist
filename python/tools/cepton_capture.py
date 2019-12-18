#!/usr/bin/env python3

import argparse
import os
import sys

from cepton_util.capture import *
from cepton_util.common import *


def main():
    description = """
Captures camera, network, ROS, and serial.

Dependencies: ffmpeg, wireshark.
"""
    parser = argparse.ArgumentParser(
        usage="%(prog)s [OPTIONS] output_dir",
        description=description, formatter_class=argparse.RawTextHelpFormatter)
    OutputDataDirectory.add_arguments(parser)
    parser.add_argument(
        "--network", action="store_true", help="Capture network.")
    parser.add_argument("--camera_devices")
    parser.add_argument("--network_interface")
    parser.add_argument("--ros_topics")
    parser.add_argument("--settings_dir")
    parser.add_argument("--serial_ports")
    args = parser.parse_args()

    capture = OutputDataDirectory.from_arguments(args)
    print("Capture Path: {}".format(capture.path))

    settings_dir = fix_path(args.settings_dir)
    if settings_dir is None:
        settings_dir = os.getcwd()
    print("Settings Path: {}".format(settings_dir))
    capture.copy_settings(settings_dir)

    if args.camera_devices is not None:
        camera_devices = sorted(args.camera_devices.split(","))
        print("Camera Devices: {}".format(camera_devices))
        for i, device in enumerate(camera_devices):
            device = device.strip()
            camera_capture = CameraCapture(device, capture.camera_path(i))

    if args.network:
        network_interface = args.network_interface
        if network_interface is None:
            network_interface = find_network_interface()
        print("Network Interface: {}".format(network_interface))
        network_capture = NetworkCapture(
            capture.network_path, interface=network_interface)

    if args.ros_topics is not None:
        ros_topics = sorted(args.ros_topics.split(","))
        print("ROS Topics: {}".format(ros_topics))
        ROSCapture(ros_topics, capture.ros_path)

    if args.serial_ports is not None:
        serial_ports = sorted(args.serial_ports.split(","))
        print("Serial Ports: {}".format(serial_ports))
        for i, port in enumerate(serial_ports):
            SerialCapture(port, capture.serial_path(i))

    print("Capturing...")
    capture.wait()
    print("Processing...")


if __name__ == "__main__":
    main()
