#!/usr/bin/env python3

import argparse
import os
import sys

from cepton_util.capture import *
from cepton_util.common import *


def main():
    description = """
Captures network and video.

Dependencies: dumpcap, ffmpeg
"""
    parser = argparse.ArgumentParser(
        usage="%(prog)s [OPTIONS] output_dir",
        description=description, formatter_class=argparse.RawTextHelpFormatter)
    OutputDataDirectory.add_arguments(parser)
    parser.add_argument(
        "--network", action="store_true", help="Capture network.")
    parser.add_argument("--network_interface")
    parser.add_argument("--ros_topics")
    parser.add_argument("--settings_dir")
    parser.add_argument("--video_devices")
    args = parser.parse_args()

    if (not args.network) and (args.video_devices is None):
        raise ValueError("Nothing to capture!")

    capture = OutputDataDirectory.from_arguments(args)
    print("Capture path: {}".format(capture.path))

    settings_dir = fix_path(args.settings_dir)
    if settings_dir is None:
        settings_dir = os.getcwd()
    print("Settings path: {}".format(settings_dir))
    capture.copy_settings(settings_dir)

    if args.network:
        network_interface = args.network_interface
        if network_interface is None:
            network_interface = find_network_interface()
        print("Network interface: {}".format(network_interface))
        pcap_capture = PCAPCapture(
            capture.pcap_path, interface=network_interface)

    if args.ros_topics is not None:
        ros_topics = sorted(args.ros_topics.split(","))
        print("ROS topics: {}".format(ros_topics))
        BagCapture(ros_topics, capture.bag_path)

    if args.video_devices is not None:
        video_devices = sorted(args.video_devices.split(","))
        print("Video devices: {}".format(video_devices))
        for i, device in enumerate(video_devices):
            device = device.strip()
            camera_capture = CameraCapture(device, capture.camera_path(i))

    capture.wait()


if __name__ == "__main__":
    main()
