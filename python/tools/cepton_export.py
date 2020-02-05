#!/usr/bin/env python3
"""
Sample script to export frames to files. Can be modified to export to any
file format.
"""

import argparse
import collections
import os.path
import shutil

import numpy

import cepton_sdk
import cepton_sdk.capture_replay
import cepton_sdk.export
import cepton_sdk.load
from cepton_util.common import *


def filter_points(points):
    is_valid = numpy.logical_and.reduce([
        points.valid,
        points.distances < 1e4,
    ])
    return points[is_valid]


def process_points(points):
    points = filter_points(points)

    # TODO: add extra processing here

    return points


def main():
    description = """
Exports Cepton LiDAR points from live sensors or a capture file.
By default, exports frames to individual files.

- Export single frame from capture `lidar.pcap` to CSV files in directory `points`:
    python3 cepton_export --capture_path lidar.pcap --format CSV points
- Export all data from capture `lidar.pcap` to CSV files in directory `points`:
    python3 cepton_export --capture_path lidar.pcap --format CSV --duration -1 points
"""
    parser = argparse.ArgumentParser(
        usage="%(prog)s [OPTIONS] output_dir",
        description=description, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument("output_dir", help="Output directory.")
    cepton_sdk.load.Loader.add_arguments(parser)
    parser.add_argument("--combine_frames", action="store_true",
                        help="Combine points into single file per sensor.")
    parser.add_argument("--combine_sensors", action="store_true",
                        help="Combine points into single file per frame.")
    parser.add_argument("--duration", default="0",
                        help="Export duration (if negative, export entire capture file).")
    all_file_types = [x.name for x in cepton_sdk.export.PointsFileType]
    parser.add_argument("--format", default="LAS", choices=all_file_types,
                        type=str.upper, help="Output file format.")
    parser.add_argument("--version", action="version",
                        version="cepton_sdk {}".format(cepton_sdk.__version__))
    args = parser.parse_args()

    if args.combine_sensors and (not args.combine_frames):
        raise NotImplementedError()

    file_type = cepton_sdk.export.PointsFileType[args.format]
    duration = parse_time_hms(args.duration)

    # Create directory
    output_dir = fix_path(remove_extension(args.output_dir))
    if not (args.combine_sensors and args.combine_frames):
        os.makedirs(output_dir, exist_ok=True)

    # Initialize
    loader = cepton_sdk.load.Loader.from_arguments(args)
    loader.sdk_options.update({
        "enable_wait": True,
    })
    # TODO: uncomment to set custom frame length
    # loader.sdk_options.update({
    #     "frame_length": 0.1,
    #     "frame_mode": cepton_sdk.FrameMode.TIMED,
    # })
    if args.combine_frames and ((duration < 0) or (duration > 10)):
        # Performance optimization
        loader.sdk_options.update({
            "frame_length": 1,
            "frame_mode": cepton_sdk.FrameMode.TIMED,
        })
    loader.initialize()

    # Run
    listener = cepton_sdk.FramesListener()
    if args.combine_frames:
        # Get points
        cepton_sdk.wait(duration)
        points_dict = listener.get_points()
        points_list = []
        for serial_number, points_list_tmp in points_dict.items():
            # Process points
            points = cepton_sdk.combine_points(points_list_tmp)
            points = loader.process_sensor_points(serial_number, points)
            points = process_points(points)
            points_list.append(points)

            # Save
            if not args.combine_sensors:
                path = os.path.join(output_dir, str(serial_number))
                cepton_sdk.export.save_points(
                    points, path, file_type=file_type)
        # Save
        if args.combine_sensors:
            path = output_dir
            points = cepton_sdk.combine_points(points_list)
            cepton_sdk.export.save_points(points, path, file_type=file_type)
    else:
        t_0 = cepton_sdk.get_time()
        while True:
            # Check if done
            if (duration >= 0) and ((cepton_sdk.get_time() - t_0) > duration):
                break
            # Get points
            try:
                points_dict = listener.get_points()
            except:
                break
            for serial_number, points_list in points_dict.items():
                sensor_dir = os.path.join(output_dir, str(serial_number))
                if not os.path.isdir(sensor_dir):
                    os.makedirs(sensor_dir)
                for points in points_list:
                    # Process points
                    points = loader.process_sensor_points(
                        serial_number, points)
                    points = process_points(points)

                    # Save
                    path = os.path.join(sensor_dir, str(
                        int(1e6 * cepton_sdk.get_time())))
                    cepton_sdk.export.save_points(
                        points, path, file_type=file_type)


if __name__ == "__main__":
    main()
