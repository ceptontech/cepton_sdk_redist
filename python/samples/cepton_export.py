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
from cepton_util.common import *


def filter_points(points):
    is_valid = numpy.logical_and.reduce([
        points.valid,
        points.distances < 1e4,
    ])
    return points[is_valid]


def process_points(points):
    # TODO: process points before exporting
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
        usage="%(prog)s [OPTIONS] output_dir", description=description, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument("output_dir", help="Output directory.")
    parser.add_argument("--capture_path", help="Path to PCAP capture file.")
    parser.add_argument("--capture_seek", type=float,
                        help="Capture file seek position [seconds].")
    parser.add_argument("--combine", action="store_true",
                        help="Combine points into single file per sensor.")
    parser.add_argument("--duration", default="0",
                        help="Export duration (if negative, export entire capture file).")
    all_file_types = [x.name for x in cepton_sdk.export.PointsFileType]
    parser.add_argument("--format", default="LAS", choices=all_file_types,
                        type=str.upper, help="Output file format.")
    parser.add_argument("--version", action="version",
                        version="cepton_sdk {}".format(cepton_sdk.__version__))
    args = parser.parse_args()

    file_type = cepton_sdk.export.PointsFileType[args.format]
    duration = parse_time_hms(args.duration)

    output_dir = fix_path(remove_extension(args.output_dir))
    shutil.rmtree(output_dir, ignore_errors=True)
    os.makedirs(output_dir)

    # Initialize
    options = {}
    if args.combine:
        options.update({
            "frame_length": 1,
            "frame_mode": cepton_sdk.FrameMode.TIMED,
        })
    if args.capture_path is not None:
        options["capture_path"] = fix_path(args.capture_path)
    cepton_sdk.initialize(**options)
    if args.capture_seek is not None:
        cepton_sdk.capture_replay.seek(args.capture_seek)

    listener = cepton_sdk.FramesListener()
    if args.combine:
        cepton_sdk.wait(duration)
        points_dict = listener.get_points()
        for serial_number, points_list in points_dict.items():
            points = cepton_sdk.combine_points(points_list)
            points = process_points(points)
            points = filter_points(points)

            # Save
            path = os.path.join(output_dir, str(serial_number))
            cepton_sdk.export.save_points(points, path, file_type=file_type)
    else:
        t_0 = cepton_sdk.get_time()
        i_frame = collections.defaultdict(lambda: 0)
        while True:
            if duration >= 0:
                if (cepton_sdk.get_time() - t_0) > duration:
                    break
            try:
                points_dict = listener.get_points()
            except:
                break
            for serial_number, points_list in points_dict.items():
                sensor_dir = os.path.join(output_dir, str(serial_number))
                if not os.path.isdir(sensor_dir):
                    os.makedirs(sensor_dir)
                for points in points_list:
                    points = process_points(points)
                    points = filter_points(points)

                    # Save
                    i_frame_tmp = i_frame[serial_number]
                    i_frame[serial_number] += 1
                    path = os.path.join(sensor_dir, str(i_frame_tmp))
                    cepton_sdk.export.save_points(
                        points, path, file_type=file_type)


if __name__ == "__main__":
    main()
