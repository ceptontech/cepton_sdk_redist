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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("output", help="Output directory.")
    parser.add_argument("--capture_path")
    parser.add_argument("--capture_seek", type=float)
    all_file_types = [x.name for x in cepton_sdk.export.PointsFileType]
    parser.add_argument("--format", default="LAS", choices=all_file_types)
    parser.add_argument("--t_length", type=float, help="Maximum export time.")
    args = parser.parse_args()

    output_dir = fix_path(args.output)
    shutil.rmtree(output_dir, ignore_errors=True)

    options = {
        "capture_path": fix_path(args.capture_path),
    }
    cepton_sdk.initialize(**options)
    if args.capture_seek is not None:
        cepton_sdk.capture_replay.seek(args.capture_seek)

    file_type = cepton_sdk.export.PointsFileType[args.format.upper()]

    t_0 = cepton_sdk.get_time()
    i_frame = collections.defaultdict(lambda: 0)
    while True:
        if args.t_length is not None:
            if (cepton_sdk.get_time() - t_0) > args.t_length:
                break
        try:
            image_points_dict = cepton_sdk.get_image_frames()
        except:
            break
        for serial_number, image_points_list in image_points_dict.items():
            sensor_dir = os.path.join(output_dir, str(serial_number))
            if not os.path.isdir(sensor_dir):
                os.makedirs(sensor_dir)
            for image_points in image_points_list:
                i_frame_tmp = i_frame[serial_number]
                points = image_points.to_points()
                path = os.path.join(sensor_dir, str(i_frame_tmp))

                # Export to file
                cepton_sdk.export.save_points(
                    points, path, file_type=file_type)

                i_frame[serial_number] += 1


if __name__ == "__main__":
    main()
