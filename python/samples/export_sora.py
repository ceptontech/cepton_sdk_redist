#!/usr/bin/env python3
"""
Sample script to export SORA point cloud to file. Assumes constant speed
in the +Y direction (e.g. mounted to a car).
"""

import argparse

import numpy

import cepton_sdk
import cepton_sdk.export
import cepton_sdk.load
from cepton_util.common import *


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("output")
    cepton_sdk.load.Loader.add_arguments(parser)
    all_file_types = [x.name for x in cepton_sdk.export.PointsFileType]
    parser.add_argument("--format", default="LAS", choices=all_file_types)
    parser.add_argument("--speed", type=float)
    parser.add_argument("--t_length", default="1", help="Maximum export time.")
    args = parser.parse_args()

    file_type = cepton_sdk.export.PointsFileType[args.format.upper()]
    output_path = fix_path(args.output)
    speed = args.speed if (args.speed is not None) else 0
    t_length = None
    if args.t_length is not None:
        t_length = parse_time_hms(args.t_length)

    # Get points
    loader = cepton_sdk.load.Loader.from_arguments(args)
    image_points_dict = \
        cepton_sdk.get_image_points(t_length, return_partial=True)
    points = loader.process_points(image_points_dict)

    # Apply speed
    t = points.timestamps - numpy.amin(points.timestamps)
    points.positions[:, 1] -= speed * t

    # Save
    cepton_sdk.export.save_points(points, output_path, file_type=file_type)


if __name__ == "__main__":
    main()
