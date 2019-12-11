#!/usr/bin/env python3

import argparse
import datetime
import os
import shutil
import subprocess
import sys

from cepton_util.capture import *
from cepton_util.common import *


def main():
    description = """
Clips camera, network, and ROS.

Dependencies: ffmpeg, wireshark.
"""
    parser = argparse.ArgumentParser(
        usage="%(prog)s [OPTIONS]",
        description=description, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument("-i", "--input", required=True,
                        help="Input directory.")
    parser.add_argument("-o", "--output", help="Output directory.")
    parser.add_argument("--duration")
    parser.add_argument("--end")
    parser.add_argument("--start")
    args = parser.parse_args()

    input_dir = fix_path(args.input)
    if args.output is None:
        output_dir = input_dir + "_clip"
    else:
        output_dir = fix_path(args.output)
    print("Output Directory: {}".format(output_dir))

    start = 0
    if args.start is not None:
        start = parse_time_hms(args.start)
    end = None
    if args.end is not None:
        end = parse_time_hms(args.end)
    elif args.duration is not None:
        end = start + parse_time_hms(args.duration)
    clip_capture(input_dir, output_dir, start=start, end=end)


if __name__ == "__main__":
    main()
