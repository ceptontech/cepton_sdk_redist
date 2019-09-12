#!/usr/bin/env python3
import argparse

import cepton_sdk
import cepton_sdk.load
from cepton_util.common import *


def main():
    parser = argparse.ArgumentParser(
        usage="%(prog)s [OPTIONS] output_path", formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument("output", help="Output path.")
    cepton_sdk.load.Loader.add_arguments(parser)
    parser.add_argument("--duration", default="-1",
                        help="Export duration (if negative, export entire capture file).")
    parser.add_argument("--version", action="version",
                        version="cepton_sdk {}".format(cepton_sdk.__version__))
    args = parser.parse_args()

    duration = parse_time_hms(args.duration)
    output_path = fix_path(args.output)

    # Initialize
    loader = cepton_sdk.load.Loader.from_arguments(args)
    loader.initialize()

    listener = cepton_sdk.SerialLinesListener()
    cepton_sdk.wait(duration)
    lines_dict = listener.get_lines()
    if not lines_dict:
        return
    lines = next(iter(lines_dict.values()))
    with open(output_path, "wb") as f:
        f.writelines(lines)


if __name__ == "__main__":
    main()
