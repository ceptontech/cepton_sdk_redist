#!/usr/bin/env python3
"""
Sample script to list attached sensors.
"""

import argparse
import pprint

import cepton_sdk
from cepton_util.common import *

if __name__ == "__main__":
    parser = argparse.ArgumentParser(usage="%(prog)s [OPTIONS]")
    parser.add_argument("--capture_path")
    parser.add_argument("--version", action="version",
                        version="cepton_sdk {}".format(cepton_sdk.__version__))
    args = parser.parse_args()

    capture_path = fix_path(args.capture_path)

    cepton_sdk.initialize(capture_path=capture_path)

    sensors_dict = cepton_sdk.get_sensors()
    for sensor in sensors_dict.values():
        pprint.pprint(sensor.information.to_dict())
