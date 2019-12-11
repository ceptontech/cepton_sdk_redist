#!/usr/bin/env python3
"""
Sample script for getting points from a single sensor.
"""

import pprint

import numpy

import cepton_sdk
import cepton_sdk.plot
from common import *

if __name__ == "__main__":
    # Variables
    capture_path = get_sample_capture_path()

    # Initialize
    cepton_sdk.initialize(capture_path=capture_path, enable_wait=True)

    # Get sensor
    sensor = cepton_sdk.Sensor.create_by_index(0)
    pprint.pprint(sensor.information.to_dict())

    # Get points
    listener = cepton_sdk.SensorFramesListener(sensor.serial_number)
    points_list = listener.get_points()
    del listener
    points = points_list[0]

    # Plot
    cepton_sdk.plot.plot_points(points)
