#!/usr/bin/env python3

import pprint

import cepton_sdk
import cepton_sdk.plot
from common import *

if __name__ == "__main__":
    # Variables
    capture_path = get_sample_capture_path()

    # Initialize
    cepton_sdk.initialize(capture_path=capture_path)

    # Get sensors
    sensors_dict = cepton_sdk.get_sensors()

    # Get points
    listener = cepton_sdk.FramesListener()
    points_dict = listener.get_points()
    del listener
    points_list = next(iter(points_dict.values()))
    points = points_list[0]

    # Plot
    cepton_sdk.plot.plot_points(points)
