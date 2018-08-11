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
    listener = cepton_sdk.ImageFramesListener()
    image_points_dict = listener.get_points()
    del listener
    image_points_list = next(iter(image_points_dict.values()))
    image_points = image_points_list[0]
    points = image_points.to_points()

    # Plot
    cepton_sdk.plot.plot_points(points)
