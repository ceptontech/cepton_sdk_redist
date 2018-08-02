#!/usr/bin/env python3

import pprint

import cepton_sdk
import cepton_sdk.plot
from common import *

if __name__ == "__main__":
    # Variables
    capture_path = get_sample_capture_path()
    frame_length = 0.1

    # Initialize
    cepton_sdk.initialize(capture_path=capture_path)

    # Get points
    image_points_dict = cepton_sdk.get_image_points(frame_length)
    image_points = \
        cepton_sdk.ImagePoints.combine(list(image_points_dict.values()))
    points = image_points.to_points()

    # Plot
    cepton_sdk.plot.plot_points(points)
