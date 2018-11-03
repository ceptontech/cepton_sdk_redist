#!/usr/bin/env python3

import pprint

import cepton_sdk
import cepton_sdk.plot
from common import *

if __name__ == "__main__":
    # Variables
    frame_length = 0.1

    # Initialize
    cepton_sdk.initialize()

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
