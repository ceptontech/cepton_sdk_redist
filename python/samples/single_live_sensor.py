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
    sensors_dict = cepton_sdk.get_sensors()
    sensor = next(iter(sensors_dict.values()))
    pprint.pprint(sensor.information.to_dict())

    # Get points
    listener = cepton_sdk.SensorImageFramesListener(sensor.serial_number)
    image_points_list = listener.get_points()
    del listener
    points = image_points.to_points()

    # Plot
    cepton_sdk.plot.plot_points(points)
