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

    # Get sensor
    sensors_dict = cepton_sdk.get_sensors()
    sensor = next(iter(sensors_dict.values()))
    pprint.pprint(sensor.information.to_dict())

    # Get points
    listener = cepton_sdk.SensorImageFramesListener(sensor.serial_number)
    image_points_list = listener.get_points()
    del listener
    points = image_points_list[0].to_points()

    # Plot
    cepton_sdk.plot.plot_points(points)
