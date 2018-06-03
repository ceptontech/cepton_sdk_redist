#!/usr/bin/env python3

import pprint

import cepton_sdk
import common

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
    image_points = \
        cepton_sdk.get_sensor_image_points(sensor.serial_number, frame_length)
    points = image_points.to_points()

    # Plot
    common.plot_points(points)
