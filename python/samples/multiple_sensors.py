#!/usr/bin/env python3

import pprint

import cepton_sdk
import cepton_sdk.plot
import common

if __name__ == "__main__":
    # Variables
    capture_path = common.get_sample_capture_path()
    sensor_serial_number = 4165
    frame_length = 0.1

    # Initialize
    cepton_sdk.initialize(capture_path=capture_path)

    # Get sensors
    sensors_dict = cepton_sdk.get_sensors()
    sensor = sensors_dict[sensor_serial_number]
    pprint.pprint(sensor.information.to_dict())

    # Get points
    image_points_dict = cepton_sdk.get_image_points(frame_length)
    image_points = image_points_dict[sensor_serial_number]
    points = image_points.to_points()

    # Plot
    cepton_sdk.plot.plot_points(points)
