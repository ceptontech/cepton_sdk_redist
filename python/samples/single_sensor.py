#!/usr/bin/env python3

import pprint

import cepton_sdk

import common

if __name__ == "__main__":
    # Variables
    capture_path = common.get_sample_capture_path()
    sensor_serial_number = 4165
    frame_length = 0.1

    # Initialize
    cepton_sdk.initialize(capture_path=capture_path)

    # Get sensor
    sensor = cepton_sdk.Sensor.create(sensor_serial_number)
    pprint.pprint(sensor.information.to_dict())

    # Get points
    image_points = \
        cepton_sdk.get_sensor_image_points(sensor_serial_number, frame_length)
    points = image_points.to_points()

    # Plot
    common.plot_points(points)
