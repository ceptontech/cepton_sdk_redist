#!/usr/bin/env python3

import argparse
import vispy.app
import vispy.io
import vispy.scene
import vispy.util.keys

from common import *
import cepton_sdk


class Renderer(object):
    def __init__(self, serial_number):
        self.serial_number = serial_number
        self.image_points_list = []
        self.listener = cepton_sdk.SensorImageFramesListener(
            self.serial_number)

        # Initialize canvas
        options = {
            "keys": "interactive",
            "position": [0, 0],
        }
        self.canvas = vispy.scene.SceneCanvas(**options)
        self.canvas.show()

        # Initialize view
        self.view = self.canvas.central_widget.add_view()
        self.view.camera = vispy.scene.cameras.make_camera("turntable")
        self.view.camera.azimuth = 0
        self.view.camera.depth_value = 1e3
        self.view.camera.elevation = 0
        self.view.camera.fov = 0
        self.view.camera.scale_factor = 40

        # Initialize events
        self.canvas.connect(self.on_key_press)

        # Initialize plot
        self.plot_handles = {}

    def on_key_press(self, event):
        if (event.key is not None):
            if (event.key == "N"):
                self.update()
                return True
        return False

    def add_plot(self, plot_name, plot_handle):
        assert (plot_name not in self.plot_handles)
        self.view.add(plot_handle)
        self.plot_handles[plot_name] = plot_handle

    def run(self):
        self.update()
        vispy.app.run()

    def update(self):
        while not self.image_points_list:
            try:
                image_points_list = self.listener.get_points()
            except:
                return
            self.image_points_list.extend(image_points_list)
        image_points = self.image_points_list.pop(0)
        points = image_points.to_points()

        plot_name = "points"
        if plot_name not in self.plot_handles:
            plot_handle = vispy.scene.visuals.Markers()
            plot_handle.antialias = 0
            self.add_plot(plot_name, plot_handle)
        plot_handle = self.plot_handles[plot_name]
        colors = numpy.ones([len(points), 4])
        if True:
            colors[numpy.logical_not(points.valid), :] = [1, 0, 0, 1]
        else:
            colors[numpy.logical_not(points.valid), :] = 0
        options = {
            "edge_width": 0,
            "face_color": colors,
            "pos": points.positions,
            "size": 2,
        }
        plot_handle.set_data(**options)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(usage='%(prog)s [OPTIONS]')
    parser.add_argument("--capture_path")
    parser.add_argument("--sensor", type=int)
    args = parser.parse_args()

    options = {}
    if args.capture_path is not None:
        options["capture_path"] = args.capture_path
    cepton_sdk.initialize(**options)

    if args.sensor is None:
        sensors_dict = cepton_sdk.get_sensors()
        serial_number = next(iter(sensors_dict.keys()))
    else:
        serial_number = args.sensor

    Renderer(serial_number).run()
