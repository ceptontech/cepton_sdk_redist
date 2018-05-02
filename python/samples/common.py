from __future__ import (absolute_import, division, generators, nested_scopes,
                        print_function, unicode_literals, with_statement)

import os.path

import numpy
import vispy.app
import vispy.scene


def fix_path(path):
    return os.path.abspath(os.path.expanduser(path))


def get_sample_capture_path():
    module_dir_path = os.path.dirname(__file__)
    capture_path = fix_path(os.path.join(module_dir_path, "1.pcap"))
    return capture_path


def print_points(points):
    options = {
        "precision": 3,
        "suppress_small": True,
    }
    s = numpy.array_str(points.positions, **options)
    print("Positions [m]:\n{}".format(s))


def plot_points(points):
    # Initialize canvas
    options = {
        "keys": "interactive",
        "position": [0, 0],
    }
    canvas = vispy.scene.SceneCanvas(**options)
    canvas.show()

    # Initialize view
    view = canvas.central_widget.add_view()
    view.camera = vispy.scene.cameras.make_camera("turntable")
    view.camera.azimuth = 0
    view.camera.depth_value = 1e3
    view.camera.elevation = 0
    view.camera.fov = 0
    view.camera.scale_factor = 100

    # Plot
    plot_handle = vispy.scene.visuals.Markers()
    plot_handle.antialias = 0
    view.add(plot_handle)
    options = {
        "edge_width": 0,
        "face_color": 'white',
        "pos": points.positions,
        "size": 2,
    }
    plot_handle.set_data(**options)

    # Run
    vispy.app.run()
