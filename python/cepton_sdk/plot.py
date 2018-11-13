import numpy
import vispy.app
import vispy.scene
import vispy.visuals.transforms
from cepton_sdk.common import *

_all_builder = AllBuilder(__name__)


def plot_points(points, show_invalid=False):
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

    # Plot points
    points_visual = vispy.scene.visuals.Markers()
    points_visual.antialias = 0
    view.add(points_visual)
    colors = numpy.ones([len(points), 4])
    if show_invalid:
        colors[numpy.logical_not(points.valid), :] = [1, 0, 0, 1]
    else:
        colors[numpy.logical_not(points.valid), :] = 0
    options = {
        "edge_width": 0,
        "face_color": colors,
        "pos": points.positions,
        "size": 2,
    }
    points_visual.set_data(**options)

    # Run
    vispy.app.run()


__all__ = _all_builder.get()
