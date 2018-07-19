import vispy.scene
import vispy.app

__all__ = [
    "plot_points",
]


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
