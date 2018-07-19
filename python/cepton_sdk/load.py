import cepton_sdk
import cepton_sdk.settings
import cepton_util.common


class Loader(cepton_util.common.ArgumentParserMixin):
    """Initializes SDK and loads settings files from command line parameters.
    
    For example usage see `samples/export_sora.py`.
    """

    def __init__(self, capture_path=None, capture_seek=None,
                 sensor_clip_manager=None, sensor_transform_manager=None):
        if sensor_clip_manager is None:
            sensor_clip_manager = cepton_sdk.settings.SensorClipManager()
        if sensor_transform_manager is None:
            sensor_transform_manager = cepton_sdk.settings.SensorTransformManager()

        self.sensor_clip_manager = sensor_clip_manager
        self.sensor_transform_manager = sensor_transform_manager

        # Initialize sdk
        options = {}
        if capture_path is not None:
            options["capture_path"] = capture_path
        cepton_sdk.initialize(**options)
        if capture_seek is not None:
            cepton_sdk.capture_replay.seek(capture_seek)

    @classmethod
    def add_arguments(cls, parser):
        group = parser.add_argument_group('Loader')
        group.add_argument("--capture_path")
        group.add_argument("--capture_seek")
        group.add_argument("--clips_path")
        group.add_argument("--transforms_path")
        return group

    @classmethod
    def parse_arguments(cls, args):
        options = {}
        if args.capture_path is not None:
            options["capture_path"] = \
                cepton_util.common.fix_path(args.capture_path)
        if args.capture_seek is not None:
            options["capture_seek"] = \
                cepton_util.common.parse_time_hms(args.capture_seek)
        if args.clips_path is not None:
            clips_path = cepton_util.common.fix_path(args.clips_path)
            with open(clips_path, "r") as f:
                options["sensor_clip_manager"] = \
                    cepton_sdk.settings.SensorClipManager.from_file(f)
        if args.transforms_path is not None:
            transforms_path = cepton_util.common.fix_path(args.transforms_path)
            with open(transforms_path, "r") as f:
                options["sensor_transform_manager"] = \
                    cepton_sdk.settings.SensorTransformManager.from_file(f)
        return options

    def process_points(self, image_points_dict):
        image_points_dict = \
            self.sensor_clip_manager.process_points(image_points_dict)
        points_dict = {key: value.to_points()
                       for key, value in image_points_dict.items()}
        points_dict = self.sensor_transform_manager.process_points(points_dict)
        points = cepton_sdk.Points.combine(list(points_dict.values()))
        return points
