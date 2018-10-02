import os

import cepton_sdk
import cepton_sdk.settings
import cepton_util.common


class Loader(cepton_util.common.ArgumentParserMixin):
    """Initializes SDK and loads settings files from command line parameters.

    For example usage see `samples/export_sora.py`.
    """

    def __init__(self, capture_path=None, capture_seek=None, sdk_options={},
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
        options.update(sdk_options)
        cepton_sdk.initialize(**options)
        if capture_seek is not None:
            cepton_sdk.capture_replay.seek(capture_seek)

    @classmethod
    def add_arguments(cls, parser):
        group = parser.add_argument_group("Loader")
        group.add_argument("--capture_path")
        group.add_argument("--capture_seek")
        group.add_argument("--load_auto", action="store_true",
                           help="Load settings from capture directory.")
        group.add_argument("--load_path",
                           help="Load settings from directory.")
        return group

    @classmethod
    def parse_arguments(cls, args):
        options = {
            "capture_path": cepton_util.common.fix_path(args.capture_path),
            "capture_seek": cepton_util.common.parse_time_hms(
                args.capture_seek),
        }
        load_path = cepton_util.common.fix_path(args.load_path)
        if load_path is None:
            if args.load_auto and (options["capture_path"] is not None):
                load_path = os.path.dirname(options["capture_path"])
        if load_path is not None:
            clips_path = cepton_util.common.find_file_by_name(
                "cepton_clips.json", path=load_path)
            transforms_path = cepton_util.common.find_file_by_name(
                "cepton_transforms.json", path=load_path)
            if clips_path is not None:
                with open(clips_path, "r") as f:
                    options["sensor_clip_manager"] = \
                        cepton_sdk.settings.SensorClipManager.from_file(f)
            if transforms_path is not None:
                with open(transforms_path, "r") as f:
                    options["sensor_transform_manager"] = \
                        cepton_sdk.settings.SensorTransformManager.from_file(f)
        return cepton_util.common.process_options(options)

    def process_points(self, image_points_dict):
        image_points_dict = \
            self.sensor_clip_manager.process_points(image_points_dict)
        points_dict = {key: value.to_points()
                       for key, value in image_points_dict.items()}
        points_dict = self.sensor_transform_manager.process_points(points_dict)
        points = cepton_sdk.Points.combine(list(points_dict.values()))
        return points
