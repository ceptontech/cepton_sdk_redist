import copy
import os

import cepton_sdk.api
import cepton_sdk.point
import cepton_sdk.settings
import cepton_util.common
from cepton_sdk.common import *

_all_builder = AllBuilder(__name__)

from cepton_util.common import InputDataDirectory, OutputDataDirectory  # noqa isort:skip


def load_clips(path=None):
    if path is None:
        return cepton_sdk.settings.SensorClipManager()
    with open(path, "r") as f:
        return cepton_sdk.settings.SensorClipManager.from_file(f)


def load_transforms(path=None):
    if path is None:
        return cepton_sdk.settings.SensorTransformManager()
    with open(path, "r") as f:
        return cepton_sdk.settings.SensorTransformManager.from_file(f)


class Loader(cepton_util.common.ArgumentParserMixin):
    """Initializes SDK and loads settings files from command line parameters.

    For example usage see `samples/export_sora.py`.
    """

    def __init__(self, settings_dir=None, sdk_options={}):
        self.settings_dir = InputDataDirectory(settings_dir)
        self.sensor_clip_manager = load_clips(self.settings_dir.clips_path)
        self.sensor_transform_manager = load_transforms(
            self.settings_dir.transforms_path)
        self.sdk_options = copy.deepcopy(sdk_options)

    @classmethod
    def add_arguments(cls, parser):
        group = parser.add_argument_group("Load")
        group.add_argument("--capture_path", help="Path to PCAP capture file.")
        group.add_argument("--capture_seek",
                           help="Capture file seek position [seconds].")
        group.add_argument("--settings_dir",
                           help="Load settings from directory.")
        # Default control_flags is ControlFlags.ENABLE_MULTIPLE_RETURNS
        group.add_argument("--control_flags", default=16, help="SDK control flags")
        return group

    @classmethod
    def parse_arguments(cls, args):
        capture_path = cepton_util.common.fix_path(args.capture_path)

        settings_dir = cepton_util.common.fix_path(args.settings_dir)
        if (settings_dir is None) and (capture_path is not None):
            settings_dir = os.path.dirname(capture_path)

        sdk_options = {
            "capture_path": capture_path,
            "capture_seek": cepton_util.common.parse_time_hms(
                args.capture_seek),
            "control_flags": int(args.control_flags),
        }
        sdk_options = cepton_util.common.process_options(sdk_options)

        options = {
            "settings_dir": settings_dir,
            "sdk_options": sdk_options,
        }
        return cepton_util.common.process_options(options)

    def initialize(self):
        cepton_sdk.api.initialize(**self.sdk_options)

    def process_points(self, points_dict, combine=True):
        points_dict = self.sensor_transform_manager.process_points(points_dict)
        points_dict = self.sensor_clip_manager.process_points(points_dict)
        if not combine:
            return points_dict
        points = cepton_sdk.point.Points.combine(list(points_dict.values()))
        return points

    def process_sensor_points(self, serial_number, points):
        self.sensor_transform_manager.process_sensor_points(
            serial_number, points)
        self.sensor_clip_manager.process_sensor_points(serial_number, points)
        return points


__all__ = _all_builder.get()
