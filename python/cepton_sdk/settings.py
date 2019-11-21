import collections
import copy
import json

import numpy

import cepton_sdk.common.transform
from cepton_sdk.common import *

_all_builder = AllBuilder(__name__)


def _convert_keys_to_int(d, ignore_invalid=False):
    d_int = {}
    for key, value in d.items():
        try:
            key = int(key)
        except:
            if ignore_invalid:
                continue
            else:
                raise
        d_int[key] = value
    return d_int


def _convert_keys_to_string(d):
    return {str(key): value for (key, value) in d.items()}


def _get_pretty_json(d):
    d = _convert_keys_to_string(d)
    return json.dumps(d, sort_keys=True, indent=2, separators=(',', ': '))


def _save_pretty_json(d, f):
    f.write(_get_pretty_json(d))


class _ManagerBase:
    def update_from_dict(self, input_dict):
        raise NotImplementedError()

    def to_dict(self):
        raise NotImplementedError()

    def update_from_json(self, input_json):
        input_dict = input_json
        self.update_from_dict(input_dict)

    @classmethod
    def from_json(cls, input_json):
        self = cls()
        self.update_from_json(input_json)
        return self

    def to_json(self):
        input_dict = self.to_dict()
        input_json = _convert_keys_to_string(input_dict)
        return input_json

    def update_from_file(self, input_file):
        input_json = json.load(input_file)
        self.update_from_json(input_json)

    @classmethod
    def from_file(cls, input_file):
        self = cls()
        self.update_from_file(input_file)
        return self

    def to_file(self, output_file):
        output_json = self.to_json()
        _save_pretty_json(output_json, output_file)

    def process_sensor_points(self, sensor_serial_number, points):
        raise NotImplementedError

    def process_points(self, points_dict):
        for sensor_serial_number, points in points_dict.items():
            self.process_sensor_points(sensor_serial_number, points)
        return points_dict


class SensorTransformManager(_ManagerBase):
    def __init__(self):
        self.transforms = collections.defaultdict(
            cepton_sdk.common.transform.Transform3d)

    def update_from_dict(self, transforms_dict):
        for key, transform_dict in transforms_dict.items():
            try:
                sensor_serial_number = int(key)
            except:
                continue
            transform = cepton_sdk.common.transform.Transform3d()
            transform.translation = \
                numpy.array(transform_dict["translation"], dtype=float)
            rotation = numpy.array(transform_dict["rotation"], dtype=float)
            transform.rotation = \
                cepton_sdk.common.transform.Quaternion.from_vector(rotation)
            self.transforms[sensor_serial_number] = transform

    def to_dict(self):
        transforms_dict = {}
        for sensor_serial_number, transform in self.transforms.items():
            transform_dict = {}
            transform_dict["translation"] = transform.translation.tolist()
            transform_dict["rotation"] = transform.rotation.to_vector().tolist()
            transforms_dict[sensor_serial_number] = transform_dict
        return transforms_dict

    def process_sensor_points(self, sensor_serial_number, points):
        if sensor_serial_number not in self.transforms:
            return points
        if len(points) == 0:
            return points

        transform = self.transforms[sensor_serial_number]
        points.positions[:, :] = transform.apply(points.positions)
        return points


class SensorClip:
    def __init__(self):
        self.distance_lb = -numpy.inf
        self.distance_ub = numpy.inf
        self.image_lb = numpy.full([2], -numpy.inf)
        self.image_ub = numpy.full([2], numpy.inf)

    @classmethod
    def from_dict(cls, d):
        self = cls()
        if "min_distance" in d:
            self.distance_lb = d["min_distance"]
        if "max_distance" in d:
            self.distance_ub = d["max_distance"]
        if "min_image_x" in d:
            self.image_lb[0] = d["min_image_x"]
        if "max_image_x" in d:
            self.image_ub[0] = d["max_image_x"]
        if "min_image_z" in d:
            self.image_lb[1] = d["min_image_z"]
        if "max_image_z" in d:
            self.image_ub[1] = d["max_image_z"]
        return self

    def find_points(self, points):
        if len(points) == 0:
            return numpy.array([], dtype=bool)

        return numpy.logical_or.reduce([
            points.distances <= self.distance_lb,
            points.distances > self.distance_ub,
            numpy.any(points.image_positions < self.image_lb, axis=-1),
            numpy.any(points.image_positions > self.image_ub, axis=-1),
        ])


class FocusClip:
    def __init__(self):
        self.lb = numpy.full([3], -numpy.inf)
        self.ub = numpy.full([3], numpy.inf)

    @classmethod
    def from_dict(cls, d):
        self = cls()
        if "min_x" in d:
            self.lb[0] = d["min_x"]
        if "max_x" in d:
            self.ub[0] = d["max_x"]
        if "min_y" in d:
            self.lb[1] = d["min_y"]
        if "max_y" in d:
            self.ub[1] = d["max_y"]
        if "min_z" in d:
            self.lb[2] = d["min_z"]
        if "max_z" in d:
            self.ub[2] = d["max_z"]
        return self

    def find_points(self, points):
        if len(points) == 0:
            return numpy.array([], dtype=bool)

        return numpy.logical_or.reduce([
            numpy.any(points.positions < self.lb, axis=-1),
            numpy.any(points.positions > self.ub, axis=-1),
        ])


class GroundClip:
    def __init__(self):
        self.height = numpy.inf
        self.distance_ub = 0

    @classmethod
    def from_dict(cls, d):
        self = cls()
        if "height" in d:
            self.height = d["height"]
        if "max_distance" in d:
            self.distance_ub = d["max_distance"]
        return self

    def find_points(self, points):
        if len(points) == 0:
            return numpy.array([], dtype=bool)

        return numpy.logical_and.reduce([
            points.positions[:, 2] < self.height,
            points.distances < self.distance_ub,
        ])


class SensorClipManager(_ManagerBase):
    def __init__(self):
        self.focus_clip = FocusClip()
        self.ground_clip = GroundClip()
        self.clips = {}

    def update_from_dict(self, d):
        for key, d_tmp in d.items():
            if key == "focus":
                self.focus_clip = FocusClip.from_dict(d_tmp)
            elif key == "ground":
                self.ground_clip = GroundClip.from_dict(d_tmp)
            else:
                try:
                    sensor_serial_number = int(key)
                except:
                    continue
                self.clips[sensor_serial_number] = SensorClip.from_dict(d_tmp)

    def process_sensor_points(self, sensor_serial_number, points):
        if len(points) == 0:
            return points

        is_clipped_list = [
            self.focus_clip.find_points(points),
            self.ground_clip.find_points(points),
        ]
        if sensor_serial_number in self.clips:
            is_clipped_list.append(
                self.clips[sensor_serial_number].find_points(points))
        is_clipped = numpy.logical_or.reduce(is_clipped_list)
        points.flags[is_clipped, cepton_sdk.PointFlag.VALID] = False
        return points


__all__ = _all_builder.get()
