import copy
import json

import numpy

import cepton_sdk.transform


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


class _ManagerBase(object):
    def update_from_dict(self, input_dict):
        raise NotImplementedError()

    def to_dict(self):
        raise NotImplementedError()

    def update_from_json(self, input_json):
        options = {
            "ignore_invalid": True,
        }
        input_dict = _convert_keys_to_int(input_json, **options)
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
        processed_points_dict = {}
        for sensor_serial_number, points in points_dict.items():
            processed_points_dict[sensor_serial_number] = \
                self.process_sensor_points(sensor_serial_number, points)
        return processed_points_dict


class SensorTransformManager(_ManagerBase):
    def __init__(self):
        self.transforms = {}

    def update_from_dict(self, transforms_dict):
        for sensor_serial_number, transform_dict in transforms_dict.items():
            transform = cepton_sdk.transform.Transform3d()
            transform.translation = \
                numpy.array(transform_dict["translation"], dtype=float)
            rotation = numpy.array(transform_dict["rotation"], dtype=float)
            transform.rotation = \
                cepton_sdk.transform.Quaternion.from_vector(rotation)
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
            return copy.deepcopy(points)
        if len(points) == 0:
            return points

        transform = self.transforms[sensor_serial_number]
        transformed_points = copy.deepcopy(points)
        transformed_points.positions[:, :] = \
            transform.apply(points.positions)
        return transformed_points


class SensorClip(object):
    def __init__(self):
        self.min_distance = -numpy.inf
        self.max_distance = numpy.inf
        self.min_image = -numpy.inf * numpy.ones([2])
        self.max_image = numpy.inf * numpy.ones([2])

    @classmethod
    def from_dict(cls, d):
        self = cls()
        if "min_distance" in d:
            self.min_distance = d["min_distance"]
        if "max_distance" in d:
            self.max_distance = d["max_distance"]
        if "min_image_x" in d:
            self.min_image[0] = d["min_image_x"]
        if "max_image_x" in d:
            self.max_image[0] = d["max_image_x"]
        if "min_image_z" in d:
            self.min_image[1] = d["min_image_z"]
        if "max_image_z" in d:
            self.max_image[1] = d["max_image_z"]
        return self

    def find_points(self, image_points):
        if len(image_points) == 0:
            return numpy.array([], dtype=bool)

        is_clipped_list = []
        is_clipped_list.append(
            image_points.distances < self.min_distance)
        is_clipped_list.append(
            image_points.distances > self.max_distance)
        is_clipped_list.append(
            numpy.any(image_points.positions < self.min_image, axis=-1))
        is_clipped_list.append(
            numpy.any(image_points.positions > self.max_image, axis=-1))
        is_clipped = numpy.logical_or.reduce(is_clipped_list)

        return is_clipped

    def clip_points(self, image_points):
        if len(image_points) == 0:
            return image_points

        is_clipped = self.find_points(image_points)
        return image_points[numpy.logical_not(is_clipped)]


class SensorClipManager(_ManagerBase):
    def __init__(self):
        self.clips = {}

    def update_from_dict(self, clips_dict):
        for sensor_serial_number, clips_dict_tmp in clips_dict.items():
            self.clips[sensor_serial_number] = \
                SensorClip.from_dict(clips_dict_tmp)

    def process_sensor_points(self, sensor_serial_number, image_points):
        if sensor_serial_number not in self.clips:
            return copy.deepcopy(image_points)
        if len(image_points) == 0:
            return image_points

        clip_tmp = self.clips[sensor_serial_number]
        return clip_tmp.clip_points(image_points)
