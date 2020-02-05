import enum

import numpy

import cepton_sdk.c
import cepton_sdk.common.c
import cepton_util.common
from cepton_sdk.common import *

_all_builder = AllBuilder(__name__)


def combine_points(points_list):
    """Combine list of points (`ImagePoints`, `Points`, etc).

    List must be nonempty.
    Returns:
        combined_points
    """
    cls = type(points_list[0])
    return cls.combine(points_list)


def convert_image_points_to_points(image_positions, distances):
    """Convert image points to 3d points.

    Returns:
        positions
    """
    hypotenuse_small = numpy.sqrt(
        image_positions[:, 0]**2 +
        image_positions[:, 1]**2 + 1.0)
    ratio = distances / hypotenuse_small
    n = image_positions.shape[0]
    positions = numpy.zeros([n, 3])
    positions[:, 0] = -image_positions[:, 0] * ratio
    positions[:, 1] = ratio
    positions[:, 2] = -image_positions[:, 1] * ratio
    return positions


def convert_points_to_image_points(positions):
    """Convert 3d points to image points.

    Returns:
        image_positions
        distances
    """
    n = positions.shape[0]
    image_positions = numpy.zeros([n, 2])
    distances = numpy.zeros([n])
    is_valid = positions[:, 1] > 0
    positions_tmp = positions[is_valid, :]
    image_positions[is_valid, :] = \
        -positions_tmp[:, [0, 2]] / positions_tmp[:, [1]]
    distances[is_valid] = numpy.linalg.norm(positions_tmp, axis=-1)
    return image_positions, distances


class ReturnType(enum.IntEnum):
    STRONGEST = 0
    FARTHEST = 1


class PointFlag(enum.IntEnum):
    VALID = 0
    SATURATED = 1


class Points(StructureOfArrays, ToCArrayMixin):
    """3D points array.

    Attributes:
        timestamps_usec
        timestamps
        image_positions
        distances
        positions
        intensities
        return_strongest
        return_farthest
        valid
        saturated
    """

    def __init__(self, n=0):
        super().__init__(n)
        self.timestamps_usec = numpy.zeros([n], dtype=numpy.int64)
        self.image_positions = numpy.zeros([n, 2])
        self.distances = numpy.zeros([n])
        self.positions = numpy.zeros([n, 3])
        self.intensities = numpy.zeros([n])
        self.return_types = numpy.zeros([n, 8], dtype=bool)
        self.flags = numpy.zeros([n, 8], dtype=bool)
        self.flags[:, PointFlag.VALID] = True
        self.segment_ids = numpy.zeros([n], dtype=numpy.uint8)

    @classmethod
    def _get_array_member_names(cls):
        return ["timestamps_usec", "image_positions", "distances", "positions",
                "intensities", "return_types", "flags", "segment_ids"]

    @classmethod
    def _get_c_class(cls):
        return cepton_sdk.c.C_SensorImagePoint

    def _from_c_impl(self, data):
        self.timestamps_usec[:] = data["timestamp"]
        self.image_positions[:, 0] = data["image_x"]
        self.image_positions[:, 1] = data["image_z"]
        self.distances[:] = data["distance"]
        self.intensities[:] = data["intensity"]

        self.return_types[:, :] = cepton_sdk.common.c.unpack_bits(
            data["return_type"])
        self.flags[:, :] = cepton_sdk.common.c.unpack_bits(data["flags"])
        self.segment_ids[:] = data["segment_id"]

        self.positions[:, :] = convert_image_points_to_points(
            self.image_positions, self.distances)

    def _to_c_impl(self, data):
        data["timestamp"][:] = self.timestamps_usec
        data["image_x"][:] = self.positions[:, 0]
        data["image_z"][:] = self.positions[:, 1]
        data["distance"][:] = self.distances
        data["intensity"][:] = self.intensities
        data["return_type"][:] = self.return_types
        data["segment_id"][:] = self.segment_ids
        data["flags"] = self.flags

    @numpy_property
    def timestamps(self):
        return cepton_util.common.from_usec(self.timestamps_usec)

    @numpy_property
    def return_strongest(self):
        return self.return_types[:, ReturnType.STRONGEST]

    @numpy_property
    def return_farthest(self):
        return self.return_types[:, ReturnType.FARTHEST]

    @numpy_property
    def valid(self):
        return self.flags[:, PointFlag.VALID]

    @numpy_property
    def saturated(self):
        return self.flags[:, PointFlag.SATURATED]

__all__ = _all_builder.get()
