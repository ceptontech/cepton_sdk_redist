import cepton_sdk.c
import numpy
import cepton_sdk.common.c
from cepton_sdk.common.mixin import *

__all__ = [
    "combine_points",
    "convert_image_points_to_points",
    "convert_points_to_image_points",
    "ImagePoints",
    "Points",
]


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


class Points(StructureOfArrays):
    """3D points array.

    Attributes:
        timestamps: time since epoch [microseconds]
        positions: x, y, z cartesian coordinates
        intensities: 0-1 scaled intensity
        return_types
        valid
        saturated
    """

    def __init__(self, n=0):
        super().__init__(n)
        self.timestamps_usec = numpy.zeros([n], dtype=numpy.int64)
        self.positions = numpy.zeros([n, 3])
        self.intensities = numpy.zeros([n])
        self.return_types = numpy.zeros([n], dtype=numpy.uint8)
        self.valid = numpy.zeros([n], dtype=bool)
        self.saturated = numpy.zeros([n], dtype=bool)

    @classmethod
    def _get_array_member_names(cls):
        return ["timestamps_usec", "positions", "intensities",
                "return_types", "valid", "saturated"]

    @property
    def timestamps(self):
        return 1e-6 * self.timestamps_usec.astype(float)


class ImagePoints(StructureOfArrays, ToCMixin):
    """Image points array.

    Attributes:
        timestamps: time since epoch [microseconds]
        positions: x, z image coordinates
        distances: distance [meters]
        intensities: 0-1 scaled intensity
        return_types
        valid
        saturated
    """

    def __init__(self, n=0):
        super().__init__(n)
        self.timestamps_usec = numpy.zeros([n], dtype=numpy.int64)
        self.positions = numpy.zeros([n, 2])
        self.distances = numpy.zeros([n])
        self.intensities = numpy.zeros([n])
        self.return_types = numpy.zeros([n], dtype=numpy.uint8)
        self.valid = numpy.zeros([n], dtype=bool)
        self.saturated = numpy.zeros([n], dtype=bool)

    @classmethod
    def _get_c_class(cls):
        return cepton_sdk.c.C_SensorImagePoint

    @classmethod
    def _get_array_member_names(cls):
        return ["timestamps_usec", "positions",
                "distances", "intensities", "return_types", "valid",
                "saturated"]

    @property
    def timestamps(self):
        return 1e-6 * self.timestamps_usec.astype(float)

    def _from_c_impl(self, data):
        self.timestamps_usec[:] = data["timestamp"]
        self.positions[:, 0] = data["image_x"]
        self.positions[:, 1] = data["image_z"]
        self.distances[:] = data["distance"]
        self.intensities[:] = data["intensity"]
        self.return_types[:] = data["return_type"]
        flags = cepton_sdk.common.c.unpack_bits(data["flags"])
        self.valid[:] = flags[:, 0]
        self.saturated[:] = flags[:, 1]

    @classmethod
    def from_c(cls, n_points, c_image_points):
        data = \
            cepton_sdk.c.convert_c_array_to_ndarray(
                c_image_points, n_points, cls._get_c_class())
        image_points = cls(len(data))
        image_points._from_c_impl(data)
        return image_points

    def _to_c_impl(self, data):
        data["timestamp"][:] = self.timestamps_usec
        data["image_x"][:] = self.positions[:, 0]
        data["image_z"][:] = self.positions[:, 1]
        data["distance"][:] = self.distances
        data["intensity"][:] = self.intensities
        data["return_type"][:] = self.return_types
        flags = numpy.zeros([len(self), 8], dtype=bool)
        flags[:, 0] = self.valid
        flags[:, 1] = self.saturated
        data["flags"] = cepton_sdk.common.c.pack_bits(flags)

    def to_c(self, c_type=None):
        if c_type is None:
            c_type = self._get_c_class()

        dtype = numpy.dtype(c_type)
        data = numpy.zeros(len(self), dtype=dtype)
        self._to_c_impl(data)
        c_image_points_ptr = \
            cepton_sdk.c.convert_ndarray_to_c_array(data, c_type)
        return c_image_points_ptr

    def to_points(self, cls=Points):
        """Convert to 3d points.
        """
        assert issubclass(cls, Points)

        points = cls(len(self))
        points.timestamps_usec[:] = self.timestamps_usec
        points.positions[:, :] = convert_image_points_to_points(
            self.positions, self.distances)
        points.intensities[:] = self.intensities
        points.return_types[:] = self.return_types
        points.valid[:] = self.valid
        points.saturated[:] = self.saturated
        return points

    @classmethod
    def from_points(cls, points):
        """Create from 3d points.
        """
        assert isinstance(points, Points)

        image_points = cls(len(points))
        image_points.timestamps_usec[:] = points.timestamps_usec
        image_points.positions[:, :], image_points.distances[:] = \
            convert_points_to_image_points(points.positions)
        image_points.intensities[:] = points.intensities
        image_points.return_types[:] = points.return_types
        image_points.valid[:] = points.valid
        image_points.saturated[:] = points.saturated
        return image_points
