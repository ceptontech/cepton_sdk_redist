import cepton_sdk.c
import numpy
import cepton_sdk.common.c
from cepton_sdk.common.mixin import *

__all__ = [
    "convert_image_points_to_points",
    "convert_points_to_image_points",
    "ImagePoints",
    "Points",
]


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
        self.timestamps = numpy.zeros([n])
        self.timestamps_usec = numpy.zeros([n], dtype=numpy.int64)
        self.positions = numpy.zeros([n, 3])
        self.intensities = numpy.zeros([n])
        self.return_types = numpy.zeros([n], dtype=numpy.uint8)
        self.valid = numpy.zeros([n], dtype=bool)
        self.saturated = numpy.zeros([n], dtype=bool)
        self._check_array_members()

    @classmethod
    def _get_array_member_names(cls):
        return ["timestamps_usec", "timestamps", "positions", "intensities",
                "return_types", "valid", "saturated"]


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
        self.timestamps = numpy.zeros([n])
        self.positions = numpy.zeros([n, 2])
        self.distances = numpy.zeros([n])
        self.intensities = numpy.zeros([n])
        self.return_types = numpy.zeros([n], dtype=numpy.uint8)
        self.valid = numpy.zeros([n], dtype=bool)
        self.saturated = numpy.zeros([n], dtype=bool)
        self._check_array_members()

    @classmethod
    def _get_c_class(cls):
        return cepton_sdk.c.C_SensorImagePoint

    @classmethod
    def _get_array_member_names(cls):
        return ["timestamps_usec", "timestamps", "positions",
                "distances", "intensities", "return_types", "valid",
                "saturated"]

    def _from_c_impl(self, data):
        self.timestamps_usec[:] = data["timestamp"]
        self.timestamps[:] = 1e-6 * data["timestamp"].astype(float)
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
        points.timestamps[:] = self.timestamps
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
        image_points.timestamps[:] = points.timestamps
        image_points.positions[:, :], image_points.distances[:] = \
            convert_points_to_image_points(points.positions)
        image_points.intensities[:] = points.intensities
        image_points.return_types[:] = points.return_types
        image_points.valid[:] = points.valid
        image_points.saturated[:] = points.saturated
        return image_points


class NPointsAccumulator(object):
    """Points accumulator used by internal listeners."""

    def __init__(self, ):
        self._points = None
        self.reset()

    def reset(self):
        if self._points is not None:
            cls = type(self._points)
            self._points = cls(0)

    def add_points(self, points):
        if len(points) == 0:
            return

        if self._points is None:
            self._points = points
        else:
            self._points = self._points.combine([self._points, points])

    def has_points(self, n):
        if self._points is None:
            return False
        return len(self._points) >= n

    def get_points(self, n):
        if self._points is None:
            return None
        points = self._points[:n]
        self._points = self._points[n:]
        return points


class _TimePointsAccumulatorBase(object):
    """Points accumulator used by internal listeners."""

    def __init__(self, t=-numpy.inf):
        self.reset()
        self._t = t

    def reset(self):
        self._t = -numpy.inf

    @property
    def t(self):
        return self._t

    def _check_t(self, t):
        if t <= self._t:
            raise ValueError("requested times must be increasing")

    def is_empty(self):
        return numpy.isinf(self._t)


class TimePointsAccumulator(_TimePointsAccumulatorBase):
    """Points accumulator used by internal listeners."""

    def __init__(self, **kwargs):
        self._points = None
        super().__init__(**kwargs)

    @property
    def t_max(self):
        return self._t_max

    def reset(self):
        super().reset()
        self._t_max = -numpy.inf
        if self._points is not None:
            cls = type(self._points)
            self._points = cls(0)

    def add_points(self, points):
        if len(points) == 0:
            return

        if numpy.isinf(self._t):
            self._t = numpy.amin(points.timestamps)
        else:
            select = points.timestamps > self._t
            points = points[select]
            if len(points) == 0:
                return

        if self._points is None:
            self._points = points
        else:
            self._points = self._points.combine([self._points, points])

        # Sort
        sort_indices = numpy.argsort(self._points.timestamps)
        self._points = self._points[sort_indices]

        self._t_max = self._points.timestamps[-1]

    def has_points_by_t(self, t):
        if self.is_empty():
            return False
        self._check_t(t)
        return t <= self._t_max

    def has_points(self, t_length):
        t = self._t + t_length
        return self.has_points_by_t(t)

    def get_points_by_t(self, t):
        self._check_t(t)
        if self._points is None:
            points = None
        else:
            is_valid = self._points.timestamps <= t
            points = self._points[is_valid]

        self._t = t
        if self._points is not None:
            is_valid = self._points.timestamps > self._t
            self._points = self._points[is_valid]

        return points

    def get_points(self, t_length):
        t = self._t + t_length
        return self.get_points_by_t(t)


class MultipleTimePointsAccumulator(_TimePointsAccumulatorBase):
    """Points accumulator used by internal listeners."""

    def __init__(self, t_latency=0, **kwargs):
        self._t_latency = t_latency
        super().__init__(**kwargs)

    @property
    def t_max(self):
        return max(
            map(lambda x: x.t_max, self._accumulators.values()),
            default=-numpy.inf)

    def reset(self):
        super().reset()
        self._accumulators = {}

    def add_points(self, serial_number, points):
        if len(points) == 0:
            return

        if numpy.isinf(self._t):
            self._t = numpy.amin(points.timestamps) + self._t_latency

        if serial_number not in self._accumulators:
            options = {
                "t": self._t,
            }
            self._accumulators[serial_number] = \
                TimePointsAccumulator(**options)
        self._accumulators[serial_number].add_points(points)

    def has_points_by_t(self, t):
        if self.is_empty():
            return False
        self._check_t(t)
        for accumulator in self._accumulators.values():
            if accumulator.has_points_by_t(t + self._t_latency):
                return True
        return False

    def has_points(self, t_length):
        t = self._t + t_length
        return self.has_points_by_t(t)

    def get_points_by_t(self, t):
        self._check_t(t)
        self._t = t
        points_dict = {}
        for serial_number, accumulator in self._accumulators.items():
            points = accumulator.get_points_by_t(t)
            if (points is None) or (len(points) == 0):
                continue
            points_dict[serial_number] = points
        return points_dict

    def get_points(self, t_length):
        t = self._t + t_length
        return self.get_points_by_t(t)
