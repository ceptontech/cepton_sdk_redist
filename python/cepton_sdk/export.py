import ctypes
import enum
import os.path
import uuid

import laspy
import numpy
import plyfile

import cepton_sdk.point
import cepton_util.common
from cepton_sdk.common import *

_all_builder = AllBuilder(__name__)


def convert_points_to_spherical(points):
    radii = numpy.sqrt(
        points.image_positions[:, 0]**2 + points.image_positions[:, 1]**2 + 1)
    azimuths = numpy.arctan2(points.image_positions[:, 0], 1)
    elevations = numpy.arcsin(-points.image_positions[:, 1] / radii)
    return (azimuths, elevations)


def save_points_las(points, path):
    """Save points to LAS file.
    """
    header = laspy.header.Header()
    header.data_format_id = 1

    with laspy.file.File(path, mode="w", header=header) as f:
        f.header.gps_time_type = 1
        f.header.guid = uuid.uuid1()
        f.header.scale = [0.001] * 3
        f.header.offset = numpy.median(points.positions, axis=0)

        f.gps_time = points.timestamps - 1e9
        f.x = points.positions[:, 0]
        f.y = points.positions[:, 1]
        f.z = points.positions[:, 2]
        f.intensity = 65536 * numpy.clip(points.intensities, 0, 1)


def load_points_las(load_path, cls=cepton_sdk.point.Points):
    """Load points from LAS file.

    Returns:
        Points, extra_data
    """
    extra_data = {}
    with laspy.file.File(load_path, mode="r") as f:
        points = cls(len(f.x))
        points.timestamps_usec[:] = 1e6 * (f.gps_time + 1e9)
        points.positions[:, 0] = f.x
        points.positions[:, 1] = f.y
        points.positions[:, 2] = f.z
        points.intensities[:] = f.intensity / 65536
    return points, extra_data


class PlyPoint(ctypes.Structure):
    _fields_ = [
        ("position", 3 * ctypes.c_float),
        ("color", 4 * ctypes.c_ubyte),
        ("size", ctypes.c_float),
    ]


def save_points_ply(points, path, colors=None, sizes=None):
    """Save points to PLY file.
    """
    n = len(points)

    if colors is None:
        colors = numpy.ones([n, 4])
    if sizes is None:
        sizes = numpy.ones([n])

    with open(path, "w") as f:
        lines = [
            "ply\n",
            "format binary_little_endian 1.0\n",
            "element vertex {}\n".format(len(points)),
            "property float x\n",
            "property float y\n",
            "property float z\n",
            "property uchar red\n"
            "property uchar green\n"
            "property uchar blue\n"
            "property uchar alpha\n"
            "property float size\n"
            "end_header\n",
        ]
        f.writelines(lines)

    dtype = numpy.dtype(PlyPoint)
    data = numpy.zeros([n], dtype=dtype)
    data["position"][:, :] = points.positions
    data["color"][:, :] = (255 * numpy.clip(colors, 0, 1)).round()
    data["size"][:] = sizes
    with open(path, "ab") as f:
        f.write(data.tobytes())


def load_points_ply(path):
    """Load points from PLY file.

    Returns:
        Points, extra_data
    """
    plydata = plyfile.PlyData.read(path)
    data = plydata.elements[0].data
    n = len(data)
    points = cepton_sdk.Points(n)
    points.positions[:, 0] = data["x"]
    points.positions[:, 1] = data["y"]
    points.positions[:, 2] = data["z"]
    colors = numpy.ones([n, 4])
    colors[:, 0] = data["red"]
    colors[:, 1] = data["green"]
    colors[:, 2] = data["blue"]
    colors[:, 3] = data["alpha"]
    sizes = numpy.ones([n])
    sizes[:] = data["size"]
    extra_data = {
        "colors": colors,
        "sizes": sizes,
    }
    return points, extra_data


class PcdPoint(ctypes.Structure):
    _fields_ = [
        ("position", 3 * ctypes.c_float),
    ]


def save_points_pcd(points, path):
    """Save points to PCD file.
    """
    n = len(points)

    with open(path, "w") as f:
        lines = [
            "VERSION 0.7\n",
            "FIELDS x y z\n",
            "SIZE 4 4 4\n",
            "TYPE F F F\n",
            "COUNT 1 1 1\n",
            "WIDTH {}\n".format(n),
            "HEIGHT 1\n",
            "VIEWPOINT 0 0 0 1 0 0 0\n",
            "POINTS {}\n".format(n),
            "DATA binary\n",
        ]
        f.writelines(lines)

    dtype = numpy.dtype(PcdPoint)
    data = numpy.zeros([n], dtype=dtype)
    data["position"][:, :] = points.positions
    with open(path, "ab") as f:
        f.write(data.tobytes())


def save_points_csv(points, path):
    azimuths, elevations = convert_points_to_spherical(points)
    fields = [
        ("timestamp", "unix timestamp [us]", points.timestamps_usec, "%d"),
        ("image_x", "image x coordinate (f=1)",
         points.image_positions[:, 0], "%.3f"),
        ("image_z", "image z coordinate (f=1)",
         points.image_positions[:, 1], "%.3f"),
        ("distance", "distance measurement [m]", points.distances, "%.3f"),
        ("x", "cartesian x coordinate (right)[m]",
         points.positions[:, 0], "%.3f"),
        ("y", "cartesian y coordinate (forward)[m]",
         points.positions[:, 1], "%.3f"),
        ("z", "cartesian z coordinate (up)[m]",
         points.positions[:, 2], "%.3f"),
        ("azimuth",
         "spherical azimuth coordinate (from +y axis)[rad]", azimuths, "%.3f"),
        ("elevation",
         "spherical elevation coordinate (from +y axis)[rad]", elevations, "%.3f"),
        ("intensity", "diffuse reflectance (normal: 0-1 | retroreflective: >1)",
         points.intensities, "%.2f"),
        ("return_strongest", "measurement is strongest (multi-return)",
         points.return_strongest, "%i"),
        ("return_farthest", "measurement is farthest (multi-return)",
         points.return_farthest, "%i"),
        ("valid", "measurement is valid", points.valid, "%i"),
        ("saturated", "measurement is saturated", points.saturated, "%i"),
        ("segment_id", "segment ID", points.segment_ids, "%i"),
    ]
    dtype = numpy.dtype([(x[0], x[2].dtype) for x in fields])
    data = numpy.zeros(len(points), dtype=dtype)
    for field in fields:
        data[field[0]] = field[2]
    options = {
        "delimiter": ",",
        "header": "{}\n{}".format(
            ",".join([x[0] for x in fields]),
            ",".join([x[1] for x in fields])),
        "fmt": ",".join([x[3] for x in fields]),
    }
    numpy.savetxt(path, data, **options)


@enum.unique
class PointsFileType(enum.Enum):
    CSV = 1
    LAS = 2
    PCD = 3
    PLY = 4


def get_points_file_type(ext):
    return PointsFileType[ext[1:].upper()]


def get_points_file_type_extension(file_type):
    return ".{}".format(file_type.name.lower())


def save_points(points, path, file_type=PointsFileType.LAS):
    """Save points to file.

    Sets file extension based on type.
    """
    ext = get_points_file_type_extension(file_type)
    path = cepton_util.common.set_extension(path, ext)
    if file_type is PointsFileType.CSV:
        save_points_csv(points, path)
    elif file_type is PointsFileType.LAS:
        save_points_las(points, path)
    elif file_type is PointsFileType.PCD:
        save_points_pcd(points, path)
    elif file_type is PointsFileType.PLY:
        save_points_ply(points, path)
    else:
        raise NotImplementedError()


def load_points(path, file_type=None):
    """Load points from file.

    File type is inferred from extension.

    Returns:
        Points, extra_data
    """
    if file_type is None:
        ext = os.path.splitext(path)[1]
        file_type = get_points_file_type(ext)
    if file_type is PointsFileType.LAS:
        return load_points_las(path)
    elif file_type is PointsFileType.PLY:
        return load_points_ply(path)
    else:
        raise NotImplementedError()


__all__ = _all_builder.get()
