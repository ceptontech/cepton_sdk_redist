import laspy
import numpy
import uuid

import cepton_sdk.point

def save_points_las(points, save_path):
    """Save points to LAS file.
    """
    header = laspy.header.Header()
    header.data_format_id = 1

    with laspy.file.File(save_path, mode="w", header=header) as f:
        f.header.gps_time_type = 1
        f.header.guid = uuid.uuid1()
        f.header.scale = [0.001] * 3

        f.gps_time = numpy.remainder(points.timestamps, 1e9)
        f.x = points.positions[:, 0]
        f.y = points.positions[:, 1]
        f.z = points.positions[:, 2]
        f.intensity = points.intensities
        f.return_num = points.return_numbers


def load_points_las(load_path, cls=cepton_sdk.point.Points):
    """Load points from LAS file.

    Returns:
        Points.
    """
    data = {}
    with laspy.file.File(load_path, mode="r") as f:
        points = cls(len(f.x))
        points.timestamps[:] = f.gps_time
        points.positions[:, 0] = f.x
        points.positions[:, 1] = f.y
        points.positions[:, 2] = f.z
        points.intensities[:] = f.intensity
        points.return_numbers[:] = f.return_num
    return points, data
