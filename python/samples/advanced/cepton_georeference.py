#!/usr/bin/env python3
"""
Sample script to combine LiDAR data to generate point cloud.
"""

import argparse
import datetime

import matplotlib.pyplot
import numpy
import pytz
import scipy.interpolate
import scipy.spatial.transform
import utm
from mpl_toolkits.mplot3d import Axes3D

import cepton_sdk.export
import cepton_sdk.plot
from cepton_sdk.common import *
from cepton_util.common import *


def from_gps_time(weeks, seconds):
    d = datetime.datetime(1980, 1, 6) + \
        datetime.timedelta(weeks=weeks, seconds=seconds)
    # leapseconds
    d -= datetime.timedelta(seconds=18)
    return pytz.utc.localize(d).timestamp()


class Transforms(StructureOfArrays):
    def __init__(self, n=0):
        super().__init__(n)
        self.timestamps = numpy.zeros([n])
        self.translations = numpy.zeros([n, 3])
        self.quaternions = numpy.zeros([n, 4])

    @classmethod
    def _get_array_member_names(cls):
        return ["timestamps", "translations", "quaternions"]

    @property
    def rotations(self):
        return scipy.spatial.transform.Rotation(self.quaternions)


def main():
    parser = argparse.ArgumentParser(
        usage="%(prog)s [OPTIONS] output_path",
        formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument("output", help="Output path.")
    parser.add_argument("--downsample", action="store_true")
    parser.add_argument("--points_path", help="Path to points", required=True)
    parser.add_argument(
        "--serial_path", help="Path to serial data", required=True)
    parser.add_argument("--version", action="version",
                        version="cepton_sdk {}".format(cepton_sdk.__version__))
    args = parser.parse_args()

    output_path = fix_path(args.output)

    # Load points
    points_path = fix_path(args.points_path)
    points = cepton_sdk.export.load_points_las(points_path)[0]

    # Load serial
    serial_path = fix_path(args.serial_path)
    with open(serial_path, "r") as f:
        serial_lines = f.readlines()
    transforms = Transforms(len(serial_lines))
    i_transform = 0
    for line in serial_lines:
        if line.startswith("#INSPVAA"):
            # Novatel
            line = line.lstrip("#").split("*")[0]
            header, data = line.split(";")
            header = header.split(",")
            data = [None, None] + data.split(",")
            if data[13] != "INS_SOLUTION_GOOD":
                continue
            transforms.timestamps[i_transform] = \
                from_gps_time(float(data[2]), float(data[3]))
            transforms.translations[i_transform, :2] = utm.from_latlon(
                float(data[4]), float(data[5]))[:2]
            if i_transform == 0:
                print("UTM: {}".format(utm.from_latlon(
                    float(data[4]), float(data[5]))[2:]))
            transforms.translations[i_transform, 2] = float(data[6])
            transforms.quaternions[i_transform, :] = \
                scipy.spatial.transform.Rotation.from_euler(
                "zxy",
                [-float(data[12]), float(data[11]), float(data[10])],
                degrees=True).as_quat()
            i_transform += 1
    transforms = transforms[:i_transform]
    t_diff = numpy.diff(transforms.timestamps)
    assert (numpy.all(t_diff > 0))

    # HACK: no PPS
    # print(datetime.datetime.utcfromtimestamp(points.timestamps[0]))
    # print(datetime.datetime.utcfromtimestamp(transforms.timestamps[0]))
    points.timestamps_usec[:] += \
        int(1e6 * (transforms.timestamps[0] - points.timestamps[0]))

    # Plot point timestamps
    # matplotlib.pyplot.plot(points.timestamps)
    # matplotlib.pyplot.show()
    # return

    # Plot 3d trajectory
    # fig = matplotlib.pyplot.figure()
    # ax = fig.add_subplot(projection="3d")
    # matplotlib.pyplot.plot(
    #     transforms.translations[:, 0], transforms.translations[:, 1],
    #     transforms.translations[:, 2], 'o')
    # matplotlib.pyplot.show()
    # return

    # Plot 2d trajectory with directions
    # matplotlib.pyplot.axis("equal")
    # matplotlib.pyplot.plot(
    #     transforms.translations[:, 0], transforms.translations[:, 1], 'o')
    # directions = numpy.zeros([len(transforms), 3])
    # directions[:, 1] = 1.0
    # directions = transforms.rotations.apply(directions)
    # matplotlib.pyplot.quiver(
    #     transforms.translations[::10, 0], transforms.translations[::10, 1],
    #     directions[::10, 0], directions[::10, 1])
    # matplotlib.pyplot.show()
    # return

    indices = numpy.arange(0, len(points))

    # Apply pose
    # TODO: skip if measurement spacing large
    is_valid = numpy.logical_and(
        points.timestamps > transforms.timestamps[0],
        points.timestamps < transforms.timestamps[-1])
    indices = indices[is_valid]
    translations_tmp = scipy.interpolate.interp1d(
        transforms.timestamps, transforms.translations, axis=0)(
            points.timestamps[indices])
    rotations_tmp = \
        scipy.spatial.transform.Slerp(
            transforms.timestamps, transforms.rotations)(
                points.timestamps[indices])
    points.positions[indices, :] = \
        rotations_tmp.apply(points.positions[indices, :]) + \
        translations_tmp

    # Grid downsample
    if args.downsample:
        grid_ub = numpy.full([3], 1e4)
        grid_lb = -grid_ub
        grid_spacing = numpy.full([3], 0.01)
        grid_shape = ((grid_ub - grid_lb) / grid_spacing).astype(int)

        def get_flat_grid_indices(positions):
            grid_indices = ((positions - grid_lb) / grid_spacing).astype(int)
            is_valid = numpy.logical_and(
                numpy.all(grid_indices >= 0, axis=1),
                numpy.all(grid_indices < grid_shape, axis=1))
            flat_grid_indices = numpy.full(grid_indices.shape[0], -1)
            flat_grid_indices[is_valid] = numpy.ravel_multi_index(
                grid_indices[is_valid, :].transpose(), grid_shape)
            return flat_grid_indices
        grid_indices = get_flat_grid_indices(points.positions[indices, :])
        is_valid = grid_indices >= 0
        indices = indices[is_valid]
        grid_indices = grid_indices[is_valid]
        is_valid = numpy.unique(grid_indices, return_index=True)[1]
        indices = indices[is_valid]
        grid_indices = grid_indices[is_valid]

    assert (len(indices) > 0)

    points = points[indices]

    # Save
    cepton_sdk.export.save_points(
        points, output_path, file_type=cepton_sdk.export.PointsFileType.LAS)

    # Load
    points_tmp = cepton_sdk.export.load_points(output_path)[0]
    assert (numpy.max(numpy.abs(points.positions - points_tmp.positions)) < 1e-3)
    points = points_tmp

    # Plot points
    cepton_sdk.plot.plot_points(points)


if __name__ == "__main__":
    main()
