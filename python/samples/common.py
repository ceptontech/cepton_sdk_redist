import os.path

import numpy

from cepton_util.common import *


def get_sample_capture_path():
    module_dir_path = os.path.dirname(__file__)
    capture_path = fix_path(os.path.join(module_dir_path, "1.pcap"))
    return capture_path


def print_points(points):
    options = {
        "precision": 3,
        "suppress_small": True,
    }
    s = numpy.array_str(points.positions, **options)
    print("Positions [m]:\n{}".format(s))
