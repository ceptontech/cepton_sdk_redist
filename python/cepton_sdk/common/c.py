import os
import os.path
import platform
import sys
from ctypes import *

import numpy

# ------------------------------------------------------------------------------
# Load library
# ------------------------------------------------------------------------------


def load_c_library(parent_dir, name):
    if sys.platform.startswith("linux"):
        if platform.machine().startswith("armv"):
            os_name = "linux-arm"
        else:
            os_name = "linux-{}".format(platform.machine())
        lib_name = "lib{}.so".format(name)
    elif sys.platform.startswith("darwin"):
        os_name = "osx"
        lib_name = "lib{}.dylib".format(name)
    elif sys.platform.startswith("win"):
        os_name = "win64"
        lib_name = "{}.dll".format(name)
    else:
        raise NotImplementedError("Platform not supported!")
    lib_dir = "lib/{}/".format(os_name)

    # Try local and global search paths
    path = os.path.join(parent_dir, lib_dir, lib_name)
    if not os.path.exists(path):
        path = lib_name
    return CDLL(path)

# ------------------------------------------------------------------------------
# Structures
# ------------------------------------------------------------------------------


def set_c_fields_from_dict(c_obj, d):
    # TODO: set arrays properly
    valid = set([])
    for name, _ in c_obj._fields_:
        if name in d:
            setattr(c_obj, name, d[name])
            valid.add(name)
    if valid != set(d.keys()):
        raise AttributeError("invalid keys")


def from_bytes(c_type, buffer):
    assert (sizeof(c_type) <= len(buffer))
    c_value = c_type()
    memmove(addressof(c_value), buffer, sizeof(c_value))
    return c_value


def to_bytes(c_value):
    buffer = create_string_buffer(sizeof(c_value))
    memmove(buffer, addressof(c_value), sizeof(c_value))
    return buffer


# ------------------------------------------------------------------------------
# Arrays
# ------------------------------------------------------------------------------
_C_NDARRAY_REQUIREMENTS = ["C_CONTIGUOUS", "ALIGNED", "WRITEABLE", "OWNDATA"]


def get_c_ndpointer_type(dtype, ndim=1):
    return numpy.ctypeslib.ndpointer(
        dtype=dtype, ndim=ndim, flags=_C_NDARRAY_REQUIREMENTS)


def get_c_ndarray(a, dtype):
    return numpy.require(
        a, dtype=dtype, requirements=_C_NDARRAY_REQUIREMENTS)


def create_c_ndarray(size, dtype):
    a = numpy.zeros(size, dtype=dtype)
    return get_c_ndarray(a, dtype)


def convert_bytes_to_ndarray(a_bytes, c_type):
    """Convert bytes array to structured array"""
    dtype = numpy.dtype(c_type)
    assert (sizeof(c_type) == dtype.itemsize)
    a = a_bytes.view(dtype=dtype)
    assert (len(a) == len(a_bytes) / sizeof(c_type))
    return a


def convert_ndarray_to_bytes(a):
    """Convert structured array to bytes array"""
    return a.view(dtype=numpy.uint8)


def convert_c_array_to_ndarray(c_a, n, c_type):
    """Convert ctypes array to structured array"""
    n_bytes = n * sizeof(c_type)
    a_bytes = \
        numpy.ctypeslib.as_array(cast(c_a, POINTER(c_byte)), shape=(n_bytes,))
    return convert_bytes_to_ndarray(a_bytes, c_type)


def convert_ndarray_to_c_array(a, c_type):
    """Convert structured array to ctypes array"""
    a_bytes = convert_ndarray_to_bytes(a)
    assert (len(a_bytes) == len(a) * sizeof(c_type))
    c_a_bytes = numpy.ctypeslib.as_ctypes(a_bytes)
    c_a = cast(c_a_bytes, POINTER(c_type))
    return c_a


def unpackbits(a):
    """Convert array of integers to array of bool"""
    bits = numpy.unpackbits(a.flatten().view(numpy.uint8)).astype(bool)
    bits = bits.reshape([a.size, -1, 8])
    bits = bits[:, :, ::-1]
    bits = bits.reshape(list(a.shape) + [-1])
    return bits
