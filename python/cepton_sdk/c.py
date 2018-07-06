import ctypes.util
import enum
import os
import os.path
import platform
import sys
import warnings
from ctypes import *

import numpy

SDK_VERSION = 13

# ------------------------------------------------------------------------------
# Load library
# ------------------------------------------------------------------------------


def _load_c_library():
    module_path = os.path.dirname(os.path.abspath(__file__))
    if sys.platform.startswith("linux"):
        if platform.machine().startswith("armv"):
            os_name = "linux-arm"
        else:
            os_name = "linux-{}".format(platform.machine())
        lib_name = "libcepton_sdk.so"
    elif sys.platform.startswith("darwin"):
        os_name = "osx"
        lib_name = "libcepton_sdk.dylib"
    elif sys.platform.startswith("win"):
        os_name = "win64"
        lib_name = "cepton_sdk.dll"
    else:
        raise NotImplementedError("Platform not supported!")
    lib_dir = "lib/{}/".format(os_name)

    # Try local and global search paths
    module_file = os.path.join(module_path, lib_dir, lib_name)
    if not os.path.exists(module_file):
        module_file = lib_name
    return CDLL(module_file)


libcepton_sdk = _load_c_library()

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
    bits = numpy.unpackbits(a.flatten().view(numpy.uint8)).astype(bool)
    bits = bits.reshape([a.size, -1, 8])
    bits = bits[:, :, ::-1]
    bits = bits.reshape(list(a.shape) + [-1])
    return bits


# ------------------------------------------------------------------------------
# General
# ------------------------------------------------------------------------------
C_SensorHandle = c_uint64


class C_ErrorCode(enum.IntEnum):
    CEPTON_SUCCESS = 0
    CEPTON_ERROR_GENERIC = -1
    CEPTON_ERROR_OUT_OF_MEMORY = -2
    CEPTON_ERROR_SENSOR_NOT_FOUND = -4
    CEPTON_ERROR_SDK_VERSION_MISMATCH = -5
    CEPTON_ERROR_COMMUNICATION = -6
    CEPTON_ERROR_TOO_MANY_CALLBACKS = -7
    CEPTON_ERROR_INVALID_ARGUMENTS = -8
    CEPTON_ERROR_ALREADY_INITIALIZED = -9
    CEPTON_ERROR_NOT_INITIALIZED = -10
    CEPTON_ERROR_INVALID_FILE_TYPE = -11
    CEPTON_ERROR_FILE_IO = -12
    CEPTON_ERROR_CORRUPT_FILE = -13
    CEPTON_ERROR_NOT_OPEN = -14
    CEPTON_ERROR_EOF = -15

    CEPTON_FAULT_INTERNAL = -1000
    CEPTON_FAULT_EXTREME_TEMPERATURE = -1001
    CEPTON_FAULT_EXTREME_HUMIDITY = -1002
    CEPTON_FAULT_EXTREME_ACCELERATION = -1003
    CEPTON_FAULT_ABNORMAL_FOV = -1004
    CEPTON_FAULT_ABNORMAL_FRAME_RATE = -1005
    CEPTON_FAULT_MOTOR_MALFUNCTION = -1006
    CEPTON_FAULT_LASER_MALFUNCTION = -1007
    CEPTON_FAULT_DETECTOR_MALFUNCTION = -1008


c_get_error_code_name = libcepton_sdk.cepton_get_error_code_name
c_get_error_code_name.argtypes = [c_int]
c_get_error_code_name.restype = c_char_p

c_is_error_code = libcepton_sdk.cepton_is_error_code
c_is_error_code.argtypes = [c_int]
c_is_error_code.restype = c_int

c_is_fault_code = libcepton_sdk.cepton_is_fault_code
c_is_fault_code.argtypes = [c_int]
c_is_fault_code.restype = c_int


def get_error_code_name(error_code):
    error_code_name = c_get_error_code_name(error_code)
    return error_code_name.decode("utf-8")


def is_error_code(error_code):
    return bool(c_is_error_code(error_code))


def is_fault_code(error_code):
    return bool(c_is_fault_code(error_code))


class C_Error(Exception):
    """Error thrown by most sdk functions.

    Attributes:
        error_code: `cepton_sdk.C_ErrorCode`
    """

    def __init__(self, message, error_code):
        super().__init__(message)
        self.error_code = error_code

    @classmethod
    def create(cls, error_code):
        message = get_error_code_name(error_code)
        return cls(message, error_code)


class C_Warning(Warning):
    pass


def check_error_code(error_code, warning=False):
    if not error_code:
        return
    if is_error_code(error_code) and (not warning):
        raise C_Error.create(error_code)
    else:
        message = get_error_code_name(error_code)
        warnings.warn(message, C_Warning, stacklevel=2)


def _c_error_check(error_code, func, args):
    check_error_code(error_code)


def _add_c_error_check(c_func):
    c_func.restype = c_int
    c_func.errcheck = _c_error_check


# ------------------------------------------------------------------------------
# Setup
# ------------------------------------------------------------------------------
class C_FrameOptions(Structure):
    _fields_ = [
        ("signature", c_size_t),
        ("mode", c_uint32),
        ("length", c_float),
    ]


c_create_frame_options = libcepton_sdk.cepton_sdk_create_frame_options
c_create_frame_options.restype = C_FrameOptions


class C_Options(Structure):
    _fields_ = [
        ("signature", c_size_t),
        ("control_flags", c_uint32),
        ("frame", C_FrameOptions),
        ("port", c_uint16),
    ]


c_create_options = libcepton_sdk.cepton_sdk_create_options
c_create_options.restype = C_Options

C_SensorErrorCallback = CFUNCTYPE(
    None, C_SensorHandle, c_int, c_char_p, c_void_p, c_size_t, c_void_p)

c_is_initialized = libcepton_sdk.cepton_sdk_is_initialized
c_is_initialized.restype = c_int

c_initialize = libcepton_sdk.cepton_sdk_initialize
c_initialize.argtypes = \
    [c_int, POINTER(C_Options), C_SensorErrorCallback, c_void_p]
_add_c_error_check(c_initialize)

c_deinitialize = libcepton_sdk.cepton_sdk_deinitialize
c_deinitialize.argtypes = []
_add_c_error_check(c_deinitialize)

c_set_control_flags = libcepton_sdk.cepton_sdk_set_control_flags
c_set_control_flags.argtypes = [c_uint32, c_uint32]
_add_c_error_check(c_set_control_flags)

c_get_control_flags = libcepton_sdk.cepton_sdk_get_control_flags
c_get_control_flags.argtypes = []
c_get_control_flags.restype = c_uint32

c_has_control_flag = libcepton_sdk.cepton_sdk_has_control_flag
c_has_control_flag.argtypes = [c_uint32]
c_has_control_flag.restype = c_int

c_get_port = libcepton_sdk.cepton_sdk_get_port
c_get_port.restype = c_uint16

c_set_port = libcepton_sdk.cepton_sdk_set_port
c_set_port.argtypes = [c_uint16]
_add_c_error_check(c_set_port)

c_set_frame_options = libcepton_sdk.cepton_sdk_set_frame_options
c_set_frame_options.argtypes = [POINTER(C_FrameOptions)]
_add_c_error_check(c_set_frame_options)

c_get_frame_mode = libcepton_sdk.cepton_sdk_get_frame_mode
c_get_frame_mode.restype = c_uint32

c_get_frame_length = libcepton_sdk.cepton_sdk_get_frame_length
c_get_frame_length.restype = c_float

c_clear_cache = libcepton_sdk.cepton_sdk_clear_cache
_add_c_error_check(c_clear_cache)

# ------------------------------------------------------------------------------
# Sensors
# ------------------------------------------------------------------------------


class C_SensorInformation(Structure):
    _fields_ = [
        ("handle", C_SensorHandle),
        ("serial_number", c_uint64),
        ("model_name", c_char * 28),
        ("model", c_int32),
        ("firmware_version", c_char * 32),

        ("last_reported_temperature", c_float),
        ("last_reported_humidity", c_float),
        ("last_reported_age", c_float),
        ("", c_float),

        ("ptp_ts", c_int64),

        ("gps_ts_year", c_uint8),
        ("gps_ts_month", c_uint8),
        ("gps_ts_day", c_uint8),
        ("gps_ts_hour", c_uint8),
        ("gps_ts_min", c_uint8),
        ("gps_ts_sec", c_uint8),

        ("return_count", c_uint8),
        ("", c_uint8),

        ("is_mocked", c_uint32, 1),
        ("is_pps_connected", c_uint32, 1),
        ("is_nmea_connected", c_uint32, 1),
        ("is_ptp_connected", c_uint32, 1),
        ("is_calibrated", c_uint32, 1),
    ]


c_cepton_sensor_information_size = \
    c_size_t.in_dll(libcepton_sdk, "cepton_sensor_information_size").value
assert (sizeof(C_SensorInformation) == c_cepton_sensor_information_size)

c_get_n_sensors = libcepton_sdk.cepton_sdk_get_n_sensors
c_get_n_sensors.restype = c_size_t

c_get_sensor_handle_by_serial_number = \
    libcepton_sdk.cepton_sdk_get_sensor_handle_by_serial_number
c_get_sensor_handle_by_serial_number.argtypes = \
    [c_uint64, POINTER(C_SensorHandle)]
_add_c_error_check(c_get_sensor_handle_by_serial_number)

c_get_sensor_information = libcepton_sdk.cepton_sdk_get_sensor_information
c_get_sensor_information.argtypes = \
    [C_SensorHandle, POINTER(C_SensorInformation)]
_add_c_error_check(c_get_sensor_information)

c_get_sensor_information_by_index = \
    libcepton_sdk.cepton_sdk_get_sensor_information_by_index
c_get_sensor_information_by_index.argtypes = \
    [c_size_t, POINTER(C_SensorInformation)]
_add_c_error_check(c_get_sensor_information_by_index)

# ------------------------------------------------------------------------------
# Points
# ------------------------------------------------------------------------------


class C_SensorImagePoint(Structure):
    _fields_ = [
        ("timestamp", c_int64),
        ("image_x", c_float),
        ("distance", c_float),
        ("image_z", c_float),
        ("intensity", c_float),
        ("return_number", c_uint8),
        ("valid", c_uint8),
        ("saturated", c_uint8),
        ("reserved", c_uint8)
    ]


c_cepton_sensor_image_point_size = \
    c_size_t.in_dll(libcepton_sdk, "cepton_sensor_image_point_size").value
assert (sizeof(C_SensorImagePoint) == c_cepton_sensor_image_point_size)

# ------------------------------------------------------------------------------
# Listen
# ------------------------------------------------------------------------------
C_SensorImageDataCallback = \
    CFUNCTYPE(None,
              C_SensorHandle, c_size_t, POINTER(C_SensorImagePoint), c_void_p)

c_listen_image_frames = libcepton_sdk.cepton_sdk_listen_image_frames
c_listen_image_frames.argtypes = [C_SensorImageDataCallback, c_void_p]
_add_c_error_check(c_listen_image_frames)

c_unlisten_image_frames = libcepton_sdk.cepton_sdk_unlisten_image_frames
_add_c_error_check(c_unlisten_image_frames)

# ------------------------------------------------------------------------------
# Capture
# ------------------------------------------------------------------------------
c_capture_replay_is_open = libcepton_sdk.cepton_sdk_capture_replay_is_open
c_capture_replay_is_open.restype = c_int

c_capture_replay_open = libcepton_sdk.cepton_sdk_capture_replay_open
c_capture_replay_open.argtypes = [c_char_p]
_add_c_error_check(c_capture_replay_open)

c_capture_replay_close = libcepton_sdk.cepton_sdk_capture_replay_close
_add_c_error_check(c_capture_replay_close)

c_capture_replay_get_start_time = \
    libcepton_sdk.cepton_sdk_capture_replay_get_start_time
c_capture_replay_get_start_time.restype = c_int64

c_capture_replay_get_position = \
    libcepton_sdk.cepton_sdk_capture_replay_get_position
c_capture_replay_get_position.restype = c_float

c_capture_replay_get_length = \
    libcepton_sdk.cepton_sdk_capture_replay_get_length
c_capture_replay_get_length.restype = c_float

c_capture_replay_is_end = \
    libcepton_sdk.cepton_sdk_capture_replay_is_end
c_capture_replay_is_end.restype = c_int

c_capture_replay_rewind = \
    libcepton_sdk.cepton_sdk_capture_replay_rewind
_add_c_error_check(c_capture_replay_rewind)

c_capture_replay_seek = \
    libcepton_sdk.cepton_sdk_capture_replay_seek
c_capture_replay_seek.argtypes = [c_float]
_add_c_error_check(c_capture_replay_seek)

c_capture_replay_get_enable_loop = \
    libcepton_sdk.cepton_sdk_capture_replay_get_enable_loop
c_capture_replay_get_enable_loop.restype = c_int

c_capture_replay_set_enable_loop = \
    libcepton_sdk.cepton_sdk_capture_replay_set_enable_loop
c_capture_replay_set_enable_loop.argtypes = [c_bool]
_add_c_error_check(c_capture_replay_set_enable_loop)

c_capture_replay_get_speed = \
    libcepton_sdk.cepton_sdk_capture_replay_get_speed
c_capture_replay_get_speed.restype = c_float

c_capture_replay_set_speed = \
    libcepton_sdk.cepton_sdk_capture_replay_set_speed
c_capture_replay_set_speed.argtypes = [c_float]
_add_c_error_check(c_capture_replay_set_speed)

c_capture_replay_resume_blocking_once = \
    libcepton_sdk.cepton_sdk_capture_replay_resume_blocking_once
_add_c_error_check(c_capture_replay_resume_blocking_once)

c_capture_replay_resume_blocking = \
    libcepton_sdk.cepton_sdk_capture_replay_resume_blocking
c_capture_replay_resume_blocking.argtypes = [c_float]
_add_c_error_check(c_capture_replay_resume_blocking)

c_capture_replay_is_running = \
    libcepton_sdk.cepton_sdk_capture_replay_is_running
c_capture_replay_is_running.restype = c_int

c_capture_replay_resume = libcepton_sdk.cepton_sdk_capture_replay_resume
_add_c_error_check(c_capture_replay_resume)

c_capture_replay_pause = libcepton_sdk.cepton_sdk_capture_replay_pause
_add_c_error_check(c_capture_replay_pause)
