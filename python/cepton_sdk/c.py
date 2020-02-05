from cepton_sdk.common.c import *  # noqa isort:skip
import enum
import os.path
import warnings
from ctypes import *

from cepton_sdk.common import *

_all_builder = AllBuilder(__name__)


SDK_VERSION = 19

_module_dir = os.path.dirname(os.path.abspath(__file__))
lib = load_c_library(_module_dir, "cepton_sdk")

# ------------------------------------------------------------------------------
# General
# ------------------------------------------------------------------------------
C_SensorHandle = c_uint64

c_get_version_string = lib.cepton_sdk_get_version_string
c_get_version_string.restype = c_char_p

c_get_version_major = lib.cepton_sdk_get_version_major
c_get_version_major.restype = c_int

c_get_version_minor = lib.cepton_sdk_get_version_minor
c_get_version_minor.restype = c_int

c_get_version_patch = lib.cepton_sdk_get_version_patch
c_get_version_patch.restype = c_int


def get_version_string():
    return c_get_version_string().decode("utf-8")


def get_version_major():
    return c_get_version_major()


def get_version_minor():
    return c_get_version_minor()


def get_version_patch():
    return c_get_version_patch()


# ------------------------------------------------------------------------------
# Errors
# ------------------------------------------------------------------------------


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


c_get_error_code_name = lib.cepton_get_error_code_name
c_get_error_code_name.argtypes = [c_int]
c_get_error_code_name.restype = c_char_p

c_is_error_code = lib.cepton_is_error_code
c_is_error_code.argtypes = [c_int]
c_is_error_code.restype = c_int

c_is_fault_code = lib.cepton_is_fault_code
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
        code: `cepton_sdk.C_ErrorCode`
    """

    def __init__(self, code=C_ErrorCode.CEPTON_SUCCESS, msg=None, data=None):
        code_name = get_error_code_name(code)
        if msg:
            msg = "{}: {}".format(code_name, msg)
        else:
            msg = code_name
        super().__init__(msg)
        self.code = code
        self.data = data

    def __bool__(self):
        return self.code != C_ErrorCode.CEPTON_SUCCESS

    def __int__(self):
        return self.code

    @property
    def name(self):
        return get_error_code_name(self.code)

    def is_error(self):
        return is_error_code(self.code)

    def is_fault(self):
        return is_fault_code(self.code)


class C_Warning(Warning):
    pass


c_get_error = lib.cepton_sdk_get_error
c_get_error.argtypes = [POINTER(c_char_p)]
c_get_error.restype = c_int


def get_error():
    error_msg = c_char_p()
    error_code = c_get_error(byref(error_msg))
    return C_Error(error_code, error_msg.value.decode("utf-8"))


def check_error(error):
    if not error:
        return error
    if error.is_error():
        raise error
    else:
        warnings.warn(str(error), C_Warning, stacklevel=2)
    return error


def log_error(error):
    if not error:
        return error
    warnings.warn(str(error), C_Warning, stacklevel=2)
    return error


def _c_error_check(error_code, func, args):
    check_error(get_error())


def add_c_error_check(c_func):
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


c_create_frame_options = lib.cepton_sdk_create_frame_options
c_create_frame_options.restype = C_FrameOptions


class C_Options(Structure):
    _fields_ = [
        ("signature", c_size_t),
        ("control_flags", c_uint32),
        ("frame", C_FrameOptions),
        ("port", c_uint16),
    ]


c_create_options = lib.cepton_sdk_create_options
c_create_options.restype = C_Options

C_SensorErrorCallback = \
    CFUNCTYPE(None,
              C_SensorHandle, c_int, c_char_p, c_void_p, c_size_t, c_void_p)

c_is_initialized = lib.cepton_sdk_is_initialized
c_is_initialized.restype = c_int

c_initialize = lib.cepton_sdk_initialize
c_initialize.argtypes = \
    [c_int, POINTER(C_Options), C_SensorErrorCallback, c_void_p]
add_c_error_check(c_initialize)

c_deinitialize = lib.cepton_sdk_deinitialize
c_deinitialize.argtypes = []
add_c_error_check(c_deinitialize)

c_set_control_flags = lib.cepton_sdk_set_control_flags
c_set_control_flags.argtypes = [c_uint32, c_uint32]
add_c_error_check(c_set_control_flags)

c_get_control_flags = lib.cepton_sdk_get_control_flags
c_get_control_flags.argtypes = []
c_get_control_flags.restype = c_uint32

c_has_control_flag = lib.cepton_sdk_has_control_flag
c_has_control_flag.argtypes = [c_uint32]
c_has_control_flag.restype = c_int

c_get_port = lib.cepton_sdk_get_port
c_get_port.restype = c_uint16

c_set_port = lib.cepton_sdk_set_port
c_set_port.argtypes = [c_uint16]
add_c_error_check(c_set_port)

c_set_frame_options = lib.cepton_sdk_set_frame_options
c_set_frame_options.argtypes = [POINTER(C_FrameOptions)]
add_c_error_check(c_set_frame_options)

c_get_frame_mode = lib.cepton_sdk_get_frame_mode
c_get_frame_mode.restype = c_uint32

c_get_frame_length = lib.cepton_sdk_get_frame_length
c_get_frame_length.restype = c_float

# ------------------------------------------------------------------------------
# Sensors
# ------------------------------------------------------------------------------


class C_FirmwareVersion(Structure):
    _fields_ = [
        ('major', c_uint8),
        ('minor', c_uint8),
        ('padding', c_uint8 * 2)
    ]


class C_SensorInformation(Structure):
    _fields_ = [
        ("handle", C_SensorHandle),
        ("serial_number", c_uint64),
        ("model_name", c_char * 28),
        ("model", c_uint16),
        ("", c_uint16),
        ("firmware_version", c_char * 28),
        ("formal_firmware_version", C_FirmwareVersion),

        ("last_reported_temperature", c_float),
        ("last_reported_humidity", c_float),
        ("last_reported_age", c_float),

        ("measurement_period", c_float),

        ("ptp_ts", c_int64),

        ("gps_ts_year", c_uint8),
        ("gps_ts_month", c_uint8),
        ("gps_ts_day", c_uint8),
        ("gps_ts_hour", c_uint8),
        ("gps_ts_min", c_uint8),
        ("gps_ts_sec", c_uint8),

        ("return_count", c_uint8),
        ("segment_count", c_uint8),

        ("is_mocked", c_uint32, 1),
        ("is_pps_connected", c_uint32, 1),
        ("is_nmea_connected", c_uint32, 1),
        ("is_ptp_connected", c_uint32, 1),
        ("is_calibrated", c_uint32, 1),
    ]


check_c_size(lib, C_SensorInformation, "cepton_sensor_information_size")

c_get_n_sensors = lib.cepton_sdk_get_n_sensors
c_get_n_sensors.restype = c_size_t

c_get_sensor_handle_by_serial_number = \
    lib.cepton_sdk_get_sensor_handle_by_serial_number
c_get_sensor_handle_by_serial_number.argtypes = \
    [c_uint64, POINTER(C_SensorHandle)]
add_c_error_check(c_get_sensor_handle_by_serial_number)

c_get_sensor_information = lib.cepton_sdk_get_sensor_information
c_get_sensor_information.argtypes = \
    [C_SensorHandle, POINTER(C_SensorInformation)]
add_c_error_check(c_get_sensor_information)

c_get_sensor_information_by_index = \
    lib.cepton_sdk_get_sensor_information_by_index
c_get_sensor_information_by_index.argtypes = \
    [c_size_t, POINTER(C_SensorInformation)]
add_c_error_check(c_get_sensor_information_by_index)

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
        ("return_type", c_uint8),
        ("flags", c_uint8),
        ("segment_id", c_uint8),
        ("reserved", c_uint8),
    ]


check_c_size(lib, C_SensorImagePoint, "cepton_sensor_image_point_size")


C_SensorImageDataCallback = \
    CFUNCTYPE(None,
              C_SensorHandle, c_size_t, POINTER(C_SensorImagePoint), c_void_p)

c_listen_image_frames = lib.cepton_sdk_listen_image_frames
c_listen_image_frames.argtypes = [C_SensorImageDataCallback, c_void_p]
add_c_error_check(c_listen_image_frames)

c_unlisten_image_frames = lib.cepton_sdk_unlisten_image_frames
add_c_error_check(c_unlisten_image_frames)

# ------------------------------------------------------------------------------
# Serial
# ------------------------------------------------------------------------------

C_SerialReceiveCallback = \
    CFUNCTYPE(None,
              C_SensorHandle, c_char_p, c_void_p)

c_listen_serial_lines = lib.cepton_sdk_listen_serial_lines
c_listen_serial_lines.argtypes = [C_SerialReceiveCallback, c_void_p]
add_c_error_check(c_listen_serial_lines)

c_unlisten_serial_lines = lib.cepton_sdk_unlisten_serial_lines
add_c_error_check(c_unlisten_serial_lines)

# ------------------------------------------------------------------------------
# Capture
# ------------------------------------------------------------------------------
c_capture_replay_is_open = lib.cepton_sdk_capture_replay_is_open
c_capture_replay_is_open.restype = c_int

c_capture_replay_open = lib.cepton_sdk_capture_replay_open
c_capture_replay_open.argtypes = [c_char_p]
add_c_error_check(c_capture_replay_open)

c_capture_replay_close = lib.cepton_sdk_capture_replay_close
add_c_error_check(c_capture_replay_close)

c_capture_replay_get_filename = lib.cepton_sdk_capture_replay_get_filename
c_capture_replay_get_filename.restype = c_char_p

c_capture_replay_get_start_time = lib.cepton_sdk_capture_replay_get_start_time
c_capture_replay_get_start_time.restype = c_int64

c_capture_replay_get_position = lib.cepton_sdk_capture_replay_get_position
c_capture_replay_get_position.restype = c_float

c_capture_replay_get_length = lib.cepton_sdk_capture_replay_get_length
c_capture_replay_get_length.restype = c_float

c_capture_replay_is_end = lib.cepton_sdk_capture_replay_is_end
c_capture_replay_is_end.restype = c_int

c_capture_replay_rewind = lib.cepton_sdk_capture_replay_rewind
add_c_error_check(c_capture_replay_rewind)

c_capture_replay_seek = lib.cepton_sdk_capture_replay_seek
c_capture_replay_seek.argtypes = [c_float]
add_c_error_check(c_capture_replay_seek)

c_capture_replay_get_enable_loop = lib.cepton_sdk_capture_replay_get_enable_loop
c_capture_replay_get_enable_loop.restype = c_int

c_capture_replay_set_enable_loop = lib.cepton_sdk_capture_replay_set_enable_loop
c_capture_replay_set_enable_loop.argtypes = [c_bool]
add_c_error_check(c_capture_replay_set_enable_loop)

c_capture_replay_get_speed = lib.cepton_sdk_capture_replay_get_speed
c_capture_replay_get_speed.restype = c_float

c_capture_replay_set_speed = lib.cepton_sdk_capture_replay_set_speed
c_capture_replay_set_speed.argtypes = [c_float]
add_c_error_check(c_capture_replay_set_speed)

c_capture_replay_resume_blocking_once = \
    lib.cepton_sdk_capture_replay_resume_blocking_once
add_c_error_check(c_capture_replay_resume_blocking_once)

c_capture_replay_resume_blocking = lib.cepton_sdk_capture_replay_resume_blocking
c_capture_replay_resume_blocking.argtypes = [c_float]
add_c_error_check(c_capture_replay_resume_blocking)

c_capture_replay_is_running = lib.cepton_sdk_capture_replay_is_running
c_capture_replay_is_running.restype = c_int

c_capture_replay_resume = lib.cepton_sdk_capture_replay_resume
add_c_error_check(c_capture_replay_resume)

c_capture_replay_pause = lib.cepton_sdk_capture_replay_pause
add_c_error_check(c_capture_replay_pause)

__all__ = _all_builder.get()
