import collections
import atexit
import enum
import warnings

import cepton_sdk.c

__all__ = [
    "ControlFlag",
    "deinitialize",
    "FrameMode",
    "get_control_flags",
    "get_frame_length",
    "get_frame_mode",
    "get_port",
    "has_control_flag",
    "is_initialized",
    "SensorError",
    "set_control_flags",
    "set_frame_options",
    "set_port",
]

SensorError = collections.namedtuple(
    "SensorError", "error_code message error_data")


@enum.unique
class ControlFlag(enum.IntEnum):
    DISABLE_NETWORK = 1 << 1
    DISABLE_IMAGE_CLIP = 1 << 2
    DISABLE_DISTANCE_CLIP = 1 << 3
    ENABLE_MULTIPLE_RETURNS = 1 << 4


@enum.unique
class FrameMode(enum.IntEnum):
    STREAMING = 0
    TIMED = 1
    COVER = 2
    CYCLE = 3


class _Manager(object):
    def __init__(self):
        _error_callback = None
        _c_on_error = None

    def initialize(self, control_flags=None, error_callback=None,
                   frame_mode=None, frame_length=None, port=None):
        if error_callback is not None:
            self._error_callback = error_callback
        self._c_on_error = \
            cepton_sdk.c.C_SensorErrorCallback(self._global_on_error)

        c_options = cepton_sdk.c.c_create_options()
        if control_flags is not None:
            c_options.control_flags = control_flags
        if frame_length is not None:
            c_options.frame.length = frame_length
        if frame_mode is not None:
            c_options.frame.mode = frame_mode
        if port is not None:
            c_options.port = port
        cepton_sdk.c.c_initialize(
            cepton_sdk.c.SDK_VERSION, c_options, self._c_on_error, None)

    def deinitialize(self):
        try:
            cepton_sdk.c.c_deinitialize()
        except BaseException as e:
            traceback.print_exc()

        self._callback = None

    def set_error_callback(self, error_callback):
        self._error_callback = error_callback

    def clear_error_callback(self):
        self._error_callback = None

    @staticmethod
    def _global_on_error(*args):
        _manager._on_error(*args)

    def _on_error(self, sensor_handle, error_code, message, error_data, error_data_size, user_data):
        cepton_sdk.c.check_error_code(error_code)
        if error_code:
            return

        if self._error_callback is not None:
            error = SensorError(
                error_code=error_code, message=message, error_data=None)
            self._error_callback(sensor_handle, error)


_manager = _Manager()


def is_initialized():
    return bool(cepton_sdk.c.c_is_initialized())


deinitialize = _manager.deinitialize

set_error_callback = _manager.set_error_callback
clear_error_callback = _manager.clear_error_callback


def get_control_flags():
    return int(cepton_sdk.c.c_get_control_flags())


def set_control_flags(mask, flags):
    cepton_sdk.c.c_set_control_flags(mask, flags)


def has_control_flag(flag):
    return bool(cepton_sdk.c.c_has_control_flag(flag))


def get_port():
    return int(cepton_sdk.c.c_get_port())


def set_port(port):
    cepton_sdk.c.c_set_port(port)


def set_frame_options(mode, length=None):
    c_options = cepton_sdk.c.c_create_frame_options()
    if length is not None:
        c_options.length = length
    cepton_sdk.c.c_set_frame_options(c_options)


def get_frame_length():
    return float(cepton_sdk.c.c_get_frame_length())


def get_frame_mode():
    return FrameMode(cepton_sdk.c.c_get_frame_mode())
