import atexit
import collections
import enum
import threading
import warnings

import cepton_sdk.c
from cepton_sdk.common import *

_all_builder = AllBuilder(__name__)


@enum.unique
class ControlFlag(enum.IntEnum):
    DISABLE_NETWORK = 1 << 1
    ENABLE_MULTIPLE_RETURNS = 1 << 4
    HOST_TIMESTAMPS = 1 << 6


@enum.unique
class FrameMode(enum.IntEnum):
    STREAMING = 0
    TIMED = 1
    COVER = 2
    CYCLE = 3


class _Manager:
    def __init__(self):
        self._lock = threading.Lock()
        self._error_callback = None

    def __del__(self):
        self.deinitialize()

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
        except:
            pass
        self._error_callback = None

    @staticmethod
    def _global_on_error(*args):
        _manager._on_error(*args)

    def _on_error(self, sensor_handle, error_code, error_msg, error_data, error_data_size, user_data):
        with self._lock:
            error_code_name = cepton_sdk.c.get_error_code_name(error_code)
            if not error_msg:
                msg = error_code_name
            else:
                msg = "{}: {}".format(
                    error_code_name, error_msg.decode("utf-8"))
            error = cepton_sdk.c.C_Error(error_code, msg=msg)
            if self._error_callback is None:
                cepton_sdk.c.log_error(error)
            else:
                self._error_callback(sensor_handle, error)


_manager = _Manager()


def is_initialized():
    return bool(cepton_sdk.c.c_is_initialized())


def get_control_flags():
    return int(cepton_sdk.c.c_get_control_flags())


def has_control_flags(mask):
    return (get_control_flags() & mask) == mask


def set_control_flags(mask, flags):
    cepton_sdk.c.c_set_control_flags(mask, flags)


def enable_control_flags(flags):
    set_control_flags(flags, flags)


def disable_control_flags(flags):
    set_control_flags(flags, 0)


def has_control_flag(flag):
    return bool(cepton_sdk.c.c_has_control_flag(flag))


def get_port():
    return int(cepton_sdk.c.c_get_port())


def set_port(port):
    cepton_sdk.c.c_set_port(port)


def set_frame_options(mode, length=None):
    c_options = cepton_sdk.c.c_create_frame_options()
    c_options.mode = mode
    if length is not None:
        c_options.length = length
    cepton_sdk.c.c_set_frame_options(c_options)


def get_frame_length():
    return float(cepton_sdk.c.c_get_frame_length())


def get_frame_mode():
    return FrameMode(cepton_sdk.c.c_get_frame_mode())


class _Callback:
    def __init__(self):
        self._lock = threading.Lock()
        self._callbacks = {}
        self._i_callback = 0

    def clear(self):
        self._callbacks = {}

    def listen(self, callback):
        with self._lock:
            callback_id = self._i_callback
            self._i_callback += 1
            self._callbacks[callback_id] = callback
            return callback_id

    def unlisten(self, callback_id):
        with self._lock:
            del self._callbacks[callback_id]

    def _on_callback(self, *args, **kwargs):
        with self._lock:
            for callback in self._callbacks.values():
                callback(*args, **kwargs)


class _FramesCallback(_Callback):
    def __del__(self):
        self.deinitialize()

    def initialize(self):
        self.deinitialize()
        self._c_on_callback = \
            cepton_sdk.c.C_SensorImageDataCallback(
                lambda *args: self._on_frame(*args[:-1]))
        cepton_sdk.c.c_listen_image_frames(self._c_on_callback, None)

    def deinitialize(self):
        try:
            cepton_sdk.c.c_unlisten_image_frames()
        except:
            pass
        self.clear()

    def _on_frame(self, sensor_handle, n_points, c_image_points_ptr):
        sensor_info = \
            cepton_sdk.sensor.get_sensor_information_by_handle(sensor_handle)
        points = cepton_sdk.point.Points.from_c(n_points, c_image_points_ptr)
        self._on_callback(sensor_info, points)


_frames_callback = _FramesCallback()


class _SerialLinesCallback(_Callback):
    def __del__(self):
        self.deinitialize()

    def initialize(self):
        self.deinitialize()
        self._c_on_callback = \
            cepton_sdk.c.C_SerialReceiveCallback(
                lambda *args: self._on_line(*args[:-1]))
        cepton_sdk.c.c_listen_serial_lines(self._c_on_callback, None)

    def deinitialize(self):
        try:
            cepton_sdk.c.c_unlisten_serial_lines()
        except:
            pass
        self.clear()

    def _on_line(self, sensor_handle, line):
        sensor_info = \
            cepton_sdk.sensor.get_sensor_information_by_handle(sensor_handle)
        self._on_callback(sensor_info, line)


_serial_lines_callback = _SerialLinesCallback()


__all__ = _all_builder.get()
