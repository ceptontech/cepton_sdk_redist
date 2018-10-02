import collections
import atexit
import enum
import threading
import warnings

import cepton_sdk.c

__all__ = [
    "ControlFlag",
    "disable_control_flags",
    "enable_control_flags",
    "FrameMode",
    "get_control_flags",
    "get_frame_length",
    "get_frame_mode",
    "get_port",
    "has_control_flag",
    "has_control_flags",
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
    ENABLE_STRAY_FILTER = 1 << 5


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

    def listen(self, callback):
        with self._lock:
            callback_id = self._i_callback
            self._i_callback += 1
            self._callbacks[callback_id] = callback
            return callback_id

    def unlisten(self, callback_id):
        with self._lock:
            del self._callbacks[callback_id]


class _ImageFramesCallback(_Callback):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    def __del__(self):
        self.deinitialize()

    def initialize(self):
        self.deinitialize()
        self._c_on_callback = \
            cepton_sdk.c.C_SensorImageDataCallback(
                lambda *args: self._on_callback(*args[:-1]))
        cepton_sdk.c.c_listen_image_frames(self._c_on_callback, None)

    def deinitialize(self):
        try:
            cepton_sdk.c.c_unlisten_image_frames()
        except cepton_sdk.c.C_Error:
            pass
        self._callbacks.clear()

    def _on_callback(self, sensor_handle, n_points, c_image_points_ptr):
        sensor_info = \
            cepton_sdk.sensor.get_sensor_information_by_handle(sensor_handle)
        image_points = \
            cepton_sdk.point.ImagePoints.from_c(n_points, c_image_points_ptr)
        with self._lock:
            for callback in self._callbacks.values():
                callback(sensor_info, image_points)


_image_frames_callback = _ImageFramesCallback()
