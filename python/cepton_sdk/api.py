import collections
import datetime
import threading
import time

import cepton_sdk.c
import cepton_sdk.capture_replay
import cepton_sdk.core
import cepton_sdk.sensor
from cepton_sdk.common.function import *

__all__ = [
    "get_sensors",
    "get_time",
    "get_timestamp",
    "has_sensor",
    "ImageFramesListener",
    "initialize",
    "deinitialize",
    "is_end",
    "is_live",
    "is_realtime",
    "listen_image_frames",
    "Sensor",
    "SensorImageFramesListener",
    "unlisten_image_frames",
    "wait",
]


def is_live():
    """Returns true if capture replay is not open."""
    return not cepton_sdk.capture_replay.is_open()


def is_realtime():
    """Returns true if live or capture replay is running."""
    return is_live() or cepton_sdk.capture_replay.is_running()


def is_end():
    """Returns true if next call to `wait` will throw `CEPTON_ERROR_EOF`"""
    if cepton_sdk.capture_replay.is_open():
        if cepton_sdk.capture_replay.get_enable_loop():
            return False
        return cepton_sdk.capture_replay.is_end()
    return False


def get_timestamp():
    """Returns unix timestamp"""
    return datetime.datetime.utcnow().timestamp()


def get_time():
    """Returns capture replay time or live time."""
    if is_live():
        return get_timestamp()
    else:
        return cepton_sdk.capture_replay.get_time()


def _wait(t_length):
    if is_realtime():
        time.sleep(t_length)
    else:
        cepton_sdk.capture_replay.resume_blocking(t_length)


def wait(t_length=0):
    """Resumes capture replay or sleeps for duration.

    If `t_length` is `0`, then waits forever.
    """
    if t_length == 0:
        while True:
            _wait(0.1)
            if is_end():
                break
    else:
        _wait(t_length)


def initialize(capture_path=None, capture_seek=0, control_flags=0,
               error_callback=None, port=None, wait_for_sensors=True, **kwargs):
    """Initializes SDK. Optionally starts capture replay.

    Arguments:
        control_flags: :class:`cepton_sdk.ControlFlag`
    """
    if capture_path is not None:
        control_flags |= cepton_sdk.core.ControlFlag.DISABLE_NETWORK
    options = {
        "control_flags": control_flags,
        "frame_mode": cepton_sdk.core.FrameMode.COVER,
    }
    options.update(kwargs)
    if error_callback is not None:
        options["error_callback"] = error_callback
    if port is not None:
        options["port"] = port
    cepton_sdk.core._manager.initialize(**options)
    cepton_sdk.core._image_frames_callback.initialize()

    if capture_path is not None:
        cepton_sdk.capture_replay.open(capture_path)
    if wait_for_sensors:
        wait(3)
        if capture_path is not None:
            cepton_sdk.capture_replay.seek(capture_seek)


def deinitialize():
    cepton_sdk.core._image_frames_callback.deinitialize()
    cepton_sdk.core._manager.deinitialize()


def listen_image_frames(callback):
    """Register image frames callback.

    Throws error if `callback_id` is currently registered.

    Returns:
        callback_id
    """
    return cepton_sdk.core._image_frames_callback.listen(callback)


def unlisten_image_frames(callback_id):
    """Unregisters image frames callback.

    Throws error if `callback_id` is not currently registered.
    """
    cepton_sdk.core._image_frames_callback.unlisten(callback_id)


def _wait_on_func(func, timeout=None):
    if timeout is not None:
        t_start = get_timestamp()
    while not func():
        wait(0.001)
        if timeout is not None:
            if (get_timestamp() - t_start) > timeout:
                raise RuntimeError("Timed out!")


class _ListenerBase:
    def __init__(self):
        self._lock = threading.Lock()
        self._callback_id = self._frames_callback().listen(self._on_points)

    def __del__(self):
        try:
            self._frames_callback().unlisten(self._callback_id)
        except:
            pass


class _FramesListener(_ListenerBase):
    """Listener for getting all sensor frames."""

    def __init__(self):
        self.i_frame_dict = collections.defaultdict(lambda: 0)
        self.points_dict = collections.defaultdict(list)
        super().__init__()

    def reset(self):
        with self._lock:
            self.i_frame_dict = collections.defaultdict(lambda: 0)
            self.points_dict = collections.defaultdict(list)

    def _on_points(self, sensor_info, points):
        with self._lock:
            self.i_frame_dict[sensor_info.serial_number] += 1
            if self.i_frame_dict[sensor_info.serial_number] > 2:
                self.points_dict[sensor_info.serial_number].append(points)

    def has_points(self):
        with self._lock:
            return len(self.points_dict) > 0

    def _get_points(self):
        with self._lock:
            points_dict = self.points_dict
            self.points_dict = collections.defaultdict(list)
        return points_dict

    def get_points(self, **kwargs):
        _wait_on_func(self.has_points, **kwargs)
        return self._get_points()


class _SensorFramesListener(_ListenerBase):
    def __init__(self, serial_number):
        self.serial_number = serial_number
        self.i_frame = 0
        self.points_list = []
        super().__init__()

    def reset(self):
        with self._lock:
            self.i_frame = 0
            self.points_list = []

    def _on_points(self, sensor_info, points):
        with self._lock:
            if sensor_info.serial_number != self.serial_number:
                return
            self.i_frame += 1
            if self.i_frame > 2:
                self.points_list.append(points)

    def has_points(self):
        with self._lock:
            return len(self.points_list) > 0

    def _get_points(self):
        with self._lock:
            points_list = self.points_list
            self.points_list = []
        return points_list

    def get_points(self, **kwargs):
        _wait_on_func(self.has_points, **kwargs)
        return self._get_points()


class _ImageListenerMixin:
    @classmethod
    def _frames_callback(cls):
        return cepton_sdk.core._image_frames_callback


class ImageFramesListener(_FramesListener, _ImageListenerMixin):
    pass


class SensorImageFramesListener(_SensorFramesListener, _ImageListenerMixin):
    pass


class Sensor:
    """
    Attributes:
        information (:class:`cepton_sdk.SensorInformation`)
    """

    def __init__(self, sensor_info):
        if type(sensor_info) is dict:
            self.information = SensorInformation.from_dict(sensor_info)
        else:
            self.information = sensor_info

    @property
    def handle(self):
        return self.information.handle

    @property
    def serial_number(self):
        return self.information.serial_number

    @classmethod
    def create_by_index(cls, sensor_index):
        sensor_info = \
            cepton_sdk.sensor.get_sensor_information_by_index(sensor_index)
        return cls(sensor_info)

    @classmethod
    def create_by_handle(cls, sensor_handle):
        sensor_info = \
            cepton_sdk.sensor.get_sensor_information_by_handle(sensor_handle)
        return cls(sensor_info)

    @classmethod
    def create(cls, serial_number):
        sensor_info = cepton_sdk.sensor.get_sensor_information(serial_number)
        return cls(sensor_info)

    def update(self):
        """Update sensor information.

        Should be called often, to pull latest sensor information.
        """
        self.information = \
            cepton_sdk.sensor.get_sensor_information_by_handle(self.handle)


def has_sensor(serial_number):
    """Returns true if sensor is attached"""
    try:
        cepton_sdk.sensor.get_sensor_handle(serial_number)
    except:
        return False
    return True


def get_sensors(cls=Sensor):
    """Returns attached sensors.

    Returns:
        Dictionary of sensors, indexed by serial number.
    """
    sensors_dict = {}
    for i_sensor in range(cepton_sdk.sensor.get_n_sensors()):
        sensor = cls.create_by_index(i_sensor)
        sensors_dict[sensor.serial_number] = sensor
    return sensors_dict
