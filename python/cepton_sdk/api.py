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
    "ImageFramesListener",
    "initialize",
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


def wait(t_length=0.1):
    """Resumes capture replay or sleeps for duration."""
    if is_realtime():
        time.sleep(t_length)
    else:
        cepton_sdk.capture_replay.resume_blocking(t_length)


def initialize(capture_path=None, control_flags=0, error_callback=None, port=None, **kwargs):
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
    wait(3)
    if capture_path is not None:
        cepton_sdk.capture_replay.seek(0)


def listen_image_frames(callback, callback_id=None):
    """Register image frames callback.

    Throws error if `callback_id` is currently registered.

    Returns:
        callback_id
    """
    return cepton_sdk.core._image_frames_callback.listen(callback, callback_id=None)


def unlisten_image_frames(callback_id):
    """Unregisters image frames callback.

    Throws error if `callback_id` is not currently registered.
    """
    cepton_sdk.core._image_frames_callback.unlisten(callback_id)


def _wait_on_func(func, timeout=None):
    if timeout is not None:
        t_start = get_timestamp()
    while not func():
        wait()
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
        self.points_dict = collections.defaultdict(list)
        super().__init__()

    def _reset(self):
        self.points_dict.clear()

    def reset(self):
        with self._lock:
            self._reset()

    def _on_points(self, serial_number, points):
        with self._lock:
            self.points_dict[serial_number].append(points)

    def has_points(self):
        with self._lock:
            return len(self.points_dict) > 0

    def _get_points(self):
        with self._lock:
            points_dict = dict(self.points_dict)
            self._reset()
        return points_dict

    def get_points(self, **kwargs):
        _wait_on_func(self.has_points, **kwargs)
        return self._get_points()


class _SensorFramesListener(_ListenerBase):
    def __init__(self, serial_number):
        self.serial_number = serial_number
        self.points_list = []
        super().__init__()

    def _reset(self):
        del self.points_list[:]

    def reset(self):
        with self._lock:
            self._reset()

    def _on_points(self, serial_number, points):
        with self._lock:
            if serial_number != self.serial_number:
                return
            self.points_list.append(points)

    def has_points(self):
        with self._lock:
            return len(self.points_list) > 0

    def _get_points(self):
        with self._lock:
            points_list = copy.copy(self.points_list)
        self._reset()
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
    def create(cls, sensor_serial_number):
        sensor_info = \
            cepton_sdk.sensor.get_sensor_information(sensor_serial_number)
        return cls(sensor_info)

    def update(self):
        """Update sensor information.

        Should be called often, to pull latest sensor information.
        """
        self.information = \
            cepton_sdk.sensor.get_sensor_information_by_handle(self.handle)

    def get_image_frames(self, *args, **kwargs):
        """See `cepton_sdk.get_sensor_image_frames`"""
        return get_sensor_image_frames(self.serial_number, *args, **kwargs)

    def get_image_points(self, *args, **kwargs):
        """See `cepton_sdk.get_sensor_image_points`"""
        return get_sensor_image_points(self.serial_number, *args, **kwargs)

    def get_image_points_by_n(self, *args, **kwargs):
        """See `cepton_sdk.get_sensor_image_points_by_n`"""
        return get_sensor_image_points_by_n(self.serial_number, *args, **kwargs)


def has_sensor(serial_number):
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
