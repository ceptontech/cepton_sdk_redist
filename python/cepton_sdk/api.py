import collections
import threading
import time

import cepton_sdk.c
import cepton_sdk.capture_replay
import cepton_sdk.core
import cepton_sdk.sensor
from cepton_sdk.common import *

_all_builder = AllBuilder(__name__)


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
    return time.time()


def get_time():
    """Returns capture replay time or live time."""
    if is_live():
        return get_timestamp()
    else:
        return cepton_sdk.capture_replay.get_time()


def _wait(duration):
    if is_realtime():
        time.sleep(duration)
    else:
        cepton_sdk.capture_replay.resume_blocking(duration)


def wait(duration=-1):
    """Resumes capture replay or sleeps for duration.

    If `duration` is `0`, then waits forever.
    """
    if duration < 0:
        while True:
            _wait(0.1)
            if is_end():
                break
    else:
        _wait(duration)


def initialize(capture_path=None, capture_seek=0, control_flags=0,
               enable_wait=False, error_callback=None, port=None, **kwargs):
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
    cepton_sdk.core._frames_callback.initialize()
    cepton_sdk.core._serial_lines_callback.initialize()

    if capture_path is None:
        if enable_wait:
            wait(3)
    else:
        open_replay(capture_path, capture_seek=capture_seek,
                    enable_wait=enable_wait)


def deinitialize():
    cepton_sdk.core._frames_callback.deinitialize()
    cepton_sdk.core._manager.deinitialize()


def open_replay(capture_path, capture_seek=0, enable_loop=False, enable_wait=False):
    if cepton_sdk.capture_replay.is_open():
        cepton_sdk.capture_replay.close()
    cepton_sdk.capture_replay.open(capture_path)
    cepton_sdk.capture_replay.set_enable_loop(enable_loop)
    if enable_wait:
        cepton_sdk.capture_replay.resume_blocking(3)
    cepton_sdk.capture_replay.seek(capture_seek)


close_replay = cepton_sdk.capture_replay.close


def listen_frames(callback):
    """Register frames callback.

    Throws error if `callback_id` is currently registered.

    Returns:
        callback_id
    """
    return cepton_sdk.core._frames_callback.listen(callback)


def unlisten_frames(callback_id):
    """Unregisters frames callback.

    Throws error if `callback_id` is not currently registered.
    """
    cepton_sdk.core._frames_callback.unlisten(callback_id)


def _wait_on_func(func, timeout=None):
    if timeout is not None:
        t_start = get_timestamp()
    while not func():
        wait(0.1)
        if timeout is not None:
            if (get_timestamp() - t_start) > timeout:
                raise RuntimeError("Timed out!")


class _ListenerBase:
    def __init__(self):
        self._lock = threading.Lock()
        self._callback_id = self._callback().listen(self._on_callback)

    def __del__(self):
        try:
            self._callback().unlisten(self._callback_id)
        except:
            pass

    def _on_callback(self):
        raise NotImplementedError()


class FramesListener(_ListenerBase):
    """Listener for getting all sensor frames."""

    def __init__(self):
        self._i_frame_dict = collections.defaultdict(lambda: 0)
        self._points_dict = collections.defaultdict(list)
        super().__init__()

    def _callback(self):
        return cepton_sdk.core._frames_callback

    def _on_callback(self, sensor_info, points):
        with self._lock:
            self._i_frame_dict[sensor_info.serial_number] += 1
            if self._i_frame_dict[sensor_info.serial_number] > 2:
                self._points_dict[sensor_info.serial_number].append(points)

    def reset(self):
        with self._lock:
            self._i_frame_dict = collections.defaultdict(lambda: 0)
            self._points_dict = collections.defaultdict(list)

    def has_points(self):
        with self._lock:
            return len(self._points_dict) > 0

    def _get_points(self):
        with self._lock:
            points_dict = self._points_dict
            self._points_dict = collections.defaultdict(list)
        return points_dict

    def get_points(self, **kwargs):
        _wait_on_func(self.has_points, **kwargs)
        return self._get_points()


class SensorFramesListener(_ListenerBase):
    def __init__(self, serial_number):
        self._serial_number = serial_number
        self._i_frame = 0
        self._points_list = []
        super().__init__()

    def _callback(self):
        return cepton_sdk.core._frames_callback

    def _on_callback(self, sensor_info, points):
        with self._lock:
            if sensor_info.serial_number != self._serial_number:
                return
            self._i_frame += 1
            if self._i_frame > 2:
                self._points_list.append(points)

    def reset(self):
        with self._lock:
            self._i_frame = 0
            self._points_list = []

    def has_points(self):
        with self._lock:
            return len(self._points_list) > 0

    def _get_points(self):
        with self._lock:
            points_list = self._points_list
            self._points_list = []
        return points_list

    def get_points(self, **kwargs):
        _wait_on_func(self.has_points, **kwargs)
        return self._get_points()


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


def listen_serial_lines(callback):
    return cepton_sdk.core._serial_lines_callback.listen(callback)


def unlisten_serial_lines(callback_id):
    cepton_sdk.core._serial_lines_callback.unlisten(callback_id)


class SerialLinesListener(_ListenerBase):
    def __init__(self):
        self._lines_dict = collections.defaultdict(list)
        super().__init__()

    def _callback(self):
        return cepton_sdk.core._serial_lines_callback

    def _on_callback(self, sensor_info, line):
        with self._lock:
            self._lines_dict[sensor_info.serial_number].append(line)

    def get_lines(self):
        with self._lock:
            lines_dict = self._lines_dict
            self._lines_dict = collections.defaultdict(list)
        return lines_dict


__all__ = _all_builder.get()
