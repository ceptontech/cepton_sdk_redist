import datetime
import time

import cepton_sdk.c
import cepton_sdk.capture_replay
import cepton_sdk.core
import cepton_sdk.listener
import cepton_sdk.sensor
from cepton_sdk.common import *

__all__ = [
    "clear_cache",
    "get_image_frames",
    "get_image_points",
    "get_sensor_image_frames",
    "get_sensor_image_points_by_n",
    "get_sensor_image_points",
    "get_sensors",
    "get_time",
    "initialize",
    "get_timestamp",
    "is_end",
    "is_live",
    "Sensor",
    "wait",
]


def is_live():
    """Returns true if capture replay is not open, false otherwise."""
    return not cepton_sdk.capture_replay.is_open()


def is_end():
    """Returns true if capture replay is open and at end, false otherwise."""
    if is_live():
        return False  # Live sensors never end
    if cepton_sdk.capture_replay.get_enable_loop():
        return False  # Looping captures never end
    return cepton_sdk.capture_replay.is_end()


def get_timestamp():
    """Returns unix timestamp"""
    return datetime.datetime.utcnow().timestamp()


def get_time():
    """Returns capture replay time or live time"""
    if is_live():
        return get_timestamp()
    else:
        return cepton_sdk.capture_replay.get_time()


def wait(t_length=0.1):
    """Resumes capture replay or sleeps for duration."""
    if cepton_sdk.capture_replay.is_open():
        cepton_sdk.capture_replay.resume_blocking(t_length)
    else:
        time.sleep(t_length)


def _wait_on_func(func, timeout=None):
    if timeout is not None:
        t_start = time.time()
    while not func():
        wait()
        if timeout is not None:
            if (time.time() - t_start) > timeout:
                raise RuntimeError("timed out")


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
    cepton_sdk.listener.initialize()

    if capture_path is None:
        time.sleep(1)
    else:
        cepton_sdk.capture_replay.open(capture_path)
        cepton_sdk.capture_replay.resume_blocking(1)
        cepton_sdk.capture_replay.seek(0)


def clear_cache():
    cepton_sdk.c.c_clear_cache()
    cepton_sdk.listener.clear_cache()


def _get_points_wait(wait_func, return_partial=False, timeout=None):
    if not cepton_sdk.core.is_initialized():
        raise cepton_sdk.c.C_Error(cepton_sdk.c.C_Error.CEPTON_ERROR_NOT_INITIALIZED)

    if wait_func():
        return
    if is_end():
        error_code = cepton_sdk.c.C_ErrorCode.CEPTON_ERROR_EOF
        raise cepton_sdk.c.C_Error.create(error_code)
    try:
        _wait_on_func(wait_func, timeout=timeout)
    except cepton_sdk.c.C_Error as e:
        if return_partial:
            if e.error_code == cepton_sdk.c.C_ErrorCode.CEPTON_ERROR_EOF:
                return
        raise


def _get_frames(listener, **kwargs):
    def wait_func(): return listener.has_points()
    _get_points_wait(wait_func, **kwargs)
    return listener.get_points()


@static_vars(listener=cepton_sdk.listener.PointsListener())
def get_image_frames(**kwargs):
    """Returns next frames of points for all sensors.

    Returns:
        Dictionary of lists of image points, indexed by serial number.
    """
    return _get_frames(get_image_frames.listener, **kwargs)


@static_vars(listeners={})
def get_sensor_image_frames(serial_number, **kwargs):
    """Returns next frames of points for specified sensor.

    Args:
        serial_number: Sensor serial number.
    Returns:
        List of image points.
    """
    listener = _get_sensor_listener(
        get_sensor_image_frames.listeners, serial_number,
        cepton_sdk.listener.SensorPointsListener)
    return _get_frames(listener, **kwargs)


def _get_points(listener, frame_length, **kwargs):
    if frame_length is None:
        if is_live():
            raise ValueError("must specify frame lenghth for live sensor")
        kwargs["return_partial"] = True

        def wait_func():
            listener.update()
            return False
    else:
        def wait_func(): return listener.has_points(frame_length)
    _get_points_wait(wait_func, **kwargs)
    if frame_length is None:
        listener.update()
        return listener.get_points_by_t(listener.t_max)
    else:
        return listener.get_points(frame_length)


@static_vars(listener=cepton_sdk.listener.TimeAccumulatedPointsListener(t_latency=0.1))
def get_image_points(frame_length, return_partial=False, timeout=None):
    """Returns next chunk of points for all sensors.

    If using capture replay and frame length is None, returns all remaining
    points in capture file.

    Args:
        frame_length: length of time chunk [sec].
    Returns:
        Dictionary of image points, indexed by serial number.
    """
    return _get_points(
        get_image_points.listener, frame_length, return_partial=return_partial,
        timeout=timeout)


def _get_sensor_listener(listeners, serial_number, cls, *args, **kwargs):
    if serial_number not in listeners:
        listeners[serial_number] = cls(serial_number, *args, **kwargs)
    return listeners[serial_number]


@static_vars(listeners={})
def get_sensor_image_points(serial_number, frame_length, return_partial=False, timeout=None):
    """Returns next chunk of points for specified sensor.

    If using capture replay and frame length is None, returns all remaining
    points in capture file.

    Args:
        serial_number: Sensor serial number.
        frame_length: Length of time chunk [sec].
    Returns:
        Image points.
    """
    listener = _get_sensor_listener(
        get_sensor_image_points.listeners, serial_number,
        cepton_sdk.listener.TimeAccumulatedSensorPointsListener)
    return _get_points(
        listener, frame_length, return_partial=return_partial, timeout=timeout)


def _get_points_by_n(listener, n_points, **kwargs):
    def wait_func(): return listener.has_points(n_points)
    _get_points_wait(wait_func, **kwargs)
    return listener.get_points(n_points)


@static_vars(listeners={})
def get_sensor_image_points_by_n(serial_number, n_points, **kwargs):
    """Returns next chunk of points for specified sensor.

    Args:
        serial_number: Sensor serial number.
        n_points: Number of points.
    Returns:
        Image points.
    """
    listener = _get_sensor_listener(
        get_sensor_image_points_by_n.listeners, serial_number,
        cepton_sdk.listener.NAccumulatedSensorPointsListener)
    return _get_points_by_n(listener, n_points, **kwargs)


class Sensor(object):
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

    @property
    def model_name(self):
        return self.information.model_name

    @property
    def firmware_version(self):
        return self.information.firmware_version

    @property
    def model(self):
        return self.information.model  # This is SensorModel enum

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

    def is_hr80t(self):
        return self.model == cepton_sdk.sensor.SensorModel.HR80T

    def is_hr80t_r2(self):
        return self.model == cepton_sdk.sensor.SensorModel.HR80T_R2

    def is_hr80w(self):
        return self.model == cepton_sdk.sensor.SensorModel.HR80W

    def is_sora(self):
        return self.model == cepton_sdk.sensor.SensorModel.SORA_200

    def is_vista860(self):
        return self.model == cepton_sdk.sensor.SensorModel.VISTA_860


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
