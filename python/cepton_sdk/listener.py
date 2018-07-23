import collections
import threading

import cepton_sdk.c
import cepton_sdk.point
import cepton_sdk.sensor
from cepton_sdk.common.function import *


class _Frame(object):
    def __init__(self, frame_id, sensor_serial_number, points):
        self.id = frame_id
        self.sensor_serial_number = sensor_serial_number
        self.points = points


class _FramesListenerBase(object):
    """Global points listener class.

    Stores history of most recently received frames.
    Use PointsListener and SensorPointsListener for higher level api.
    """

    _instances = []

    def __init__(self):
        self._instances.append(self)
        self._lock = threading.Lock()
        self._reset_id = 0
        self._frame_id = 0
        self._frames = collections.deque()
        self._n_points = 0
        self._max_n_points = int(1e6)

    def initialize(self):
        raise NotImplementedError()

    def reset(self):
        with self._lock:
            self._reset_id += 1
            self._frames.clear()
            self._n_points = 0

    @property
    def reset_id(self):
        with self._lock:
            return self._reset_id

    @property
    def frame_id(self):
        with self._lock:
            return self._frame_id

    def _is_valid_frame(self, frame, sensor_serial_number=None, frame_id_lb=None):
        if sensor_serial_number is not None:
            if (frame.sensor_serial_number != sensor_serial_number):
                return False
        if frame_id_lb is not None:
            if frame.id < frame_id_lb:
                return False
        return True

    def has_frames(self, **kwargs):
        with self._lock:
            for frame in reversed(self._frames):
                if self._is_valid_frame(frame, **kwargs):
                    return True
        return False

    def get_frames(self, **kwargs):
        with self._lock:
            frames_list = \
                [frame for frame in self._frames
                 if self._is_valid_frame(frame, **kwargs)]
        return frames_list

    def _add_points(self, sensor_handle, points):
        sensor_info = \
            cepton_sdk.sensor.get_sensor_information_by_handle(sensor_handle)
        with self._lock:
            frame = \
                cepton_sdk.listener._Frame(
                    self._frame_id, sensor_info.serial_number, points)
            self._frame_id += 1
            self._frames.append(frame)
            self._n_points += len(frame.points)
            while self._n_points > self._max_n_points:
                frame_tmp = self._frames.popleft()
                self._n_points -= len(frame_tmp.points)


class _FramesListener(_FramesListenerBase):
    def initialize(self):
        self._c_on_points = \
            cepton_sdk.c.C_SensorImageDataCallback(
                lambda *args: self._on_image_points(*args[:-1]))
        cepton_sdk.c.c_listen_image_frames(self._c_on_points, None)

    def _on_image_points(self, sensor_handle, n_points, c_image_points_ptr):
        image_points = \
            cepton_sdk.point.ImagePoints.from_c(n_points, c_image_points_ptr)
        self._add_points(sensor_handle, image_points)


_frames_listener = _FramesListener()


def initialize():
    for frames_listener in _FramesListenerBase._instances:
        frames_listener.initialize()


def clear_cache():
    for frames_listener in _FramesListenerBase._instances:
        frames_listener.reset()


class _ListenerBase(object):
    def __init__(self, frames_listener=_frames_listener):
        self._frames_listener = frames_listener
        self.reset()

    def reset(self):
        self._frame_id = self._frames_listener.frame_id
        self._reset_id = self._frames_listener.reset_id

    def update(self):
        if self._reset_id != self._frames_listener.reset_id:
            self.reset()


class PointsListener(_ListenerBase):
    """Listener for getting all sensor frames."""

    def has_points(self):
        self.update()
        options = {
            "frame_id_lb": self._frame_id,
        }
        return self._frames_listener.has_frames(**options)

    def get_points(self):
        self.update()
        frames_list = self._frames_listener.get_frames(
            frame_id_lb=self._frame_id)
        if len(frames_list) == 0:
            return {}

        self._frame_id = frames_list[-1].id + 1

        points_dict = collections.defaultdict(list)
        for frame in frames_list:
            points_dict[frame.sensor_serial_number].append(frame.points)
        return points_dict


class SensorPointsListener(_ListenerBase):
    def __init__(self, sensor_serial_number, **kwargs):
        self.sensor_serial_number = sensor_serial_number
        super().__init__(**kwargs)

    def has_points(self):
        self.update()
        return self._frames_listener.has_frames(
            frame_id_lb=self._frame_id,
            sensor_serial_number=self.sensor_serial_number)

    def get_points(self):
        self.update()
        frames = self._frames_listener.get_frames(
            frame_id_lb=self._frame_id,
            sensor_serial_number=self.sensor_serial_number)
        if not frames:
            return []
        self._frame_id = frames[-1].id + 1
        points_list = [frame.points for frame in frames]
        return points_list


class _NAccumulatedPointsListenerMixin(object):
    def has_points(self, n_points):
        self.update()
        return self._accumulator.has_points(n_points)

    def get_points(self, n_points):
        self.update()
        return self._accumulator.get_points(n_points)


class _TimeAccumulatedPointsListenerMixin(object):
    @property
    def t(self):
        return self._accumulator.t

    @property
    def t_max(self):
        return self._accumulator.t_max

    def has_points_by_t(self, t):
        self.update()
        return self._accumulator.has_points_by_t(t)

    def has_points(self, t_length):
        self.update()
        return self._accumulator.has_points(t_length)

    def get_points_by_t(self, t):
        self.update()
        return self._accumulator.get_points_by_t(t)

    def get_points(self, t_length):
        self.update()
        return self._accumulator.get_points(t_length)


class _MultipleAccumulatedPointsListenerBase(_ListenerBase):
    def __init__(self, frames_listener=_frames_listener):
        self._listener = PointsListener(frames_listener=frames_listener)
        super().__init__(frames_listener=frames_listener)

    def update(self):
        super().update()
        if not self._listener.has_points():
            return
        points_dict = self._listener.get_points()
        for sensor_serial_number, points_list in points_dict.items():
            points = points_list[0].combine(points_list)
            self._accumulator.add_points(sensor_serial_number, points)

    def reset(self):
        super().reset()
        self._accumulator.reset()
        self._listener.reset()


class _SensorAccumulatedPointsListenerBase(_ListenerBase):
    def __init__(self, sensor_serial_number, frames_listener=_frames_listener):
        self._listener = \
            cepton_sdk.listener.SensorPointsListener(
                sensor_serial_number, frames_listener=frames_listener)
        super().__init__(frames_listener=frames_listener)

    def update(self):
        super().update()

        if not self._listener.has_points():
            return
        points_list = self._listener.get_points()
        if len(points_list) == 0:
            return
        points = points_list[0].combine(points_list)
        self._accumulator.add_points(points)

    def reset(self):
        super().reset()
        self._accumulator.reset()
        self._listener.reset()


class NAccumulatedSensorPointsListener(_SensorAccumulatedPointsListenerBase,
                                       _NAccumulatedPointsListenerMixin):
    def __init__(self, sensor_serial_number, frames_listener=_frames_listener):
        self._accumulator = cepton_sdk.point.NPointsAccumulator()
        super().__init__(sensor_serial_number, frames_listener=frames_listener)


class TimeAccumulatedPointsListener(_MultipleAccumulatedPointsListenerBase,
                                    _TimeAccumulatedPointsListenerMixin):
    def __init__(self, frames_listener=_frames_listener, **kwargs):
        self._accumulator = \
            cepton_sdk.point.MultipleTimePointsAccumulator(**kwargs)
        super().__init__(frames_listener=frames_listener)


class TimeAccumulatedSensorPointsListener(_SensorAccumulatedPointsListenerBase,
                                          _TimeAccumulatedPointsListenerMixin):
    def __init__(self, sensor_serial_number, frames_listener=_frames_listener, **kwargs):
        self._accumulator = cepton_sdk.point.TimePointsAccumulator(**kwargs)
        super().__init__(sensor_serial_number, frames_listener=frames_listener)
