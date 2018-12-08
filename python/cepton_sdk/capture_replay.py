import ctypes
import os.path

import cepton_sdk.c
from cepton_sdk.common import *

_all_builder = AllBuilder(__name__)


def is_open():
    return bool(cepton_sdk.c.c_capture_replay_is_open())


def open(capture_path):
    capture_path = capture_path.encode("utf-8")
    cepton_sdk.c.c_capture_replay_open(capture_path)


def close():
    try:
        cepton_sdk.c.c_capture_replay_close()
    except IOError as e:
        warnings.warn(e)


def get_filename():
    return cepton_sdk.c.c_capture_replay_get_filename().decode("utf-8")


def get_start_time():
    return 1e-6 * float(cepton_sdk.c.c_capture_replay_get_start_time())


def get_position():
    return float(cepton_sdk.c.c_capture_replay_get_position())


def get_time():
    return get_start_time() + get_position()


def get_length():
    return float(cepton_sdk.c.c_capture_replay_get_length())


def is_end():
    return bool(cepton_sdk.c.c_capture_replay_is_end())


def seek(t):
    cepton_sdk.c.c_capture_replay_seek(t)


def seek_relative(t):
    t += get_position()
    seek(t)


def get_enable_loop():
    return bool(cepton_sdk.c.c_capture_replay_get_enable_loop())


def set_enable_loop(enable_loop):
    cepton_sdk.c.c_capture_replay_set_enable_loop(enable_loop)


def get_speed():
    return float(cepton_sdk.c.c_capture_replay_get_speed())


def set_speed(speed):
    cepton_sdk.c.c_capture_replay_set_speed(speed)


def resume_blocking(duration=None):
    if duration is None:
        cepton_sdk.c.c_capture_replay_resume_blocking_once()
    else:
        cepton_sdk.c.c_capture_replay_resume_blocking(duration)


def is_running():
    return bool(cepton_sdk.c.c_capture_replay_is_running())


def resume():
    cepton_sdk.c.c_capture_replay_resume()


def pause():
    cepton_sdk.c.c_capture_replay_pause()


__all__ = _all_builder.get()
