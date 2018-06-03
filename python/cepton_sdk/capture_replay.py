import ctypes
import os.path

import cepton_sdk.c
import cepton_sdk.listener

__all__ = [
    "get_length",
    "get_position",
    "get_start_time",
    "get_time",
    "is_end",
    "is_open",
    "rewind",
    "seek_relative",
    "seek",
]


def is_open():
    return bool(cepton_sdk.c.c_capture_replay_is_open())


def open(capture_path):
    capture_path = capture_path.encode("UTF-8")
    cepton_sdk.c.c_capture_replay_open(capture_path)
    cepton_sdk.listener.clear_cache()


def close():
    try:
        cepton_sdk.c.c_capture_replay_close()
    except IOError as e:
        warnings.warn(e)
    cepton_sdk.listener.clear_cache()


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


def rewind():
    cepton_sdk.c.c_capture_replay_rewind()
    cepton_sdk.listener.clear_cache()


def seek(t):
    cepton_sdk.c.c_capture_replay_seek(t)
    cepton_sdk.listener.clear_cache()


def seek_relative(t):
    t += get_position()
    seek(t)


def get_enable_loop():
    return bool(cepton_sdk.c.c_capture_replay_get_enable_loop())


def set_enable_loop(enable_loop):
    cepton_sdk.c.c_capture_replay_set_enable_loop(enable_loop)


def resume_blocking(t_length=None):
    if t_length is None:
        cepton_sdk.c.c_capture_replay_resume_blocking_once()
    else:
        cepton_sdk.c.c_capture_replay_resume_blocking(t_length)


def is_running():
    return bool(cepton_sdk.c.c_capture_replay_is_running())


def resume():
    cepton_sdk.c.c_capture_replay_resume()


def pause():
    cepton_sdk.c.c_capture_replay_pause()
