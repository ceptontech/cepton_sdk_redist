import argparse
import datetime
import os
import os.path
import signal
import subprocess
import sys
import threading
import time

import serial
import serial.tools.list_ports

__all__ = [
    "add_execute_command_arguments",
    "add_output_path_arguments",
    "ArgumentParserMixin",
    "create_directory",
    "execute_command",
    "fix_path",
    "get_environment",
    "get_output_path",
    "get_timestamp_str",
    "get_timestamp",
    "modify_path",
    "parse_execute_command_arguments",
    "parse_time_hms",
    "remove_extension",
    "run_background",
    "set_extension",
    "wait_on_background",
]


def get_timestamp():
    return datetime.datetime.utcnow().timestamp()


def get_timestamp_str():
    return datetime.datetime.now().strftime("%y%m%d_%H%M%S")


def parse_time_hms(s):
    """Convert HH:MM:SS to seconds"""
    parts = [float(x) for x in s.split(":")]
    sec = 0
    for i, part in enumerate(reversed(parts)):
        sec += part * 60**i
    return sec


STRING_BOOL_LOOKUP = {
    "true": True,
    "false": False,
}


def get_environment(key):
    s = os.environ[key]
    if s.lower() in STRING_BOOL_LOOKUP:
        return STRING_BOOL_LOOKUP[s.lower()]
    return s


# ------------------------------------------------------------------------------
# Execute process
# ------------------------------------------------------------------------------


__local = {}
__local["procs"] = []
__local["threads"] = []


class BackgroundProcess(object):
    def __init__(self, *args, **kwargs):
        self.proc = subprocess.Popen(*args, **kwargs)

    def __del__(self):
        self.proc.terminate()
        for i in range(10):
            if self.proc.poll() is not None:
                return
            time.sleep(0.1)
        self.proc.kill()


class BackgroundThread(object):
    def __init__(self, *args, **kwargs):
        self.shutdown_event = threading.Event()
        kwargs["kwargs"]["shutdown_event"] = self.shutdown_event
        self.thread = threading.Thread(*args, **kwargs)

    def __del__(self):
        self.shutdown_event.set()
        self.thread.join(1.0)


def wait_on_background():
    for proc in __local["procs"]:
        proc.proc.wait()
    __local["procs"] = []
    for thread in __local["threads"]:
        thread.thread.join()
    __local["threads"] = []


def run_background(func, args=(), kwargs={}):
    thread = BackgroundThread(target=func, args=args, kwargs=kwargs)
    thread.thread.start()
    __local["threads"].append(thread)


def execute_command(cmd_list, background=False, quiet=False, **kwargs):
    options = {
        # "check": True,
    }
    if quiet:
        options.update({
            "stdout": subprocess.DEVNULL,
            "stderr": subprocess.DEVNULL,
        })
    options.update(kwargs)
    if background:
        proc = BackgroundProcess(cmd_list, **options)
        __local["procs"].append(proc)
        return proc.proc
    else:
        subprocess.run(cmd_list, **options)


def add_execute_command_arguments(parser):
    parser.add_argument("-q", "--quiet", action="store_true")


def parse_execute_command_arguments(args):
    return {
        "quiet": args.quiet,
    }


# ------------------------------------------------------------------------------
# Path
# ------------------------------------------------------------------------------
def create_directory(path):
    if not os.path.exists(path):
        os.makedirs(path)


def fix_path(path):
    path = os.path.expanduser(path)
    path = os.path.expandvars(path)
    path = os.path.realpath(path)
    path = os.path.abspath(path)
    return path


def remove_extension(path):
    return os.path.splitext(path)[0]


def set_extension(path, new_ext):
    return os.path.splitext(path)[0] + new_ext


def modify_path(path, new_ext=None, prefix="", postfix=""):
    head, basename = os.path.split(path)
    name, ext = os.path.splitext(path)
    if new_ext is not None:
        ext = new_ext
    basename = "{}{}{}{}".format(prefix, name, postfix, ext)
    path = os.path.join(head, basename)
    return path


def add_output_path_arguments(parser, prefix="", postfix="_processed"):
    parser.add_argument("-o", "--output")
    parser.add_argument("--prefix", default=prefix)
    parser.add_argument("--postfix", default=postfix)


def get_output_path(input_path, args):
    if args.output is None:
        options = {
            "prefix": args.prefix,
            "postfix": args.postfix,
        }
        return modify_path(input_path, **options)
    else:
        return fix_path(args.output)

# ------------------------------------------------------------------------------
# Arguments
# ------------------------------------------------------------------------------


class ArgumentParserMixin(object):
    @classmethod
    def add_arguments(cls, parser):
        raise NotImplementedError()

    @classmethod
    def parse_arguments(cls, args):
        raise NotImplementedError()

    @classmethod
    def from_arguments(cls, parser_args, *args, **kwargs):
        options = cls.parse_arguments(parser_args)
        options.update(kwargs)
        return cls(*args, **options)
