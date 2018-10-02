import argparse
import datetime
import os
import os.path
import shutil
import signal
import subprocess
import sys
import threading
import time

import numpy
import serial
import serial.tools.list_ports

__all__ = [
    "add_execute_command_arguments",
    "add_io_path_arguments",
    "add_simple_io_path_arguments",
    "ArgumentParserMixin",
    "create_directory",
    "execute_command",
    "find_file_by_extension",
    "find_file_by_name",
    "find_file",
    "fix_path",
    "get_day_str",
    "get_environment",
    "get_io_paths",
    "get_sec_str",
    "get_simple_io_paths",
    "get_timestamp_str",
    "get_timestamp",
    "has_environment",
    "kill_background",
    "modify_path",
    "parse_execute_command_arguments",
    "parse_list",
    "parse_time_hms",
    "process_options",
    "remove_extension",
    "run_background",
    "set_extension",
    "wait_for_input",
    "wait_on_background",
]


def optional_function(func):
    def wrapper(x, *args, **kwargs):
        if x is None:
            return None
        return func(x, *args, **kwargs)
    return wrapper


def get_timestamp():
    return datetime.datetime.utcnow().timestamp()


def get_day_str():
    return datetime.datetime.now().strftime("%y%m%d")


def get_sec_str():
    return datetime.datetime.now().strftime("%H%M%S")


def get_timestamp_str():
    return datetime.datetime.now().strftime("{}_{}".format(get_day_str(), get_sec_str()))


@optional_function
def parse_time_hms(s):
    """Convert HH:MM:SS to seconds"""
    parts = [float(x) for x in s.split(":")]
    sec = 0
    for i, part in enumerate(reversed(parts)):
        sec += part * 60**i
    return sec


@optional_function
def parse_list(s, **kwargs):
    options = {
        "sep": ",",
    }
    options.update(kwargs)
    return numpy.fromstring(s, **options)


@optional_function
def parse_enum(s, enum_type):
    return enum_type[s.upper()]


STRING_BOOL_LOOKUP = {
    "true": True,
    "false": False,
}


def has_environment(key):
    return key in os.environ


def get_environment(key, default=None):
    try:
        s = os.environ[key]
    except:
        if default is not None:
            return default
        raise
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
        self.close()

    def close(self):
        try:
            if os.name == "nt":
                os.kill(self.proc.pid, signal.CTRL_C_EVENT)
            else:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
        except:
            pass


class BackgroundThread(object):
    def __init__(self, *args, **kwargs):
        self.shutdown_event = threading.Event()
        kwargs["kwargs"]["shutdown_event"] = self.shutdown_event
        self.thread = threading.Thread(*args, **kwargs)

    def __del__(self):
        self.close()

    def close(self):
        try:
            self.shutdown_event.set()
        except:
            pass
        try:
            self.thread.join(1.0)
        except:
            pass


def wait_for_input():
    time.sleep(1)
    print("Press ENTER to exit...")
    input()
    kill_background()


def wait_on_background():
    for proc in __local["procs"]:
        proc.proc.wait()
    del __local["procs"][:]
    for thread in __local["threads"]:
        thread.thread.join()
    del __local["threads"][:]


def kill_background():
    del __local["procs"][:]
    del __local["threads"][:]


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
        return proc
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
def find_file(func, path=None, depth=None):
    if path is None:
        path = os.getcwd()
    level = 0
    while os.path.isdir(path):
        for name in os.listdir(path):
            if func(name):
                return os.path.join(path, name)

        old_path = path
        path = os.path.dirname(path)
        if path == old_path:
            break

        level += 1
        if (depth is not None) and (level >= depth):
            break
    return None


def find_file_by_name(name, **kwargs):
    return find_file(lambda x: x == name, **kwargs)


def find_file_by_extension(ext, **kwargs):
    return find_file(lambda x: x.endswith(ext), **kwargs)


def create_directory(path):
    if not os.path.exists(path):
        os.makedirs(path)


@optional_function
def fix_path(path):
    path = os.path.expanduser(path)
    path = os.path.expandvars(path)
    path = os.path.realpath(path)
    path = os.path.abspath(path)
    return path


@optional_function
def remove_extension(path):
    return os.path.splitext(path)[0]


@optional_function
def set_extension(path, new_ext):
    return os.path.splitext(path)[0] + new_ext


@optional_function
def modify_path(path, new_ext=None, prefix="", postfix=""):
    head, basename = os.path.split(path)
    name, ext = os.path.splitext(path)
    if new_ext is not None:
        ext = new_ext
    basename = "{}{}{}{}".format(prefix, name, postfix, ext)
    path = os.path.join(head, basename)
    return path


def add_simple_io_path_arguments(parser):
    parser.add_argument("input")
    parser.add_argument("--dry_run", action="store_true")
    parser.add_argument("-o", "--output")


def get_simple_io_paths(args, output_name):
    input_path = fix_path(args.input)
    if args.output is not None:
        output_path = fix_path(args.output)
    else:
        dir_path = os.path.dirname(input_path)
        output_path = os.path.join(dir_path, output_name)
    return input_path, output_path


def add_io_path_arguments(parser, prefix="", postfix=""):
    add_simple_io_path_arguments(parser)
    parser.add_argument("--inplace", action="store_true")
    parser.add_argument("--prefix", default=prefix)
    parser.add_argument("--postfix", default=postfix)


def get_io_paths(args, output_ext=None, output_name=None):
    input_path = fix_path(args.input)
    if args.output is not None:
        output_path = fix_path(args.output)
    elif args.inplace:
        output_path = input_path
    else:
        if output_name is None:
            options = {
                "postfix": args.postfix,
                "prefix": args.prefix,
            }
            output_path = modify_path(input_path, **options)
        else:
            dir_path = os.path.dirname(input_path)
            output_path = os.path.join(dir_path, output_name)
    if output_ext is not None:
        output_path = modify_path(output_path, new_ext=output_ext)
    if (not args.dry_run) and (input_path == output_path):
        input_path = modify_path(input_path, postfix="_old")
        shutil.move(output_path, input_path)
    return input_path, output_path

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


def process_options(options):
    return {key: value for key, value in options.items() if value is not None}
