import argparse
import datetime
import glob
import os
import os.path
import pkgutil
import re
import shutil
import signal
import subprocess
import sys
import threading
import time

import numpy
import serial
import serial.tools.list_ports


class AllBuilder:
    def __init__(self, module_name, init=None):
        if init is None:
            init = []

        self._module_name = module_name
        self._init = init
        self._exclude = set(dir(sys.modules[self._module_name]))

    def get(self):
        result = \
            set(dir(sys.modules[self._module_name])) - self._exclude
        result = [x for x in result if not x.startswith("_")]
        result.extend(self._init)
        return result


_all_builder = AllBuilder(__name__, init=["AllBuilder"])


def get_package_path(name):
    return os.path.dirname(pkgutil.get_loader(name).path)


def optional_function(func):
    """Only apply function if input value is not None.
    Function must be unary.
    """
    def wrapper(x, *args, **kwargs):
        if x is None:
            return None
        return func(x, *args, **kwargs)
    return wrapper


def array_function(func):
    """Convert input to numpy and output to original type.
    Function must be unary
    """
    def wrapper(value, *args, **kwargs):
        result = func(numpy.array(value), *args, **kwargs)
        if numpy.isscalar(value):
            result = numpy.asscalar(result)
        return result
    return wrapper


@array_function
def from_usec(timestamps_usec):
    """Convert microseconds to seconds."""
    return 1e-6 * timestamps_usec.astype(float)


@array_function
def to_usec(timestamps):
    """Convert seconds to microseconds."""
    return (1e6 * timestamps).astype(numpy.int64)


def get_timestamp():
    return datetime.datetime.utcnow().timestamp()


def get_timestamp_usec():
    return to_usec(get_timestamp())


def get_day_str():
    return datetime.datetime.now().strftime("%Y-%m-%d")


def get_sec_str():
    return datetime.datetime.now().strftime("%H%M%S")


def get_timestamp_str():
    return datetime.datetime.now().strftime("{}_{}".format(get_day_str(), get_sec_str()))


@optional_function
def parse_time_hms(s):
    """Convert HH:MM:SS to seconds."""
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
    if isinstance(s, int):
        return enum_type(s)
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


class BackgroundProcess:
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


class BackgroundThread:
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


def create_directory(path, overwrite=False):
    if overwrite:
        delete_directory(path)
    if not os.path.exists(path):
        os.makedirs(path)


def delete_directory(path):
    if not os.path.isdir(path):
        return
    os.rmdir(path)


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


def backup_file(path):
    backup_path = modify_path(path, postfix=".orig")
    shutil.copy2(path, backup_path)
    return backup_path


def add_simple_io_path_arguments(parser):
    parser.add_argument("input")
    parser.add_argument("-o", "--output")


def get_simple_io_paths(args, output_name, inplace=False):
    input_path = fix_path(args.input)
    if args.output is not None:
        output_path = fix_path(args.output)
    else:
        dir_path = os.path.dirname(input_path)
        output_path = os.path.join(dir_path, output_name)
    if inplace:
        if input_path != output_path:
            shutil.copy2(input_path, output_path)
        return output_path
    else:
        return input_path, output_path


def add_io_path_arguments(parser, prefix="", postfix=""):
    add_simple_io_path_arguments(parser)
    parser.add_argument("--inplace", action="store_true")
    parser.add_argument("--prefix", default=prefix)
    parser.add_argument("--postfix", default=postfix)


def get_io_paths(args, inplace=False, output_ext=None, output_name=None):
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
    if inplace:
        if input_path != output_path:
            shutil.copy2(input_path, output_path)
        return output_path
    else:
        if input_path == output_path:
            input_path = backup_file(input_path)
        return input_path, output_path

# ------------------------------------------------------------------------------
# Arguments
# ------------------------------------------------------------------------------


class ArgumentParserMixin:
    """Mixin for class that has command line arguments."""
    @classmethod
    def add_arguments(cls, parser):
        return parser

    @classmethod
    def parse_arguments(cls, args):
        return {}

    @classmethod
    def from_arguments(cls, parser_args, *args, **kwargs):
        options = cls.parse_arguments(parser_args)
        options.update(kwargs)
        return cls(*args, **options)


def process_options(options):
    return {key: value for key, value in options.items() if value is not None}


class _OptionsMixinMeta(type):
    def __call__(cls, *args, **kwargs):
        self = super().__call__(*args, **kwargs)
        self.set_options()
        return self


class OptionsMixin(metaclass=_OptionsMixinMeta):
    """Mixin for class that has options (usually from json)."""

    def get_options(self):
        return {}

    def set_options(self, **kwargs):
        pass

# ------------------------------------------------------------------------------
# Capture
# ------------------------------------------------------------------------------


class InputDataDirectory:
    def __init__(self, path=None):
        self.path = path

    def _find_file(self, name):
        if self.path is None:
            return None
        return find_file_by_name(name, path=self.path, depth=1)

    @property
    def pcap_path(self):
        return self._find_file("lidar.pcap")

    @property
    def viewer_config_path(self):
        return self._find_file("cepton_viewer_config.json")

    @property
    def transforms_path(self):
        return self._find_file("cepton_transforms.json")

    @property
    def clips_path(self):
        return self._find_file("cepton_clips.json")


def copy_settings(src, dst):
    if src == dst:
        return
    patterns = ["*.json"]
    for pattern in patterns:
        for path in glob.glob(os.path.join(src, pattern)):
            name = os.path.basename(path)
            shutil.copy2(path, os.path.join(dst, name))


class OutputDataDirectory(ArgumentParserMixin):
    def __init__(self, path=None, duration=None, name="", postfix="", root_dir="~/Captures"):
        self.duration = duration

        if path is None:
            self.name = os.path.join(
                get_day_str(), name, get_sec_str() + postfix)
            self.path = fix_path(os.path.join(root_dir, self.name))
        else:
            self.name = os.path.basename(path)
            self.path = path
        create_directory(self.path)

    @classmethod
    def add_arguments(cls, parser):
        parser.add_argument("--duration")
        parser.add_argument("--name", default="")

    @classmethod
    def parse_arguments(cls, args):
        options = {
            "duration": parse_time_hms(args.duration),
            "name": args.name,
        }
        return process_options(options)

    def copy_settings(self, input_path=None):
        if input_path is None:
            input_path = os.getcwd()
        copy_settings(input_path, self.path)

    @property
    def serial_numbers(self):
        return parse_list(get_environment("CEPTON_SENSORS", ""), dtype=int)

    def _get_path(self, name):
        return os.path.join(self.path, name)

    @property
    def pcap_path(self):
        return self._get_path("lidar.pcap")


__all__ = _all_builder.get()
