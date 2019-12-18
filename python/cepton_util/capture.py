import datetime
import glob
import ipaddress
import os
import platform
import shutil
import subprocess
import sys

import netifaces
import serial.tools.list_ports
import yaml

from cepton_util.common import *

_all_builder = AllBuilder(__name__)


class CaptureBase:
    def __init__(self):
        self.start_time = get_timestamp()

    def __del__(self):
        self.close()

    def close(self):
        pass

    @property
    def length(self):
        return get_timestamp() - self.start_time


def get_all_camera_devices():
    if sys.platform.startswith("linux"):
        return sorted(glob.glob("/dev/video[0-9]"))
    else:
        raise NotImplementedError("OS not supported!")


def process_video(input_path, **kwargs):
    output_path = modify_path(input_path, postfix="_tmp")
    cmd_list = [
        "ffmpeg",
        "-nostdin",
        "-y",
        "-i", input_path,
        "-c:v", "libx265",
        "-preset", "veryfast",
        "-an",
        "-movflags", "faststart",
        output_path,
    ]
    execute_command(cmd_list, quiet=True)
    os.remove(input_path)
    os.rename(output_path, input_path)


class CameraCapture(CaptureBase):
    def __init__(self, video_device, output_path, video_size="1920x1080", **kwargs):
        super().__init__(**kwargs)

        self.output_path = output_path
        cmd_list = [
            "ffmpeg",
            "-loglevel", "error",
            "-nostdin",
            "-y",
            "-f", "v4l2",
            "-framerate", "30",
            "-video_size", video_size,
            "-i", str(video_device),
            "-c:v", "copy",
            "-an",
            output_path,
        ]
        options = {
            "background": True,
        }
        self._proc = execute_command(cmd_list, **options)

    def close(self):
        if self._proc is None:
            return
        try:
            self._proc.close()
        except:
            pass
        self._proc = None
        run_background(process_video, args=(self.output_path,))


def find_network_interface():
    interfaces = []
    for interface in netifaces.interfaces():
        try:
            if netifaces.ifaddresses(interface)[netifaces.AF_INET][0]["broadcast"] == "192.168.255.255":
                interfaces.append(interface)
        except:
            continue
    if len(interfaces) == 0:
        raise RuntimeError("No network interface found!")
    if len(interfaces) > 1:
        raise RuntimeError("Multiple network interfaces found!")
    return interfaces[0]


class NetworkCapture(CaptureBase):
    def __init__(self, output_path, interface=None, **kwargs):
        if interface is None:
            interface = find_network_interface()

        super().__init__(**kwargs)

        cmd_list = [
            "dumpcap",
            "-q",
            "-i", interface,
            "-w", output_path,
            "-P",
            "-B", "1024",
        ]
        options = {
            "background": True,
            "quiet": True,
        }
        self._proc = execute_command(cmd_list, **options)

    def close(self):
        if self._proc is None:
            return
        try:
            self._proc.close()
        except:
            pass
        self._proc = None


def get_all_ros_topics():
    try:
        result = subprocess.check_output(["rostopic", "list"], timeout=1)
        return sorted(result.decode("utf-8").split())
    except:
        return []


class ROSCapture(CaptureBase):
    def __init__(self, ros_topics, output_path, **kwargs):
        super().__init__(**kwargs)

        cmd_list = [
            "rosbag", "record",
            "--lz4",
            "-O", output_path,
        ]
        cmd_list.extend(ros_topics)
        options = {
            "background": True,
        }
        self._proc = execute_command(cmd_list, **options)

    def close(self):
        if self._proc is None:
            return
        try:
            self._proc.close()
        except:
            pass
        self._proc = None


def get_all_serial_ports():
    return sorted([x.device for x in serial.tools.list_ports.comports()])


class SerialCapture(CaptureBase):
    def __init__(self, port, output_path, **kwargs):
        super().__init__(**kwargs)

        cmd_list = [
            "(stty raw; cat > {}) < {}".format(output_path, port)
        ]
        options = {
            "background": True,
            "shell": True,
        }
        self._proc = execute_command(cmd_list, **options)

    def close(self):
        if self._proc is None:
            return
        try:
            self._proc.close()
        except:
            pass
        self._proc = None


def clip_pcap(input_path, output_path, start, end):
    cmd_list = [
        "capinfos",
        "-S",
        "-a",
        input_path
    ]
    result = subprocess.check_output(cmd_list).decode("utf-8")
    info = yaml.load(result, Loader=yaml.FullLoader)
    t_0 = float(info["First packet time"])
    time_template = "%Y-%m-%d %H:%M:%S"

    cmd_list = [
        "editcap",
        "-F", "pcap",
    ]
    if start is not None:
        start += t_0
        start_str = \
            datetime.datetime.fromtimestamp(start).strftime(time_template)
        cmd_list += ["-A", start_str]
    if end is not None:
        end += t_0
        end_str = datetime.datetime.fromtimestamp(end).strftime(time_template)
        cmd_list += ["-B", end_str]
    cmd_list += [input_path, output_path]
    execute_command(cmd_list, quiet=True)


def clip_video(input_path, output_path, start=None, end=None):
    cmd_list = [
        "ffmpeg",
        "-nostdin",
        "-y",
        "-i", input_path,
        "-c:v", "libx265",
        "-preset", "veryfast",
        "-movflags", "faststart",
    ]
    if start is not None:
        cmd_list += ["-ss", str(start)]
    if end is not None:
        cmd_list += ["-to", str(end)]
    cmd_list += [output_path]
    execute_command(cmd_list, quiet=True)


def clip_ros(input_path, output_path, start=None, end=None):
    cmd_list = [
        "rosbag", "info",
        "--yaml",
        "--key=start",
        input_path,
    ]
    t_0 = float(subprocess.check_output(cmd_list))

    filter_command = []
    if start is not None:
        start += t_0
        filter_command += ["(t.secs >= {})".format(start)]
    if end is not None:
        end += t_0
        filter_command += ["(t.secs <= {})".format(end)]
    cmd_list = [
        "rosbag", "filter",
        input_path,
        output_path,
        " and ".join(filter_command),
    ]
    execute_command(cmd_list, quiet=True)


def clip_capture(input_dir, output_dir, start=None, end=None):
    input_dir = InputDataDirectory(input_dir)
    output_dir = OutputDataDirectory(output_dir)
    shutil.rmtree(output_dir.path)
    output_dir.copy_settings(input_dir.path)

    options = {
        "start": start,
        "end": end,
    }
    if input_dir.network_path is not None:
        clip_pcap(input_dir.network_path, output_dir.network_path, **options)
    if input_dir.ros_path is not None:
        try:
            clip_ros(input_dir.ros_path, output_dir.ros_path, **options)
        except FileNotFoundError:
            pass
    for input_path in input_dir.camera_paths():
        output_path = os.path.join(
            output_dir.path, os.path.basename(input_path))
        clip_video(input_path, output_path, **options)


__all__ = _all_builder.get()
