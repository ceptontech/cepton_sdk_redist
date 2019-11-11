import datetime
import glob
import ipaddress
import platform
import shutil
import subprocess
import sys

import netifaces
import serial.tools.list_ports

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


class CameraCapture(CaptureBase):
    def __init__(self, video_device, output_path, video_size="1280x720", **kwargs):
        if shutil.which("dumpcap") is None:
            raise OSError("Cannot find ffmpeg!")
        if not sys.platform.startswith("linux"):
            raise NotImplementedError("OS not supported!")

        super().__init__(**kwargs)

        cmd_list = [
            "ffmpeg",
            "-loglevel", "error",  # Quiet
            "-nostdin",  # Disable capturing keyboard
            "-y",  # Overwrite
            "-f", "v4l2",
            "-video_size", video_size,  # Video size
            "-input_format", "mjpeg",  # Video codec
            "-ts", "mono2abs",
            "-i", str(video_device),
            "-c:v", "copy",  # Copy video codec
            "-an",  # No audio
            "-copyts",  # Copy timestamps
            output_path,
        ]
        options = {
            "background": True,
        }
        self._proc = execute_command(cmd_list, **options)

    def close(self):
        try:
            self._proc.close()
        except:
            pass
        self._proc = None


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
        if shutil.which("dumpcap") is None:
            raise RuntimeError("Cannot find dumpcap!")

        if interface is None:
            interface = find_network_interface()

        super().__init__(**kwargs)

        cmd_list = [
            "dumpcap",
            "-q",  # Quiet
            "-i", interface,  # Interface
            "-w", output_path,  # Output path
            "-P",  # PCAP
            "-B", "1024",  # Buffer size
        ]
        options = {
            "background": True,
        }
        self._proc = execute_command(cmd_list, **options)

    def close(self):
        try:
            self._proc.close()
        except:
            pass
        self._proc = None


def get_all_ros_topics():
    try:
        return sorted(subprocess.check_output(["rostopic", "list"]).decode("utf-8").split())
    except:
        return []


class ROSCapture(CaptureBase):
    def __init__(self, ros_topics, output_path, **kwargs):
        if shutil.which("rosbag") is None:
            raise OSError("Cannot find rosbag!")

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
        try:
            self._proc.close()
        except:
            pass
        self._proc = None


def get_all_serial_ports():
    return sorted([x.device for x in serial.tools.list_ports.comports()])


class SerialCapture(CaptureBase):
    def __init__(self, port, output_path, **kwargs):
        if not platform.startswith("linux"):
            raise OSError("Unsupported platform!")

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
        try:
            self._proc.close()
        except:
            pass
        self._proc = None


__all__ = _all_builder.get()
