import datetime
import ipaddress

from cepton_util.common import *
import netifaces

_all_builder = AllBuilder(__name__)


class CaptureBase:
    def __init__(self):
        self.start_time = get_timestamp()

    def __del__(self):
        self.close()

    @property
    def length(self):
        return get_timestamp() - self.start_time


def find_interface(network="192.168.0.0/16"):
    network = ipaddress.ip_network(network)
    interfaces = []
    for interface in netifaces.interfaces():
        try:
            interface_address = \
                netifaces.ifaddresses(interface)[netifaces.AF_INET][0]["addr"]
        except:
            continue
        interface_address = ipaddress.ip_address(interface_address)
        if interface_address in network:
            interfaces.append(interface)
    if len(interfaces) == 0:
        raise RuntimeError("No network interface found!")
    if len(interfaces) > 1:
        raise RuntimeError("Multiple network interfaces found!")
    return interfaces[0]


class PCAPCapture(CaptureBase):
    def __init__(self, output_path, interface=None, **kwargs):
        if interface is None:
            interface = find_interface()

        super().__init__(**kwargs)

        cmd_list = [
            "dumpcap",
            "-q",
            "-i", interface,
            "-w", output_path,
            "-P",  # PCAP
            "-B", "1024",  # Buffer size
        ]
        options = {
            "background": True,
            "quiet": True,
        }
        self._proc = execute_command(cmd_list, **options)

    def close(self):
        try:
            self._proc.close()
        except:
            pass
        self._proc = None


class CameraCapture(CaptureBase):
    def __init__(self, video_device, output_path, video_size="1280x720", **kwargs):
        super().__init__(**kwargs)

        cmd_list = [
            "ffmpeg",
            "-nostdin",
            "-y",
            "-f", "v4l2",
            "-video_size", video_size,
            "-input_format", "mjpeg",
            "-ts", "mono2abs",
            "-i", str(video_device),
            "-c:v", "copy",
            "-an",
            "-copyts",
            output_path,
        ]
        options = {
            "background": True,
            "quiet": True,
        }
        self._proc = execute_command(cmd_list, **options)

    def close(self):
        try:
            self._proc.close()
        except:
            pass
        self._proc = None


__all__ = _all_builder.get()
