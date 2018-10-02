import ipaddress

import netifaces

import cepton_util.common


def find_interface(addr="192.168.0.0/16"):
    for interface in netifaces.interfaces():
        try:
            addr = netifaces.ifaddresses(
                interface)[netifaces.AF_INET][0]["addr"]
        except:
            continue
        if ipaddress.ip_address(addr) in ipaddress.ip_network(addr):
            return interface
    return None


class CaptureWriter:
    def __init__(self, output_path, interface=None):
        if interface is None:
            interface = find_interface()

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
        self._proc = cepton_util.common.execute_command(cmd_list, **options)

    def __del__(self):
        self.close()

    def close(self):
        self._proc.close()
        self._proc = None
