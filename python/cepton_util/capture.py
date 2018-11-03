import datetime
import ipaddress

import cepton_util.common
import netifaces


def find_interface(network="192.168.0.0/16"):
    network = ipaddress.ip_network(network)
    for interface in netifaces.interfaces():
        try:
            interface_address = \
                netifaces.ifaddresses(interface)[netifaces.AF_INET][0]["addr"]
        except:
            continue
        interface_address = ipaddress.ip_address(interface_address)
        if interface_address in network:
            return interface
    return None


class CaptureWriter:
    def __init__(self, output_path, interface=None):
        if interface is None:
            interface = find_interface()
        if interface is None:
            raise RuntimeError("No network interface found!")

        self.start_time = cepton_util.common.get_timestamp()

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

    @property
    def length(self):
        return cepton_util.common.get_timestamp() - self.start_time

    def close(self):
        try:
            self._proc.close()
        except:
            pass
        self._proc = None
