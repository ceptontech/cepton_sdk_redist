#!/usr/bin/env python3

from __future__ import (absolute_import, division, generators, nested_scopes,
                        print_function, unicode_literals, with_statement)

import argparse
import ipaddress
import serial
import serial.tools.list_ports
import time
import re
import sys


def ip_str_to_hex(ip_str):
    return ipaddress.IPv4Address(ip_str).packed.hex()


def mac_str_to_hex(mac_str):
    mac_str = mac_str.replace(":", "").replace("-", "")
    if len(mac_str) != 12:
        raise ValueError("MAC must be 6 bytes")
    return "{:012x}".format(int(mac_str, 16))


class CeptonSerial(object):
    def __init__(self, port, debug=False):
        if not debug:
            options = {
                "baudrate": 9600,
                "port": port,
                "timeout": 0.1,
                "write_timeout": 0.1,
            }
            self.ser = serial.Serial(**options)
        self.debug = debug

    def send_command(self, command, value):
        message = "$SCT{}{}*\n".format(command, value)
        if self.debug:
            print(message, end="")
        else:
            self.ser.write(message.encode("utf-8"))
            time.sleep(1)

    def reset_subfield_control(self):
        self.reset_subfield_control("11111111")

    def set_subfield_control(self, ctrl_str):
        if not re.match('^[1|0]{8}$', ctrl_str):
            print("Error: Invalid subfield control format")
            sys.exit(1)
        self.send_command("LAS", ctrl_str)

    def reset_destination_ip(self):
        self.set_destination_ip("255.255.255.255")

    def set_destination_ip(self, ip_str):
        ip_hex = ip_str_to_hex(ip_str)
        self.send_command("IPD", ip_hex)

    def reset_source_ip(self, serial_number):
        base_ip = ipaddress.IPv4Address("192.168.32.32")
        ip_str = str(base_ip + serial_number)
        if self.debug:
            print(ip_str)
        self.set_source_ip(ip_str)

    def set_source_ip(self, ip_str):
        ip_hex = ip_str_to_hex(ip_str)
        self.send_command("IPS", ip_hex)

    def reset_destination_mac(self, serial_number):
        self.set_destination_mac("ff:ff:ff:ff:ff:ff", serial_number)

    def set_destination_mac(self, destination_mac_str, serial_number):
        destination_mac_hex = mac_str_to_hex(destination_mac_str)
        source_mac_hex = "181212{:06x}".format(serial_number)
        self.send_command("MAC", destination_mac_hex + source_mac_hex)

    def reset_port(self):
        self.set_port(8808)

    def set_port(self, destination_port):
        source_port_hex = "{:04x}".format(443)
        destination_port_hex = "{:04x}".format(destination_port)
        self.send_command("POT", source_port_hex + destination_port_hex)

    def set_fixed_power(self, powers):
        self.send_command("PWRF", powers)

    def set_auto_power(self):
        self.send_command("PWRA", "")

    def reset(self, serial_number):
        self.reset_destination_ip()
        self.reset_source_ip(serial_number)
        # self.reset_subfield_control()
        self.reset_destination_mac(serial_number)
        self.reset_port()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--debug", help="used for command debugging", action="store_true")
    parser.add_argument("--serial", help="Lidar unit serial number", type=int)
    parser.add_argument("--list_ports", help="Show a list of all COM ports available",
                        action='store_true', default=False)
    parser.add_argument("--port", help="Serial port name")
    parser.add_argument("--reset", action="store_true")
    parser.add_argument("--reset_destination_ip", action="store_true")
    parser.add_argument(
        "--destination_ip", help="Destination IP address (default: 255.255.255.255)")
    parser.add_argument("--reset_destination_mac", action="store_true")
    parser.add_argument(
        "--destination_mac", help="Destination MAC (default: ff:ff:ff:ff:ff:ff)")
    parser.add_argument("--reset_destination_port", action="store_true")
    parser.add_argument(
        "--destination_port", help="Destination port (default: 8808)", type=int)
    parser.add_argument("--reset_source_ip", action="store_true")
    parser.add_argument(
        "--source_ip", help="Sensor IP address (default: 192.168.*.*)")
    parser.add_argument("--reset_subfield_control", action="store_true")
    parser.add_argument(
        "--subfield_control", help="Enable/disable subfields (default: 11111111)")
    parser.add_argument("--fixed_power", help="Set power level to fixed value between 1-f (e.g. ffffffff)")
    parser.add_argument("--auto_power", action="store_true", help="Set power level to auto adjusting")
    args = parser.parse_args()

    if args.list_ports or not args.port:
        print("All ports:")
        for cp in serial.tools.list_ports.comports():
            print("  " + cp.device)
        sys.exit()

    if args.serial is None:
        print("Must specify sensor serial number")
        sys.exit(1)

    cepton_serial = CeptonSerial(args.port, debug=args.debug)

    if args.reset:
        print("Resetting...")
        cepton_serial.reset(args.serial)
    else:
        if args.reset_destination_ip:
            print("Resetting destination IP...")
            cepton_serial.reset_destination_ip()
        if args.reset_destination_mac:
            print("Resetting destination MAC...")
            cepton_serial.reset_destination_mac(args.serial)
        if args.reset_destination_port:
            print("Resetting destination port...")
            cepton_serial.reset_destination_port()
        if args.reset_source_ip:
            print("Setting source IP...")
            cepton_serial.reset_source_ip(args.serial)
        if args.reset_subfield_control:
            print("Resetting subfield controls...")
            cepton_serial.reset_subfield_control()

    if args.destination_ip is not None:
        print("Setting destination IP...")
        cepton_serial.set_destination_ip(args.destination_ip)

    if args.destination_mac is not None:
        print("Setting destination MAC...")
        cepton_serial.set_destination_mac(args.destination_mac, args.serial)

    if args.destination_port is not None:
        print("Setting destination port...")
        cepton_serial.set_port(args.destination_port)

    if args.source_ip is not None:
        print("Setting source IP...")
        cepton_serial.set_source_ip(args.source_ip, args.serial)

    if args.subfield_control is not None:
        print("Setting subfield controls...")
        cepton_serial.set_subfield_control(args.subfield_control)

    if args.fixed_power is not None:
        print("Setting fixed power...")
        cepton_serial.set_fixed_power(args.fixed_power)

    if args.auto_power:
        print("Setting auto power...")
        cepton_serial.set_auto_power()

    print("Done.")


if __name__ == "__main__":
    main()
