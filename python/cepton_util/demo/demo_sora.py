"""SORA scan demo with Polulu Micro Maestro."""

import re
import sys
import time

import numpy
import serial
import serial.tools.list_ports
import transforms3d.quaternions

import cepton_util.demo.maestro

# ------------------------------------------------------------------------------
# Config
# ------------------------------------------------------------------------------

"""Minimum scan angle [degrees]. Must be in range [-45, 45]."""
ANGLE_MIN = -30

"""Maximum scan angle [degrees]. Must be in range [-45, 45]."""
ANGLE_MAX = 30

"""Scan speed. Must be in range [1, 10]."""
SPEED = 3

# ------------------------------------------------------------------------------

CHANNEL = 0


def main():
    # Get port by vid, pid
    def filter_func(x): return ((x.vid == 8187) and (x.pid == 137))
    ports = serial.tools.list_ports.comports()
    ports = list(filter(filter_func, ports))
    connected = False
    for port in ports:
        controller = cepton_util.demo.maestro.AngleController(port=port.device)
        try:
            controller.getPosition(CHANNEL)
        except serial.SerialTimeoutException:
            continue
        connected = True
        break

    if not connected:
        raise IOError("Maestro not connected!")

    controller.setSpeed(CHANNEL, SPEED)
    while True:
        for target in [ANGLE_MIN, ANGLE_MAX]:
            target = numpy.radians(target)
            controller.setAngleTarget(CHANNEL, target)
            while controller.isMoving(CHANNEL):
                angle = controller.getAngle(CHANNEL)

                q_1 = transforms3d.quaternions.axangle2quat(
                    [0, 1, 0], -numpy.pi / 2)
                q_2 = transforms3d.quaternions.axangle2quat([0, 0, 1], -angle)
                q = transforms3d.quaternions.qmult(q_2, q_1)

                print("{} {} {} {} 0 0 0".format(q[0], q[1], q[2], q[3]))
                time.sleep(0.1)


if __name__ == "__main__":
    main()
