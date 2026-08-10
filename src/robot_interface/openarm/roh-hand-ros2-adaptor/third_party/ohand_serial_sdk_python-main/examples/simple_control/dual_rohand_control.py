#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
File: dual_rohand_control.py
Description:
    Demonstrates how to use the SDK to control the OYMotion multi-degree-of-freedom dexterous hand. In subsequent example code, use the following default configuration,
    Please modify the corresponding configuration parameters according to actual conditions:
    Left hand hand_id: 0x02
    Right hand hand_id: 0x03
    Communication interface: 485
    Default port: "/dev/ttyUSB0"
    Baud rate: 115200

"""

from serial.tools import list_ports
from ohand.constants import *
from ohand.interface.uart import *
from ohand.OHandSerialAPI import OHandSerialAPI

NUM_MOTORS = 6
HAND_ID = [0x02, 0x03]
DEFAULT_PORT = "/dev/ttyUSB0"  # Modify to the corresponding serial port number according to actual ports

MAX_POS = [30000] * NUM_MOTORS
MIN_POS = [0] * NUM_MOTORS
MAX_SPEED = [255] * NUM_MOTORS

def find_comport(port_name):
    """
    Find available serial port automatically
    :param port_name: Characterization of the port description, such as "CH340"
    :return: Comport of device if successful, None otherwise
    """
    ports = list_ports.comports()
    for port in ports:
        if port_name in port.description:
            return port.device
    return None


def main():
    interface_instance = Serial_Init(port_name=find_comport("CH340") or find_comport("Serial") or DEFAULT_PORT, baudrate=115200)
    ohand_instane = OHandSerialAPI(interface_instance, HAND_PROTOCOL_UART, 0x01, send_data_impl, recv_data_impl)
    ohand_instane.HAND_SetTimerFunction(get_milli_seconds_impl, delay_milli_seconds_impl)
    ohand_instane.HAND_SetCommandTimeOut(255)

    while True:
        for hand_id in HAND_ID:
            err = ohand_instane.HAND_SetFingerPosAll(hand_id, MAX_POS, MAX_SPEED, NUM_MOTORS, [])
            if err != HAND_RESP_SUCCESS:
                print(f"Left rohand: HAND_SetFingerPosAll returned error: {err}")
            delay_milli_seconds_impl(500)

            err = ohand_instane.HAND_SetFingerPosAll(hand_id, MIN_POS, MAX_SPEED, NUM_MOTORS, [])
            if err != HAND_RESP_SUCCESS:
                print(f"Left rohand: HAND_SetFingerPosAll returned error: {err}")
            delay_milli_seconds_impl(500)

            delay_milli_seconds_impl(1000)


if __name__ == "__main__":
    main()
