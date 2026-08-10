from serial.tools import list_ports
import os
import sys
from ohand.constants import *
from ohand.OHandSerialAPI import OHandSerialAPI


# sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))
# from OHandSerialAPI import HAND_RESP_SUCCESS, HAND_PROTOCOL_UART, OHandSerialAPI


PORT_UART = 0
PORT_CAN = 1

# Modify PORT_TYPE to select the communication port
PORT_TYPE = PORT_CAN

if PORT_TYPE == PORT_UART:
    # from interface.uart_interface import *
    from ohand.interface.uart import *
else:
    from ohand.interface.can import *
    # from interface.can_interface import *

ADDRESS_MASTER = 0x01

# Modify ADDRESS_HAND to select the hand ID
ADDRESS_HAND = 0x02

HAS_THUMB_ROOT_MOTOR = True
NUM_FINGERS = 5
THUMB_ROOT_ID = 5

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
    interface_instance = None
    ohand_instance = None
    if PORT_TYPE == PORT_UART:
        interface_instance = Serial_Init(port_name=find_comport("CH340") or find_comport("Serial"), baudrate=115200)
    else:
        interface_instance = CAN_Init(port_name="1", baudrate=1000000)

    if interface_instance is None:
        print("Port init failed\n")
        return

    ohand_instance = OHandSerialAPI(interface_instance, HAND_PROTOCOL_UART, ADDRESS_MASTER,
                                           send_data_impl, recv_data_impl)

    ohand_instance.HAND_SetTimerFunction(get_milli_seconds_impl, delay_milli_seconds_impl)
    ohand_instance.HAND_SetCommandTimeOut(255)
    print(ohand_instance.get_private_data(), "\n")

    for finger_id in range(0, NUM_FINGERS):
        err = ohand_instance.HAND_SetFingerPos(ADDRESS_HAND, finger_id, 0, 255, [])
        if err != HAND_RESP_SUCCESS:
            print(f"HAND_SetFingerPos returned error: {err}")

    if HAS_THUMB_ROOT_MOTOR:
        err = ohand_instance.HAND_SetFingerPos(ADDRESS_HAND, THUMB_ROOT_ID, 0, 255, [])
        if err != HAND_RESP_SUCCESS:
            print(f"HAND_SetFingerPos returned error: {err}")

    delay_milli_seconds_impl(2000)

    while True:
        err = ohand_instance.HAND_SetFingerPos(ADDRESS_HAND, 0, 65535, 255, [])
        if err != HAND_RESP_SUCCESS:
            print(f"HAND_SetFingerPos returned error: {err}")
        delay_milli_seconds_impl(1500)

        err = ohand_instance.HAND_SetFingerPos(ADDRESS_HAND, 0, 0, 255, [])
        if err != HAND_RESP_SUCCESS:
            print(f"HAND_SetFingerPos returned error: {err}")
        delay_milli_seconds_impl(1500)

        for finger_id in range(1, NUM_FINGERS):
            err = ohand_instance.HAND_SetFingerPos(ADDRESS_HAND, finger_id, 65535, 255, [])
            if err != HAND_RESP_SUCCESS:
                print(f"HAND_SetFingerPos returned error: {err}")
            delay_milli_seconds_impl(200)
        delay_milli_seconds_impl(1500)

        for finger_id in range(1, NUM_FINGERS):
            err = ohand_instance.HAND_SetFingerPos(ADDRESS_HAND, finger_id, 0, 255, [])
            if err != HAND_RESP_SUCCESS:
                print(f"HAND_SetFingerPos returned error: {err}")
            delay_milli_seconds_impl(200)
        delay_milli_seconds_impl(1500)

        if HAS_THUMB_ROOT_MOTOR:
            err = ohand_instance.HAND_SetFingerPos(ADDRESS_HAND, THUMB_ROOT_ID, 65535, 255, [])
            if err != HAND_RESP_SUCCESS:
                print(f"HAND_SetFingerPos returned error: {err}")
            delay_milli_seconds_impl(1500)

            err = ohand_instance.HAND_SetFingerPos(ADDRESS_HAND, THUMB_ROOT_ID, 0, 255, [])
            if err != HAND_RESP_SUCCESS:
                print(f"HAND_SetFingerPos returned error: {err}")
            delay_milli_seconds_impl(1500)


if __name__ == '__main__':
    main()