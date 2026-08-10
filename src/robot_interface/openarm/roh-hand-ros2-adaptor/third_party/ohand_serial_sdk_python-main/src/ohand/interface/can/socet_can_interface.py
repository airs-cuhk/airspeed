import time
import can

# Send data function (matches OHandSerialAPI interface)
def send_data_impl(addr, data, length, context):
    """
    Send data in frames, max 8 bytes per frame.
    Interface matches OHandSerialAPI: (addr, data, length, context)
    """
    if not context:
        print("Error: null context")
        return 1

    can_interface = context

    if not can_interface or not hasattr(can_interface, "send"):
        print("Error: CAN bus not properly initialized")
        return 1

    try:
        for i in range(0, length, 8):
            current_size = min(8, length - i)
            # Create CAN message
            msg = can.Message(arbitration_id=addr, data=data[i : i + current_size], is_extended_id=False)
            # Send message
            can_interface.send(msg)
            # print(f"Sent frame: ID=0x{addr:03X}, LEN={current_size}, DATA=", end="")
            # for byte in data[i : i + current_size]:
            #     print(f"{byte:02X} ", end="")
            # print()
        return 0
    except can.CanError as e:
        print(f"CAN send failed, error: {e}")
        return 1
    except Exception as e:
        print(f"Send exception: {e}")
        return 1


# Receive data function (matches OHandSerialAPI interface)
def recv_data_impl(context, api_instance=None):
    """
    Receive CAN data and process.
    Interface matches OHandSerialAPI: (context)
    """
    if not context:
        print("Error: null context")
        return 1

    can_interface = context

    if not can_interface or not hasattr(can_interface, "recv"):
        print("Error: CAN bus not properly initialized")
        return 1

    try:
        # Non-blocking receive (timeout 0.005 seconds)
        msg = can_interface.recv(timeout=0.005)
        if msg is not None:
            # Print received CAN frame info
            # print(f"Received frame: ID=0x{msg.arbitration_id:03X}, LEN={msg.dlc}, DATA=", end="")
            # for byte in msg.data:
            #     print(f"{byte:02X} ", end="")
            # print()

            # If the message is sent to the master device, call HAND_OnData
            if msg.arbitration_id == 0x01:  # ADDRESS_MASTER
                for byte in msg.data:
                    api_instance.HAND_OnData(byte)
    except can.CanError as e:
        print(f"CAN receive error: {e}")
    except Exception as e:
        print(f"Receive exception: {e}")


# Time related functions
_start_time = None


def get_milli_seconds_impl():
    """Return milliseconds since program start"""
    global _start_time
    if _start_time is None:
        _start_time = time.time() * 1000  # Initialize start time
    return int(time.time() * 1000 - _start_time)


def delay_milli_seconds_impl(ms):
    """Pause execution for the specified milliseconds"""
    time.sleep(ms / 1000.0)


# CAN initialization function
def CAN_Init(port_name, baudrate):
    """Initialize CAN bus connection, return can.interface.Bus instance"""
    try:
        port_num = int(port_name)
        if port_num < 1 or port_num > 16:
            print(f"\nError: Invalid port number {port_name}, must be a number between 1 and 16")
            return None

        if baudrate not in [250000, 500000, 1000000]:
            print(f"\nError: Unsupported baudrate {baudrate}, must be 250000, 500000, or 1000000")
            return None

        bus = can.interface.Bus(interface="socketcan", channel=f"can{port_num}", bitrate=baudrate)
        print(f"\nCAN bus initialized successfully: port={port_name}, baudrate={baudrate}")
        return bus

    except ValueError as e:
        print(f"\nError: Port name parsing failed, {str(e)}")
        return None
    except can.CanError as e:
        print(f"\nError: CAN initialization failed, {str(e)}")
        return None
    except Exception as e:
        print(f"\nInitialization exception: {str(e)}")
        return None

