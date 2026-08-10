import time
import serial

__all__ = [
    'send_data_impl',
    'recv_data_impl',
    'get_milli_seconds_impl',
    'delay_milli_seconds_impl',
    'Serial_Init',
]

# Send data function (adapted for Serial)
def send_data_impl(addr, data, length, context):
    """
    Send serial data
    Interface consistent with OHandSerialAPI: (addr, data, length, private_data)
    """
    if not context or not hasattr(context, 'write'):
        print("Error: Serial port not properly initialized")
        return 1
        
    try:
        # Directly send all data via serial port
        context.write(data[:length])
        # print(f"Send data: LEN={length}, DATA=", end='')
        # for byte in data[:length]:
        #     print(f"{byte:02X} ", end='')
        # print()
        return 0
    except serial.SerialException as e:
        print(f"Serial send failed, error: {e}")
        return 1
    except Exception as e:
        print(f"Send exception: {e}")
        return 1

# Receive data function (adapted for Serial)
def recv_data_impl(context, api_instance=None):
    """
    Receive serial data and process (read all available data at once)
    Interface consistent with OHandSerialAPI: (private_data)
    """
    if not context or not hasattr(context, 'read'):
        print("Error: Serial port not properly initialized")
        return

    uart_interface = context
        
    try:
        msg_bytes = uart_interface.read(uart_interface.in_waiting or 1)
        if msg_bytes:
            # Print received byte data
            # print(f"Receive data: LEN={len(msg_bytes)}, DATA=", end="")
            # for byte in msg_bytes:
            #     print(f"{byte:02X} ", end="")
            # print()

            # Parse data according to actual protocol, here assume bytes are passed to HAND_OnData
            for byte in msg_bytes:
                if api_instance:
                    api_instance.HAND_OnData(byte)
    except serial.SerialException as e:
        print(f"Serial receive error: {e}")
    except Exception as e:
        print(f"Receive exception: {e}")

# Time related functions (unchanged)
_start_time = None
def get_milli_seconds_impl():
    """Return milliseconds since program started"""
    global _start_time
    if _start_time is None:
        _start_time = time.time() * 1000  # Initialize start time
    return int(time.time() * 1000 - _start_time)

def delay_milli_seconds_impl(ms):
    """Pause execution for specified milliseconds"""
    time.sleep(ms / 1000.0)

# Serial initialization function
def Serial_Init(port_name, baudrate):
    """Initialize serial connection, return serial.Serial instance"""
    try:
        # Configure serial parameters
        ser = serial.Serial(
            port=port_name,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.005  # Set read timeout (consistent with CAN version)
        )
        
        if ser.is_open:
            print(f"\nSerial port initialized successfully: port={port_name}, baudrate={baudrate}")
            return ser
        else:
            print(f"\nError: Unable to open serial port {port_name}")
            return None

    except serial.SerialException as e:
        print(f"\nError: Serial port initialization failed, {str(e)}")
        return None
    except Exception as e:
        print(f"\nInitialization exception: {str(e)}")
        return None
