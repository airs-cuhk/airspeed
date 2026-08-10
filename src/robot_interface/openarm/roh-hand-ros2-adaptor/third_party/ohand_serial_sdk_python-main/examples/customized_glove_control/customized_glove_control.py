from serial.tools import list_ports
import asyncio
import signal

from ohand.OHandSerialAPI import *
from ohand.constants import *
from pos_input_ble_glove import PosInputBleGlove as PosInput

PORT_UART = 0
PORT_CAN = 1

# Modify PORT_TYPE to select the communication port
PORT_TYPE = PORT_UART

if PORT_TYPE == PORT_UART:
    from ohand.interface.uart import *
else:
    from ohand.interface.can import *

ADDRESS_MASTER = 0x01

# Modify ADDRESS_HAND to select the hand ID
ADDRESS_HAND = 0x02

HAS_THUMB_ROOT_MOTOR = True
NUM_FINGERS = 5
NUM_MOTORS = 6
THUMB_ROOT_ID = 5
POS_THRESHOLD = 4096

def clamp(n, smallest, largest):
    return max(smallest, min(n, largest))

def interpolate(n, from_min, from_max, to_min, to_max):
    return (n - from_min) / (from_max - from_min) * (to_max - to_min) + to_min

class Application:
    def __init__(self):
        signal.signal(signal.SIGINT, lambda signal, frame: self._signal_handler())
        self.terminated = False

    def _signal_handler(self):
        print("You pressed ctrl-c, exit")
        self.terminated = True

    def find_comport(self, port_name):
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

    def set_custom(self, instance, hand_id, speed=None, position=None, angle=None, get_flag=0x00):
            data_flag = get_flag
            data = bytearray(1)

            if speed is not None:
                data_flag |= SUB_CMD_SET_SPEED
                for i in range(len(speed)):
                    value = int(speed[i])
                    value = clamp(value, 0, 65535)
                    data.append(value & 0xFF)
                    data.append((value >> 8) & 0xFF)

            if position is not None:
                data_flag |= SUB_CMD_SET_POS
                for i in range(len(position)):
                    value = int(position[i])
                    value = clamp(value, 0, 65535)
                    data.append(value & 0xFF)
                    data.append((value >> 8) & 0xFF)

            if angle is not None:
                data_flag |= SUB_CMD_SET_ANGLE
                for i in range(len(angle)):
                    value = int(angle[i] * 100)  # scale
                    if value < 0:
                        value += 65536
                    value = clamp(value, 0, 65535)
                    data.append(value & 0xFF)
                    data.append((value >> 8) & 0xFF)

            data[0] = data_flag

            err = instance.HAND_SetCustom(hand_id, data, len(data), [])
            if err != HAND_RESP_SUCCESS:
                return (err, None, None, None, None, None)

            # Parse data
            position = None
            angle = None
            current = None
            force = None
            status = None

            motor_cnt = 0
            data_entry = 0
            data_len = len(data)
            # print(f"data_len: {data_len}")

            if (data_flag & SUB_CMD_GET_POS): data_entry += 2
            if (data_flag & SUB_CMD_GET_ANGLE): data_entry += 2
            if (data_flag & SUB_CMD_GET_CURRENT): data_entry += 2
            if (data_flag & SUB_CMD_GET_FORCE): data_entry += 2
            if (data_flag & SUB_CMD_GET_STATUS): data_entry += 1

            if data_entry != 0:
                motor_cnt = int(data_len / data_entry)

            if motor_cnt * data_entry != data_len:
                return (HAND_RESP_DATA_INVALID, None, None, None, None, None)

            # print(f"motor_cnt: {motor_cnt}")
            offset = 0

            if (data_flag & SUB_CMD_GET_POS):
                position = []
                for i in range(motor_cnt):
                    value = data[offset] | (data[offset + 1] << 8)
                    position.append(float(value))
                    offset += 2

            if (data_flag & SUB_CMD_GET_ANGLE):
                angle = []
                for i in range(motor_cnt):
                    value = data[offset] | (data[offset + 1] << 8)
                    angle.append(float(value))
                    offset += 2

            if (data_flag & SUB_CMD_GET_CURRENT):
                current = []
                for i in range(motor_cnt):
                    value = data[offset] | (data[offset + 1] << 8)
                    current.append(float(value))
                    offset += 2

            if (data_flag & SUB_CMD_GET_FORCE):
                force = []
                for i in range(motor_cnt):
                    value = data[offset] | (data[offset + 1] << 8)
                    force.append(float(value))
                    offset += 2

            if (data_flag & SUB_CMD_GET_STATUS):
                status = []
                for i in range(motor_cnt):
                    value = data[offset]
                    status.append(int(value))
                    offset += 1

            return (err, position, angle, current, force, status)


    async def main(self):
        interface_instance = None
        ohand_instance = None

        if PORT_TYPE == PORT_UART:
            interface_instance = Serial_Init(port_name=self.find_comport("CH340") or self.find_comport("Serial"), baudrate=115200)
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

        speed = [65535 for _ in range(NUM_MOTORS)]
            
        pos_input = PosInput()
        await pos_input.start()

        while not self.terminated:
            # Data from motion capture
            finger_data = await pos_input.get_position()

            # Send to OHand and read
            err, position, angle, current, force, status = self.set_custom(ohand_instance,
                ADDRESS_HAND, speed=speed, position=finger_data, angle= None, get_flag= SUB_CMD_GET_POS
            )

            if err != HAND_RESP_SUCCESS:    
                print(f"HAND_SetCustom returned error: {err}")
            else:
                for i in range(NUM_MOTORS):
                    pos_err = abs(finger_data[i] - position[i])
                    if pos_err < POS_THRESHOLD:
                        speed[i] = interpolate(pos_err, 0, POS_THRESHOLD, 0, 65535)
                    else:
                        speed[i] = 65535

        await pos_input.stop()


if __name__ == '__main__':
    app = Application()
    asyncio.run(app.main())