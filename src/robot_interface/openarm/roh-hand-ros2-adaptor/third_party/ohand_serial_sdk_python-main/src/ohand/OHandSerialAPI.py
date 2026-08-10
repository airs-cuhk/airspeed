import struct
import time

from .constants import *

__all__ = [
    'OHandSerialAPI',
]

def match_data_type(data, type):
    if type == UINT8_T:
        return 0x00 <= data <= 0xFF
    elif type == INT8_T:
        return -0x80 <= data <= 0x7F
    elif type == UINT16_T:
        return 0x0000 <= data <= 0xFFFF
    elif type == INT16_T:
        return -0x8000 <= data <= 0x7FFF
    else:
        # print("Unsupported data type: ", type)
        return False


def match_list_type(data, type):
    if isinstance(data, list):
        for item in data:
            if not match_data_type(item, type):
                return False
        return True
    else:
        return False


class OHandSerialAPI:
    def __init__(self, private_data, protocol, address_master, send_data_impl, recv_data_impl=None):
        self.private_data = private_data
        self.protocol = protocol
        self.address_master = address_master
        self.send_data_impl = send_data_impl
        self.recv_data_impl = recv_data_impl
        self.timeout = 255  # Default timeout in ms
        self._get_milli_seconds_impl = None
        self._delay_milli_seconds_impl = None
        self.packet_data = bytearray(MAX_PROTOCOL_DATA_SIZE + 5)
        self.is_whole_packet = False
        self.decode_state = self._initial_state()
        self.byte_count = 0

    def _initial_state(self):
        if self.protocol == HAND_PROTOCOL_UART:
            return "WAIT_ON_HEADER_0"
        elif self.protocol == HAND_PROTOCOL_I2C:
            return "WAIT_ON_ADDRESSED_NODE_ID"

    def get_private_data(self):
        return self.private_data

    def HAND_ProtocolLRC(self, lrcBytes):
        lrc = 0
        for byte in lrcBytes:
            lrc ^= byte
        return lrc

    def HAND_SendCmd(self, addr, cmd, data, nb_data):
        if not self.send_data_impl:
            return HAND_RESP_INVALID_CONTEXT

        if not self._delay_milli_seconds_impl or not self._get_milli_seconds_impl:
            return HAND_RESP_TIMER_FUNC_NOT_SET

        if nb_data >= MAX_PROTOCOL_DATA_SIZE:
            return HAND_RESP_DATA_SIZE_TOO_BIG

        send_buf = bytearray(7 + nb_data)
        send_buf[0] = 0x55
        send_buf[1] = 0xAA
        send_buf[2] = addr
        send_buf[3] = self.address_master
        send_buf[4] = cmd
        send_buf[5] = nb_data

        # 处理data为None或空的情况
        if data is not None:
            send_buf[6 : 6 + nb_data] = data  # 确保data是可迭代的字节数据

        # 计算LRC校验（从addr开始到data结束）
        lrc = 0
        for i in range(2, 6 + nb_data):
            lrc ^= send_buf[i]
        send_buf[6 + nb_data] = lrc

        # 发送数据并返回结果（假设send_data_impl返回0表示成功）
        if self.send_data_impl(addr, send_buf, len(send_buf), self.private_data) != 0:
            return HAND_RESP_HAND_ERROR

        return HAND_RESP_SUCCESS

    def HAND_GetResponse(self, addr, cmd, time_out, resp_bytes, remote_err):
        wait_start = self._get_milli_seconds_impl()
        wait_timeout = wait_start + time_out

        while not self.is_whole_packet:
            time.sleep(0.001)  # Delay 1ms

            if self.recv_data_impl:
                self.recv_data_impl(self.private_data, self)

            if self._get_milli_seconds_impl() > wait_timeout:
                self.decode_state = self._initial_state()
                return HAND_RESP_TIMEOUT

        # Validate LRC
        lrc = self.HAND_ProtocolLRC(self.packet_data[: self.packet_data[3] + 4])
        if lrc != self.packet_data[self.packet_data[3] + 4]:
            self.is_whole_packet = False
            return ERR_PROTOCOL_WRONG_LRC

        # Check if response is error
        if (self.packet_data[2] & CMD_ERROR_MASK) != 0:
            self.is_whole_packet = False
            if remote_err is not None:
                remote_err.append(self.packet_data[4])
            return HAND_RESP_HAND_ERROR

        if self.packet_data[1] != addr and addr != 0xFF:
            self.is_whole_packet = False
            return HAND_RESP_UNMATCHED_ADDR

        if self.packet_data[2] != cmd:
            self.is_whole_packet = False
            return HAND_RESP_UNMATCHED_CMD

        # Copy response data
        if resp_bytes:
            packet_byte_count = self.packet_data[3]
            if packet_byte_count > len(resp_bytes):
                return HAND_RESP_INVALID_OUT_BUFFER_SIZE
            else:
                resp_bytes[:] = self.packet_data[4 : 4 + packet_byte_count]

        self.is_whole_packet = False
        return HAND_RESP_SUCCESS

    def HAND_SetTimerFunction(self, get_milli_seconds_impl, delay_milli_seconds_impl):
        self._get_milli_seconds_impl = get_milli_seconds_impl
        self._delay_milli_seconds_impl = delay_milli_seconds_impl

    def HAND_GetTick(self):
        if self._get_milli_seconds_impl:
            return self._get_milli_seconds_impl()
        else:
            return 0

    def HAND_SetCommandTimeOut(self, timeout):
        self.timeout = timeout

    def HAND_OnData(self, data):
        if self.is_whole_packet:
            return  # Old packet is not processed, ignore

        if self.decode_state == "WAIT_ON_HEADER_0":
            if data == 0x55:
                self.decode_state = "WAIT_ON_HEADER_1"
        elif self.decode_state == "WAIT_ON_HEADER_1":
            if data == 0xAA:
                self.decode_state = "WAIT_ON_ADDRESSED_NODE_ID"
            else:
                self.decode_state = "WAIT_ON_HEADER_0"
        elif self.decode_state == "WAIT_ON_ADDRESSED_NODE_ID":
            self.packet_data[0] = data
            self.decode_state = "WAIT_ON_OWN_NODE_ID"
        elif self.decode_state == "WAIT_ON_OWN_NODE_ID":
            self.packet_data[1] = data
            self.decode_state = "WAIT_ON_COMMAND_ID"
        elif self.decode_state == "WAIT_ON_COMMAND_ID":
            self.packet_data[2] = data
            self.decode_state = "WAIT_ON_BYTECOUNT"
        elif self.decode_state == "WAIT_ON_BYTECOUNT":
            self.packet_data[3] = data
            self.byte_count = data
            if self.byte_count > MAX_PROTOCOL_DATA_SIZE:
                self.decode_state = self._initial_state()
            elif self.byte_count > 0:
                self.decode_state = "WAIT_ON_DATA"
            else:
                self.decode_state = "WAIT_ON_LRC"
        elif self.decode_state == "WAIT_ON_DATA":
            index = 4 + self.packet_data[3] - self.byte_count
            self.packet_data[index] = data
            self.byte_count -= 1
            if self.byte_count == 0:
                self.decode_state = "WAIT_ON_LRC"
        elif self.decode_state == "WAIT_ON_LRC":
            index = 4 + self.packet_data[3]
            self.packet_data[index] = data
            if self.packet_data[0] == self.address_master:
                self.is_whole_packet = True
            self.decode_state = self._initial_state()

    def HAND_GetProtocolVersion(self, hand_id, major, minor, remote_err):
        out = bytearray(2)
        err = self.HAND_SendCmd(hand_id, HAND_CMD_GET_PROTOCOL_VERSION, None, 0)
        if err == HAND_RESP_SUCCESS:
            byte_count = len(out)
            err = self.HAND_GetResponse(hand_id, HAND_CMD_GET_PROTOCOL_VERSION, self.timeout, out, remote_err)
            if err == HAND_RESP_SUCCESS:
                minor[0] = out[0]
                major[0] = out[1]
        return err, major[0], minor[0]

    def HAND_GetFirmwareVersion(self, hand_id, major, minor, revision, remote_err):
        out = bytearray(4)  # Assuming revision is 2 bytes, and major/minor are 1 byte each
        err = self.HAND_SendCmd(hand_id, HAND_CMD_GET_FW_VERSION, None, 0)
        if err == HAND_RESP_SUCCESS:
            byte_count = len(out)
            err = self.HAND_GetResponse(hand_id, HAND_CMD_GET_FW_VERSION, self.timeout, out, remote_err)
            if err == HAND_RESP_SUCCESS:
                revision[0] = out[0] | (out[1] << 8)
                minor[0] = out[2]
                major[0] = out[3]
        return err, major[0], minor[0], revision[0]

    def HAND_GetHardwareVersion(self, hand_id, hw_type, hw_ver, boot_version, remote_err):
        out = bytearray(4)  # Assuming boot_version is 2 bytes, and hw_type/hw_ver are 1 byte each
        err = self.HAND_SendCmd(hand_id, HAND_CMD_GET_HW_VERSION, None, 0)
        if err == HAND_RESP_SUCCESS:
            byte_count = len(out)
            err = self.HAND_GetResponse(hand_id, HAND_CMD_GET_HW_VERSION, self.timeout, out, remote_err)
            if err == HAND_RESP_SUCCESS:
                hw_type[0] = out[0]
                hw_ver[0] = out[1]
                boot_version[0] = (out[2] << 8) | out[3]
        return err, hw_type[0], hw_ver[0], boot_version[0]

    def HAND_GetCaliData(self, hand_id, end_pos, start_pos, motor_cnt, thumb_root_pos, thumb_root_pos_cnt, remote_err):
        out = bytearray(2 + 2 * MAX_MOTOR_CNT + 2 * MAX_MOTOR_CNT + 1 + 2 * MAX_THUMB_ROOT_POS)
        err = self.HAND_SendCmd(hand_id, HAND_CMD_GET_CALI_DATA, None, 0)
        if err == HAND_RESP_SUCCESS:
            byte_count = len(out)
            err = self.HAND_GetResponse(hand_id, HAND_CMD_GET_CALI_DATA, self.timeout, out, remote_err)
            if err == HAND_RESP_SUCCESS:
                p_data = iter(out)
                motor_cnt_ret = next(p_data)
                thumb_root_pos_cnt_ret = next(p_data)
                if motor_cnt[0] < motor_cnt_ret or thumb_root_pos_cnt[0] < thumb_root_pos_cnt_ret:
                    return HAND_RESP_DATA_SIZE_TOO_BIG
                motor_cnt[0] = motor_cnt_ret
                thumb_root_pos_cnt[0] = thumb_root_pos_cnt_ret

                end_pos_size = 2 * motor_cnt_ret
                start_pos_size = 2 * motor_cnt_ret
                thumb_root_pos_size = 2 * thumb_root_pos_cnt_ret

                if end_pos:
                    for i in range(motor_cnt_ret):
                        end_pos[i] = next(p_data) | (next(p_data) << 8)

                if start_pos:
                    for i in range(motor_cnt_ret):
                        start_pos[i] = next(p_data) | (next(p_data) << 8)

                if thumb_root_pos:
                    for i in range(thumb_root_pos_cnt_ret):
                        thumb_root_pos[i] = next(p_data) | (next(p_data) << 8)

        return err, end_pos, start_pos, thumb_root_pos

    def HAND_GetFingerPID(self, hand_id, finger_id, p, i, d, g, remote_err):
        if not match_data_type(finger_id, UINT8_T):
            return HAND_RESP_DATA_INVALID

        data = bytearray(1)
        data[0] = finger_id
        out = bytearray(1 + 4 + 4 + 4 + 4)  # Assuming float is 4 bytes
        err = self.HAND_SendCmd(hand_id, HAND_CMD_GET_FINGER_PID, data, len(data))
        if err == HAND_RESP_SUCCESS:
            byte_count = len(out)
            err = self.HAND_GetResponse(hand_id, HAND_CMD_GET_FINGER_PID, self.timeout, out, remote_err)
            if err == HAND_RESP_SUCCESS and out[0] == finger_id:
                p[0] = struct.unpack('f', out[1:5])
                i[0] = struct.unpack('f', out[5:9])
                d[0] = struct.unpack('f', out[9:13])
                g[0] = struct.unpack('f', out[13:17])
            else:
                err = HAND_RESP_DATA_INVALID
        return err, p[0][0], i[0][0], d[0][0], g[0][0]

    def HAND_GetFingerCurrentLimit(self, hand_id, finger_id, current_limit, remote_err):
        if not match_data_type(finger_id, UINT8_T):
            return HAND_RESP_DATA_INVALID

        data = bytearray(1)
        data[0] = finger_id
        out = bytearray(3)
        err = self.HAND_SendCmd(hand_id, HAND_CMD_GET_FINGER_CURRENT_LIMIT, data, len(data))
        if err == HAND_RESP_SUCCESS:
            byte_count = len(out)
            err = self.HAND_GetResponse(hand_id, HAND_CMD_GET_FINGER_CURRENT_LIMIT, self.timeout, out, remote_err)
            if err == HAND_RESP_SUCCESS and out[0] == finger_id:
                current_limit[0] = out[1] | (out[2] << 8)
            else:
                err = HAND_RESP_DATA_INVALID
        return err, current_limit[0]

    def HAND_GetFingerCurrent(self, hand_id, finger_id, current, remote_err):
        if not match_data_type(finger_id, UINT8_T):
            return HAND_RESP_DATA_INVALID

        data = bytearray(1)
        data[0] = finger_id
        out = bytearray(3)
        err = self.HAND_SendCmd(hand_id, HAND_CMD_GET_FINGER_CURRENT, data, len(data))
        if err == HAND_RESP_SUCCESS:
            byte_count = len(out)
            err = self.HAND_GetResponse(hand_id, HAND_CMD_GET_FINGER_CURRENT, self.timeout, out, remote_err)
            if err == HAND_RESP_SUCCESS and out[0] == finger_id:
                current[0] = out[1] | (out[2] << 8)
            else:
                err = HAND_RESP_DATA_INVALID
        return err, current[0]

    def HAND_GetFingerForceTarget(self, hand_id, finger_id, force_target, remote_err):
        if not match_data_type(finger_id, UINT8_T):
            return HAND_RESP_DATA_INVALID

        data = bytearray(1)
        data[0] = finger_id
        out = bytearray(3)
        err = self.HAND_SendCmd(hand_id, HAND_CMD_GET_FINGER_FORCE_TARGET, data, len(data))
        if err == HAND_RESP_SUCCESS:
            byte_count = len(out)
            err = self.HAND_GetResponse(hand_id, HAND_CMD_GET_FINGER_FORCE_TARGET, self.timeout, out, remote_err)
            if err == HAND_RESP_SUCCESS and out[0] == finger_id:
                force_target[0] = out[1] | (out[2] << 8)
            else:
                err = HAND_RESP_DATA_INVALID
        return err, force_target[0]

    def HAND_GetFingerForce(self, hand_id, finger_id, force_entry_cnt, force, remote_err):
        if not match_data_type(finger_id, UINT8_T):
            return HAND_RESP_DATA_INVALID

        data = bytearray(1)
        data[0] = finger_id
        out = bytearray(1 + MAX_FORCE_ENTRIES * 2)  # Assuming each entry is 2 bytes
        err = self.HAND_SendCmd(hand_id, HAND_CMD_GET_FINGER_FORCE, data, len(data))
        if err == HAND_RESP_SUCCESS:
            byte_count = len(out)
            err = self.HAND_GetResponse(hand_id, HAND_CMD_GET_FINGER_FORCE, self.timeout, out, remote_err)
            if err == HAND_RESP_SUCCESS:
                if finger_id != out[0]:
                    err = HAND_RESP_DATA_INVALID
                else:
                    force_entry_cnt[0] = out[1]
                    for i in range(force_entry_cnt[0]):
                        if i < len(force):
                            force[i] = out[2 + i]
        return err, force

    def HAND_GetFingerPosLimit(self, hand_id, finger_id, low_limit, high_limit, remote_err):
        if not match_data_type(finger_id, UINT8_T):
            return HAND_RESP_DATA_INVALID

        data = bytearray(1)
        data[0] = finger_id
        out = bytearray(5)
        err = self.HAND_SendCmd(hand_id, HAND_CMD_GET_FINGER_POS_LIMIT, data, len(data))
        if err == HAND_RESP_SUCCESS:
            byte_count = len(out)
            err = self.HAND_GetResponse(hand_id, HAND_CMD_GET_FINGER_POS_LIMIT, self.timeout, out, remote_err)
            if err == HAND_RESP_SUCCESS and out[0] == finger_id:
                low_limit[0] = out[1] | (out[2] << 8)
                high_limit[0] = out[3] | (out[4] << 8)
            else:
                err = HAND_RESP_DATA_INVALID
        return err, low_limit[0], high_limit[0]

    def HAND_GetFingerPosAbs(self, hand_id, finger_id, target_pos, current_pos, remote_err):
        if not match_data_type(finger_id, UINT8_T):
            return HAND_RESP_DATA_INVALID

        data = bytearray(1)
        data[0] = finger_id
        out = bytearray(5)
        err = self.HAND_SendCmd(hand_id, HAND_CMD_GET_FINGER_POS_ABS, data, len(data))
        if err == HAND_RESP_SUCCESS:
            byte_count = len(out)
            err = self.HAND_GetResponse(hand_id, HAND_CMD_GET_FINGER_POS_ABS, self.timeout, out, remote_err)
            if err == HAND_RESP_SUCCESS and out[0] == finger_id:
                if target_pos:
                    target_pos[0] = out[1] | (out[2] << 8)
                if current_pos:
                    current_pos[0] = out[3] | (out[4] << 8)
            else:
                err = HAND_RESP_DATA_INVALID
        return err, target_pos[0], current_pos[0]

    def HAND_GetFingerPos(self, hand_id, finger_id, target_pos, current_pos, remote_err):
        if not match_data_type(finger_id, UINT8_T):
            return HAND_RESP_DATA_INVALID

        data = bytearray(1)
        data[0] = finger_id
        out = bytearray(5)
        err = self.HAND_SendCmd(hand_id, HAND_CMD_GET_FINGER_POS, data, len(data))
        if err == HAND_RESP_SUCCESS:
            byte_count = len(out)
            err = self.HAND_GetResponse(hand_id, HAND_CMD_GET_FINGER_POS, self.timeout, out, remote_err)
            if err == HAND_RESP_SUCCESS and out[0] == finger_id:
                if target_pos:
                    target_pos[0] = out[1] | (out[2] << 8)
                if current_pos:
                    current_pos[0] = out[3] | (out[4] << 8)
            else:
                err = HAND_RESP_DATA_INVALID
        return err, target_pos[0], current_pos[0]

    def HAND_GetFingerAngle(self, hand_id, finger_id, target_angle, current_angle, remote_err):
        if not match_data_type(finger_id, UINT8_T):
            return HAND_RESP_DATA_INVALID

        data = bytearray(1)
        data[0] = finger_id
        out = bytearray(5)
        err = self.HAND_SendCmd(hand_id, HAND_CMD_GET_FINGER_ANGLE, data, len(data))
        if err == HAND_RESP_SUCCESS:
            byte_count = len(out)
            err = self.HAND_GetResponse(hand_id, HAND_CMD_GET_FINGER_ANGLE, self.timeout, out, remote_err)
            if err == HAND_RESP_SUCCESS and out[0] == finger_id:
                if target_angle:
                    target_angle[0] = int.from_bytes(out[1:3], byteorder="little", signed=True)
                if current_angle:
                    current_angle[0] = int.from_bytes(out[3:5], byteorder="little", signed=True)
            else:
                err = HAND_RESP_DATA_INVALID
        return err, target_angle[0], current_angle[0]

    def HAND_GetThumbRootPos(self, hand_id, raw_encoder, pos, remote_err):
        out = bytearray(3)
        err = self.HAND_SendCmd(hand_id, HAND_CMD_GET_THUMB_ROOT_POS, None, 0)
        if err == HAND_RESP_SUCCESS:
            byte_count = len(out)
            err = self.HAND_GetResponse(hand_id, HAND_CMD_GET_THUMB_ROOT_POS, self.timeout, out, remote_err)
            if err == HAND_RESP_SUCCESS:
                raw_encoder[0] = out[0] | (out[1] << 8)
                pos[0] = out[2]
        return err, raw_encoder[0], pos[0]

    def HAND_GetFingerPosAbsAll(self, hand_id, target_pos, current_pos, motor_cnt, remote_err):
        out = bytearray(2 * MAX_MOTOR_CNT * 2)
        err = self.HAND_SendCmd(hand_id, HAND_CMD_GET_FINGER_POS_ABS_ALL, None, 0)
        if err == HAND_RESP_SUCCESS:
            byte_count = len(out)
            err = self.HAND_GetResponse(hand_id, HAND_CMD_GET_FINGER_POS_ABS_ALL, self.timeout, out, remote_err)
            if err == HAND_RESP_SUCCESS:
                motor_cnt_ret = byte_count // (2 + 2)
                if motor_cnt[0] < motor_cnt_ret:
                    return HAND_RESP_DATA_SIZE_TOO_BIG
                motor_cnt[0] = motor_cnt_ret
                if target_pos:
                    for i in range(motor_cnt_ret):
                        target_pos[i] = out[2 * i] | (out[2 * i + 1] << 8)
                if current_pos:
                    for i in range(motor_cnt_ret):
                        current_pos[i] = out[2 * motor_cnt_ret + 2 * i] | (out[2 * motor_cnt_ret + 2 * i + 1] << 8)
        return err, target_pos, current_pos

    def HAND_GetFingerPosAll(self, hand_id, target_pos, current_pos, motor_cnt, remote_err):
        out = bytearray(2 * MAX_MOTOR_CNT * 2)
        err = self.HAND_SendCmd(hand_id, HAND_CMD_GET_FINGER_POS_ALL, None, 0)
        if err == HAND_RESP_SUCCESS:
            byte_count = len(out)
            err = self.HAND_GetResponse(hand_id, HAND_CMD_GET_FINGER_POS_ALL, self.timeout, out, remote_err)
            if err == HAND_RESP_SUCCESS:
                motor_cnt_ret = byte_count // (2 + 2)
                if motor_cnt[0] < motor_cnt_ret:
                    return HAND_RESP_DATA_SIZE_TOO_BIG
                motor_cnt[0] = motor_cnt_ret
                if target_pos:
                    for i in range(motor_cnt_ret):
                        target_pos[i] = out[2 * i] | (out[2 * i + 1] << 8)
                if current_pos:
                    for i in range(motor_cnt_ret):
                        current_pos[i] = out[2 * motor_cnt_ret + 2 * i] | (out[2 * motor_cnt_ret + 2 * i + 1] << 8)
        return err, target_pos, current_pos

    def HAND_GetFingerAngleAll(self, hand_id, target_angle, current_angle, motor_cnt, remote_err):
        out = bytearray(2 * MAX_MOTOR_CNT * 2)
        err = self.HAND_SendCmd(hand_id, HAND_CMD_GET_FINGER_ANGLE_ALL, None, 0)
        if err == HAND_RESP_SUCCESS:
            byte_count = len(out)
            err = self.HAND_GetResponse(hand_id, HAND_CMD_GET_FINGER_ANGLE_ALL, self.timeout, out, remote_err)
            if err == HAND_RESP_SUCCESS:
                motor_cnt_ret = byte_count // (2 + 2)
                if motor_cnt[0] < motor_cnt_ret:
                    return HAND_RESP_DATA_SIZE_TOO_BIG
                motor_cnt[0] = motor_cnt_ret
                if target_angle:
                    for i in range(motor_cnt_ret):
                        target_angle[i] = int.from_bytes(out[2 * i : 2 * i + 2], byteorder="little", signed=True)
                if current_angle:
                    for i in range(motor_cnt_ret):
                        current_angle[i] = int.from_bytes(
                            out[2 * motor_cnt_ret + 2 * i : 2 * motor_cnt_ret + 2 * i + 2], byteorder="little", signed=True
                        )
        return err, target_angle, current_angle

    def HAND_GetFingerStopParams(self, hand_id, finger_id, speed, stop_current, stop_after_period, retry_interval, remote_err):
        if not match_data_type(finger_id, UINT8_T):
            return HAND_RESP_DATA_INVALID

        data = bytearray(1)
        data[0] = finger_id
        out = bytearray(9)  # Assuming 4 bytes of stop params
        err = self.HAND_SendCmd(hand_id, HAND_CMD_GET_FINGER_STOP_PARAMS, data, len(data))
        if err == HAND_RESP_SUCCESS:
            err = self.HAND_GetResponse(hand_id, HAND_CMD_GET_FINGER_STOP_PARAMS, self.timeout, out, remote_err)
            if err == HAND_RESP_SUCCESS and out[0] == finger_id:
                speed[0] = out[1] | (out[2] << 8)
                stop_current[0] = out[3] | (out[4] << 8)
                stop_after_period[0] = out[5] | (out[6] << 8)
                retry_interval[0] = out[7] | (out[8] << 8)
            else:
                err = HAND_RESP_DATA_INVALID
        return err, speed[0], stop_current[0], stop_after_period[0], retry_interval[0]

    def HAND_GetFingerForcePID(self, hand_id, finger_id, p, i, d, g, remote_err):
        if not match_data_type(finger_id, UINT8_T):
            return HAND_RESP_DATA_INVALID

        data = bytearray(1)
        data[0] = finger_id
        out = bytearray(1 + 4 + 4 + 4 + 4)  # Assuming float is 4 bytes
        err = self.HAND_SendCmd(hand_id, HAND_CMD_GET_FINGER_FORCE_PID, data, len(data))
        if err == HAND_RESP_SUCCESS:
            err = self.HAND_GetResponse(hand_id, HAND_CMD_GET_FINGER_FORCE_PID, self.timeout, out, remote_err)
            if err == HAND_RESP_SUCCESS and out[0] == finger_id:
                p[0] = struct.unpack('f', out[1:5])
                i[0] = struct.unpack('f', out[5:9])
                d[0] = struct.unpack('f', out[9:13])
                g[0] = struct.unpack('f', out[13:17])
            else:
                err = HAND_RESP_DATA_INVALID
        return err, p[0][0], i[0][0], d[0][0], g[0][0]

    def HAND_GetSelfTestLevel(self, hand_id, self_test_level, remote_err):
        out = bytearray(1)
        err = self.HAND_SendCmd(hand_id, HAND_CMD_GET_SELF_TEST_LEVEL, None, 0)
        if err == HAND_RESP_SUCCESS:
            byte_count = len(out)
            err = self.HAND_GetResponse(hand_id, HAND_CMD_GET_SELF_TEST_LEVEL, self.timeout, out, remote_err)
            if err == HAND_RESP_SUCCESS:
                self_test_level[0] = out[0]
        return err, self_test_level[0]

    def HAND_GetBeepSwitch(self, hand_id, beep_switch, remote_err):
        out = bytearray(1)
        err = self.HAND_SendCmd(hand_id, HAND_CMD_GET_BEEP_SWITCH, None, 0)
        if err == HAND_RESP_SUCCESS:
            byte_count = len(out)
            err = self.HAND_GetResponse(hand_id, HAND_CMD_GET_BEEP_SWITCH, self.timeout, out, remote_err)
            if err == HAND_RESP_SUCCESS:
                beep_switch[0] = out[0]
        return err, beep_switch[0]

    def HAND_GetButtonPressedCnt(self, hand_id, pressed_cnt, remote_err):
        out = bytearray(1)
        err = self.HAND_SendCmd(hand_id, HAND_CMD_GET_BUTTON_PRESSED_CNT, None, 0)
        if err == HAND_RESP_SUCCESS:
            byte_count = len(out)
            err = self.HAND_GetResponse(hand_id, HAND_CMD_GET_BUTTON_PRESSED_CNT, self.timeout, out, remote_err)
            if err == HAND_RESP_SUCCESS:
                pressed_cnt[0] = out[0]
        return err, pressed_cnt[0]

    def HAND_GetUID(self, hand_id, uid_w0, uid_w1, uid_w2, remote_err):
        out = bytearray(12)  # 96 bits UID
        err = self.HAND_SendCmd(hand_id, HAND_CMD_GET_UID, None, 0)
        if err == HAND_RESP_SUCCESS:
            byte_count = len(out)
            err = self.HAND_GetResponse(hand_id, HAND_CMD_GET_UID, self.timeout, out, remote_err)
            if err == HAND_RESP_SUCCESS:
                uid_w0[0] = out[0] | (out[1] << 8) | (out[2] << 16) | (out[3] << 24)
                uid_w1[0] = out[4] | (out[5] << 8) | (out[6] << 16) | (out[7] << 24)
                uid_w2[0] = out[8] | (out[9] << 8) | (out[10] << 16) | (out[11] << 24)
        return err, uid_w0[0], uid_w1[0], uid_w2[0]

    def HAND_GetBatteryVoltage(self, hand_id, voltage, remote_err):
        out = bytearray(2)
        err = self.HAND_SendCmd(hand_id, HAND_CMD_GET_BATTERY_VOLTAGE, None, 0)
        if err == HAND_RESP_SUCCESS:
            byte_count = len(out)
            err = self.HAND_GetResponse(hand_id, HAND_CMD_GET_BATTERY_VOLTAGE, self.timeout, out, remote_err)
            if err == HAND_RESP_SUCCESS:
                voltage[0] = out[0] | (out[1] << 8)
        return err

    def HAND_GetUsageStat(self, hand_id, total_use_time, total_open_times, motor_cnt, remote_err):
        data = bytearray(1)
        data[0] = motor_cnt
        out = bytearray(4 + 4 * MAX_MOTOR_CNT)
        err = self.HAND_SendCmd(hand_id, HAND_CMD_GET_USAGE_STAT, data, len(data))
        if err == HAND_RESP_SUCCESS:
            byte_count = len(out)
            err = self.HAND_GetResponse(hand_id, HAND_CMD_GET_USAGE_STAT, self.timeout, out, remote_err)
            if err == HAND_RESP_SUCCESS:
                total_use_time[0] = out[0] | (out[1] << 8) | (out[2] << 16) | (out[3] << 24)
                for i in range(MAX_MOTOR_CNT):
                    index = 4 + i * 4
                    total_open_times[i] = out[index] | (out[index + 1] << 8) | (out[index + 2] << 16) | (out[index + 3] << 24)
        return err

    def HAND_GetManufactureData(self, hand_id, sub_model, hw_revision, serial_number, customer_tag, remote_err):
        out = bytearray(26)
        err = self.HAND_SendCmd(hand_id, HAND_CMD_GET_MANUFACTURE_DATA, None, 0)
        if err == HAND_RESP_SUCCESS:
            err = self.HAND_GetResponse(hand_id, HAND_CMD_GET_MANUFACTURE_DATA, self.timeout, out, remote_err)
            if err == HAND_RESP_SUCCESS:
                sub_model[0] = out[0]
                hw_revision[0] = out[1]
                serial_bytes = out[2:18]
                customer_bytes = out[18:26]

                serial_numbers = [int(byte) for byte in serial_bytes]
                serial_number_str = ''.join(map(str, serial_numbers))
                serial_number[0] = serial_number_str

                customer_numbers = [int(byte) for byte in customer_bytes]
                customer_numbers_str = ''.join(map(str, customer_numbers))
                customer_tag[0] = customer_numbers_str
            else:
                err = HAND_RESP_DATA_INVALID
        return err, sub_model[0], hw_revision[0], serial_number[0], customer_tag[0]

    def HAND_GetFingerSpeedCtrlParams(self, hand_id, brake_distance, accel_distance, speed_ratio, remote_err):
        out = bytearray(2 + 2 + 4)
        err = self.HAND_SendCmd(hand_id, HAND_CMD_GET_SPEED_CTRL_PARAMS, None, 0)
        if err == HAND_RESP_SUCCESS:
            err = self.HAND_GetResponse(hand_id, HAND_CMD_GET_SPEED_CTRL_PARAMS, self.timeout, out, remote_err)
            if err == HAND_RESP_SUCCESS:
                brake_distance[0] = out[0] | (out[1] << 8)
                accel_distance[0] = out[2] | (out[3] << 8)
                speed_ratio[0] = struct.unpack('f', out[4:8])
            else:
                err = HAND_RESP_DATA_INVALID

        return err, brake_distance[0], accel_distance[0], speed_ratio[0][0]

    def HAND_Reset(self, hand_id, mode, remote_err):
        if not match_data_type(mode, UINT8_T):
            return HAND_RESP_DATA_INVALID

        data = bytearray(1)
        data[0] = mode
        err = self.HAND_SendCmd(hand_id, HAND_CMD_RESET, data, len(data))
        if err == HAND_RESP_SUCCESS:
            err = self.HAND_GetResponse(hand_id, HAND_CMD_RESET, self.timeout, None, remote_err)
        return err

    def HAND_PowerOff(self, hand_id, remote_err):
        err = self.HAND_SendCmd(hand_id, HAND_CMD_POWER_OFF, None, 0)
        if err == HAND_RESP_SUCCESS:
            err = self.HAND_GetResponse(hand_id, HAND_CMD_POWER_OFF, self.timeout, None, remote_err)
        return err

    def HAND_SetID(self, hand_id, new_id, remote_err):
        if not match_data_type(new_id, UINT8_T):
            return HAND_RESP_DATA_INVALID

        data = bytearray(1)
        data[0] = new_id
        err = self.HAND_SendCmd(hand_id, HAND_CMD_SET_NODE_ID, data, len(data))
        if err == HAND_RESP_SUCCESS:
            err = self.HAND_GetResponse(hand_id, HAND_CMD_SET_NODE_ID, self.timeout, None, remote_err)
        return err

    def HAND_Calibrate(self, hand_id, key, remote_err):
        data = bytearray(2)
        data[0] = key & 0xFF
        data[1] = (key >> 8) & 0xFF
        err = self.HAND_SendCmd(hand_id, HAND_CMD_CALIBRATE, data, len(data))
        if err == HAND_RESP_SUCCESS:
            err = self.HAND_GetResponse(hand_id, HAND_CMD_CALIBRATE, self.timeout, None, remote_err)
        return err

    def HAND_SetCaliData(self, hand_id, end_pos, start_pos, motor_cnt, thumb_root_pos, thumb_root_pos_cnt, remote_err):
        if not match_data_type(motor_cnt, UINT8_T) or not match_data_type(thumb_root_pos_cnt, UINT8_T):
            return HAND_RESP_DATA_INVALID

        data = bytearray(1 + 2 * MAX_MOTOR_CNT + 2 * MAX_MOTOR_CNT + 1 + 2 * MAX_THUMB_ROOT_POS)
        p_data = 0
        data[p_data] = motor_cnt
        p_data += 1

        for i in range(motor_cnt):
            data[p_data] = end_pos[i] & 0xFF
            data[p_data + 1] = (end_pos[i] >> 8) & 0xFF
            p_data += 2

        for i in range(motor_cnt):
            data[p_data] = start_pos[i] & 0xFF
            data[p_data + 1] = (start_pos[i] >> 8) & 0xFF
            p_data += 2

        data[p_data] = thumb_root_pos_cnt
        p_data += 1

        for i in range(thumb_root_pos_cnt):
            data[p_data] = thumb_root_pos[i] & 0xFF
            data[p_data + 1] = (thumb_root_pos[i] >> 8) & 0xFF
            p_data += 2

        err = self.HAND_SendCmd(hand_id, HAND_CMD_SET_CALI_DATA, data, p_data)
        if err == HAND_RESP_SUCCESS:
            err = self.HAND_GetResponse(hand_id, HAND_CMD_SET_CALI_DATA, self.timeout, None, remote_err)
        return err

    def HAND_SetFingerPID(self, hand_id, finger_id, p, i, d, g, remote_err):
        if not match_data_type(finger_id, UINT8_T):
            return HAND_RESP_DATA_INVALID

        data = bytearray(1 + 4 + 4 + 4 + 4)
        data[0] = finger_id
        data[1:5] =   struct.pack('f', p)
        data[5:9] =   struct.pack('f', i)
        data[9:13] =  struct.pack('f', d)
        data[13:17] = struct.pack('f', g)

        err = self.HAND_SendCmd(hand_id, HAND_CMD_SET_FINGER_PID, data, len(data))
        if err == HAND_RESP_SUCCESS:
            err = self.HAND_GetResponse(hand_id, HAND_CMD_SET_FINGER_PID, self.timeout, None, remote_err)
        return err

    def HAND_SetFingerCurrentLimit(self, hand_id, finger_id, current_limit, remote_err):
        if not match_data_type(finger_id, UINT8_T) or not match_data_type(current_limit, UINT16_T):
            return HAND_RESP_DATA_INVALID

        data = bytearray(3)
        data[0] = finger_id
        data[1] = current_limit & 0xFF
        data[2] = (current_limit >> 8) & 0xFF

        err = self.HAND_SendCmd(hand_id, HAND_CMD_SET_FINGER_CURRENT_LIMIT, data, len(data))
        if err == HAND_RESP_SUCCESS:
            err = self.HAND_GetResponse(hand_id, HAND_CMD_SET_FINGER_CURRENT_LIMIT, self.timeout, None, remote_err)
        return err

    def HAND_SetFingerForceTarget(self, hand_id, finger_id, force_limit, remote_err):
        if not match_data_type(finger_id, UINT8_T) or not match_data_type(force_limit, UINT16_T):
            return HAND_RESP_DATA_INVALID

        data = bytearray(3)
        data[0] = finger_id
        data[1] = force_limit & 0xFF
        data[2] = (force_limit >> 8) & 0xFF

        err = self.HAND_SendCmd(hand_id, HAND_CMD_SET_FINGER_FORCE_TARGET, data, len(data))
        if err == HAND_RESP_SUCCESS:
            err = self.HAND_GetResponse(hand_id, HAND_CMD_SET_FINGER_FORCE_TARGET, self.timeout, None, remote_err)
        return err

    def HAND_SetFingerPosLimit(self, hand_id, finger_id, pos_limit_low, pos_limit_high, remote_err):
        if (
            not match_data_type(finger_id, UINT8_T)
            or not match_data_type(pos_limit_low, UINT16_T)
            or not match_data_type(pos_limit_high, UINT16_T)
            or pos_limit_low > pos_limit_high
        ):
            return HAND_RESP_DATA_INVALID

        data = bytearray(5)
        data[0] = finger_id
        data[1] = pos_limit_low & 0xFF
        data[2] = (pos_limit_low >> 8) & 0xFF
        data[3] = pos_limit_high & 0xFF
        data[4] = (pos_limit_high >> 8) & 0xFF

        err = self.HAND_SendCmd(hand_id, HAND_CMD_SET_FINGER_POS_LIMIT, data, len(data))
        if err == HAND_RESP_SUCCESS:
            err = self.HAND_GetResponse(hand_id, HAND_CMD_SET_FINGER_POS_LIMIT, self.timeout, None, remote_err)
        return err

    def HAND_FingerStart(self, hand_id, finger_id_bits, remote_err):
        if not match_data_type(finger_id_bits, UINT8_T):
            return HAND_RESP_DATA_INVALID

        data = bytearray(1)
        data[0] = finger_id_bits

        err = self.HAND_SendCmd(hand_id, HAND_CMD_FINGER_START, data, len(data))
        if err == HAND_RESP_SUCCESS:
            err = self.HAND_GetResponse(hand_id, HAND_CMD_FINGER_START, self.timeout, None, remote_err)
        return err

    def HAND_FingerStop(self, hand_id, finger_id_bits, remote_err):
        if not match_data_type(finger_id_bits, UINT8_T):
            return HAND_RESP_DATA_INVALID

        data = bytearray(1)
        data[0] = finger_id_bits

        err = self.HAND_SendCmd(hand_id, HAND_CMD_FINGER_STOP, data, len(data))
        if err == HAND_RESP_SUCCESS:
            err = self.HAND_GetResponse(hand_id, HAND_CMD_FINGER_STOP, self.timeout, None, remote_err)
        return err

    def HAND_SetFingerPosAbs(self, hand_id, finger_id, raw_pos, speed, remote_err):
        if (
            not match_data_type(finger_id, UINT8_T)
            or not match_data_type(raw_pos, UINT16_T)
            or not match_data_type(speed, UINT8_T)
        ):
            return HAND_RESP_DATA_INVALID

        data = bytearray(4)
        data[0] = finger_id
        data[1] = raw_pos & 0xFF
        data[2] = (raw_pos >> 8) & 0xFF
        data[3] = speed

        err = self.HAND_SendCmd(hand_id, HAND_CMD_SET_FINGER_POS_ABS, data, len(data))
        if err == HAND_RESP_SUCCESS:
            err = self.HAND_GetResponse(hand_id, HAND_CMD_SET_FINGER_POS_ABS, self.timeout, None, remote_err)
        return err

    def HAND_SetFingerPos(self, hand_id, finger_id, pos, speed, remote_err):
        if not match_data_type(finger_id, UINT8_T) or not match_data_type(pos, UINT16_T) or not match_data_type(speed, UINT8_T):
            return HAND_RESP_DATA_INVALID

        data = bytearray(4)
        data[0] = finger_id
        data[1] = pos & 0xFF
        data[2] = (pos >> 8) & 0xFF
        data[3] = speed

        err = self.HAND_SendCmd(hand_id, HAND_CMD_SET_FINGER_POS, data, len(data))
        if err == HAND_RESP_SUCCESS:
            err = self.HAND_GetResponse(hand_id, HAND_CMD_SET_FINGER_POS, self.timeout, None, remote_err)
        return err

    def HAND_SetFingerAngle(self, hand_id, finger_id, angle, speed, remote_err):
        if not match_data_type(finger_id, UINT8_T) or not match_data_type(angle, INT16_T) or not match_data_type(speed, UINT8_T):
            return HAND_RESP_DATA_INVALID

        data = bytearray(4)
        data[0] = finger_id
        data[1] = angle & 0xFF
        data[2] = (angle >> 8) & 0xFF
        data[3] = speed

        err = self.HAND_SendCmd(hand_id, HAND_CMD_SET_FINGER_ANGLE, data, len(data))
        if err == HAND_RESP_SUCCESS:
            err = self.HAND_GetResponse(hand_id, HAND_CMD_SET_FINGER_ANGLE, self.timeout, None, remote_err)
        return err

    def HAND_SetThumbRootPos(self, hand_id, pos, speed, remote_err):
        if not match_data_type(pos, UINT8_T) or not match_data_type(speed, UINT8_T):
            return HAND_RESP_DATA_INVALID

        data = bytearray(2)
        data[0] = pos
        data[1] = speed

        err = self.HAND_SendCmd(hand_id, HAND_CMD_SET_THUMB_ROOT_POS, data, len(data))
        if err == HAND_RESP_SUCCESS:
            err = self.HAND_GetResponse(hand_id, HAND_CMD_SET_THUMB_ROOT_POS, self.timeout, None, remote_err)
        return err

    def HAND_SetFingerPosAbsAll(self, hand_id, raw_pos, speed, motor_cnt, remote_err):
        if (
            not match_list_type(raw_pos, UINT16_T)
            or not match_list_type(speed, UINT8_T)
            or not match_data_type(motor_cnt, UINT8_T)
            or motor_cnt > MAX_MOTOR_CNT
        ):
            return HAND_RESP_DATA_INVALID

        data = bytearray(3 * motor_cnt)
        p_data = 0
        for i in range(motor_cnt):
            data[p_data] = raw_pos[i] & 0xFF
            data[p_data + 1] = (raw_pos[i] >> 8) & 0xFF
            data[p_data + 2] = speed[i]
            p_data += 3

        err = self.HAND_SendCmd(hand_id, HAND_CMD_SET_FINGER_POS_ABS_ALL, data, p_data)
        if err == HAND_RESP_SUCCESS:
            err = self.HAND_GetResponse(hand_id, HAND_CMD_SET_FINGER_POS_ABS_ALL, self.timeout, None, remote_err)
        return err

    def HAND_SetFingerPosAll(self, hand_id, pos, speed, motor_cnt, remote_err):
        if (
            not match_list_type(pos, UINT16_T)
            or not match_list_type(speed, UINT8_T)
            or not match_data_type(motor_cnt, UINT8_T)
            or motor_cnt > MAX_MOTOR_CNT
        ):
            return HAND_RESP_DATA_INVALID

        data = bytearray(3 * motor_cnt)
        p_data = 0
        for i in range(motor_cnt):
            data[p_data] = pos[i] & 0xFF
            data[p_data + 1] = (pos[i] >> 8) & 0xFF
            data[p_data + 2] = speed[i]
            p_data += 3

        err = self.HAND_SendCmd(hand_id, HAND_CMD_SET_FINGER_POS_ALL, data, p_data)
        if err == HAND_RESP_SUCCESS:
            err = self.HAND_GetResponse(hand_id, HAND_CMD_SET_FINGER_POS_ALL, self.timeout, None, remote_err)
        return err

    def HAND_SetFingerAngleAll(self, hand_id, angle, speed, motor_cnt, remote_err):
        if (
            not match_list_type(angle, INT16_T)
            or not match_list_type(speed, UINT8_T)
            or not match_data_type(motor_cnt, UINT8_T)
            or motor_cnt > MAX_MOTOR_CNT
        ):
            return HAND_RESP_DATA_INVALID

        data = bytearray(3 * motor_cnt)
        p_data = 0
        for i in range(motor_cnt):
            data[p_data] = angle[i] & 0xFF
            data[p_data + 1] = (angle[i] >> 8) & 0xFF
            data[p_data + 2] = speed[i]
            p_data += 3

        err = self.HAND_SendCmd(hand_id, HAND_CMD_SET_FINGER_ANGLE_ALL, data, p_data)
        if err == HAND_RESP_SUCCESS:
            err = self.HAND_GetResponse(hand_id, HAND_CMD_SET_FINGER_ANGLE_ALL, self.timeout, None, remote_err)
        return err

    def HAND_SetFingerStopParams(self, hand_id, finger_id, speed, stop_current, stop_after_period, retry_interval, remote_err):
        if (
            not match_data_type(finger_id, UINT8_T)
            or not match_data_type(speed, UINT16_T)
            or not match_data_type(stop_current, UINT16_T)
            or not match_data_type(stop_after_period, UINT16_T)
            or not match_data_type(retry_interval, UINT16_T)
        ):
            return HAND_RESP_DATA_INVALID

        data = bytearray(1 + 2 + 2 + 2 + 2)
        data[0] = finger_id
        data[1] = speed & 0xFF
        data[2] = (speed >> 8) & 0xFF
        data[3] = stop_current & 0xFF
        data[4] = (stop_current >> 8) & 0xFF
        data[5] = stop_after_period & 0xFF
        data[6] = (stop_after_period >> 8) & 0xFF
        data[7] = retry_interval & 0xFF
        data[8] = (retry_interval >> 8) & 0xFF

        err = self.HAND_SendCmd(hand_id, HAND_CMD_SET_FINGER_STOP_PARAMS, data, len(data))
        if err == HAND_RESP_SUCCESS:
            err = self.HAND_GetResponse(hand_id, HAND_CMD_SET_FINGER_STOP_PARAMS, self.timeout, None, remote_err)
        return err

    def HAND_SetFingerForcePID(self, hand_id, finger_id, p, i, d, g, remote_err):
        if not match_data_type(finger_id, UINT8_T):
            return HAND_RESP_DATA_INVALID
        
        data = bytearray(1 + 4 + 4 + 4 + 4)
        data[0] = finger_id
        data[1:5] =   struct.pack('f', p)
        data[5:9] =   struct.pack('f', i)
        data[9:13] =  struct.pack('f', d)
        data[13:17] = struct.pack('f', g)

        err = self.HAND_SendCmd(hand_id, HAND_CMD_SET_FINGER_FORCE_PID, data, len(data))
        if err == HAND_RESP_SUCCESS:
            err = self.HAND_GetResponse(hand_id, HAND_CMD_SET_FINGER_FORCE_PID, self.timeout, None, remote_err)
        return err

    def HAND_ResetForce(self, hand_id, remote_err):
        err = self.HAND_SendCmd(hand_id, HAND_CMD_RESET_FORCE, None, 0)
        if err == HAND_RESP_SUCCESS:
            err = self.HAND_GetResponse(hand_id, HAND_CMD_RESET_FORCE, self.timeout, None, remote_err)
        return err

    def HAND_SetCustom(self, hand_id, data, send_data_size, remote_err):
        err = self.HAND_SendCmd(hand_id, HAND_CMD_SET_CUSTOM, data, send_data_size)
        if err == HAND_RESP_SUCCESS:
            err = self.HAND_GetResponse(hand_id, HAND_CMD_SET_CUSTOM, self.timeout, data, remote_err)
        return err

    def HAND_SetSelfTestLevel(self, hand_id, self_test_level, remote_err):
        if not match_data_type(self_test_level, UINT8_T):
            return HAND_RESP_DATA_INVALID
        
        data = bytearray(1)
        data[0] = self_test_level

        err = self.HAND_SendCmd(hand_id, HAND_CMD_SET_SELF_TEST_LEVEL, data, len(data))
        if err == HAND_RESP_SUCCESS:
            err = self.HAND_GetResponse(hand_id, HAND_CMD_SET_SELF_TEST_LEVEL, self.timeout, None, remote_err)
        return err

    def HAND_SetBeepSwitch(self, hand_id, beep_on, remote_err):
        if not match_data_type(beep_on, UINT8_T):
            return HAND_RESP_DATA_INVALID
        
        data = bytearray(1)
        data[0] = beep_on

        err = self.HAND_SendCmd(hand_id, HAND_CMD_SET_BEEP_SWITCH, data, len(data))
        if err == HAND_RESP_SUCCESS:
            err = self.HAND_GetResponse(hand_id, HAND_CMD_SET_BEEP_SWITCH, self.timeout, None, remote_err)
        return err

    def HAND_Beep(self, hand_id, duration, remote_err):
        if not match_data_type(duration, UINT16_T):
            return HAND_RESP_DATA_INVALID
        
        data = bytearray(2)
        data[0] = duration & 0xFF
        data[1] = (duration >> 8) & 0xFF

        err = self.HAND_SendCmd(hand_id, HAND_CMD_BEEP, data, len(data))
        if err == HAND_RESP_SUCCESS:
            err = self.HAND_GetResponse(hand_id, HAND_CMD_BEEP, self.timeout, None, remote_err)
        return err

    def HAND_SetButtonPressedCnt(self, hand_id, pressed_cnt, remote_err):
        if not match_data_type(pressed_cnt, UINT8_T):
            return HAND_RESP_DATA_INVALID
        
        data = bytearray(1)
        data[0] = pressed_cnt

        err = self.HAND_SendCmd(hand_id, HAND_CMD_SET_BUTTON_PRESSED_CNT, data, len(data))
        if err == HAND_RESP_SUCCESS:
            err = self.HAND_GetResponse(hand_id, HAND_CMD_SET_BUTTON_PRESSED_CNT, self.timeout, None, remote_err)
        return err

    def HAND_StartInit(self, hand_id, remote_err):
        err = self.HAND_SendCmd(hand_id, HAND_CMD_START_INIT, None, 0)
        if err == HAND_RESP_SUCCESS:
            err = self.HAND_GetResponse(hand_id, HAND_CMD_START_INIT, self.timeout, None, remote_err)
        return err

    def HAND_SetManufactureData(self, hand_id, key, sub_model, hw_revision, serial_number, customer_tag, remote_err):
        data = bytearray(28)
        data[0:2] = key
        data[2] = sub_model
        data[3] = hw_revision
        data[4:20] = serial_number
        data[20:28] = customer_tag

        err = self.HAND_SendCmd(hand_id, HAND_CMD_SET_MANUFACTURE_DATA, data, len(data))
        if err == HAND_RESP_SUCCESS:
            err = self.HAND_GetResponse(hand_id, HAND_CMD_SET_MANUFACTURE_DATA, self.timeout, None, remote_err)
        return err

    def HAND_SetFingerSpeedCtrlParams(self, hand_id, brake_distance, accel_distance, speed_ratio, remote_err):
        if not match_data_type(brake_distance, UINT16_T) or not match_data_type(accel_distance, UINT16_T):
            return HAND_RESP_DATA_INVALID
        
        data = bytearray(8)
        data[0] = brake_distance & 0xFF
        data[1] = brake_distance >> 8 & 0xFF
        data[2] = accel_distance & 0xFF
        data[3] = accel_distance >> 8 & 0xFF
        data[4:8] = struct.pack('f', speed_ratio)

        err = self.HAND_SendCmd(hand_id, HAND_CMD_SET_SPEED_CTRL_PARAMS, data, len(data))
        if err == HAND_RESP_SUCCESS:
            err = self.HAND_GetResponse(hand_id, HAND_CMD_SET_SPEED_CTRL_PARAMS, self.timeout, None, remote_err)
        return err
