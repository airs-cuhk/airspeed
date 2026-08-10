from typing import Final


MAX_MOTOR_CNT: Final = 6
MAX_THUMB_ROOT_POS: Final = 3
MAX_FORCE_ENTRIES: Final = 5 * 12

# Constants from the C header file
HAND_PROTOCOL_UART: Final = 0
HAND_PROTOCOL_I2C: Final = 1

# Error codes
ERR_PROTOCOL_WRONG_LRC: Final = 0x01
ERR_COMMAND_INVALID: Final = 0x11
ERR_COMMAND_INVALID_BYTE_COUNT: Final = 0x12
ERR_COMMAND_INVALID_DATA: Final = 0x13
ERR_STATUS_INIT: Final = 0x21
ERR_STATUS_CALI: Final = 0x22
ERR_STATUS_STUCK: Final = 0x23
ERR_OP_FAILED: Final = 0x31
ERR_SAVE_FAILED: Final = 0x32


# API return values
HAND_RESP_HAND_ERROR: Final = 0xFF  # device error, error call back will be called with OHand error codes listed above
HAND_RESP_SUCCESS: Final = 0x00
HAND_RESP_TIMER_FUNC_NOT_SET: Final = 0x01  # local error, timer function not set, call HAND_SetTimerFunction(...) first
HAND_RESP_INVALID_CONTEXT: Final = 0x02  # local error, invalid context, NULL or send data function not set
HAND_RESP_TIMEOUT: Final = 0x03  # local error, time out when waiting node response
HAND_RESP_INVALID_OUT_BUFFER_SIZE: Final = 0x04  # local error, out buffer size not matched to returned data
HAND_RESP_UNMATCHED_ADDR: Final = 0x05  # local error, unmatched node id between returned and waiting
HAND_RESP_UNMATCHED_CMD: Final = 0x06  # local error, unmatched command between returned and waiting
HAND_RESP_DATA_SIZE_TOO_BIG: Final = 0x07  # local error, size of data to send exceeds the buffer size
HAND_RESP_DATA_INVALID: Final = 0x08  # local error, data content invalid

# Sub-commands for HAND_CMD_SET_CUSTOM
SUB_CMD_SET_SPEED: Final = 1 << 0
SUB_CMD_SET_POS: Final = 1 << 1
SUB_CMD_SET_ANGLE: Final = 1 << 2
SUB_CMD_GET_POS: Final = 1 << 3
SUB_CMD_GET_ANGLE: Final = 1 << 4
SUB_CMD_GET_CURRENT: Final = 1 << 5
SUB_CMD_GET_FORCE: Final = 1 << 6
SUB_CMD_GET_STATUS: Final = 1 << 7

# Command definitions

# Chief GET commands
HAND_CMD_GET_PROTOCOL_VERSION: Final = 0x00  # Get protocol version, Please don't modify!
HAND_CMD_GET_FW_VERSION: Final = 0x01  # Get firmware version
HAND_CMD_GET_HW_VERSION: Final = 0x02  # Get hardware version, [HW_TYPE, HW_VER, BOOT_VER_MAJOR, BOOT_VER_MINOR]
HAND_CMD_GET_CALI_DATA: Final = 0x03  # Get calibration data
HAND_CMD_GET_FINGER_PID: Final = 0x04  # Get PID of finger
HAND_CMD_GET_FINGER_CURRENT_LIMIT: Final = 0x05  # Get motor current limit of finger
HAND_CMD_GET_FINGER_CURRENT: Final = 0x06  # Get motor current of finger
HAND_CMD_GET_FINGER_FORCE_TARGET: Final = 0x07  # Get force limit of finger
HAND_CMD_GET_FINGER_FORCE: Final = 0x08  # Get force of finger
HAND_CMD_GET_FINGER_POS_LIMIT: Final = 0x09  # Get absolute position limit of finger
HAND_CMD_GET_FINGER_POS_ABS: Final = 0x0A  # Get current absolute position of finger
HAND_CMD_GET_FINGER_POS: Final = 0x0B  # Get current logical position of finger
HAND_CMD_GET_FINGER_ANGLE: Final = 0x0C  # Get first joint angle of finger
HAND_CMD_GET_THUMB_ROOT_POS: Final = 0x0D  # Get preset position of thumb root, [0, 1, 2, 255], 255 as invalid
HAND_CMD_GET_FINGER_POS_ABS_ALL: Final = 0x0E  # Get current absolute position of all fingers
HAND_CMD_GET_FINGER_POS_ALL: Final = 0x0F  # Get current logical position of all fingers
HAND_CMD_GET_FINGER_ANGLE_ALL: Final = 0x10  # Get first joint angle of all fingers
HAND_CMD_GET_FINGER_STOP_PARAMS: Final = 0x11  # Get finger finger stop parametres
HAND_CMD_GET_FINGER_FORCE_PID: Final = 0x12  # Get finger force PID

# Auxiliary GET commands
HAND_CMD_GET_SELF_TEST_LEVEL: Final = 0x20  # Get self-test level state
HAND_CMD_GET_BEEP_SWITCH: Final = 0x21  # Get beep switch state
HAND_CMD_GET_BUTTON_PRESSED_CNT: Final = 0x22  # Get button press count
HAND_CMD_GET_UID: Final = 0x23  # Get 96 bits UID
HAND_CMD_GET_BATTERY_VOLTAGE: Final = 0x24  # Get battery voltage
HAND_CMD_GET_USAGE_STAT: Final = 0x25  # Get usage stat
HAND_CMD_GET_SPEED_CTRL_PARAMS = 0x3D  # Get speed control parameters
HAND_CMD_GET_MANUFACTURE_DATA: Final = 0x3E    # Get manufacture data

# Chief SET commands
HAND_CMD_RESET: Final = 0x40  # Please don't modify
HAND_CMD_POWER_OFF: Final = 0x41  # Power off
HAND_CMD_SET_NODE_ID: Final = 0x42  # Set node ID
HAND_CMD_CALIBRATE: Final = 0x43  # Recalibrate hand
HAND_CMD_SET_CALI_DATA: Final = 0x44  # Set finger pos range & thumb pos set
HAND_CMD_SET_FINGER_PID: Final = 0x45  # Set PID of finger
HAND_CMD_SET_FINGER_CURRENT_LIMIT: Final = 0x46  # Set motor current limit of finger
HAND_CMD_SET_FINGER_FORCE_TARGET: Final = 0x47  # Set force limit of finger
HAND_CMD_SET_FINGER_POS_LIMIT: Final = 0x48  # Get current absolute position of finger
HAND_CMD_FINGER_START: Final = 0x49  # Start motor
HAND_CMD_FINGER_STOP: Final = 0x4A  # Stop motor
HAND_CMD_SET_FINGER_POS_ABS: Final = 0x4B  # Move finger to physical position, [0, 65535]
HAND_CMD_SET_FINGER_POS: Final = 0x4C  # Move finger to logical position, [0, 65535]
HAND_CMD_SET_FINGER_ANGLE: Final = 0x4D  # Set first joint angle of finger
HAND_CMD_SET_THUMB_ROOT_POS: Final = 0x4E  # Move thumb root to preset position, {0, 1, 2}
HAND_CMD_SET_FINGER_POS_ABS_ALL: Final = 0x4F  # Set current absolute position of all fingers
HAND_CMD_SET_FINGER_POS_ALL: Final = 0x50  # Set current logical position of all fingers
HAND_CMD_SET_FINGER_ANGLE_ALL: Final = 0x51  # Set first joint angle of all fingers
HAND_CMD_SET_FINGER_STOP_PARAMS: Final = 0x52  # Set finger finger stop parametres
HAND_CMD_SET_FINGER_FORCE_PID: Final = 0x53  # Set finger force PID
HAND_CMD_RESET_FORCE: Final = 0x54  # Reset force

HAND_CMD_SET_CUSTOM: Final = 0x5F  # Custom set command

# Auxiliary SET commands
HAND_CMD_SET_SELF_TEST_LEVEL: Final = 0x60  # Set self-test level, level, 0: wait command, 1: semi self-test, 2: full self-test
HAND_CMD_SET_BEEP_SWITCH: Final = 0x61  # Set beep ON/OFF
HAND_CMD_BEEP: Final = 0x62  # Beep for duration if beep switch is on
HAND_CMD_SET_BUTTON_PRESSED_CNT: Final = 0x63  # Set button press count, for ROH calibration only
HAND_CMD_START_INIT: Final = 0x64  # Start init in case of SELF_TEST_LEVEL=0
HAND_CMD_SET_MANUFACTURE_DATA: Final = 0x65  # Set manufacture data
HAND_CMD_SET_SPEED_CTRL_PARAMS = 0x66  # Set speed control parameters
CMD_ERROR_MASK: Final = 1 << 7  # bit mask for command error

MAX_PROTOCOL_DATA_SIZE: Final = 64

# Data type
UINT8_T = 0
INT8_T = 1
UINT16_T = 2
INT16_T = 3