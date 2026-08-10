# !/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys

current_dir = os.path.dirname(os.path.realpath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

from lib_gforce import gforce
from lib_gforce.gforce import EmgRawDataConfig, SampleResolution


# Device filters
DEV_NAME_PREFIX = "gForceBLE"
DEV_MIN_RSSI = -128

# sample resolution:BITS_8 or BITS_12
SAMPLE_RESOLUTION = 12

# Channel0: thumb, Channel1: index, Channel2: middle, Channel3: ring, Channel4: pinky, Channel5: thumb root
INDEX_CHANNELS = [7, 6, 0, 3, 4, 5]

NUM_FINGERS = 6


class PosInputBleGlove:

    def __init__(self):
        self._gforce_device = gforce.GForce(DEV_NAME_PREFIX, DEV_MIN_RSSI)
        self._emg_data = [0 for _ in range(NUM_FINGERS)]
        self._emg_min = [65535 for _ in range(NUM_FINGERS)]
        self._emg_max = [0 for _ in range(NUM_FINGERS)]
        self._q = None

    def clamp(self, n, smallest, largest):
        return max(smallest, min(n, largest))

    def interpolate(self, n, from_min, from_max, to_min, to_max):
        # 将n从from_min到from_max的范围映射到to_min到to_max的范围
        return (n - from_min) / (from_max - from_min) * (to_max - to_min) + to_min

    async def start(self) -> bool:
        # GForce.connect() may get exception, but we just ignore for gloves
        try:
            await self._gforce_device.connect()
        except Exception as e:
            print(e)

        if self._gforce_device.client == None or not self._gforce_device.client.is_connected:
            exit(-1)

        print("Connected to {0}".format(self._gforce_device.device_name))

        # Set the EMG raw data configuration, default configuration is 8 bits, 16 batch_len
        if SAMPLE_RESOLUTION == 12:
            cfg = EmgRawDataConfig(fs=100, channel_mask=0xFF, batch_len=48, resolution=SampleResolution.BITS_12)
            await self._gforce_device.set_emg_raw_data_config(cfg)

        baterry_level = await self._gforce_device.get_battery_level()
        print("电池电量: {0}%\nDevice baterry level: {0}%".format(baterry_level))

        await self._gforce_device.set_subscription(gforce.DataSubscription.EMG_RAW)
        self._q = await self._gforce_device.start_streaming()

        print("校正模式，请执行握拳和张开动作若干次\nCalibrating mode, please perform a fist and open action several times")

        for _ in range(256):
            v = await self._q.get()
            # print(v)

            emg_sum = [0 for _ in range(NUM_FINGERS)]

            for j in range(len(v)):
                for i in range(NUM_FINGERS):
                    emg_sum[i] += v[j][INDEX_CHANNELS[i]]

            for i in range(NUM_FINGERS):
                temp = emg_sum[i] / len(v)
                self._emg_max[i] = max(self._emg_max[i], temp)
                self._emg_min[i] = min(self._emg_min[i], temp)

            # print(emg_min, emg_max)

        range_valid = True

        for i in range(NUM_FINGERS):
            print("MIN/MAX of finger {0}: {1}-{2}".format(i, self._emg_min[i], self._emg_max[i]))
            if self._emg_min[i] >= self._emg_max[i]:
                range_valid = False

        return range_valid

    async def get_position(self):
        while True:
            v = await self._q.get()
            if self._q.empty():
                break

        # print(v)

        finger_data = [0 for _ in range(NUM_FINGERS)]
        emg_sum = [0 for _ in range(NUM_FINGERS)]

        for j in range(len(v)):
            for i in range(NUM_FINGERS):
                emg_sum[i] += v[j][INDEX_CHANNELS[i]]

        for i in range(NUM_FINGERS):
            self._emg_data[i] = (self._emg_data[i] * 3 + emg_sum[i] / len(v)) / 4

            finger_data[i] = round(self.interpolate(self._emg_data[i], self._emg_min[i], self._emg_max[i], 65535, 0))
            finger_data[i] = self.clamp(finger_data[i], 0, 65535)

        return finger_data

    async def stop(self):
        await self._gforce_device.stop_streaming()
        await self._gforce_device.disconnect()
        print("Disconnected from {0}".format(self._gforce_device.device_name))
