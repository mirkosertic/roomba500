#!/usr/bin/env python3

import smbus
import time
import sys

class HMC5883L:

    __scales = {
        0.88: [0, 0.73],
        1.30: [1, 0.92],
        1.90: [2, 1.22],
        2.50: [3, 1.52],
        4.00: [4, 2.27],
        4.70: [5, 2.56],
        5.60: [6, 3.03],
        8.10: [7, 4.35],
    }

    def __init__(self, port=1, address=0x1E, gauss=1.3):
        self.bus = smbus.SMBus(port)
        self.address = address

        (reg, self.__scale) = self.__scales[gauss]
        self.bus.write_byte_data(self.address, 0x00, 0x70) # 8 Average, 15 Hz, normal measurement
        self.bus.write_byte_data(self.address, 0x01, reg << 5) # Scale
        self.bus.write_byte_data(self.address, 0x02, 0x00) # Continuous measurement

    def twos_complement(self, val, len):
        # Convert twos compliment to integer
        if (val & (1 << len - 1)):
            val = val - (1<<len)
        return val

    def convert(self, val):
        if val == -4096:
            return None;
        return round(val * self.__scale, 4)

    def axes(self):
        # Wait for data available
        data = self.bus.read_byte_data(self.address, 0x09)

        while (data & 0x01 == 0):
            data = self.bus.read_byte_data(self.address, 0x09)

        x = self.convert(self.twos_complement(self.bus.read_byte_data(self.address, 0x03) << 8 | self.bus.read_byte_data(self.address, 0x04), 16))
        z = self.convert(self.twos_complement(self.bus.read_byte_data(self.address, 0x05) << 8 | self.bus.read_byte_data(self.address, 0x06), 16))
        y = self.convert(self.twos_complement(self.bus.read_byte_data(self.address, 0x07) << 8 | self.bus.read_byte_data(self.address, 0x08), 16))

        return (x,y,z)
