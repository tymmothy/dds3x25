#!/usr/bin/env python

import struct

class PointCountPacket(object):
    PACKET_TYPE = 0xa1
    TRANSFER_START_BIT = (1 << 15)
    MAX_POINTS = 4095

    def __init__(self, num_points, is_start=False):
        if num_points > 4095:
            raise ValueError("Number of points cannot exceed {0}.".format(PointCountPacket.MAX_POINTS))

        self.num_points = num_points
        self.is_start = is_start

    def __str__(self):
        value = self.num_points - 1
        if self.is_start: value |= PointCountPacket.TRANSFER_START_BIT
        s = struct.pack("<BH", self.PACKET_TYPE, value)
        return s


class ConfigurePacket(object):
    PACKET_TYPE = 0xa0

    def __init__(self, dds, reset_counter=False, reset_trigger=False):
        b0 = 1 if dds._counter_mode else 0
        b0 |= 2 if reset_counter else 0
        b0 |= 4 if dds._oneshot else 0
        b0 |= 8 if dds._ext_trigger == 1 else 0
        b0 |= 16 if dds._ext_trigger is not None else 0
        b0 |= 32 if reset_trigger else 0
        self._b0 = b0

        self._b1 = 1 if dds._programmable_output else 0

        self._b2_3 = dds._digital_output
        self._b4_5 = dds._clock_divider / 2

    def __str__(self):
        s = struct.pack('<B2BHH', ConfigurePacket.PACKET_TYPE, self._b0, self._b1, self._b2_3, self._b4_5)
        return s


