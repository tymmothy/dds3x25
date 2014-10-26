#!/usr/bin/env python
"""
This is an interface library for Hantek DDS-3X25 arbitrary waveform generator.

What works:
* (unreliable) waveform sending
* clock divider setting
* digital IO pins
* frequency and pulse counter

Requirements:
https://pypi.python.org/pypi/libusb1 >=1.0.0 version

Installation:
Copy the folder to the Python library path or use it in current folder.

NOTE: To use this interface as a regular user:
Move 91-hantek_dds.rules to /etc/udev/rules.d/ and run 'service udev restart'.
Reconnect the device. It should work without root rights.

Licenced LGPL2+
Copyright (C) 2013 Domas Jokubauskis (domas@jokubauskis.lt)

Protocol reference: http://sigrok.org/wiki/Hantek_DDS-3X25
Started by tymm on EEVblog Electronics Community Forum:
http://www.eevblog.com/forum/chat/review-hantek-dds-3x25-anyone-own-one/msg290311/#msg290311
Information for sync output correction:
http://www.soasystem.com/eng/goltek/synch.htm
"""

#TODO: make a system for waveform generation at a reliable frequency
#TODO: save waveform to memory
#TODO: test in Windows

import usb.core
import usb.util
import struct
import math


PACKET_TYPE_CONFIGURE =       0xa0


class SamplePoint(object):
    MAX_AMPLITUDE = (1 << 11) - 1

    SIGN_BIT = (1 << 11)

    # ??? these don't actually seem to have any effect.
    # SYNC_BIT = (1 << 12)
    # BIT13 = (1 << 13)

    def __init__(self, value, sync=False, bit13=True, **kwargs):
        self._point_value = SamplePoint.munge(value)

    def __str__(self):
        return struct.pack("<H", self._point_value)

    def __repr__(self):
        s = "<SamplePoint value={0}".format(SamplePoint.unmunge(self._point_value))
        s += ">"

        return s

    @property
    def value(self):
        return SamplePoint.unmunge(self._point_value)

    @value.setter
    def value(self, value):
        self._point_value = SamplePoint.munge(value)

    @property
    def volts(self):
        return (3.5 * self.value) / SamplePoint.MAX_AMPLITUDE

    @volts.setter
    def volts(self, value):
        value = (volts / 3.5) * SamplePoint.MAX_AMPLITUDE
        self.value = value

    @staticmethod
    def from_volts(volts, **kwargs):
        value = (volts / 3.5) * SamplePoint.MAX_AMPLITUDE
        return SamplePoint(value, **kwargs)

    @staticmethod
    def from_float(value, **kwargs):
        point_value = value * SamplePoint.MAX_AMPLITUDE
        return SamplePoint(value, **kwargs)

    @staticmethod
    def munge(value):
        result = abs(value)

        if result > SamplePoint.MAX_AMPLITUDE:
            msg = "Value {0} is out of range (max {1})".format(value, SamplePoint.MAX_AMPLITUDE)
            raise ValueError(msg)

        # Note: 0 is negative value
        if value > 0:
            result = (SamplePoint.MAX_AMPLITUDE + 1) - result
        else:
            result = result | SamplePoint.SIGN_BIT

        return result

    @staticmethod
    def unmunge(value):
        if value & SamplePoint.SIGN_BIT:
            return -(value & SamplePoint.MAX_AMPLITUDE)
        else:
            return (SamplePoint.MAX_AMPLITUDE + 1) - value


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


def usb_configure(idVendor, idProduct, ep_directions):
    usb_dev = usb.core.find(idVendor=idVendor, idProduct=idProduct)
    if not usb_dev: raise ValueError('Device not found.')

    # set the active configuration. With no arguments, the first
    # configuration will be the active one
    usb_dev.set_configuration()

    # get an endpoint instance
    cfg = usb_dev.get_active_configuration()
    interface_number = cfg[(0,0)].bInterfaceNumber
    alternate_setting = usb.control.get_interface(usb_dev, interface_number)
    intf = usb.util.find_descriptor(              \
        cfg, bInterfaceNumber = interface_number, \
        bAlternateSetting = alternate_setting     \
    )

    endpoints = []

    for ep_direction in ep_directions:
        ep = usb.util.find_descriptor(
             intf,
             # match the first OUT endpoint
             custom_match = \
             lambda e: \
                 usb.util.endpoint_direction(e.bEndpointAddress) == \
                 ep_direction
        )
        assert ep is not None

        endpoints.append(ep)

    return endpoints


def samplepoint_chunks(data):
    """Cut samplepoint data into 64-byte chunks.
       If necessary, add padding to the last chunk to make it 64 bytes.
    """
    SAMPLEPOINT_CHUNK_SIZE=64

    for i in xrange(0, len(data), SAMPLEPOINT_CHUNK_SIZE):
        chunk = data[i:i+SAMPLEPOINT_CHUNK_SIZE]
        if len(chunk) < SAMPLEPOINT_CHUNK_SIZE:
            chunk += "\x91\x1c" * ((SAMPLEPOINT_CHUNK_SIZE - len(chunk)) / 2)

        yield chunk


class DDS(object):
    # Hantek 3x25 USB Vendor & Product IDs
    USB_VID = 0x0483
    USB_PID = 0x5721

    # Core DAC clock -> 200 MHz
    DAC_CLOCK = 200e6

    # Maximum DAC clock divider
    DAC_CLOCK_DIV_MAX = 131070

    # Maximum # of sample points
    MAX_POINTS = 4095

    NUM_DIGITAL_OUTPUTS = 12
    NUM_DIGITAL_INPUTS =   6

    CONFIGURE_PACKET_TYPE = 0xa0

    def __init__(self, idVendor=USB_VID, idProduct=USB_PID, **kwargs):
        """Initialize a DDS instance and connect to the hardware.

        Args:
            idVendor (int):  16-bit USB Vendor ID (VID) for the DDS hardware.
            idProduct (int): 16-bit USB Product ID (PID) for the DDS hardware.

        Kwargs:
            See DDS.configure() for the list of kwargs that __init__ understands.
        """
        # Set up defaults for instance variables.
        self._ext_trigger = None
        self._oneshot = False
        self._counter_mode = False
        self._programmable_output = True
        self._digital_output = 0
        self._clock_divider = 128

        # do not initialize USB device if used for unit testing
        if kwargs.get('testing', False):
            return

        # We want to set up an in endpoint and an out endpoint
        usb_ep_directions = (usb.util.ENDPOINT_IN, usb.util.ENDPOINT_OUT)

        self._in_ep, self._out_ep = usb_configure(idVendor, idProduct, usb_ep_directions)

        self._write = lambda data: self._out_ep.write(data)
        self._read = lambda: self._in_ep.read(self._in_ep.wMaxPacketSize)

        self.configure(**kwargs)

    def configure(self, **kwargs):
        """Update the 3x25's configuration settings.

        Kwargs:
           reset_trig (bool): If True, reset the DDS external trigger.

           reset_counter (bool): If True, reset the DDS counter.

           oneshot (bool): If True, only output one wave (not continuous).

           counter_mode (bool): Set true to enable counter mode.
                If True, the 3x25 counts pulses.
                If False, the 3x25 measures frequency.

           programmable_output (bool): Set true to enable programmable digital output.
                If True, digital output pins are controlled by setting digital_output.
                If False, digital output pins follow the DAC output value.

           ext_trigger ([None, 0 or 1]): Configure external trigger mode.
                If None, external triggering is disabled.
                If 1, external triggering occurs on rising pulse edges.
                If 0, external triggering occurs on falling pulse edges.

           digital_output (int): 12-bit unsigned value whose bits are written
                to the 3x25's digital output pins.
                Note: Only used when programmable_output is enabled.

           clock_divider (int): Divisor to use for 200Mhz DAC clock to generate
                sample output clock.
                Must be an even value from 0-131070
        """
        reset_trigger = bool(kwargs.get('reset_trig', False))
        reset_counter = bool(kwargs.get('reset_counter', False))

        oneshot = bool(kwargs.get('oneshot', self._oneshot))
        counter_mode = bool(kwargs.get('counter_mode', self._counter_mode))
        programmable_output = bool(kwargs.get('programmable_output', self._programmable_output))

        ext_trigger = kwargs.get('ext_trigger', self._ext_trigger)
        if ext_trigger not in [ None, 0, 1 ]:
            raise ValueError("Invalid value for ext_trigger (must be 1, 0 or None)")

        digital_output = int(kwargs.get('digital_output', self._digital_output))
        clock_divider = int(kwargs.get('clock_divider', self._clock_divider))
        if (clock_divider < 1) or (clock_divider > 131070) or (clock_divider > 1 and clock_divider & 1):
            msg = "Clock divider ({0}) must be 1 or an even value between 2 and {1}.".format(clock_divider, DDS.DAC_CLOCK_DIV_MAX)
            raise ValueError(msg)

        self._oneshot = oneshot
        self._counter_mode = counter_mode
        self._programmable_output = programmable_output
        self._ext_trigger = ext_trigger
        self._digital_output = digital_output
        self._clock_divider = clock_divider

        b0 = 1 if self._counter_mode else 0
        b0 |= 2 if reset_counter else 0
        b0 |= 4 if self._oneshot else 0
        b0 |= 8 if self._ext_trigger == 1 else 0
        b0 |= 16 if self._ext_trigger is not None else 0
        b0 |= 32 if reset_trigger else 0

        b1 = 1 if self._programmable_output else 0

        b2_3 = self._digital_output
        b4_5 = self._clock_divider / 2

        config_val = struct.pack('<B2BHH', DDS.CONFIGURE_PACKET_TYPE, b0, b1, b2_3, b4_5)

        self._write(config_val)
        response = self._read()

        response = self._parse_configure_packet_response(response)

        return response

    def _parse_configure_packet_response(self, packet):
        vals = struct.unpack("<HII", packet)

        return {
            'digital_input' : vals[0],
            'frequency' : vals[1] * 2 if self._counter_mode is False else 0,
            'ticks' : vals[2],
            'counts' : vals[1] if self._counter_mode is True else 0,
        }

    def set_waveform(self, points, **kwargs):
        count = len(points)
        point_data = ''.join([ str(SamplePoint(point)) for point in points])

        self._write(str(PointCountPacket(count, is_start=True)))
        assert self._read()[0] == 0xcc

        for chunk in samplepoint_chunks(point_data):
            self._write(chunk)
            assert self._read()[0] == 0xcc

        self._write(str(PointCountPacket(count)))
        assert self._read()[0] == 0xcc

    def reset_counter(self):
        self.configure(reset_counter=True)

    def reset_trigger(self):
        self.configure(reset_trigger=True)

    def digital_write(self, val):
        self.configure(digital_output=val)

    def digital_read(self):
        return self.configure()['digital_input']

    def frequency(self):
        return self.configure()['frequency']

    def counts(self):
        return self.configure()['counts']

    def ticks(self):
        return self.configure()['ticks']

    @property
    def ext_trigger(self):
        return self._ext_trigger

    @ext_trigger.setter
    def ext_trigger(self, trig):
        if trig is not None and trig != 0 and trig != 1:
            raise ValueError("Invalid value for external trigger (should be 1, 0 or None)")
        self.configure(ext_trigger=trig)

    @property
    def oneshot_mode(self):
        return self._oneshot

    @oneshot_mode.setter
    def oneshot_mode(self, val):
        val = True if val else False
        self.configure(oneshot=val)

    @property
    def counter_mode(self):
        return self._counter_mode

    @counter_mode.setter
    def counter_mode(self, val):
        val = True if val else False
        self.configure(counter_mode=val)

    @property
    def programmable_output(self):
        return self._programmable_output

    @programmable_output.setter
    def programmable_output(self, val):
        self.configure(programmable_output=val)

    @staticmethod
    def points_and_div_for_freq(freq):
        div = 1
        while True:
            npoints = (DDS.DAC_CLOCK + ((div * freq) / 2)) / (div * freq)
            npoints = int(npoints)
            if npoints <= DDS.MAX_POINTS:
                break
            div += 2 if div != 1 else 1

        return (npoints, div)

    def generate_sine(self, freq, amplitude=(1<<11)-1, offset=0):
        (npoints, div) = DDS.points_and_div_for_freq(freq)
        points = []
        for i in range(npoints):
            i = float(i)
            point = (amplitude * math.sin(2.0 * math.pi * i / npoints)) + offset
            points.append(int(point))

        self.configure(clock_divider=div)
        self.set_waveform(points)

    def generate_square(self, freq, duty_cycle=0.5, amplitude=(1<<11)-1, offset=0):
        (npoints, div) = DDS.points_and_div_for_freq(freq)
        points = []
        for i in range(npoints):
            point = -amplitude if i >= (duty_cycle * npoints) else amplitude
            points.append(int(point + offset))

        self.configure(clock_divider=div)
        self.set_waveform(points)


if __name__ == "__main__":
    import time

    freq = 6000000

    d = DDS()

#    print "Generating square wave @ {0} hz".format(freq)
#    d.generate_square(25000000, 0.50)
#    time.sleep(10)
    print "Generating sine wave @ {0} hz".format(freq)
    d.generate_sine(freq)

    d.programmable_output=True

    d.reset_counter()
    d.counter_mode = True

