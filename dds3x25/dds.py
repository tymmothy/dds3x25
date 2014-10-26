#!/usr/bin/env python
"""
This is an interface library for Hantek DDS-3X25 arbitrary waveform generator.

Licenced LGPL2+
Copyright (C) 2013 Domas Jokubauskis (domas@jokubauskis.lt)
Copyright (C) 2014 Tymm Twillman (tymmothy@gmail.com)
"""

import struct
import math
import collections

# dds3x25 imports...
from usb_interface import *
from packet import *


def samplepoint_encode(value):
    SIGN_BIT = (1 << 11)
    encoded = abs(value)

    if encoded > DDS.MAX_POINT_VALUE:
        msg = "Value {0} is out of range ({1}-{2})".format(value, -DDS.MAX_POINT_VALUE, DDS.MAX_POINT_VALUE)
        raise ValueError(msg)

    # Note: 0 is negative value
    if value > 0:
        encoded = (DDS.MAX_POINT_VALUE + 1) - encoded
    else:
        encoded = encoded | SIGN_BIT

    return struct.pack("<H", encoded)


def samplepoint_chunks(data):
    """Cut samplepoint data into 32-point chunks.
       If necessary, add padding to the last chunk to make it 64 bytes.
    """
    SAMPLEPOINT_CHUNK_SIZE=32

    for i in xrange(0, len(data), SAMPLEPOINT_CHUNK_SIZE):
        chunkdata = data[i:i+SAMPLEPOINT_CHUNK_SIZE]
        chunk = "".join([ samplepoint_encode(x) for x in chunkdata ])

        if len(chunk) < SAMPLEPOINT_CHUNK_SIZE * 2:
            chunk += "\x91\x1c" * ((SAMPLEPOINT_CHUNK_SIZE - (len(chunk) / 2)))

        yield chunk


class DDS(object):
    # Hantek 3x25 USB Vendor & Product IDs
    USB_VID = 0x0483
    USB_PID = 0x5721

    # Core DAC clock -> 200 MHz
    DAC_CLOCK = int(200e6)

    # Maximum DAC clock divider
    DAC_CLOCK_DIV_MAX = 131070

    # Maximum # of sample points
    MAX_POINTS = 4095

    # Maximum value of a point
    MAX_POINT_VALUE = (1 << 11) - 1

    NUM_DIGITAL_OUTPUTS = 12
    NUM_DIGITAL_INPUTS =   6

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

        self._in_ep, self._out_ep = dds_usb_open(idVendor, idProduct)

        self.configure(**kwargs)

    def transfer(self, data):
        self._out_ep.write(data)
        return self._in_ep.read(self._in_ep.wMaxPacketSize)

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

        configure_packet = ConfigurePacket(self, reset_trigger=reset_trigger, reset_counter=reset_counter)
        response = self.transfer(str(configure_packet))
        response = self._parse_configure_packet_response(response)

        return response

    def _parse_configure_packet_response(self, packet):
        vals = struct.unpack("<HII", packet)

        return {
            'digital_input' : vals[0],
            'frequency' : vals[1] * 2 if self._counter_mode is False else None,
            'ticks' : None if vals[2] == 0xffffffff else vals[2],
            'counts' : vals[1] if self._counter_mode is True else None,
        }

    def set_waveform(self, points, clock_divider=None, shift_points=0):
        count = len(points)

        if shift_points:
            points = collections.deque(points)
            points.rotate(shift_points)

        response = self.transfer(str(PointCountPacket(count, is_start=True)))
        assert response[0] == 0xcc

        for chunk in samplepoint_chunks(points):
            response = self.transfer(chunk)
            assert response[0] == 0xcc

        response = self.transfer(str(PointCountPacket(count)))
        assert response[0] == 0xcc

        if clock_divider is not None:
            self.configure(clock_divider=clock_divider)

    def reset_counter(self):
        """Reset the 3x25 counter state."""
        self.configure(reset_counter=True)

    def reset_trigger(self):
        """Reset the 3x25 external trigger."""
        self.configure(reset_trigger=True)

    def digital_write(self, pin, pin_state):
        """Set the output state of a digital output pin.

        Args:
           pin (int): Number of pin to control.
           pin_state (int/bool): If 1/True, pin will be set high.
                             If 0/False, pin will be set low.
        """
        pin_state = 1 if pin_state else 0
        digital_output = self._digital_output & ~(1 << pin)
        digital_output |= (pin_state << pin)
        self.configure(digital_output=digital_output)

    def digital_write_port(self, pin_states):
        """Set the output states of all digital output pins.

        Args:
           pin_states (int): Value comprised of bits to write to
        the digital output pins.
        """
        self.configure(digital_output=val)

    def digital_read(self, pin):
        """Read the state of a digital input pin.

        Args:
           pin (int): Input pin # to read.

        Returns:
           0 if the pin is low, 1 if the pin is high.
        """
        digital_in = self.configure()['digital_input']
        return 1 if (digital_in & (1 << pin)) else 0

    def digital_read_port(self):
        """Read the state of all input pins as one integer value.

        Returns:
           Integer w/bits set to the states of the input pins.
        """
        return self.configure()['digital_input']

    def count_in_frequency(self):
        """Get the input frequency at the 3x25's COUNT IN port.

        The frequency is only available when the 3x25 is NOT in counter mode.

        Returns:
           Frequency (in Hz) at the COUNT IN port, or None if in counter mode.
        """
        return self.configure()['frequency']

    def count_in_counts(self):
        """Get the # of pulses counted at the 3x25's COUNT IN port since last reset.

        The count is only available when the 3x25 IS in counter mode.
        use .reset_counter() to reset the value to 0.

        Returns:
           # of pulses counted at the COUNT IN port, or None if not in counter mode.
        """
        return self.configure()['counts']

    def count_in_ticks(self):
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
        # Calculate divisor based on using max # of available samples possible.
        # -- ceil( DAC_CLOCK / (frequency * MAX_POINTS) )
        freq = int(freq)
        div = (DDS.DAC_CLOCK + (freq - 1) * DDS.MAX_POINTS) / (freq * DDS.MAX_POINTS)

        # Adjust if odd value -- divisor has to be 1 or a multiple of 2
        if div > 1 and div & 1:
            div += 1

        # Calculate # of sample points to use w/this divider to get closest
        # to requested frequency
        # -- round( DAC_CLOCK / (divider * frequency) )
        npoints = (DDS.DAC_CLOCK + (div * freq / 2)) / (div * freq)

        # Calculate actual frequency
        actual = (DDS.DAC_CLOCK / div) / npoints

        return (npoints, div, actual)

    def generate_sine(self, freq, amplitude=(1<<11)-1, offset=0, phase=0.0, shift=0):
        phase = float(phase)
        npoints, div, actual = DDS.points_and_div_for_freq(freq)
        points = []
        for i in range(npoints):
            i = float(i)
            point = (amplitude * math.sin((2.0 * math.pi * i / npoints) + phase)) + offset
            points.append(int(point))

        self.set_waveform(points, clock_divider=div, shift_points=shift)
        return actual

    def generate_square(self, freq, duty_cycle=0.5, amplitude=(1<<11)-1, offset=0, phase=0.0, shift=0):
        phase = float(phase)
        npoints, div, actual = DDS.points_and_div_for_freq(freq)
        points = []
        for i in range(npoints):
            shifted = int(i + (phase * npoints) / (2.0 * math.pi)) % npoints
            point = amplitude if shifted < (duty_cycle * npoints) else -amplitude
            points.append(int(point + offset))

        self.set_waveform(points, clock_divider=div, shift_points=shift)
        return actual


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

