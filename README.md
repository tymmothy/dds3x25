dds3x25
=======

Python Library for Hantek DDS-3X25 Arbitrary Waveform Generators
    Code initially started by Tymm Twillman on eevblog forums, but paused
    for a long time due to time crunches shortly after starting.  A lot of
    work was done by Domas Jokubauskis and uploaded to a repo at
    https://bitbucket.org/kuzavas/dds3x25/src -- from where it was picked
    back up by Tymm and more work put in.

    This is still very alpha code, and much of it still not well formed,
    plenty of things that still need fleshing out for usefulness,
    and undoubtedly plenty of bugs.

Installation:
    pip install dds3x25

On Linux, to use this interface as a regular user:
    Move 91-hantek_dds.rules to /etc/udev/rules.d/ and run 'service udev restart'.
    Reconnect the device. It should work without root rights.


Protocol reference:
    http://sigrok.org/wiki/Hantek_DDS-3X25


Basics:

    In [1]: import dds3x25

    In [2]: d = dds3x25.DDS()

    # Generate a 200 KHz sine wave
    In [3]: d.generate_sine(200000)

    # Read all digital inputs (in this case lines I4 and I5 are high)
    In [4]: d.digital_read_port()
    Out[4]: 48

    # Set digital output O6 high
    In [5]: d.digital_write(6, 1)
