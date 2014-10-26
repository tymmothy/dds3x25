from distutils.core import setup
import sys

setup(name='dds3x25',
      version='0.1',
      author='Tymm Twillman, Domas Jokubauskis',
      author_email='tymm@miselu.com, domas@jokubauskis.lt',
      description='Interface library for Hantek DDS-3X25 arbitrary waveform generator',
      classifiers=[
          'Development Status :: 3 - Alpha',
          'Programming Language :: Python',
          'Intended Audience :: Science/Research',
          'License :: OSI Approved :: GNU Lesser General Public License v2 or later (LGPLv2+)',
      ],
      license='LGPL2+',
      requires=[
          'usb'
      ],
      provides=[
          'dds3x25',
      ],
      packages=['dds3x25'],
      )

