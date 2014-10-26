#!/usr/bin/env python

import usb.core
import usb.util

def dds_usb_open(idVendor, idProduct):
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

    for ep_direction in (usb.util.ENDPOINT_IN, usb.util.ENDPOINT_OUT):
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

