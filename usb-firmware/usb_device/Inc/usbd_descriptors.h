/*
 * usbd_descriptors.h
 *
 *  Created on: Nov 8, 2023
 *      Author: fahmad
 */

#ifndef USBD_DESCRIPTORS_H_
#define USBD_DESCRIPTORS_H_

#include "usb_standards.h"

const UsbDeviceDescriptor device_descriptor = {
    .bLength            = sizeof(UsbDeviceDescriptor),
    .bDescriptorType    = USB_DESCRIPTOR_TYPE_DEVICE,
    .bcdUSB             = 0x0200, // follows -> 0xJJMN -> JJ: major version, M: minor version, N: sub-minor version
    .bDeviceClass       = USB_CLASS_PER_INTERFACE,
    .bDeviceSubClass    = USB_SUBCLASS_NONE,
    .bDeviceProtocol    = USB_PROTOCOL_NONE,
    .bMaxPacketSize0    = 8,
    .idVendor           = 0x6666,
    .idProduct          = 0x13AA,
    .bcdDevice          = 0x0100,
    .iManufacturer      = 0,
    .iProduct           = 0,
    .iSerialNumber      = 0,
    .bNumConfigurations = 1,
};

#endif /* USBD_DESCRIPTORS_H_ */
