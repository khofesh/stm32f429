/*
 * usb_hid_standards.h
 *
 *  Created on: Nov 9, 2023
 *      Author: fahmad
 */

#ifndef HID_USB_HID_STANDARDS_H_
#define HID_USB_HID_STANDARDS_H_

#include "usb_hid.h"
#include "usb_hid_usage_button.h"
#include "usb_hid_usage_desktop.h"

#define USB_DESCRIPTOR_TYPE_HID 0x21
#define USB_DESCRIPTOR_TYPE_HID_REPORT 0x22
#define USB_HID_COUNTRY_NONE 0

/** \brief USB HID descriptor. */
typedef struct {
	uint8_t bLength; /* Size of the descriptor (bytes). */
	uint8_t bDescriptorType; /* Type of the descriptor. */
	uint16_t bcdHID; /* BCD encoded version that the HID descriptor and device complies to. */
	uint8_t bCountryCode; /* Country code of the localized device. */
	uint8_t bNumDescriptors; /* Total number of HID report descriptors for the interface. */
	uint8_t bDescriptorType0; /* First HID report descriptor type. */
	uint16_t wDescriptorLength0; /* First HID report descriptor length (bytes). */
} __attribute__((__packed__)) UsbHidDescriptor;

#endif /* HID_USB_HID_STANDARDS_H_ */
