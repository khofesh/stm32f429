/*
 * usb_standards.h
 *
 *  Created on: Oct 29, 2023
 *      Author: fahmad
 */

#ifndef USB_STANDARDS_H_
#define USB_STANDARDS_H_

typedef enum UsbEndpointType
{
	USB_ENDPOINT_TYPE_CONTROL,
	USB_ENDPOINT_TYPE_ISOCHRONOUS,
	USB_ENDPOINT_TYPE_BULK,
	USB_ENDPOINT_TYPE_INTERRUPT
} UsbEndpointType;

typedef struct {
	void (*on_usb_reset_received)();
	void (*on_setup_data_received)(uint8_t endpoint_number, uint16_t bcnt);
	void (*on_out_data_received)(uint8_t endpoint_number, uint16_t bcnt);
	void (*on_in_transfer_completed)(uint8_t endpoint_number);
	void (*on_out_transfer_completed)(uint8_t endpoint_number);
	void (*on_usb_polled)();
} UsbEvents;

typedef enum
{
	USB_DEVICE_STATE_DEFAULT,
	USB_DEVICE_STATE_ADDRESSED,
	USB_DEVICE_STATE_CONFIGURED,
	USB_DEVICE_STATE_SUSPENDED
} UsbDeviceState;

typedef enum
{
	USB_CONTROL_STAGE_SETUP, // USB_CONTROL_STAGE_IDLE
	USB_CONTROL_STAGE_DATA_OUT,
	USB_CONTROL_STAGE_DATA_IN,
	USB_CONTROL_STAGE_DATA_IN_IDLE,
	USB_CONTROL_STAGE_DATA_IN_ZERO,
	USB_CONTROL_STAGE_STATUS_OUT,
	USB_CONTROL_STAGE_STATUS_IN
} UsbControlTransferStage;


#endif /* USB_STANDARDS_H_ */
