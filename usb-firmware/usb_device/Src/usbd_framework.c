/*
 * usbd_framework.c
 *
 *  Created on: Oct 28, 2023
 *      Author: fahmad
 */

#include "usbd_framework.h"

static void usb_reset_received_handler();
static void setup_data_received_handler(uint8_t endpoint_number, uint16_t byte_count);
static void process_request();

static UsbDevice *usbd_handle;

UsbEvents usb_events = {
	.on_usb_reset_received = &usb_reset_received_handler,
	.on_setup_data_received = &setup_data_received_handler
};

void usbd_initialize(UsbDevice *usb_device)
{
	usbd_handle = usb_device;

	usb_driver.initialize_gpio_pins();
	usb_driver.initialize_core();
	usb_driver.connect();
}

void usbd_poll()
{
	usb_driver.poll();
}

static void process_request()
{

}

static void usb_reset_received_handler()
{
	usbd_handle->in_data_size = 0;
	usbd_handle->out_data_size = 0;
	usbd_handle->configuration_value = 0;
	usbd_handle->device_state = USB_DEVICE_STATE_DEFAULT;
	usbd_handle->control_transfer_stage = USB_CONTROL_STAGE_SETUP;
	usb_driver.set_device_address(0);
}

static void setup_data_received_handler(uint8_t endpoint_number, uint16_t byte_count)
{
	usb_driver.read_packet(usbd_handle->ptr_out_buffer, byte_count);

	log_debug_array("setup data: ", usbd_handle->ptr_out_buffer, byte_count);

	process_request();
}


