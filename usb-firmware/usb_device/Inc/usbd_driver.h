/*
 * usbd_driver.h
 *
 *  Created on: Oct 28, 2023
 *      Author: fahmad
 *
 *  USB Device Driver
 */

#ifndef USBD_DRIVER_H_
#define USBD_DRIVER_H_

#include "stm32f4xx.h"
#include "usb_standards.h"
#include "Helpers/logger.h"

#define USB_OTG_HS_GLOBAL ((USB_OTG_GlobalTypeDef *)(USB_OTG_HS_PERIPH_BASE + USB_OTG_GLOBAL_BASE))
#define USB_OTG_HS_DEVICE ((USB_OTG_DeviceTypeDef *)(USB_OTG_HS_PERIPH_BASE + USB_OTG_DEVICE_BASE))
#define USB_OTG_HS_PCGCCTL ((uint32_t *)(USB_OTG_HS_PERIPH_BASE + USB_OTG_PCGCCTL_BASE)) // Power and clock gating control register

/**
 *  Total count of IN or OUT endpoints
 *  reference manual
 *  35.2.3 Peripheral-mode features
 */
#define ENDPOINT_COUNT 6



/**
 * @params endpoint_number The number of the IN endpoint we want to access its registers.
 *
 * reference manual
 * OTG device endpoint-x control register (OTG_HS_DIEPCTLx) (x = 0..5, where x = Endpoint_number)
 * 	Address offset: 0x900 + 0x20 * x
 */
inline static USB_OTG_INEndpointTypeDef * IN_ENDPOINT(uint8_t endpoint_number)
{
	return (USB_OTG_INEndpointTypeDef *)(USB_OTG_HS_PERIPH_BASE + USB_OTG_IN_ENDPOINT_BASE + (endpoint_number * 0x20));
}

/**
 * returns the structure contains the registers of a specific OUT endpoint.
 * @param endpoint_number the number of the OUT endpoint we want to access its registers
 *
 * reference manual
 * OTG_HS device endpoint-x control register (OTG_HS_DOEPCTLx) (x = 1..5, where x = Endpoint_number)
 * 	Address offset for OUT endpoints: 0xB00 + 0x20 * x
 */
inline static USB_OTG_OUTEndpointTypeDef * OUT_ENDPOINT(uint8_t endpoint_number)
{
	return (USB_OTG_OUTEndpointTypeDef *)(USB_OTG_HS_PERIPH_BASE + USB_OTG_OUT_ENDPOINT_BASE + (endpoint_number * 0x20));
}

inline static __IO uint32_t *FIFO(uint8_t endpoint_number)
{
	return (__IO uint32_t *)(USB_OTG_HS_PERIPH_BASE + USB_OTG_FIFO_BASE + (endpoint_number * 0x1000));
}

/* USB driver functions exposed to USB framework */
typedef struct {
	void (*initialize_core)();
	void (*initialize_gpio_pins)();
	void (*connect)();
	void (*disconnect)();
	void (*flush_rxfifo)();
	void (*flush_txfifo)(uint8_t endpoint_number);
	void (*configure_in_endpoint)(uint8_t endpoint_number, enum UsbEndpointType endpoint_type, uint16_t endpoint_size);
	void (*read_packet)(void const *buffer, uint16_t size);
	void (*write_packet)(uint8_t endpoint_number, void const *buffer, uint16_t size);
	void (*poll)();
	void (*set_device_address)(uint8_t address);
} UsbDriver;

extern const UsbDriver usb_driver;
extern UsbEvents usb_events;

#endif /* USBD_DRIVER_H_ */
