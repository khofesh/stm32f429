/*
 * usbd_framework.c
 *
 *  Created on: Oct 28, 2023
 *      Author: fahmad
 */

#include "usbd_framework.h"

void usbd_initialize()
{
	usb_driver.initialize_gpio_pins();
	usb_driver.initialize_core();
	usb_driver.connect();
}
