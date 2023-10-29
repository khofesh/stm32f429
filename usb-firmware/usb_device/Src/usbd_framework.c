/*
 * usbd_framework.c
 *
 *  Created on: Oct 28, 2023
 *      Author: fahmad
 */

#include "usbd_framework.h"

void usbd_initialize()
{
	initialize_gpio_pins();
	initialize_core();
	connect();
}
