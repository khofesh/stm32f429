/*
 * usbd_framework.h
 *
 *  Created on: Oct 28, 2023
 *      Author: fahmad
 */

#ifndef USBD_FRAMEWORK_H_
#define USBD_FRAMEWORK_H_

#include "usbd_driver.h"
#include "usb_device.h"

void usbd_initialize();
void usbd_poll();

#endif /* USBD_FRAMEWORK_H_ */
