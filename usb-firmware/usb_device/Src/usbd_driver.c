/*
 * usbd_driver.c
 *
 *  Created on: Oct 28, 2023
 *      Author: fahmad
 */

#include "usbd_driver.h"

void initialize_gpio_pins()
{
	/* enable the clock for GPIOB */
	SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN);

	/* configure USB pins to work in alternate function mode (10: Alternate function mode)*/
	MODIFY_REG(GPIOB->MODER,
			GPIO_MODER_MODER14 | GPIO_MODER_MODER15,
			_VAL2FLD(GPIO_MODER_MODER14, 0x2) | _VAL2FLD(GPIO_MODER_MODER15, 0x2)
	);

	/**
	 *  set alternate function 12 -
	 *  PB14 (-) ->  OTG_HS_DM (D minus)
	 *  PB15 (+) ->  OTG_HS_DP (D plus??)
	 *
	 *  1100 (0xC): AF12
	 */
	MODIFY_REG(GPIOB->AFR[1],
			GPIO_AFRH_AFSEL14 | GPIO_AFRH_AFSEL15,
			_VAL2FLD(GPIO_AFRH_AFSEL14, 0xC) | _VAL2FLD(GPIO_AFRH_AFSEL15, 0xC)
	);
}

void initialize_core()
{
	/* enable the clock for USB core */
	RCC->AHB1ENR |= RCC_AHB1ENR_OTGHSEN;

	/**
	 * configure the USB core to run in device mode, and to use the embedded
	 * full-speed PHY
	 *
	 * (OTG_HS_GUSBCFG)
	 * Bit 30 FDMOD: Forced peripheral mode
	 * 	0: Normal mode
	 * 	1: Forced peripheral mode
	 * Bit 6 PHYSEL: USB 2.0 high-speed ULPI PHY or USB 1.1 full-speed serial transceiver select
	 * 	0: USB 2.0 high-speed ULPI PHY
	 * 	1: USB 1.1 full-speed serial transceiver
	 * Bits 13:10 TRDT: USB turnaround time
	 * 	Table 213. TRDT values
	 * 		TRDT minimum value = 0x9
	 */
	USB_OTG_HS->GUSBCFG |= USB_OTG_GUSBCFG_FDMOD;
	USB_OTG_HS->GUSBCFG |= USB_OTG_GUSBCFG_PHYSEL;
	MODIFY_REG(USB_OTG_HS->GUSBCFG,
			USB_OTG_GUSBCFG_TRDT,
			_VAL2FLD(USB_OTG_GUSBCFG_TRDT, 0x9)
	);

	/**
	 * configure the device to run in full speed mode
	 *
	 * OTG_HS device configuration register (OTG_HS_DCFG)
	 * Bits 1:0 DSPD: Device speed
	 * 		11: Full speed using internal embedded PHY
	 */
	USB_OTG_HS_DEVICE->DCFG |= (1U<<0);
	USB_OTG_HS_DEVICE->DCFG |= (1U<<1);
//	MODIFY_REG(USB_OTG_HS_DEVICE->DCFG,
//		USB_OTG_DCFG_DSPD,
//		_VAL2FLD(USB_OTG_DCFG_DSPD, 0x03)
//	);

	/**
	 *  enable VBUS sensing device
	 *  Bit 19 VBUSBSEN: Enable the VBUS sensing “B” device
	 */
	USB_OTG_HS->GCCFG |= USB_OTG_GCCFG_VBUSBSEN;

	/* unmask the main USB core interrupt */
	USB_OTG_HS->GINTMSK |= USB_OTG_GINTMSK_USBRST | USB_OTG_GINTMSK_ENUMDNEM |
			USB_OTG_GINTMSK_SOFM | USB_OTG_GINTMSK_USBSUSPM | USB_OTG_GINTMSK_WUIM |
			USB_OTG_GINTMSK_IEPINT | USB_OTG_GINTMSK_RXFLVLM;

	/* clear all pending core interrupts */
	USB_OTG_HS->GINTSTS = 0xFFFFFFFF;
	//WRITE_REG(USB_OTG_HS->GINTSTS, 0xFFFFFFFF);

	/* unmask  USB global interrupt */
	USB_OTG_HS->GAHBCFG |= USB_OTG_GAHBCFG_GINT;


}

/**
 * connect the USB device to the bus
 */
void connect()
{
	/* power the transceivers on */
	USB_OTG_HS->GCCFG |= USB_OTG_GCCFG_PWRDWN;

	/* connect the device to the bus */
	USB_OTG_HS_DEVICE->DCTL &= ~USB_OTG_DCTL_SDIS;
}

/**
 * disconnect the USB device from the bus
 */
void disconnect()
{
	/* disconnects the device from the bus */
	USB_OTG_HS_DEVICE->DCTL |= USB_OTG_DCTL_SDIS;

	/* powers the transceivers off */
	USB_OTG_HS->GCCFG &= ~USB_OTG_GCCFG_PWRDWN;
}
