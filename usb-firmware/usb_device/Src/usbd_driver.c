/*
 * usbd_driver.c
 *
 *  Created on: Oct 28, 2023
 *      Author: fahmad
 */

#include "usbd_driver.h"

static void usbrst_handler();
static void configure_endpoint0(uint8_t endpoint_size);
static void configure_in_endpoint(uint8_t endpoint_number, UsbEndpointType endpoint_type, uint16_t endpoint_size);
static void deconfigure_endpoint(uint8_t endpoint_number);

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
			USB_OTG_GINTMSK_IEPINT | USB_OTG_GINTMSK_RXFLVLM | USB_OTG_GINTSTS_OEPINT;

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

static void configure_endpoint0(uint8_t endpoint_size)
{
	/* unmask all interrupts of IN and OUT endpoint0 */
	USB_OTG_HS_DEVICE->DAINTMSK |= (1UL << 0) | (1UL << 16);

	/**
	 * Configures the maximum packet size, activates the endpoint,
	 * and NAK the endpoint (cannot send data yet).
	 *
	 * (OTG_HS_DIEPCTLx)
	 */
	MODIFY_REG(IN_ENDPOINT(0)->DIEPCTL,
		USB_OTG_DIEPCTL_MPSIZ,
		USB_OTG_DIEPCTL_USBAEP | _VAL2FLD(USB_OTG_DIEPCTL_MPSIZ, endpoint_size) | USB_OTG_DIEPCTL_SNAK
	);

	/* clear NAK and enable endpoint data transmission */
	OUT_ENDPOINT(0)->DOEPCTL |= USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_CNAK;
}

static void configure_in_endpoint(uint8_t endpoint_number, UsbEndpointType endpoint_type, uint16_t endpoint_size)
{
	/* unmasks all interrupts of the targeted IN endpoint.*/
	USB_OTG_HS_DEVICE->DAINTMSK |= (1UL<<endpoint_number);

	/**
	 * activates the endpoint, sets endpoint handshake to NAK (not ready to send data), sets DATA0 packet identifier,
	 * configures its type, its maximum packet size, and assigns it a TxFIFO.
	 */
	MODIFY_REG(IN_ENDPOINT(endpoint_number)->DIEPCTL,
		USB_OTG_DIEPCTL_MPSIZ | USB_OTG_DIEPCTL_EPTYP,
		USB_OTG_DIEPCTL_USBAEP | _VAL2FLD(USB_OTG_DIEPCTL_MPSIZ, endpoint_size) | USB_OTG_DIEPCTL_SNAK |
		_VAL2FLD(USB_OTG_DIEPCTL_EPTYP, endpoint_type) | USB_OTG_DIEPCTL_SD0PID_SEVNFRM
	);
}

static void deconfigure_endpoint(uint8_t endpoint_number)
{
    USB_OTG_INEndpointTypeDef *in_endpoint = IN_ENDPOINT(endpoint_number);
    USB_OTG_OUTEndpointTypeDef *out_endpoint = OUT_ENDPOINT(endpoint_number);

	/**
	 * masks all interrupts of the targeted IN and OUT endpoints.
	 * (OTG_HS_DAINTMSK)
	 */
	CLEAR_BIT(USB_OTG_HS_DEVICE->DAINTMSK,
		(1 << endpoint_number) | (1 << 16 << endpoint_number)
	);

	/* clears all interrupts of the endpoint */
	/* (OTG_HS_DIEPINTx) */
	in_endpoint->DIEPINT |= 0x29FF; // 0010 1001 1111 1111 (the first 16 bits)
	/* (OTG_HS_DOEPINTx) */
    out_endpoint->DOEPINT |= 0x715F; // 0111 0001 0101 1111 (the first 16 bits)

	/* disables the endpoints if possible */
    if (in_endpoint->DIEPCTL & USB_OTG_DIEPCTL_EPENA)
    {
		/* Disables endpoint transmission. */
		in_endpoint->DIEPCTL |= USB_OTG_DIEPCTL_EPDIS;
    }

	/* deactivates the endpoint */
	in_endpoint->DIEPCTL &= ~USB_OTG_DIEPCTL_USBAEP;

    if (endpoint_number != 0)
    {
		if (out_endpoint->DOEPCTL & USB_OTG_DOEPCTL_EPENA)
		{
			/* disables endpoint transmission */
			out_endpoint->DOEPCTL |= USB_OTG_DOEPCTL_EPDIS;
		}

		/* deactivates the endpoint */
		out_endpoint->DOEPCTL &= ~USB_OTG_DOEPCTL_USBAEP;
    }
}

static void usbrst_handler()
{
	for (uint8_t i = 0; i <= ENDPOINT_COUNT; i++)
	{

	}
}

/**
 * handle the USB core interrupts
 */
void gintsts_handler()
{
	volatile uint32_t gintsts = USB_OTG_HS_GLOBAL->GINTSTS;

	if (gintsts & USB_OTG_GINTSTS_USBRST)
	{
		/* do something */


		/* clears the interrupt */
		USB_OTG_HS_GLOBAL->GINTSTS |= USB_OTG_GINTSTS_USBRST;
	}
	else if (gintsts & USB_OTG_GINTSTS_ENUMDNE)
	{

	}
	else if (gintsts & USB_OTG_GINTSTS_RXFLVL)
	{

	}
	else if (gintsts & USB_OTG_GINTSTS_IEPINT)
	{

	}
	else if (gintsts & USB_OTG_GINTSTS_OEPINT)
	{

	}
}
