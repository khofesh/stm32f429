/*
 * usbd_driver.c
 *
 *  Created on: Oct 28, 2023
 *      Author: fahmad
 *
 *  34.11 		Peripheral FIFO architecture
 *  	Figure 392. Device-mode FIFO address mapping and AHB FIFO access mapping
 *  34.13		FIFO RAM allocation
 *
 *	35.13.6 	Device programming model
 */

#include <strings.h>

#include "usbd_driver.h"

static void usbrst_handler();
static void enumdne_handler();
static void rxflvl_handler();
static void configure_endpoint0(uint8_t endpoint_size);
static void configure_in_endpoint(uint8_t endpoint_number, UsbEndpointType endpoint_type, uint16_t endpoint_size);
static void deconfigure_endpoint(uint8_t endpoint_number);
static void refresh_fifo_start_addresses();
static void configure_rxfifo_size(uint16_t size);
static void configure_txfifo_size(uint8_t endpoint_number, uint16_t size);
static void read_packet(void const *buffer, uint16_t size);
static void write_packet(uint8_t endpoint_number, void const *buffer, uint16_t size);
static void flush_rxfifo();
static void flush_txfifo(uint8_t endpoint_number);
static void disconnect();
static void connect();
static void initialize_core();
static void set_device_address(uint8_t address);
static void initialize_gpio_pins();
static void iepint_handler();
static void oepint_handler();
static void gintsts_handler();

const UsbDriver usb_driver = {
	.initialize_core = &initialize_core,
	.initialize_gpio_pins = &initialize_gpio_pins,
	.connect = &connect,
	.disconnect = &disconnect,
	.flush_rxfifo = &flush_rxfifo,
	.flush_txfifo = &flush_txfifo,
	.configure_in_endpoint = &configure_in_endpoint,
	.read_packet = &read_packet,
	.write_packet = &write_packet,
	.poll = &gintsts_handler,
	.set_device_address = &set_device_address
};

static void initialize_gpio_pins()
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

static void initialize_core()
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

	/**
	 * unmask transfer completed interrupt for all endpoints
	 *
	 * OTG_HS device IN endpoint common interrupt mask register
	 * (OTG_HS_DIEPMSK)
	 * 		Bit 0 XFRCM: Transfer completed interrupt mask
	 *
	 * OTG_HS device OUT endpoint common interrupt mask register
	 * (OTG_HS_DOEPMSK)
	 * 		Bit 0 XFRCM: Transfer completed interrupt mask
	 */
	USB_OTG_HS_DEVICE->DOEPMSK |= USB_OTG_DOEPMSK_XFRCM;
	USB_OTG_HS_DEVICE->DIEPMSK |= USB_OTG_DIEPMSK_XFRCM;
}

static void set_device_address(uint8_t address)
{
	/**
	 * OTG_HS device configuration register (OTG_HS_DCFG)
	 * 		Bits 10:4 DAD: Device address
	 * 			The application must program this field after every
	 * 			SetAddress control command.
	 */
	MODIFY_REG(
		USB_OTG_HS_DEVICE->DCFG,
		USB_OTG_DCFG_DAD,
		_VAL2FLD(USB_OTG_DCFG_DAD, address)
	);
}

/**
 * connect the USB device to the bus
 */
static void connect()
{
	/* power the transceivers on */
	USB_OTG_HS->GCCFG |= USB_OTG_GCCFG_PWRDWN;

	/* connect the device to the bus */
	USB_OTG_HS_DEVICE->DCTL &= ~USB_OTG_DCTL_SDIS;
}

/**
 * disconnect the USB device from the bus
 */
static void disconnect()
{
	/* disconnects the device from the bus */
	USB_OTG_HS_DEVICE->DCTL |= USB_OTG_DCTL_SDIS;

	/* powers the transceivers off */
	USB_OTG_HS->GCCFG &= ~USB_OTG_GCCFG_PWRDWN;
}

/**
 * pop data from the RxFIFO and stores it in the buffer
 */
static void read_packet(void const *buffer, uint16_t size)
{
	/* there is only one RxFIFO */
	volatile uint32_t *fifo = FIFO(0);

	for(; size >= 4; size -=4, buffer += 4)
	{
		/* pop one 32-bit word of data (until there is less than one word remaining) */
		volatile uint32_t data = *fifo;

		/* stores the data in the buffer */
		*((uint32_t *)buffer) = data;
	}

	if(size > 0)
	{
		/* pop the last remaining bytes (which are less than one word) */
		volatile uint32_t data = *fifo;

		/* 1 byte */
		for(; size > 0; size--, buffer++, data >>= 8)
		{
			/* stores the data in the buffer with the correct alignment */
			*((uint8_t *)buffer) = 0xFF & data;
		}
	}
}

static void write_packet(uint8_t endpoint_number, void const *buffer, uint16_t size)
{
	volatile uint32_t *fifo = FIFO(endpoint_number);
	USB_OTG_INEndpointTypeDef *in_endpoint = IN_ENDPOINT(endpoint_number);

	/**
	 * configure the transmission (1 packet that has "size" bytes)
	 *
	 * OTG_HS device endpoint-x transfer size register (OTG_HS_DIEPTSIZx)
	 * 	(x = 1..5, where x = Endpoint_number)
	 *
	 *  	Bit 28:19 PKTCNT: Packet count
	 *  	Bits 18:0 XFRSIZ: Transfer size
	 */
	MODIFY_REG(
		in_endpoint->DIEPTSIZ,
		USB_OTG_DIEPTSIZ_PKTCNT | USB_OTG_DIEPTSIZ_XFRSIZ,
		_VAL2FLD(USB_OTG_DIEPTSIZ_PKTCNT, 1) | _VAL2FLD(USB_OTG_DIEPTSIZ_XFRSIZ, size)
	);

	/**
	 * enable the transmission after clearing both STALL and NAK of the endpoint
	 *
	 * OTG device endpoint-x control register (OTG_HS_DIEPCTLx)
	 * (x = 0..5, where x = Endpoint_number)
	 * 		Bit 21 STALL: STALL handshake
	 * 		Bit 26 CNAK: Clear NAK
	 * 		Bit 31 EPENA: Endpoint enable
	 */
	MODIFY_REG(
		in_endpoint->DIEPCTL,
		USB_OTG_DIEPCTL_STALL,
		USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA
	);

	/* get the size in term of 32-bit words (to avoid integer overflow in the loop) */
	size = (size + 3)/4;

	for(; size > 0; size--, buffer += 4)
	{
		/* pushes the data to the TxFIFO */
		*fifo = *((uint32_t *)buffer);
	}
}

/**
 * flushes the RxFIFI of all OUT endpoints
 */
static void flush_rxfifo()
{
	/**
	 * OTG_HS reset register (OTG_HS_GRSTCTL)
	 * 		Bit 4 RXFFLSH: RxFIFO flush
	 */
	USB_OTG_HS->GRSTCTL |= USB_OTG_GRSTCTL_RXFFLSH;
}

/**
 * flushes the TxFIFO of an IN endpoint
 * endpoint_number - the number of an IN endpoint TO FLUSH ITS TxFIFO
 */
static void flush_txfifo(uint8_t endpoint_number)
{
	/**
	 *	OTG_HS reset register (OTG_HS_GRSTCTL)
	 *		Bits 10:6 TXFNUM: TxFIFO number
	 *		Bit 5 TXFFLSH: TxFIFO flush
	 */
	MODIFY_REG(
		USB_OTG_HS->GRSTCTL,
		USB_OTG_GRSTCTL_TXFNUM,
		_VAL2FLD(USB_OTG_GRSTCTL_TXFNUM, endpoint_number) |
		USB_OTG_GRSTCTL_TXFFLSH
	);
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

	/**
	 * 64 bytes is the maximum packet size for full speed USB devices
	 */
	configure_rxfifo_size(64);
	configure_txfifo_size(0, endpoint_size);
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
		USB_OTG_DIEPCTL_MPSIZ | USB_OTG_DIEPCTL_EPTYP | USB_OTG_DIEPCTL_TXFNUM,
		USB_OTG_DIEPCTL_USBAEP | _VAL2FLD(USB_OTG_DIEPCTL_MPSIZ, endpoint_size) | USB_OTG_DIEPCTL_SNAK |
		_VAL2FLD(USB_OTG_DIEPCTL_EPTYP, endpoint_type) | _VAL2FLD(USB_OTG_DIEPCTL_TXFNUM, endpoint_number) |
		USB_OTG_DIEPCTL_SD0PID_SEVNFRM
	);

	configure_txfifo_size(endpoint_number, endpoint_size);
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

    // flushes the FIFOs
    flush_txfifo(endpoint_number);
    flush_rxfifo();
}

static void refresh_fifo_start_addresses()
{
	/* The first changeable start address begins after the region of RxFIFO */
	uint16_t start_address = _FLD2VAL(USB_OTG_GRXFSIZ_RXFD, USB_OTG_HS->GRXFSIZ) * 4;

	/**
	 *  Updates the start address of the TxFIFO0
	 *
	 *  OTG_FS Host non-periodic transmit FIFO size register
	 *  (OTG_FS_HNPTXFSIZ)/Endpoint 0 Transmit FIFO size (OTG_FS_DIEPTXF0)
	 *  	Bits 15:0 TX0FSA: Endpoint 0 transmit RAM start address
	 */
	MODIFY_REG(USB_OTG_HS->DIEPTXF0_HNPTXFSIZ,
			USB_OTG_TX0FSA,
			_VAL2FLD(USB_OTG_TX0FSA, start_address)
	);

	/* The next start address is after where the last TxFIFO ends */
	start_address += _FLD2VAL(USB_OTG_TX0FD, USB_OTG_HS->DIEPTXF0_HNPTXFSIZ) * 4;

	/* Updates the start addresses of the rest TxFIFOs */
	for (uint8_t txfifo_number = 0; txfifo_number < ENDPOINT_COUNT - 1; txfifo_number++)
	{
		/**
		 * OTG_HS device IN endpoint transmit FIFO size register (OTG_HS_DIEPTXFx)
		 * (x = 1..5, where x is the FIFO_number)
		 *
		 * Bits 15:0 INEPTXSA: IN endpoint FIFOx transmit RAM start address
		 */
		MODIFY_REG(USB_OTG_HS->DIEPTXF[txfifo_number],
//				 USB_OTG_NPTXFSA, this is wrong I guess
				USB_OTG_DIEPTXF_INEPTXSA,
				_VAL2FLD(USB_OTG_DIEPTXF_INEPTXSA, start_address)
		);

		start_address += _FLD2VAL(USB_OTG_NPTXFD, USB_OTG_HS->DIEPTXF[txfifo_number]) * 4;
	}
}

/**
 * configures the RxFIFO of all OUT endpoints.
 * the RxFIFO is shared between all OUT endpoints.
 */
static void configure_rxfifo_size(uint16_t size)
{
	/* considers the space required to save status packets in RxFIFO and gets the size in term of 32-bit words */
	size = 10 + (2 * ((size / 4) + 1));

	/**
	 *  configure the depth of the FIFO
	 *  OTG_HS Receive FIFO size register (OTG_HS_GRXFSIZ)
	 *  	Bits 15:0 	RXFD: RxFIFO depth
	 */
	MODIFY_REG(USB_OTG_HS->GRXFSIZ,
			USB_OTG_GRXFSIZ_RXFD,
			_VAL2FLD(USB_OTG_GRXFSIZ_RXFD, size)
	);

	refresh_fifo_start_addresses();
}

static void configure_txfifo_size(uint8_t endpoint_number, uint16_t size)
{
	/* get the FIFO size in term of 32-bit words */
	size = (size + 3)/4;

	/* configures the depth of the TxFIFO */
	if (endpoint_number == 0)
	{
		/**
		 * OTG_FS Host non-periodic transmit FIFO size register
		 * (OTG_FS_HNPTXFSIZ)/Endpoint 0 Transmit FIFO size (OTG_FS_DIEPTXF0)
		 *
		 * Device mode
		 * 	Bits 31:16 TX0FD: Endpoint 0 TxFIFO depth
		 */
		MODIFY_REG(USB_OTG_HS->DIEPTXF0_HNPTXFSIZ,
				USB_OTG_TX0FD,
				_VAL2FLD(USB_OTG_TX0FD, size)
		);
	}
	else
	{
		/**
		 * OTG_HS device IN endpoint transmit FIFO size register (OTG_HS_DIEPTXFx)
		 * (x = 1..5, where x is the FIFO_number)
		 *
		 * Bits 31:16 INEPTXFD: IN endpoint TxFIFO depth
		 */
		MODIFY_REG(USB_OTG_HS->DIEPTXF[endpoint_number - 1],
				USB_OTG_DIEPTXF_INEPTXFD, // I think mohammed did a mistake here, should've been INEPTXFD
				_VAL2FLD(USB_OTG_DIEPTXF_INEPTXFD, size)
		);
	}

	refresh_fifo_start_addresses();
}

static void usbrst_handler()
{
	log_info("USB reset signal was detected");

	for (uint8_t i = 0; i <= ENDPOINT_COUNT; i++)
	{
		deconfigure_endpoint(i);
	}

	usb_events.on_usb_reset_received();
}

static void enumdne_handler()
{
	log_info("USB device speed enumeration done.");

	// 8 bytes - now hardcoded
	configure_endpoint0(8);
}

static void rxflvl_handler()
{
	/* pop the status information word from RxFIFO */
	uint32_t receive_status = USB_OTG_HS_GLOBAL->GRXSTSP;

	/* the endpoint that received the data */
	uint8_t endpoint_number = _FLD2VAL(USB_OTG_GRXSTSP_EPNUM, receive_status);

	/* the count of bytes in the received packet */
	uint16_t bcnt = _FLD2VAL(USB_OTG_GRXSTSP_BCNT, receive_status);

	/* the status of the received packet */
	uint16_t pktsts = _FLD2VAL(USB_OTG_GRXSTSP_PKTSTS, receive_status);

	/**
	 * OTG_HS Receive status debug read/OTG status read and pop registers
	 * (OTG_HS_GRXSTSR/OTG_HS_GRXSTSP)
	 * 		Bits 20:17 PKTSTS: Packet status
	 * 		Indicates the status of the received packet
	 * 		0001: Global OUT NAK (triggers an interrupt)
	 * 		0010: OUT data packet received
	 * 		0011: OUT transfer completed (triggers an interrupt)
	 * 		0100: SETUP transaction completed (triggers an interrupt)
	 * 		0110: SETUP data packet received
	 * 		Others: Reserved
	 */
	switch (pktsts) {
		case 0x06: //0110: SETUP data packet received
			usb_events.on_setup_data_received(endpoint_number, bcnt);
			break;
		case 0x02: //0010: OUT data packet received
			break;
		case 0x04: //0100: SETUP transaction completed
			/* re-enable the transmission on the endpoint */
			OUT_ENDPOINT(endpoint_number)->DOEPCTL |= USB_OTG_DOEPCTL_CNAK;
			OUT_ENDPOINT(endpoint_number)->DOEPCTL |= USB_OTG_DOEPCTL_EPENA;
			break;
		case 0x03: //0011: OUT transfer completed
			/* re-enable the transmission on the endpoint */
			OUT_ENDPOINT(endpoint_number)->DOEPCTL |= USB_OTG_DOEPCTL_CNAK;
			OUT_ENDPOINT(endpoint_number)->DOEPCTL |= USB_OTG_DOEPCTL_EPENA;
			break;
	}
}

/**
 * handles the interrupt raised when an IN endpoint has a
 * raised interrupt
 */
static void iepint_handler()
{
	/**
	 * find the endpoint that caused the interrupt
	 *
	 * OTG_HS device all endpoints interrupt register (OTG_HS_DAINT)
	 * 		Bits 15:0 IEPINT: IN endpoint interrupt bits
	 *
	 * https://man7.org/linux/man-pages/man3/ffs.3.html
	 * ffs, ffsl, ffsll - find first bit set in a word
	 */
	uint8_t endpoint_number = ffs((int)USB_OTG_HS_DEVICE->DAINT) - 1;

	if (IN_ENDPOINT(endpoint_number)->DIEPINT & USB_OTG_DIEPINT_XFRC)
	{
		usb_events.on_in_transfer_completed(endpoint_number);

		/* clear the interrupt flag */
		IN_ENDPOINT(endpoint_number)->DIEPINT |= USB_OTG_DIEPINT_XFRC;
	}
}

/**
 * handles the interrupt raised when an OUT endpint has a raised interrupt
 */
static void oepint_handler()
{
	uint8_t endpoint_number = ffs(USB_OTG_HS_DEVICE->DAINT >> 16) - 1;

	if (OUT_ENDPOINT(endpoint_number)->DOEPINT & USB_OTG_DOEPINT_XFRC)
	{
		usb_events.on_out_transfer_completed(endpoint_number);

		OUT_ENDPOINT(endpoint_number)->DOEPINT |= USB_OTG_DOEPINT_XFRC;
	}
}

/**
 * handle the USB core interrupts
 *
 * global interrupt status handler
 */
static void gintsts_handler()
{
	volatile uint32_t gintsts = USB_OTG_HS_GLOBAL->GINTSTS;

	if (gintsts & USB_OTG_GINTSTS_USBRST)
	{
		/* do something */
		usbrst_handler();

		/* clears the interrupt */
		USB_OTG_HS_GLOBAL->GINTSTS |= USB_OTG_GINTSTS_USBRST;
	}
	else if (gintsts & USB_OTG_GINTSTS_ENUMDNE)
	{
		enumdne_handler();

		/* clear the interrupt */
		USB_OTG_HS_GLOBAL->GINTSTS |= USB_OTG_GINTSTS_ENUMDNE;
	}
	else if (gintsts & USB_OTG_GINTSTS_RXFLVL)
	{
		rxflvl_handler();

		/* clear interrupt */
		USB_OTG_HS_GLOBAL->GINTSTS |= USB_OTG_GINTSTS_RXFLVL;
	}
	else if (gintsts & USB_OTG_GINTSTS_IEPINT)
	{
		iepint_handler();

		USB_OTG_HS_GLOBAL->GINTSTS |= USB_OTG_GINTSTS_IEPINT;
	}
	else if (gintsts & USB_OTG_GINTSTS_OEPINT)
	{
		oepint_handler();

		USB_OTG_HS_GLOBAL->GINTSTS |= USB_OTG_GINTSTS_OEPINT;
	}

	usb_events.on_usb_polled();
}


