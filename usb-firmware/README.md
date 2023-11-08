# VBUS

The nominal VBUS voltage is normally ~5 V.

VBUS voltage can drop down to ~ 4.4 V (according to the load on the VBUS).

USB device can normally draw current from the host through the VBUS depending on its state:

- Not configured (default state):

  USB 2.0: up to 100 mA (USB 2.0). \
   USB 3.x: up to 150 mA.

- Configured:

  USB 2.0: may ask to draw up to 500 mA (high power device). \  
   USB 3.x: may ask to draw up to 900 mA (high power device).

- Suspended:

  Up to 0.5 mA (2.5 mA if configured as high power). The current of the pull up and \
   pull down resistors must be considered (they sink ~ 0.2 mA). \
   USB device can draw more current -if needed- according to battery charging and power delivery specifications.

# clock config

![stm32cubemx clock config](./images/Screenshot%20from%202023-10-28%2019-38-24.png)

# journalctl

```shell
$ journalctl -f
Oct 29 10:57:07 ryzen kernel: usb 5-3.4.1: new full-speed USB device number 17 using xhci_hcd
```

# USB core global interrupts

In STM32F429ZI microcontroller (based on ARM Cortex-M4) we basically have 5 interrupts that exist in "USB Core Global Interrupts" register. We will spend quite a lot of time implementing the handlers of these interrupts. Here I just want to tell you when every one of these interrupts is raised.

- USB Reset Signal: this interrupt is raised when the USB device received a USB reset signal on the bus from the host.
- USB Device Speed Enumeration Done: this interrupt is raised when USB reset is done and at this moment the device knows in which speed it should run.
- USB RxFIFO Not Empty: this interrupt is raised when the device has received data from the host (and the data is ready to be read from the RxFIFO of the USB device).
- USB IN Endpoint Interrupt: this interrupt is raised whenever an interrupt is raised on any endpoint. To know which interrupt exactly was raised and on which IN endpoint, another register (other than Core Global Interrupts) should be read.
- USB OUT Endpoint Interrupt: as same as USB IN Endpoint Interrupt but for OUT endpoints.

# kinda looks the same

```C
#define USB_OTG_NPTXFSA_Pos                      (0U)
#define USB_OTG_NPTXFSA_Msk                      (0xFFFFUL << USB_OTG_NPTXFSA_Pos) /*!< 0x0000FFFF */
#define USB_OTG_NPTXFSA                          USB_OTG_NPTXFSA_Msk

#define USB_OTG_DIEPTXF_INEPTXSA_Pos             (0U)
#define USB_OTG_DIEPTXF_INEPTXSA_Msk             (0xFFFFUL << USB_OTG_DIEPTXF_INEPTXSA_Pos) /*!< 0x0000FFFF */
#define USB_OTG_DIEPTXF_INEPTXSA                 USB_OTG_DIEPTXF_INEPTXSA_Msk  /*!< IN endpoint FIFOx transmit RAM start address */
```

# USB component sources

- https://www.keil.com/pack/doc/mw/USB/html/index.html
- https://www.keil.com/pack/doc/mw/USB/html/_u_s_b__device__descriptor.html
- https://wiki.st.com/stm32mcu/wiki/Introduction_to_USB_with_STM32
- usb paper - https://usb.org/sites/default/files/usbmassbulk_10.pdf

![device descriptor](./images/Screenshot%20from%202023-11-08%2020-37-52.png)

# wireshark

Wireshark is an open-source software used mainly to view and debug network traffic. It also supports viewing USB transactions. We will be using Wireshark to view and debug the packets sent OUT to / IN from our USB device.

I highly recommend using Linux to debug the USB device, because it has a very nice and reliable log for the enumeration of USB devices. You can use any Linux distribution (that has GUI).

Below I will show you the TL;DR of how to install Wireshark on Linux (Ubuntu Distribution).

NOTE: If you are using Raspberry Pi, I highly recommend that you create a new user (and add it to sudo group) and install Wireshark on the new user. Wireshark on the default user pi didn't work properly for some reason.

## Installs Wireshark and its dependencies.

sudo apt install wireshark

## Ensures that users without root permissions can capture packets.

sudo dpkg-reconfigure wireshark-common

## Adds the current user to wireshark group.

sudo adduser $USER wireshark

## You need to execute the following two commands every time you reboot your Linux system:

## Loads usbmon module to the kernel.

modprobe usbmon

## Gives regular users permissions to use the kernel module.

sudo setfacl -m u:$USER:r /dev/usbmon\*

References:

https://wiki.wireshark.org/CaptureSetup/USB
