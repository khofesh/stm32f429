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
