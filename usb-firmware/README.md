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
