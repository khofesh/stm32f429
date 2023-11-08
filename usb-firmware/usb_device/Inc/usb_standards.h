/*
 * usb_standards.h
 *
 *  Created on: Oct 29, 2023
 *      Author: fahmad
 */

#ifndef USB_STANDARDS_H_
#define USB_STANDARDS_H_

typedef enum UsbEndpointType
{
	USB_ENDPOINT_TYPE_CONTROL,
	USB_ENDPOINT_TYPE_ISOCHRONOUS,
	USB_ENDPOINT_TYPE_BULK,
	USB_ENDPOINT_TYPE_INTERRUPT
} UsbEndpointType;

typedef struct {
	void (*on_usb_reset_received)();
	void (*on_setup_data_received)(uint8_t endpoint_number, uint16_t bcnt);
	void (*on_out_data_received)(uint8_t endpoint_number, uint16_t bcnt);
	void (*on_in_transfer_completed)(uint8_t endpoint_number);
	void (*on_out_transfer_completed)(uint8_t endpoint_number);
	void (*on_usb_polled)();
} UsbEvents;

typedef enum
{
	USB_DEVICE_STATE_DEFAULT,
	USB_DEVICE_STATE_ADDRESSED,
	USB_DEVICE_STATE_CONFIGURED,
	USB_DEVICE_STATE_SUSPENDED
} UsbDeviceState;

typedef enum
{
	USB_CONTROL_STAGE_SETUP, // USB_CONTROL_STAGE_IDLE
	USB_CONTROL_STAGE_DATA_OUT,
	USB_CONTROL_STAGE_DATA_IN,
	USB_CONTROL_STAGE_DATA_IN_IDLE,
	USB_CONTROL_STAGE_DATA_IN_ZERO,
	USB_CONTROL_STAGE_STATUS_OUT,
	USB_CONTROL_STAGE_STATUS_IN
} UsbControlTransferStage;

/* USB control request. */
typedef struct
{
	uint8_t bmRequestType; /* the transfer direction, the type of request, and the recipient */
	uint8_t bRequest; /* request identity */
	uint16_t wValue; /* parameter passed to the device */
	uint16_t wIndex; /* parameter passed to the device */
	uint16_t wLength; /* the count of bytes that will be transmitted in the data data stage */
} UsbRequest;

/* USB Device Bit Mapped Request TypeFields */
#define USB_BM_REQUEST_TYPE_DIRECTION_MASK (1 << 7)
#define USB_BM_REQUEST_TYPE_DIRECTION_TODEVICE (0 << 7)
#define USB_BM_REQUEST_TYPE_DIRECTION_TOHOST (1 << 7)

#define USB_BM_REQUEST_TYPE_TYPE_MASK (3 << 5)
#define USB_BM_REQUEST_TYPE_TYPE_STANDARD (0 << 5)
#define USB_BM_REQUEST_TYPE_TYPE_CLASS (1 << 5)
#define USB_BM_REQUEST_TYPE_TYPE_VENDOR (2 << 5)

#define USB_BM_REQUEST_TYPE_RECIPIENT_MASK (3 << 0)
#define USB_BM_REQUEST_TYPE_RECIPIENT_DEVICE (0 << 0)
#define USB_BM_REQUEST_TYPE_RECIPIENT_INTERFACE (1 << 0)
#define USB_BM_REQUEST_TYPE_RECIPIENT_ENDPOINT (2 << 0)
#define USB_BM_REQUEST_TYPE_RECIPIENT_OTHER (3 << 0)

/* USB standard requests */
#define USB_STANDARD_GET_STATUS 0x00 /*  Returns the status if the specified recipient.*/
#define USB_STANDARD_CLEAR_FEATURE 0x01 /*  Clears or disables a specific feature.*/
#define USB_STANDARD_SET_FEATURE 0x03 /* Sets or enables a specific feature.*/
#define USB_STANDARD_SET_ADDRESS 0x05 /* Sets the device address for all future device communications.*/
#define USB_STANDARD_GET_DESCRIPTOR 0x06 /* Returns the specified descriptor if the descriptor exists.*/
#define USB_STANDARD_SET_DESCRIPTOR 0x07 /* Updates existing descriptors or new descriptors may be added.*/
#define USB_STANDARD_GET_CONFIG 0x08 /* Returns the current device configuration value.*/
#define USB_STANDARD_SET_CONFIG 0x09 /* Sets the device configuration.*/
#define USB_STANDARD_GET_INTERFACE 0x0A /* Returns the selected alternate setting for the specified interface.*/
#define USB_STANDARD_SET_INTERFACE 0x0B /* Allows the host to select an alternate setting for the specified interface.*/
#define USB_STANDARD_SYNCH_FRAME 0x0C /* Sets and then reports an endpoint's synchronization frame.*/

/* USB standard descriptor types */
#define USB_DESCRIPTOR_TYPE_DEVICE 0x01
#define USB_DESCRIPTOR_TYPE_CONFIGURATION 0x02
#define USB_DESCRIPTOR_TYPE_STRING 0x03
#define USB_DESCRIPTOR_TYPE_INTERFACE 0x04
#define USB_DESCRIPTOR_TYPE_ENDPOINT 0x05
#define USB_DESCRIPTOR_TYPE_QUALIFIER 0x06
#define USB_DESCRIPTOR_TYPE_OTHER 0x07
#define USB_DESCRIPTOR_TYPE_INTERFACEPOWER 0x08
#define USB_DESCRIPTOR_TYPE_OTG 0x09
#define USB_DESCRIPTOR_TYPE_DEBUG 0x0A
#define USB_DESCRIPTOR_TYPE_INTERFASEASSOC 0x0B
#define USB_DESCRIPTOR_TYPE_CS_INTERFACE 0x24
#define USB_DESCRIPTOR_TYPE_CS_ENDPOINT 0x25

/* USB classes */
#define USB_CLASS_PER_INTERFACE 0x00 /* Class defined on interface level.*/
#define USB_CLASS_AUDIO 0x01 /* Audio device class.*/
#define USB_CLASS_PHYSICAL 0x05 /* Physical device class.*/
#define USB_CLASS_STILL_IMAGE 0x06 /* Still Imaging device class.*/
#define USB_CLASS_PRINTER 0x07 /* Printer device class.*/
#define USB_CLASS_MASS_STORAGE 0x08 /* Mass Storage device class.*/
#define USB_CLASS_HUB 0x09 /* HUB device class.*/
#define USB_CLASS_CSCID 0x0B /* Smart Card device class.*/
#define USB_CLASS_CONTENT_SEC 0x0D /* Content Security device class.*/
#define USB_CLASS_VIDEO 0x0E /* Video device class.*/
#define USB_CLASS_HEALTHCARE 0x0F /* Personal Healthcare device class.*/
#define USB_CLASS_AV 0x10 /* Audio/Video device class.*/
#define USB_CLASS_BILLBOARD 0x11 /* Billboard device class.*/
#define USB_CLASS_CBRIDGE 0x12 /* USB Type-C Bridge device class.*/
#define USB_CLASS_DIAGNOSTIC 0xDC /* Diagnostic device class.*/
#define USB_CLASS_WIRELESS 0xE0 /* Wireless controller class.*/
#define USB_CLASS_MISC 0xEF /* Miscellanious device class.*/
#define USB_CLASS_IAD 0xEF /* Class defined on interface association level.*/
#define USB_CLASS_APP_SPEC 0xFE /* Application Specific class.*/
#define USB_CLASS_VENDOR 0xFF /* Vendor specific class.*/
#define USB_CLASS_HID 0x03

#define USB_SUBCLASS_NONE 0x00 /* No subclass defined.*/
#define USB_SUBCLASS_IAD 0x02 /* Subclass defined on interface association level.*/
#define USB_SUBCLASS_VENDOR 0xFF /* Vendor specific subclass.*/

#define USB_PROTOCOL_NONE 0x00 /* No protocol defined.*/
#define USB_PROTOCOL_IAD 0x01 /* Protocol defined on interface association level.*/
#define USB_PROTOCOL_VENDOR 0xFF /* Vendor specific protocol.*/

/* USB device descriptor */
typedef struct {
	uint8_t  bLength; /* Size of the descriptor (in bytes).*/
	uint8_t  bDescriptorType; /* USB_DESCRIPTOR_TYPE_DEVICE device descriptor.*/
	uint16_t bcdUSB; /* USB specification release number.*/
	uint8_t  bDeviceClass; /* USB device class.*/
	uint8_t  bDeviceSubClass; /* USB device subclass.*/
	uint8_t  bDeviceProtocol; /* USB device protocol.*/
	uint8_t  bMaxPacketSize0; /* Max packet size for the control endpoint 0 (8, 16, 32 or 64 bytes).*/
	uint16_t idVendor; /* Vendor ID for the USB device.*/
	uint16_t idProduct; /* Product ID for the USB device.*/
	uint16_t bcdDevice; /* Device release number.*/
	uint8_t  iManufacturer; /* String descriptor index for the manufacturer's name.*/
	uint8_t  iProduct; /* String descriptor index for the product name.*/
	uint8_t  iSerialNumber; /* String descriptor index for the product serial number.*/
	uint8_t  bNumConfigurations; /* Total number of configurations supported by the USB device.*/
} __attribute__((__packed__)) UsbDeviceDescriptor;

/* USB configuration descriptor */
typedef struct {
	uint8_t  bLength; /* Size of the descriptor (in bytes).*/
	uint8_t  bDescriptorType; /* \ref USB_DESCRIPTOR_TYPE_CONFIGURATION descriptor.*/
	uint16_t wTotalLength; /* Size of the configuration descriptor header, and all sub descriptors attached to the configuration.*/
	uint8_t  bNumInterfaces; /* Total number of interfaces in the configuration.*/
	uint8_t  bConfigurationValue; /* Configuration index of the current configuration descriptor.*/
	uint8_t  iConfiguration; /* Index of a string descriptor describing this configuration.*/
	uint8_t  bmAttributes; /* Configuration attributes: Self Powered and Remote Wake up.*/
	uint8_t  bMaxPower; /* Maximum power consumption of the device.*/
} __attribute__((__packed__)) UsbConfigurationDescriptor;

/* USB interface descriptor */
typedef struct {
	uint8_t bLength; /* Size of the descriptor, in bytes. */
	uint8_t bDescriptorType; /* Interface descriptor. */
	uint8_t bInterfaceNumber; /* Index of the interface in the current configuration. */
	uint8_t bAlternateSetting; /* Alternate setting for the interface number. */
	uint8_t bNumEndpoints; /* Total number of endpoints in the interface. */
	uint8_t bInterfaceClass; /* Interface class ID. */
	uint8_t bInterfaceSubClass; /* Interface subclass ID.*/
	uint8_t bInterfaceProtocol; /* Interface protocol ID. */
	uint8_t iInterface; /* Index of the string descriptor describing the interface. */
} __attribute__((__packed__)) UsbInterfaceDescriptor;

/* USB endpoint descriptor */
typedef struct {
	uint8_t  bLength; /* Size of the descriptor, in bytes. */
	uint8_t  bDescriptorType; /* Endpoint descriptor. */
	uint8_t  bEndpointAddress; /* Logical address of the endpoint including direction mask. */
	uint8_t  bmAttributes; /* Endpoint attributes, \ref USB_ENDPOINT_BMATTRIBUTES_TYPE. */
	uint16_t wMaxPacketSize; /* Maximum packet size of the endpoint */
	uint8_t  bInterval; /* Polling interval of the endpoint (frames). */
} __attribute__((__packed__)) UsbEndpointDescriptor;

/* USB_ENDPOINT_BMATTRIBUTES_TYPE */
#define USB_ENDPOINT_TYPE_CONTROL 0x00 /* Control endpoint.*/
#define USB_ENDPOINT_TYPE_ISOCHRONUS 0x01 /* Isochronous endpoint.*/
#define USB_ENDPOINT_TYPE_BULK 0x02 /* Bulk endpoint.*/
#define USB_ENDPOINT_TYPE_INTERRUPT 0x03 /* Interrupt endpoint.*/

#endif /* USB_STANDARDS_H_ */
