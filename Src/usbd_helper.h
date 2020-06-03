#pragma once
#include <stdint.h>

/* from reactOS project */

/* USB_COMMON_DESCRIPTOR.bDescriptorType constants */
#define USB_DEVICE_DESCRIPTOR_TYPE          0x01
#define USB_CONFIGURATION_DESCRIPTOR_TYPE   0x02
#define USB_STRING_DESCRIPTOR_TYPE          0x03
#define USB_INTERFACE_DESCRIPTOR_TYPE       0x04
#define USB_ENDPOINT_DESCRIPTOR_TYPE        0x05
#define USB_QUALIFIER_DESCRIPTOR_TYPE       0x06
#define USB_CONFIG_POWER_DESCRIPTOR_TYPE    0x07
#define USB_INTERFACE_POWER_DESCRIPTOR_TYPE 0x08

/* USB_ENDPOINT_DESCRIPTOR.bmAttributes constants */
#define USB_ENDPOINT_TYPE_MASK              0x03
#define USB_ENDPOINT_TYPE_CONTROL           0x00
#define USB_ENDPOINT_TYPE_ISOCHRONOUS       0x01
#define USB_ENDPOINT_TYPE_BULK              0x02
#define USB_ENDPOINT_TYPE_INTERRUPT         0x03

/* USB_CONFIGURATION_DESCRIPTOR.bmAttributes constants */
#define USB_CONFIG_POWERED_MASK             0xc0
#define USB_CONFIG_BUS_POWERED              0x80
#ifndef USB_CONFIG_SELF_POWERED
#define USB_CONFIG_SELF_POWERED             0x40
#endif
#ifndef USB_CONFIG_REMOTE_WAKEUP
#define USB_CONFIG_REMOTE_WAKEUP            0x20
#endif

#define USB_DEVICE_CLASS_RESERVED           0x00
#define USB_DEVICE_CLASS_AUDIO              0x01
#define USB_DEVICE_CLASS_COMMUNICATIONS     0x02
#define USB_DEVICE_CLASS_HUMAN_INTERFACE    0x03
#define USB_DEVICE_CLASS_MONITOR            0x04
#define USB_DEVICE_CLASS_PHYSICAL_INTERFACE 0x05
#define USB_DEVICE_CLASS_POWER              0x06
#define USB_DEVICE_CLASS_PRINTER            0x07
#define USB_DEVICE_CLASS_STORAGE            0x08
#define USB_DEVICE_CLASS_HUB                0x09
#define USB_DEVICE_CLASS_VENDOR_SPECIFIC    0xFF

#pragma pack( push, 1 )
typedef struct _USB_COMMON_DESCRIPTOR
{
  uint8_t bLength;
  uint8_t bDescriptorType;
}
USB_COMMON_DESCRIPTOR, *PUSB_COMMON_DESCRIPTOR;

typedef struct _USB_CONFIGURATION_DESCRIPTOR
{
  uint8_t  bLength;
  uint8_t  bDescriptorType;
  uint16_t wTotalLength;
  uint8_t  bNumInterfaces;
  uint8_t  bConfigurationValue;
  uint8_t  iConfiguration;
  uint8_t  bmAttributes;
  uint8_t  MaxPower;
}
USB_CONFIGURATION_DESCRIPTOR, *PUSB_CONFIGURATION_DESCRIPTOR;

typedef struct _USB_DEVICE_DESCRIPTOR
{
  uint8_t  bLength;
  uint8_t  bDescriptorType;
  uint16_t bcdUSB;
  uint8_t  bDeviceClass;
  uint8_t  bDeviceSubClass;
  uint8_t  bDeviceProtocol;
  uint8_t  bMaxPacketSize0;
  uint16_t idVendor;
  uint16_t idProduct;
  uint16_t bcdDevice;
  uint8_t  iManufacturer;
  uint8_t  iProduct;
  uint8_t  iSerialNumber;
  uint8_t  bNumConfigurations;
}
USB_DEVICE_DESCRIPTOR, *PUSB_DEVICE_DESCRIPTOR;

typedef struct _USB_DEVICE_QUALIFIER_DESCRIPTOR
{
  uint8_t  bLength;
  uint8_t  bDescriptorType;
  uint16_t bcdUSB;
  uint8_t  bDeviceClass;
  uint8_t  bDeviceSubClass;
  uint8_t  bDeviceProtocol;
  uint8_t  bMaxPacketSize0;
  uint8_t  bNumConfigurations;
  uint8_t  bReserved;
} 
USB_DEVICE_QUALIFIER_DESCRIPTOR, *PUSB_DEVICE_QUALIFIER_DESCRIPTOR;

typedef enum _USB_DEVICE_SPEED
{
  UsbLowSpeed,
  UsbFullSpeed,
  UsbHighSpeed,
  UsbSuperSpeed
}
USB_DEVICE_SPEED;

typedef struct _USB_ENDPOINT_DESCRIPTOR
{
  uint8_t  bLength;
  uint8_t  bDescriptorType;
  uint8_t  bEndpointAddress;
  uint8_t  bmAttributes;
  uint16_t wMaxPacketSize;
  uint8_t  bInterval;
}
USB_ENDPOINT_DESCRIPTOR, *PUSB_ENDPOINT_DESCRIPTOR;

typedef struct _USB_INTERFACE_DESCRIPTOR
{
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint8_t bInterfaceNumber;
  uint8_t bAlternateSetting;
  uint8_t bNumEndpoints;
  uint8_t bInterfaceClass;
  uint8_t bInterfaceSubClass;
  uint8_t bInterfaceProtocol;
  uint8_t iInterface;
}
USB_INTERFACE_DESCRIPTOR, *PUSB_INTERFACE_DESCRIPTOR;

typedef struct _USB_STRING_DESCRIPTOR
{
  uint8_t  bLength;
  uint8_t  bDescriptorType;
  uint16_t bString[1];
}
USB_STRING_DESCRIPTOR, *PUSB_STRING_DESCRIPTOR;

#pragma pack( pop )

