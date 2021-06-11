#include "usbd_core.h"
#include "usbd_conf.h"

#define USBD_VID                    0x0C72
#define USBD_PID_FS                 0x000C   
#define USBD_LANGID_STRING          1033
#define USBD_MAX_STR_DESC_SIZ       0x100U

uint8_t * USBD_FS_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t * USBD_FS_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t * USBD_FS_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t * USBD_FS_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t * USBD_FS_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t * USBD_FS_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t * USBD_FS_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);

__ALIGN_BEGIN uint8_t USBD_FS_DeviceDesc[USB_LEN_DEV_DESC] __ALIGN_END =
{
  0x12,                       /*bLength */
  USB_DESC_TYPE_DEVICE,       /*bDescriptorType*/
  0x00,                       /*bcdUSB */
  0x01,
  0x00,                       /*bDeviceClass*/
  0x00,                       /*bDeviceSubClass*/
  0x00,                       /*bDeviceProtocol*/
  USB_MAX_EP0_SIZE,           /*bMaxPacketSize*/
  LOBYTE(USBD_VID),           /*idVendor*/
  HIBYTE(USBD_VID),           /*idVendor*/
  LOBYTE(USBD_PID_FS),        /*idProduct*/
  HIBYTE(USBD_PID_FS),        /*idProduct*/
  0xff,                       /*bcdDevice*/
  0x54,
  10,                         /*Index of manufacturer  string*/
  4,                          /*Index of product string*/
  0,                          /*Index of serial number string*/
  USBD_MAX_NUM_CONFIGURATION  /*bNumConfigurations*/
};

__ALIGN_BEGIN uint8_t USBD_LangIDDesc[USB_LEN_LANGID_STR_DESC] __ALIGN_END =
{
     USB_LEN_LANGID_STR_DESC,
     USB_DESC_TYPE_STRING,
     LOBYTE(USBD_LANGID_STRING),
     HIBYTE(USBD_LANGID_STRING)
};

uint8_t * USBD_FS_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  *length = sizeof(USBD_FS_DeviceDesc);
  return USBD_FS_DeviceDesc;
}

uint8_t * USBD_FS_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  *length = sizeof(USBD_LangIDDesc);
  return USBD_LangIDDesc;
}

/* must be here: 1, 2 */
uint8_t * USBD_FS_HugeStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED( speed );
  __ALIGN_BEGIN static const uint16_t huge_descriptor[1+126] __ALIGN_END = { 0x03FE };
  *length = sizeof( huge_descriptor );
  return (uint8_t*)huge_descriptor;
}

uint8_t * USBD_FS_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);

  __ALIGN_BEGIN static const uint16_t cfg_descriptor[1+8] __ALIGN_END = 
  { 
    0x0312, 
    'P','C','A','N','-','U','S','B' 
  };
  *length = sizeof( cfg_descriptor );
  return (uint8_t*)cfg_descriptor;
}

USBD_DescriptorsTypeDef FS_Desc =
{
  .GetDeviceDescriptor = USBD_FS_DeviceDescriptor,
  .GetLangIDStrDescriptor = USBD_FS_LangIDStrDescriptor,
  .GetManufacturerStrDescriptor = USBD_FS_HugeStrDescriptor,
  .GetProductStrDescriptor = USBD_FS_HugeStrDescriptor,
  .GetSerialStrDescriptor = 0,
  .GetConfigurationStrDescriptor = USBD_FS_ConfigStrDescriptor,
  .GetInterfaceStrDescriptor = 0,
};

