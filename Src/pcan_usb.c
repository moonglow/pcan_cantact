#include <stm32f0xx_hal.h>
#include <assert.h>
#include "usbd_ctlreq.h"
#include "usbd_ioreq.h"
#include "usbd_conf.h"
#include "usbd_helper.h"
#include "pcan_protocol.h"
#include "pcan_led.h"
#include "pcan_usb.h"

USBD_HandleTypeDef hUsbDeviceFS;
extern USBD_DescriptorsTypeDef FS_Desc;
static struct t_class_data pcan_data = { 0 };

struct t_pcan_description
{
  USB_CONFIGURATION_DESCRIPTOR con0;
  USB_INTERFACE_DESCRIPTOR     if0;
  USB_ENDPOINT_DESCRIPTOR      ep1;
  USB_ENDPOINT_DESCRIPTOR      ep2;
  USB_ENDPOINT_DESCRIPTOR      ep3;
  USB_ENDPOINT_DESCRIPTOR      ep4;
};

__ALIGN_BEGIN static const USB_DEVICE_QUALIFIER_DESCRIPTOR dev_qua __ALIGN_END = 
{
  .bLength            = sizeof( USB_DEVICE_QUALIFIER_DESCRIPTOR ),
  .bDescriptorType    = USB_QUALIFIER_DESCRIPTOR_TYPE,
  .bcdUSB             = 0x0100, /* 1.0 */
  .bDeviceClass       = 0,
  .bDeviceSubClass    = 0,
  .bDeviceProtocol    = 0,
  .bMaxPacketSize0    = 64,
  .bNumConfigurations = 1,
  .bReserved          = 0,
};

__ALIGN_BEGIN static struct t_pcan_description pcan_usb_dev __ALIGN_END = 
{
  .con0 =
  {
    .bLength              = sizeof( USB_CONFIGURATION_DESCRIPTOR ),
    .bDescriptorType      = USB_CONFIGURATION_DESCRIPTOR_TYPE,
    .wTotalLength         = sizeof( struct t_pcan_description ), 
    .bNumInterfaces       = 1,
    .bConfigurationValue  = 1,
    .iConfiguration       = 0,
    .bmAttributes         = USB_CONFIG_BUS_POWERED,
    .MaxPower             = 100, /* = 200mA */
  },
  .if0 =
  {
    .bLength              = sizeof( USB_INTERFACE_DESCRIPTOR ),
    .bDescriptorType      = USB_INTERFACE_DESCRIPTOR_TYPE,
    .bInterfaceNumber     = 0,
    .bAlternateSetting    = 0,
    .bNumEndpoints        = 4,
    .bInterfaceClass      = 0,
    .bInterfaceSubClass   = 0,
    .bInterfaceProtocol   = 0,
    .iInterface           = 0,
  },
  .ep1 = 
  {
    .bLength              = sizeof( USB_ENDPOINT_DESCRIPTOR ),
    .bDescriptorType      = USB_ENDPOINT_DESCRIPTOR_TYPE,
    .bEndpointAddress     = PCAN_USB_EP_CMDIN, /* PC IN cmd resp */
    .bmAttributes         = USB_ENDPOINT_TYPE_BULK,
    .wMaxPacketSize       = 16,
    .bInterval            = 0,
  },
  .ep2 = 
  {
    .bLength              = sizeof( USB_ENDPOINT_DESCRIPTOR ),
    .bDescriptorType      = USB_ENDPOINT_DESCRIPTOR_TYPE,
    .bEndpointAddress     = PCAN_USB_EP_CMDOUT, /* PC OUT cmd */
    .bmAttributes         = USB_ENDPOINT_TYPE_BULK,
    .wMaxPacketSize       = 16,
    .bInterval            = 0,
  },
  .ep3 = 
  {
    .bLength              = sizeof( USB_ENDPOINT_DESCRIPTOR ),
    .bDescriptorType      = USB_ENDPOINT_DESCRIPTOR_TYPE,
    .bEndpointAddress     = PCAN_USB_EP_MSGIN, /* PC IN frames */
    .bmAttributes         = USB_ENDPOINT_TYPE_BULK,
    .wMaxPacketSize       = 64,
    .bInterval            = 0,
  },
  .ep4 = 
  {
    .bLength              = sizeof( USB_ENDPOINT_DESCRIPTOR ),
    .bDescriptorType      = USB_ENDPOINT_DESCRIPTOR_TYPE,
    .bEndpointAddress     = PCAN_USB_EP_MSGOUT, 
    .bmAttributes         = USB_ENDPOINT_TYPE_BULK,/* PC OUT frames */
    .wMaxPacketSize       = 64,
    .bInterval            = 0,
  },
};


static uint8_t device_init( USBD_HandleTypeDef *pdev, uint8_t cfgidx )
{
  USB_ENDPOINT_DESCRIPTOR *p_ep = &pcan_usb_dev.ep1;

  UNUSED( cfgidx );
  
  for( int i = 0; i < pcan_usb_dev.if0.bNumEndpoints; i++ )
  {
    uint8_t ep_addr = p_ep[i].bEndpointAddress;
    
    if( p_ep[i].bmAttributes == USB_ENDPOINT_TYPE_BULK )
    {
      if( pdev->dev_speed == USBD_SPEED_FULL )
        ;
      else if( pdev->dev_speed == USBD_SPEED_HIGH )
        ;
      else
        assert( 0 );
    }
    
    USBD_LL_OpenEP( pdev, ep_addr,
                          p_ep[i].bmAttributes,
                          p_ep[i].wMaxPacketSize );
    
    if( ( ep_addr & 0x80 ) != 0 )
      pdev->ep_in[ep_addr & EP_ADDR_MSK].is_used = 1;
    else
      pdev->ep_out[ep_addr & EP_ADDR_MSK].is_used = 1;
  }
    
  pdev->pClassData = (void*)&pcan_data;


  USBD_LL_PrepareReceive( pdev, PCAN_USB_EP_CMDOUT, pcan_data.buffer_cmd, sizeof( pcan_data.buffer_cmd ) );
  USBD_LL_PrepareReceive( pdev, PCAN_USB_EP_MSGOUT, pcan_data.buffer_data, sizeof( pcan_data.buffer_data ) );
  
  return USBD_OK;
}

static uint8_t device_deinit( USBD_HandleTypeDef *pdev, uint8_t cfgidx )
{
  USB_ENDPOINT_DESCRIPTOR const *p_ep = &pcan_usb_dev.ep1;

  UNUSED( cfgidx );
  
  for( int i = 0; i < pcan_usb_dev.if0.bNumEndpoints; i++ )
  {
    uint8_t ep_addr = p_ep[i].bEndpointAddress;
    USBD_LL_CloseEP( pdev, ep_addr );
    if( ( ep_addr & 0x80 ) != 0 )
      pdev->ep_in[ep_addr & EP_ADDR_MSK].is_used = 0;
    else
      pdev->ep_out[ep_addr & EP_ADDR_MSK].is_used = 0;
  }
  
  pdev->pClassData = (void*)0;
  return USBD_OK;
}

uint16_t pcan_usb_send_command_buffer( const void *p, uint16_t size )
{
  USBD_HandleTypeDef *pdev = &hUsbDeviceFS;

  if( pdev->ep_in[PCAN_USB_EP_CMDIN & 0xFU].total_length )
    return 0;
  pdev->ep_in[PCAN_USB_EP_CMDIN & 0xFU].total_length = size;
  
  if( USBD_LL_Transmit( pdev, PCAN_USB_EP_CMDIN, (void*)p, size ) == USBD_OK )
    return size;
  return 0;
}

uint16_t pcan_usb_send_data_buffer( const void *p, uint16_t size )
{
  USBD_HandleTypeDef *pdev = &hUsbDeviceFS;

  if( pdev->ep_in[PCAN_USB_EP_MSGIN & 0xFU].total_length )
    return 0;
  pdev->ep_in[PCAN_USB_EP_MSGIN & 0xFU].total_length = size;
  
  if( USBD_LL_Transmit( pdev, PCAN_USB_EP_MSGIN, (void*)p, size ) == USBD_OK )
    return size;
  return 0;
}

void pcan_usb_poll( void )
{
  HAL_PCD_IRQHandler( &hpcd_USB_FS );
}

static uint8_t  device_setup( USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req )
{
  switch( req->bRequest )
  {
    /* get info */
    case 0:
      switch( req->wValue )
      {
        case 0: /* bootloader info */
        {
          static const uint8_t bootloader_info[] = { 
                                  0x00, 0x00, 0x08, 0x04, 0x00, 0x08, 0x07, 0x00,
                                  0x04, 0x02, 0xe0, 0x07, 0x01, 0x00, 0x00, 0x00
                                };
          USBD_CtlSendData( pdev, (void*)bootloader_info, sizeof( bootloader_info ) );

          pcan_led_set_mode( LED_CH0_RX, LED_MODE_ON, 0 );
          pcan_led_set_mode( LED_CH0_TX, LED_MODE_ON, 0 );

          /* reset endpoints */
          pcan_flush_ep( PCAN_USB_EP_MSGIN );
          pcan_flush_ep( PCAN_USB_EP_CMDIN );
        }
        break;
      }
    break;  
  }
  return USBD_OK;
}

static uint8_t device_ep0_rx_ready( USBD_HandleTypeDef *pdev )
{
  UNUSED( pdev );
  return USBD_OK;
}

/* data was sent to PC */
static uint8_t device_data_in( USBD_HandleTypeDef *pdev, uint8_t epnum )
{
  struct t_class_data *p_data = (void*)pdev->pClassData;
  
  if( pdev->pClassData == 0 )
    return USBD_FAIL;
  
/* use ZLP */
#if 0
  PCD_HandleTypeDef *hpcd = pdev->pData;
  uint32_t len = pdev->ep_in[epnum].total_length;
  /* packet is multiple of maxpacket, so tell host what all transfer is done */
  if( len && ( len % hpcd->IN_ep[epnum].maxpacket ) == 0 )
  {
    /* update the packet total length */
    pdev->ep_in[epnum].total_length = 0;
    /* send ZLP */
    if( !p_data->ep_tx_data_pending[epnum] )
    {
      USBD_LL_Transmit( pdev, epnum, NULL, 0 );
    }
    else
    {
      p_data->ep_tx_in_use[epnum] = 0;
    }
  }
  else
  {
    /* tx done, no active transfer */
    p_data->ep_tx_in_use[epnum] = 0;
  }
#else
  pdev->ep_in[epnum].total_length = 0U;
  p_data->ep_tx_in_use[epnum] = 0;
#endif
  return USBD_OK;
}

/* data was received from PC */
static uint8_t device_data_out( USBD_HandleTypeDef *pdev, uint8_t epnum )
{
  int size;
   
  if( pdev->pClassData == 0 )
    return USBD_FAIL;
  
  size = USBD_LL_GetRxDataSize( pdev, epnum );
  (void)size;

  if( epnum == PCAN_USB_EP_CMDOUT )
  {
    pcan_protocol_process_command( pcan_data.buffer_cmd, size );
    USBD_LL_PrepareReceive( pdev, PCAN_USB_EP_CMDOUT, pcan_data.buffer_cmd, sizeof( pcan_data.buffer_cmd ) );
  }
  else if( epnum == PCAN_USB_EP_MSGOUT )
  {
    pcan_protocol_process_data( pcan_data.buffer_data, size );
    USBD_LL_PrepareReceive( pdev, PCAN_USB_EP_MSGOUT, pcan_data.buffer_data, sizeof( pcan_data.buffer_data ) );
  }
  else
  {
    return USBD_FAIL;
  }

  return USBD_OK;
}

static uint8_t *device_get_hs_cfg( uint16_t *length )
{
  *length = sizeof( struct t_pcan_description );
  return (void*)&pcan_usb_dev;
}

static uint8_t *device_get_fs_cfg( uint16_t *length )
{
  *length = sizeof( struct t_pcan_description );
  return (void*)&pcan_usb_dev;
}

static uint8_t *device_get_other_speed_cfg( uint16_t *length )
{
  *length = sizeof( struct t_pcan_description );
  return (void*)&pcan_usb_dev;
}

static uint8_t *device_get_device_qualifier( uint16_t *length )
{
  *length = sizeof( USB_DEVICE_QUALIFIER_DESCRIPTOR );
  
  return (void*)&dev_qua;
}


static uint8_t *device_get_user_string( USBD_HandleTypeDef *pdev, uint8_t index, uint16_t *length )
{
  UNUSED( pdev );
  if( index == 10 )
  {
    __ALIGN_BEGIN static const uint16_t vendor_descriptor[1+24] __ALIGN_END = 
    {
      0x0332,
      'P','E','A','K','-','S','y','s','t','e','m',' ','T','e','c','h','n','i','k',' ','G','m','b','H'
    };
    *length = sizeof( vendor_descriptor );
    return (uint8_t*)vendor_descriptor;
  }
  return 0;
}

USBD_ClassTypeDef usbd_pcan =
{
  .Init = device_init,
  .DeInit = device_deinit,
  .Setup = device_setup,
  .EP0_TxSent = 0,
  .EP0_RxReady = device_ep0_rx_ready,
  .DataIn = device_data_in,
  .DataOut = device_data_out,
  .SOF = 0,
  .IsoINIncomplete = 0,
  .IsoOUTIncomplete = 0,
  .GetHSConfigDescriptor = device_get_hs_cfg,
  .GetFSConfigDescriptor = device_get_fs_cfg,
  .GetOtherSpeedConfigDescriptor = device_get_other_speed_cfg,
  .GetDeviceQualifierDescriptor = device_get_device_qualifier,
#if (USBD_SUPPORT_USER_STRING_DESC == 1U)
  .GetUsrStrDescriptor = device_get_user_string,
#endif
};

void pcan_usb_init( void )
{
  if( USBD_Init( &hUsbDeviceFS, &FS_Desc, DEVICE_FS ) != USBD_OK )
  {
    assert( 0 );
  }

  if( USBD_RegisterClass( &hUsbDeviceFS, &usbd_pcan ) != USBD_OK )
  {
    assert( 0 );
  }

  if( USBD_Start(&hUsbDeviceFS) != USBD_OK )
  {
    assert( 0 );
  }
}

int pcan_flush_ep( uint8_t ep )
{
  USBD_HandleTypeDef *pdev = &hUsbDeviceFS;
  struct t_class_data *p_data = (void*)pdev->pClassData;

  p_data->ep_tx_in_use[ep&0x0F] = 0;
  return USBD_LL_FlushEP( pdev, ep ) == USBD_OK;
}

int pcan_flush_data( struct t_m2h_fsm *pfsm, void *src, int size )
{
  USBD_HandleTypeDef *pdev = &hUsbDeviceFS;
  struct t_class_data *p_data = (void*)pdev->pClassData;

  if( !p_data )
    return 0;

  switch( pfsm->state )
  {
    case 1:
      if( p_data->ep_tx_in_use[pfsm->ep_addr&0x0F] )
      {
        p_data->ep_tx_data_pending[pfsm->ep_addr&0x0F] = size;
        return 0;
      }
      pfsm->state = 0;
      /* fall through */
    case 0:
      assert( p_data->ep_tx_in_use[pfsm->ep_addr&0x0F] == 0 );
      if( size > pfsm->dbsize )
        break;
      memcpy( pfsm->pdbuf, src, size );
      p_data->ep_tx_in_use[pfsm->ep_addr&0x0F] = 1;
      /* prepare data transmit */
      pdev->ep_in[pfsm->ep_addr & EP_ADDR_MSK].total_length = size;
      /* no pending data */
      p_data->ep_tx_data_pending[pfsm->ep_addr&0x0F] = 0;
      USBD_LL_Transmit( pdev, pfsm->ep_addr, pfsm->pdbuf, size );
      
      pfsm->total_tx += size;
      pfsm->state = 1;
      return 1;
  }
  
  return 0;
}
