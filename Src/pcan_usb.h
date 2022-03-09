#pragma once
#include <stdint.h>

/* PCAN-USB Endpoints */
#define PCAN_USB_EP_CMDOUT  0x01
#define PCAN_USB_EP_CMDIN   0x81
#define PCAN_USB_EP_MSGOUT  0x02
#define PCAN_USB_EP_MSGIN   0x82

struct t_class_data
{
  uint16_t  ep_tx_data_pending[15];
  uint8_t   ep_tx_in_use[15];
  uint8_t   buffer_cmd[16];
  uint8_t   buffer_data[64];
};

struct t_m2h_fsm
{
  uint8_t   state;
  uint8_t   ep_addr;
  uint8_t   *pdbuf;
  int       dbsize;
  uint32_t  total_tx;
};

int pcan_flush_ep( uint8_t ep );
int pcan_flush_data( struct t_m2h_fsm *pfsm, void *src, int size );

uint16_t pcan_usb_send_command_buffer( const void *p, uint16_t size );
uint16_t pcan_usb_send_data_buffer( const void *p, uint16_t size );
void pcan_usb_init( void );
void pcan_usb_poll( void );

