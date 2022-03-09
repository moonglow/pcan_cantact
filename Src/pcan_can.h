#pragma once
#include <stdint.h>

#define CAN_FLAG_ECHO  (0x20)
#define CAN_FLAG_RTR   (0x40)
#define CAN_FLAG_EXTID (0x80)

#define CAN_ERROR_FLAG_BUSOFF    (1<<0)
#define CAN_ERROR_FLAG_RX_OVF    (1<<1)
#define CAN_ERROR_FLAG_TX_ERR    (1<<2)

typedef struct
{
  uint32_t  id;
  uint8_t   data[8];
  uint8_t   dlc;
  uint8_t   flags;
  /* for self receive */
  uint8_t   dummy;
  /* in pcan ticks */
  uint16_t timestamp;
}
can_message_t;

void pcan_can_init( void );
void pcan_can_poll( void );
void pcan_can_set_bitrate( uint16_t brp, uint8_t tseg1, uint8_t tseg2, uint8_t sjw );
int pcan_can_send_message( const can_message_t *msg );
void pcan_can_install_rx_callback( void (*cb)( can_message_t * ) );
void pcan_can_install_error_callback( void (*cb)( uint8_t, uint8_t, uint8_t ) );
void pcan_can_set_bus_active( uint16_t mode );
void pcan_can_set_silent( uint8_t silent_mode );
void pcan_can_set_loopback( uint8_t loopback );
