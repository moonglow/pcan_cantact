#pragma once
#include <stdint.h>

uint16_t pcan_usb_send_command_buffer( const void *p, uint16_t size );
uint16_t pcan_usb_send_data_buffer( const void *p, uint16_t size );
void pcan_usb_init( void );
void pcan_usb_poll( void );

