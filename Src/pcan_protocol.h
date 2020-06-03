#pragma once
#include <stdint.h>

void pcan_protocol_init( void );
void pcan_protocol_poll( void );
void pcan_protocol_process_command( uint8_t *ptr, uint16_t size );
void pcan_protocol_process_data( uint8_t *ptr, uint16_t size );
