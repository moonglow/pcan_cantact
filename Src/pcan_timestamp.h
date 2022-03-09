#pragma once
#include <stdint.h>

#define PCAN_TICKS_FROM_US( _us ) (((uint32_t)_us*1000u)/42666u)

void pcan_timestamp_init( void );
uint16_t pcan_timestamp_ticks( void );
uint16_t pcan_timestamp_millis( void );
