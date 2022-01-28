#pragma once

#include "io_macro.h"

#if (defined CANABLE) || (defined ENTREE) || (defined CANTACT_8 ) || (defined CANTACT_16 )
#define IOPIN_TX    B, 1, MODE_OUTPUT_PP, NOPULL, SPEED_FREQ_MEDIUM, NOAF
#define IOPIN_RX    B, 0, MODE_OUTPUT_PP, NOPULL, SPEED_FREQ_MEDIUM, NOAF
#define LED_ON      PIN_HI
#define LED_OFF     PIN_LOW

#define CAN_RX      B, 8, MODE_AF_PP, NOPULL, SPEED_FREQ_HIGH, AF4_CAN
#define CAN_TX      B, 9, MODE_AF_PP, NOPULL, SPEED_FREQ_HIGH, AF4_CAN

#define pcan_variant_io_init()
#elif (defined OLLIE)
#define CAN_RX      B, 8, MODE_AF_PP, NOPULL, SPEED_FREQ_HIGH, AF4_CAN
#define CAN_TX      B, 9, MODE_AF_PP, NOPULL, SPEED_FREQ_HIGH, AF4_CAN

#define pcan_variant_io_init()
#else
#error Unknown board variant
#endif

