#pragma once

#include <stdint.h>

/* pcan-usb parameter get an set function */
#pragma pack(push, 1)
typedef struct 
{
	uint8_t  function;
	uint8_t  number;
	uint8_t  param[14];
} 
PCAN_USB_PARAM;
#pragma pack(pop)

/* USB control cmds get/set */
#define PCAN_USB_EX0	    0x00
#define PCAN_USB_GET			0x01
#define PCAN_USB_SET			0x02
#define PCAN_USB_EX3	    0x03

/* PCAN-USB rx/tx buffers size */
#define PCAN_USB_RX_BUFFER_SIZE		64
#define PCAN_USB_TX_BUFFER_SIZE		64

#define PCAN_USB_MSG_HEADER_LEN		2

/* PCAN-USB adapter internal clock (MHz) */
#define PCAN_USB_CRYSTAL_HZ		16000000

/* PCAN-USB USB message record status/len field */
#define PCAN_USB_STATUSLEN_TIMESTAMP		(1 << 7)
#define PCAN_USB_STATUSLEN_INTERNAL	    (1 << 6)
#define PCAN_USB_STATUSLEN_EXT_ID	    	(1 << 5)
#define PCAN_USB_STATUSLEN_RTR		    	(1 << 4)
#define PCAN_USB_STATUSLEN_DLC		    	(0xf)

/* PCAN-USB error flags */
#define PCAN_USB_ERROR_TXFULL			0x01
#define PCAN_USB_ERROR_RXQOVR			0x02
#define PCAN_USB_ERROR_BUS_LIGHT	0x04
#define PCAN_USB_ERROR_BUS_HEAVY	0x08
#define PCAN_USB_ERROR_BUS_OFF		0x10
#define PCAN_USB_ERROR_RXQEMPTY		0x20
#define PCAN_USB_ERROR_QOVR       0x40
#define PCAN_USB_ERROR_TXQFULL		0x80

/* SJA1000 modes */
#define SJA1000_MODE_NORMAL		    0x00
#define SJA1000_MODE_INIT		    	0x01

/*
 * tick duration = 42.666 us =>
 * (tick_number * 44739243) >> 20 ~ (tick_number * 42666) / 1000
 * accuracy = 10^-7
 */
#define PCAN_USB_TS_DIV_SHIFTER		20
#define PCAN_USB_TS_US_PER_TICK		44739243

/* PCAN-USB messages record types */
#define PCAN_USB_REC_ERROR		1
#define PCAN_USB_REC_ANALOG		2
#define PCAN_USB_REC_BUSLOAD  3
#define PCAN_USB_REC_TS				4
#define PCAN_USB_REC_BUSEVT		5

