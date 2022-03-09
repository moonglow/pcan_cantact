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
#define PCAN_USB_EX0      0x00
#define PCAN_USB_GET      0x01
#define PCAN_USB_SET      0x02
#define PCAN_USB_EX3      0x03

/* PCAN-USB commands */
#define PCAN_USB_CMD_BITRATE	1
#define PCAN_USB_CMD_CLOCK	  2
#define PCAN_USB_CMD_BUS	    3
#define PCAN_USB_CMD_DEVID	  4
#define PCAN_USB_CMD_CFG		  5
#define PCAN_USB_CMD_SN		    6
#define PCAN_USB_CMD_REGISTER	9
#define PCAN_USB_CMD_EXT_VCC	10
#define PCAN_USB_CMD_ERR_FR	  11
#define PCAN_USB_CMD_LED	    12
#define PCAN_USB_CMD_DEVDATA	30
/* USB Mass Storage Mode command (FW >= 8.3.0) */
#define	PCAN_USB_SETCAN2FLASH		0xC8

/* PCAN_USB_CMD_BUS PCAN_USB_SET extension: */
#define PCAN_USB_SET_SILENT_MODE	3

/* PCAN-USB rx/tx buffers size */
#define PCAN_USB_RX_BUFFER_SIZE    64
#define PCAN_USB_TX_BUFFER_SIZE    64

#define PCAN_USB_MSG_HEADER_LEN    2

/* PCAN-USB adapter internal clock (MHz) */
#define PCAN_USB_CRYSTAL_HZ    16000000

/* PCAN-USB USB message record status/len field */
#define PCAN_USB_STATUSLEN_TIMESTAMP    (1 << 7)
#define PCAN_USB_STATUSLEN_INTERNAL     (1 << 6)
#define PCAN_USB_STATUSLEN_EXT_ID       (1 << 5)
#define PCAN_USB_STATUSLEN_RTR          (1 << 4)
#define PCAN_USB_STATUSLEN_DLC          (0xf)

/* PCAN-USB error flags */
#define PCAN_USB_ERROR_TXFULL      0x01
#define PCAN_USB_ERROR_RXQOVR      0x02
#define PCAN_USB_ERROR_BUS_LIGHT   0x04
#define PCAN_USB_ERROR_BUS_HEAVY   0x08
#define PCAN_USB_ERROR_BUS_OFF     0x10
#define PCAN_USB_ERROR_RXQEMPTY    0x20
#define PCAN_USB_ERROR_QOVR        0x40
#define PCAN_USB_ERROR_TXQFULL     0x80

/* SJA1000 registers */
#define SJA1000_MOD     0	/* mode register */
#define SJA1000_CMR		  1
#define SJA1000_SR		  2
#define SJA1000_IR		  3
#define SJA1000_IER		  4	/* acceptance code */
#define SJA1000_BTR0		6	/* bus timing 0 */
#define SJA1000_BTR1		7	/* bus timing 1 */
#define SJA1000_OCR		  8	/* output control */
#define SJA1000_TR		  9
#define SJA1000_CDR     31

/* SJA1000 modes */
#define SJA1000_MODE_NORMAL        0x00
#define SJA1000_MODE_INIT          0x01

/*
 * tick duration = 42.666 us =>
 * (tick_number * 44739243) >> 20 ~ (tick_number * 42666) / 1000
 * accuracy = 10^-7
 */
#define PCAN_USB_TS_DIV_SHIFTER    20
#define PCAN_USB_TS_US_PER_TICK    44739243

/* PCAN-USB messages record types */
#define PCAN_USB_REC_ERROR     1
#define PCAN_USB_REC_ANALOG    2
#define PCAN_USB_REC_BUSLOAD   3
#define PCAN_USB_REC_TS        4
#define PCAN_USB_REC_BUSEVT    5
