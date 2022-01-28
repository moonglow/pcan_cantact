#include <assert.h>
#include <stdint.h>
#include <string.h>
#include "pcan_can.h"
#include "pcan_usb.h"
#include "pcan_packet.h"
#include "pcan_led.h"
#include "pcan_timestamp.h"
#include "punker.h"

#define SJA1000_BASICCAN  0
#define SJA1000_PELICAN   1
static struct
{
  uint8_t bus_active;
  uint8_t ext_vcc_state;
  uint8_t led_mode;

  uint16_t last_timestamp_sync;
  uint16_t last_flush;
  struct
  {
    /* error handling related */
    uint8_t   err;
    uint8_t   ecc;
    uint8_t   rx_err;
    uint8_t   tx_err;
    /* config */
    uint8_t   btr0;
    uint8_t   btr1;
    uint8_t   silient;
    uint8_t   loopback;
    uint8_t   err_mask;
    uint32_t  quartz_freq;
  }
  can;
  uint8_t   sja1000_shadow[6];
  uint8_t   sja1000_ic_mode;
  uint8_t   device_id;
  uint32_t  device_serial;
}
pcan_device =
{
  .device_id = 0xFF,
  .device_serial = 0xFFFFFFFF,

  .can = 
  {
    .quartz_freq = PCAN_USB_CRYSTAL_HZ,
    .loopback = 0,
  },
};

#define BLOCK_SIZE  (64)
#define HEADER_SIZE (2)
#define PCAN_MAX_RECORD_SIZE (BLOCK_SIZE*14)
typedef struct
{
  /* raw data array */
  uint8_t buffer[PCAN_MAX_RECORD_SIZE];
  /* ts counter, need to provide correct timestamp correction ( 2 or 1 byte ) */
  uint16_t ts_counter;
  uint16_t pos;
}
PCAN_RECORD_BUFFER_EX;

static uint8_t temp_data_buffer[PCAN_MAX_RECORD_SIZE] = { 0 };
static struct t_m2h_fsm data_fsm = 
{
  .state = 0,
  .ep_addr = PCAN_USB_EP_MSGIN,
  .pdbuf = temp_data_buffer,
  .dbsize = sizeof( temp_data_buffer ),
};

static PCAN_RECORD_BUFFER_EX pcan_records;

void pcan_sja1000_write( uint8_t reg_addr, uint8_t value )
{
  if( pcan_device.sja1000_ic_mode == SJA1000_PELICAN )
  {
    reg_addr &= 0x7F;
    switch( reg_addr )
    {
      case 6:
        pcan_device.sja1000_shadow[2] = value;
      break;
      case 7:
        pcan_device.sja1000_shadow[3] = value;
      break;
      case 14:
        pcan_device.sja1000_shadow[4] = value;
      break;
      case 15:
        pcan_device.sja1000_shadow[5] = ( value == 0xFF ) ? 0x7F: value;
      break;
      case 16:
        pcan_device.sja1000_shadow[0] = value;
      break;
      case 20:
        pcan_device.sja1000_shadow[1] = value;
      break;
      case 31:
        if( (value & 0x80 ) == 0 )
        {
          pcan_device.sja1000_ic_mode = SJA1000_BASICCAN;
        }
      break;
    }
  }
  else if( pcan_device.sja1000_ic_mode == SJA1000_BASICCAN )
  {
    reg_addr &= 0x1F;
    switch( reg_addr )
    {
      case 4:
        pcan_device.sja1000_shadow[0] = value;
      break;
      case 5:
        pcan_device.sja1000_shadow[1] = value;
      break;
      case 6:
        pcan_device.sja1000_shadow[2] = value;
      break;
      case 7:
        pcan_device.sja1000_shadow[3] = value;
      break;
      case 31:
        if( (value & 0x80 ) != 0 )
        {
          pcan_device.sja1000_ic_mode = SJA1000_PELICAN;
        }
      break;
    }
  }
}

uint8_t pcan_sja1000_read( uint8_t reg_addr )
{
  if( pcan_device.sja1000_ic_mode == SJA1000_PELICAN )
  {
    reg_addr &= 0x7F;
    switch( reg_addr )
    {
      case 0:
        return 0x01;
      case 6:
        return pcan_device.sja1000_shadow[2];
      case 7:
        return pcan_device.sja1000_shadow[3];
      case 14:
        return pcan_device.sja1000_shadow[4];
      case 15:
        return pcan_device.sja1000_shadow[5];
      case 16:
        return pcan_device.sja1000_shadow[0];
      case 20:
        return pcan_device.sja1000_shadow[1];
      case 31:
        return 0x80; /* PeliCAN mode */
      default:
        return 0;
    }
  }
  else if( pcan_device.sja1000_ic_mode == SJA1000_BASICCAN )
  {
    reg_addr &= 0x1F;
    switch( reg_addr )
    {
      case 0:
        return 0x01;
      case 4:
        return pcan_device.sja1000_shadow[0];
      case 5:
        return pcan_device.sja1000_shadow[1];
      case 6:
        return pcan_device.sja1000_shadow[2];
      case 7:
        return pcan_device.sja1000_shadow[3];
      case 31:
        return 0x00; /* BasicCAN mode */
      default:
        return 0;
    }
  }

  return 0;
}

void pcan_record_write_header( uint8_t *p )
{
  p[0] = 0x02; /* record header */
  p[1] = 0x00; /* record count */
}

void pcan_record_buffer_reset( PCAN_RECORD_BUFFER_EX *prec )
{
  pcan_record_write_header( prec->buffer );
  prec->ts_counter = 0;
  prec->pos = HEADER_SIZE;
}

/* request free slot for specific amount of data */
uint8_t *pcan_record_buffer_request( PCAN_RECORD_BUFFER_EX *prec, uint16_t size )
{
  if( size > (BLOCK_SIZE-HEADER_SIZE) )
    return 0;
  uint16_t unused = BLOCK_SIZE - ( prec->pos & (BLOCK_SIZE-1) );
  /* allocate new slot ? */
  if( unused < size )
  {
    /* align data to next slot bound */
    uint8_t next = ( prec->pos + (BLOCK_SIZE-1) ) & (~((BLOCK_SIZE-1)));
    if( (next+HEADER_SIZE) >= PCAN_MAX_RECORD_SIZE )
      return 0;
    memset( &prec->buffer[next], 0x00, BLOCK_SIZE );
    pcan_record_write_header( &prec->buffer[next] );
    next += 2;
    prec->pos = next;
    /* reset ts_counter for next buffer */
    prec->ts_counter = 0;
  }

  if( ( prec->pos + size ) > PCAN_MAX_RECORD_SIZE )
    return (void*)0;

  return &prec->buffer[prec->pos];
}

void pcan_record_buffer_commit( PCAN_RECORD_BUFFER_EX *prec, uint16_t size )
{
  uint16_t pos = prec->pos&(~(BLOCK_SIZE-1));
  /* increse record count */
  ++prec->buffer[pos+1];
  prec->pos += size;
}

uint16_t pcan_record_buffer_flush( PCAN_RECORD_BUFFER_EX *prec )
{
  uint16_t ts_ticks = pcan_timestamp_ticks();
  int res, flush_size;

  if( prec->pos <= HEADER_SIZE )
  {
    /* try to send ZLP, original 800 */
    if( pcan_device.last_flush && (((uint16_t)(ts_ticks - pcan_device.last_flush)) >= PCAN_TICKS_FROM_US( 800 )) )
    {
      res = pcan_flush_data( &data_fsm, 0, 0 );
      if( res )
      {
        pcan_device.last_flush = 0;
      }
    }
    return 0;
  }

  flush_size = ( prec->pos + (BLOCK_SIZE-1) ) & (~((BLOCK_SIZE-1)));
  res = pcan_flush_data( &data_fsm, prec->buffer, flush_size );
  if( res )
  {
    pcan_record_buffer_reset( prec );
    pcan_device.last_flush = ts_ticks;
    return 1;
  }

  return 0;
}

#if 0
static void pcan_bus_event( PCAN_RECORD_BUFFER_EX *prec, uint8_t flags )
{
  uint8_t *ptr, sl, pos = 0;

  ptr = pcan_record_buffer_request( prec, 6 );
  if( !ptr )
    return;

  /* sl */
  sl = PCAN_USB_STATUSLEN_INTERNAL | 0x03;
  pack_u8( ptr, sl );
  ptr += 1;
  pos += 1;

  /* function */
  pack_u8( ptr, PCAN_USB_REC_BUSEVT );
  ptr += 1;
  pos += 1;

  /* number 0x80, 0x00 */
  pack_u8( ptr, flags );
  ptr += 1;
  pos += 1;

  /* ecc */
  pack_u8( ptr, pcan_device.can.ecc );
  ptr += 1;
  pos += 1;

  /* rx error counter */
  pack_u8( ptr, pcan_device.can.rx_err );
  ptr += 1;
  pos += 1;

  /* tx error counter */
  pack_u8( ptr, pcan_device.can.tx_err );
  ptr += 1;
  pos += 1;  

  /* add new record */
  pcan_record_buffer_commit( prec, pos );  
}
#endif

#define BUFFER_MINIMAL_GAP_SIZE (16u)
static void pcan_rx_can_frame( PCAN_RECORD_BUFFER_EX *prec, const can_message_t *msg )
{
  uint8_t *ptr, pos = 0;

  ptr = pcan_record_buffer_request( prec, BUFFER_MINIMAL_GAP_SIZE );
  /* wow ! too fast too furious !!! */
  if( !ptr )
  {
    if( !pcan_record_buffer_flush( prec ) )
    {
      pcan_device.can.err |= PCAN_USB_ERROR_RXQOVR;
      return;
    }
    ptr = pcan_record_buffer_request( prec, BUFFER_MINIMAL_GAP_SIZE );
    /* o_0 */
    if( !ptr )
      return;
  }
  /* recheck dlc for abnormal frames... */
  uint8_t sl = msg->dlc > 8 ? 8: msg->dlc;

  if( msg->flags & CAN_FLAG_EXTID )
  {
    sl |= PCAN_USB_STATUSLEN_EXT_ID;
  }
  if( msg->flags & CAN_FLAG_RTR )
  {
    sl |= PCAN_USB_STATUSLEN_RTR;
  }

  pack_u8( ptr, sl );

  ptr += 1;
  pos += 1;

  if( sl & PCAN_USB_STATUSLEN_EXT_ID )
  {
    uint32_t id = ( msg->id & 0x1FFFFFFF ) << 3;
    if( msg->flags & CAN_FLAG_ECHO )
    {
      id |= 0x01;
    }

    pack_u32( ptr, id );

    ptr += 4;
    pos += 4;
  }
  else
  {
    uint16_t id = ( msg->id & 0x7FF ) << 5;
    if( msg->flags & CAN_FLAG_ECHO )
    {
      id |= 0x01;
    }
    pack_u16( ptr, id );

    ptr += 2;
    pos += 2;
  }
  
  /* timestamp to ticks, 1 tick is 42.666us */
  uint16_t timestamp16 = msg->timestamp;

  if( prec->ts_counter == 0 )
  {
    pack_u16( ptr, timestamp16 );
    ptr += 2;
    pos += 2;  
  }
  else
  {
    pack_u8( ptr, timestamp16&0xFF );
    ptr += 1;
    pos += 1;  
  }
  ++prec->ts_counter;

  if( !( sl & PCAN_USB_STATUSLEN_RTR ) )
  {
    memcpy( ptr, msg->data, msg->dlc );

    ptr += msg->dlc;
    pos += msg->dlc;  
  }

  if( msg->flags & CAN_FLAG_ECHO )
  {
    pack_u8( ptr, msg->dummy );
    ptr += 1;
    pos += 1;
  }

  /* add new record */
  pcan_record_buffer_commit( prec, pos );
}

static void pcan_rx_message( can_message_t *msg )
{
  if( !pcan_device.bus_active )
    return;
  if( ( msg->flags & CAN_FLAG_ECHO ) == 0 )
  {
    pcan_led_set_mode( LED_CH0_RX, LED_MODE_BLINK_FAST, 237 );
  }

  pcan_rx_can_frame( &pcan_records, msg );
}

static void pcan_can_error( uint8_t err, uint8_t rx_err, uint8_t tx_err )
{
  pcan_device.can.rx_err = rx_err;
  pcan_device.can.tx_err = tx_err;

  if( err & CAN_ERROR_FLAG_BUSOFF )
  {
    pcan_device.can.err |= PCAN_USB_ERROR_BUS_OFF; 
  }
}

#define WAIT_FOR_TXSLOTS 1
static uint8_t pcan_decode_data_frame( uint8_t *ptr, uint16_t size, uint8_t flags  )
{
  can_message_t msg = { 0 };
  uint8_t rec_len = flags & PCAN_USB_STATUSLEN_DLC;
  /* return back frame to PC */
  uint8_t srr_flag = 0;

  if( !pcan_device.bus_active )
    return 0;

  if( flags & PCAN_USB_STATUSLEN_EXT_ID )
  {
    uint32_t id;
    if( size < sizeof( uint32_t ) )
      return 0;
    id = unpack_u32( ptr );
    srr_flag = id & 0x01;
    id >>= 3;

    msg.flags = CAN_FLAG_EXTID;
    msg.id = id;

    ptr += sizeof( uint32_t );
    size -= sizeof( uint32_t );
  }
  else
  {
    uint16_t id;
    if( size < sizeof( uint16_t ) )
      return 0;
    id = unpack_u16( ptr );
    srr_flag = id & 0x01;
    id >>= 5;

    msg.id = id;

    ptr += sizeof( uint16_t );
    size -= sizeof( uint16_t );
  }

  /* check for MAX DLC */
  msg.dlc = rec_len < 8 ? rec_len: 8;
  
  if( size < rec_len )
      return 0;

  if( !( flags & PCAN_USB_STATUSLEN_RTR ) )
  {
    memcpy( msg.data, ptr, msg.dlc );

    ptr  += rec_len;
    size -= rec_len;
  }
  else
  {
    msg.flags |= CAN_FLAG_RTR;
  }

  /* self receive flag ? */
  if( srr_flag )
  {
    if( size < sizeof( uint8_t ) )
      return 0;
    
    msg.dummy = unpack_u8( ptr );

    ptr  += sizeof( uint8_t );
    size -= sizeof( uint8_t );
  }

  msg.timestamp = pcan_timestamp_ticks();

  pcan_led_set_mode( LED_CH0_TX, LED_MODE_BLINK_FAST, 237 );

#if WAIT_FOR_TXSLOTS
  const uint16_t ts_poll = pcan_timestamp_ticks();
  /* need more time to send data... ? */
  while( pcan_can_send_message( &msg ) < 0 )
  {
    /* USB will get NACK and we will not miss other data */
    pcan_can_poll();
    uint16_t ts_diff = pcan_timestamp_ticks() - ts_poll;
    /* we can't tramsit couse bus off or timeout ? */
    if( ( pcan_device.can.err & PCAN_USB_ERROR_BUS_OFF ) ||
        ( ts_diff >= PCAN_TICKS_FROM_US( 1000000u ) ) )
    {
      /* tx buffer overflow, drop all data */
      pcan_device.can.err |= PCAN_USB_ERROR_TXFULL;
      return size;
    }
  }
#else
  if( pcan_can_send_message( &msg ) < 0 )
  {
    /* tx buffer overflow, drop all data */
    pcan_device.can.err |= PCAN_USB_ERROR_TXFULL;
    return size;
  }
#endif
  /* return back to PC */
  if( srr_flag )
  {
    msg.flags |= CAN_FLAG_ECHO;
    pcan_rx_message( &msg );
  }
  return size;
}

static void pcan_set_bitrate( uint8_t *ptr )
{
  /* decode BTR1 */
  volatile uint8_t tseg1 = (ptr[0] & 0xF) + 1;
  volatile uint8_t tseg2 = ((ptr[0]>>4) & 0x7) + 1;

  /* decode BTR0 */
  volatile uint16_t brp = (ptr[1] & 0x3f) + 1;
  volatile uint8_t sjw = ((ptr[1]>>6) & 0x3) + 1;

  uint32_t bitrate = (((PCAN_USB_CRYSTAL_HZ/2)/brp)/(1/*tq*/ + tseg1 + tseg2 ));
  (void)bitrate;
  
  pcan_can_set_bitrate( brp, tseg1, tseg2, sjw );
}


/* create */
void pcan_timesync_event( PCAN_RECORD_BUFFER_EX *prec )
{
  uint8_t sl = PCAN_USB_STATUSLEN_INTERNAL | 2 /* record data size */;
  uint8_t *ptr, pos = 0;

  ptr = pcan_record_buffer_request( prec, 5 );
  if( !ptr )
    return;
  
  uint16_t timestamp16 = pcan_timestamp_ticks();

  pack_u8( ptr, sl );
  ptr += 1;
  pos += 1;

  /* function */
  pack_u8( ptr, PCAN_USB_REC_TS );
  ptr += 1;
  pos += 1;

  /* number */
  pack_u8( ptr, 1 );
  ptr += 1;
  pos += 1;

  /* timestamp */
  pack_u16( ptr, timestamp16 );
  ptr += 2;
  pos += 2;

  /* add new record */
  pcan_record_buffer_commit( prec, pos );

  /* error record */
  pos = 0;
  ptr = pcan_record_buffer_request( prec, 3 );
  if( !ptr )
    return;

  pcan_device.can.err &= pcan_device.can.err_mask;

  pack_u8( ptr, PCAN_USB_STATUSLEN_INTERNAL );
  ptr += 1;
  pos += 1;

  pack_u8( ptr, PCAN_USB_REC_ERROR ); /* f */
  ptr += 1;
  pos += 1;

  pack_u8( ptr, pcan_device.can.err ); /* n */ 
  ptr += 1;
  pos += 1;

  /* add new record */
  pcan_record_buffer_commit( prec, pos );

  /* clean errors */
  pcan_device.can.err = 0x00;
}

void pcan_protocol_process_command( uint8_t *ptr, uint16_t size )
{
  PCAN_USB_PARAM *cmd = (void*)ptr;

  if( size < PCAN_USB_MSG_HEADER_LEN )
    return;

  switch( cmd->number )
  {
    case PCAN_USB_EX0:
      switch( cmd->function )
      {
        /* set mass storage mode */
        case PCAN_USB_SETCAN2FLASH:
        break;
      }
    break;
    case PCAN_USB_EX3:
      switch( cmd->function )
      {
        /*  CAN silient control*/
        case PCAN_USB_SET_SILENT_MODE:
          pcan_device.can.silient = cmd->param[0];
          pcan_can_set_silent( pcan_device.can.silient );
        break;
      }
    break;
    case PCAN_USB_SET:
      switch( cmd->function )
      {
        /* BTR0 BTR1 */
        case PCAN_USB_CMD_BITRATE:
          pcan_device.can.btr0 = cmd->param[1];
          pcan_device.can.btr1 = cmd->param[0];
          pcan_set_bitrate( cmd->param );
        break;
        /* set CAN on/off*/
        case PCAN_USB_CMD_BUS:
          pcan_device.bus_active = cmd->param[0];
          pcan_led_set_mode( LED_STAT, pcan_device.bus_active ? LED_MODE_BLINK_SLOW:LED_MODE_OFF, 0 );
          pcan_can_set_bus_active( pcan_device.bus_active );

          if( pcan_device.bus_active )
          {
            /* provide first timesync event */
            pcan_device.last_timestamp_sync = pcan_timestamp_ticks();
            pcan_timesync_event( &pcan_records );
          }
          /* led state */
          pcan_led_set_mode( LED_CH0_TX, pcan_device.bus_active ? LED_MODE_OFF:LED_MODE_ON, 0 );
          pcan_led_set_mode( LED_CH0_RX, pcan_device.bus_active ? LED_MODE_OFF:LED_MODE_ON, 0 );
        break;
        /* set device id 0-255 */
        case PCAN_USB_CMD_DEVID:
          pcan_device.device_id = cmd->param[0];
        break;
        /* unknown windows driver call */
        case PCAN_USB_CMD_CFG:
        break;
        /* sja1000 reg write */
        case PCAN_USB_CMD_REGISTER:
          pcan_sja1000_write( cmd->param[0], cmd->param[1] );
        break;
        /* ext VCC on/off */
        case PCAN_USB_CMD_EXT_VCC:
          pcan_device.ext_vcc_state = cmd->param[0];
        break;
        /* error frame mask */
        case PCAN_USB_CMD_ERR_FR:
          pcan_device.can.err_mask = cmd->param[0];
        break;
        /* set led mode */
        case PCAN_USB_CMD_LED:
          pcan_device.led_mode = cmd->param[0];
        break;
      }
    break;
    case PCAN_USB_GET:
      switch( cmd->function )
      {
        /* BTR0 BTR1 */
        case PCAN_USB_CMD_BITRATE:
          cmd->param[1] = pcan_device.can.btr0;
          cmd->param[0] = pcan_device.can.btr1;
          pcan_usb_send_command_buffer( cmd, sizeof( PCAN_USB_PARAM ) );
        break;
        /* quarts freq in Mhz */
        case PCAN_USB_CMD_CLOCK:
        {
          cmd->param[0] = pcan_device.can.quartz_freq/1000000u;
          pcan_usb_send_command_buffer( cmd, sizeof( PCAN_USB_PARAM ) );
        }
        break;
        /* get device id */
        case PCAN_USB_CMD_DEVID:
        {
          /* default: 255 */
          cmd->param[0] = pcan_device.device_id;
          pcan_usb_send_command_buffer( cmd, sizeof( PCAN_USB_PARAM ) );
        }
        break;
        /* serial */
        case PCAN_USB_CMD_SN:
        {
          /* default: 4 bytes FFFFFFFF */
          cmd->param[0] = (pcan_device.device_serial>>0x18)&0xFF;
          cmd->param[1] = (pcan_device.device_serial>>0x10)&0xFF;
          cmd->param[2] = (pcan_device.device_serial>>0x08)&0xFF;
          cmd->param[3] = (pcan_device.device_serial>>0x00)&0xFF;
          pcan_usb_send_command_buffer( cmd, sizeof( PCAN_USB_PARAM ) );
        }
        break;
        /* sja1000 reg read */
        case PCAN_USB_CMD_REGISTER:
          cmd->param[1] = pcan_sja1000_read( cmd->param[0] );
          pcan_usb_send_command_buffer( cmd, sizeof( PCAN_USB_PARAM ) );
        break;
        /* get led */
        case PCAN_USB_CMD_LED:
          cmd->param[0] = pcan_device.led_mode;
          pcan_usb_send_command_buffer( cmd, sizeof( PCAN_USB_PARAM ) );
        break;
      }
    break;
    default:
      assert( 0 );
    break;
  }  
}

void pcan_protocol_process_data( uint8_t *ptr, uint16_t size )
{
  uint8_t msg_prefix, msg_rec_count;

  /* invalid frame header len ? */
  if( size < PCAN_USB_MSG_HEADER_LEN )
    return;

  msg_prefix = *ptr++;
  msg_rec_count = *ptr++;

  (void)msg_prefix; /* avoid warning */

  size -= PCAN_USB_MSG_HEADER_LEN;

  for( uint8_t i = 0; i < msg_rec_count; i++ )
  {
    uint16_t eated = 0, sl;

    if( size < 1 )
      return;

    sl = unpack_u8( ptr );
    ptr += sizeof( uint8_t );
    size -= sizeof( uint8_t );;

    /* status, error frame */
    if( sl & PCAN_USB_STATUSLEN_INTERNAL )
    {
      /* ??? abnormal, device support only CAN data frame */
      eated = 0;
    }
    /* regular CAN data frame */
    else
    {
      eated = pcan_decode_data_frame( ptr, size, sl );
    }

    /* ? error indicator */
    if( !eated )
      return;
    
    eated = size - eated;

    if( size < eated )
      return;

    ptr += eated;
    size -= eated;
  }
}

void pcan_protocol_init( void )
{
  pcan_device.sja1000_ic_mode = SJA1000_PELICAN;
  pcan_record_buffer_reset( &pcan_records );
  pcan_can_init();
  pcan_can_install_rx_callback( pcan_rx_message  );
  pcan_can_install_error_callback( pcan_can_error );
}

void pcan_protocol_poll( void )
{
  uint16_t ts_ms = pcan_timestamp_ticks();

  if( pcan_device.bus_active  )
  {
    /* each ~1000 ms */
    if( ( ((uint16_t)( ts_ms - pcan_device.last_timestamp_sync )) >= PCAN_TICKS_FROM_US( 1000000u ) ) )
    {
      pcan_device.last_timestamp_sync = ts_ms;
      pcan_timesync_event( &pcan_records );
    }
  }
  pcan_record_buffer_flush( &pcan_records );

  pcan_can_poll();
}
