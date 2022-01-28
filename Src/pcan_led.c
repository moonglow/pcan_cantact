#include <assert.h>
#include "stm32f0xx_hal.h"
#include "pcan_timestamp.h"
#include "pcan_led.h"
#include "pcan_varian.h"

static struct
{
  uint16_t mode;
  uint16_t arg;
  uint16_t delay;
  uint16_t timestamp;
  uint8_t  state;
}
led_mode_array[LED_TOTAL] = { 0 };

void pcan_led_init( void )
{
#ifdef IOPIN_TX
  PIN_ENABLE_CLOCK( IOPIN_TX );
  PIN_INIT( IOPIN_TX );
#endif
#ifdef IOPIN_RX
  PIN_ENABLE_CLOCK( IOPIN_RX );
  PIN_INIT( IOPIN_RX );
#endif
}

void pcan_led_set_mode( int led, int mode, uint16_t arg )
{
  assert( led < LED_TOTAL );
  uint16_t ts = pcan_timestamp_millis();

  led_mode_array[led].mode = mode;
  if( !led_mode_array[led].timestamp )
  {
    led_mode_array[led].timestamp = ts|1;
  }
  led_mode_array[led].delay = 0;

  /* set guard time */
  if( mode == LED_MODE_BLINK_FAST || mode == LED_MODE_BLINK_SLOW )
  {
    led_mode_array[led].delay = ( mode == LED_MODE_BLINK_FAST ) ? 50: 200;
    arg = arg?(ts + arg)|1:0;
  }

  led_mode_array[led].arg  = arg;
}

static void pcan_led_update_state( int led, uint8_t state )
{
  switch( led )
  {
#ifdef IOPIN_TX
    case LED_CH0_TX:
      state ? (LED_ON( IOPIN_TX )): (LED_OFF( IOPIN_TX ));
    break;
#endif
#ifdef IOPIN_RX
    case LED_CH0_RX:
      state ? (LED_ON( IOPIN_RX )): (LED_OFF( IOPIN_RX ));
    break;
#endif
    default:
      (void)state;
      return;
  }
}

void pcan_led_poll( void )
{
  uint16_t ts_ms = pcan_timestamp_millis();

  for( int i = 0; i < LED_TOTAL; i++ )
  {
    if( !led_mode_array[i].timestamp  )
      continue;
    if( (uint16_t)( ts_ms - led_mode_array[i].timestamp ) < led_mode_array[i].delay )
      continue;

    switch( led_mode_array[i].mode )
    {
      default:
      case LED_MODE_NONE:
        led_mode_array[i].timestamp = 0;
      break;
      case LED_MODE_OFF:
      case LED_MODE_ON:
        led_mode_array[i].state = ( led_mode_array[i].mode == LED_MODE_ON );
        led_mode_array[i].timestamp = 0;
      break;
      case LED_MODE_BLINK_FAST:
      case LED_MODE_BLINK_SLOW:
        led_mode_array[i].state ^= 1;
        led_mode_array[i].timestamp += led_mode_array[i].delay;
        led_mode_array[i].timestamp |= 1;
        if( led_mode_array[i].arg && ( led_mode_array[i].arg <= ts_ms ) )
        {
          pcan_led_set_mode( i, LED_MODE_OFF, 0 );
        }
      break;
    }

    pcan_led_update_state( i, led_mode_array[i].state );
  }
}
