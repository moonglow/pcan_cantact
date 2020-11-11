#include <assert.h>
#include "stm32f0xx_hal.h"
#include "pcan_timestamp.h"
#include "pcan_led.h"

static struct
{
  uint16_t mode;
  uint16_t arg;
  uint16_t delay;
  uint16_t timestamp;
  uint8_t  state;
}
led_mode_array[LED_TOTAL] = { 0 };

#define IOPIN_TX    GPIO_PIN_1
#define IOPIN_RX    GPIO_PIN_0
#define IOPIN_PORT  GPIOB

void pcan_led_init( void )
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitStruct.Pin = IOPIN_TX |IOPIN_RX;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init( IOPIN_PORT, &GPIO_InitStruct );
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
  uint16_t pin = 0;

  switch( led )
  {
    case LED_CH0_TX:
      pin = IOPIN_TX;
    break;
    case LED_CH0_RX:
      pin = IOPIN_RX;
    break;
    default:
      return;
  }
  HAL_GPIO_WritePin( IOPIN_PORT, pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET );
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
