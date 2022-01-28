#pragma once
#include <stm32f0xx_hal.h>

#define GPIO_NOAF (0u)
#define _PIN_INIT( _PORT, _PIN, _MODE, _PULL, _SPEED, _AF )\
HAL_GPIO_Init( GPIO##_PORT, (GPIO_InitTypeDef[])\
{{\
  .Pin = GPIO_PIN_##_PIN,\
  .Mode = GPIO_##_MODE,\
  .Pull = GPIO_##_PULL,\
  .Speed = GPIO_##_SPEED,\
  .Alternate = GPIO_##_AF\
}} )
#define _PIN_HI( _PORT, _PIN, ... )        GPIO##_PORT->BSRR = (1u<<_PIN)
#define _PIN_LOW( _PORT, _PIN, ... )       GPIO##_PORT->BSRR = (0x10000u<<_PIN)
#define _PIN_TOGGLE( _PORT, _PIN, ... )\
do{\
  uint32_t odr = GPIO##_PORT->ODR;\
  GPIO##_PORT->BSRR = ((odr & (1u<<_PIN)) << 16u) | (~odr & (1u<<_PIN));\
}while(0)

#define _PIN_ENABLE_CLOCK( _PORT, ... ) __HAL_RCC_GPIO ## _PORT ## _CLK_ENABLE()

#define PIN_HI( CONFIG )      _PIN_HI( CONFIG )
#define PIN_LOW( CONFIG )     _PIN_LOW( CONFIG )
#define PIN_TOGGLE( CONFIG )  _PIN_TOGGLE( CONFIG )
#define PIN_INIT( CONFIG )    _PIN_INIT( CONFIG )
#define PIN_ENABLE_CLOCK( CONFIG ) _PIN_ENABLE_CLOCK( CONFIG )