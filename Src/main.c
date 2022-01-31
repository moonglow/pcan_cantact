#include "stm32f0xx_hal.h"
#include "pcan_timestamp.h"
#include "pcan_led.h"
#include "pcan_protocol.h"
#include "pcan_usb.h"
#include "pcan_varian.h"


void HAL_MspInit( void )
{
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
}

#if ( HSE_VALUE != 0 )
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* enable HSE */
   __HAL_RCC_HSE_CONFIG(RCC_HSE_ON);
  while( __HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY) == RESET );

  /* enable PLL */
  __HAL_RCC_PLL_DISABLE();
  while( __HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY)  != RESET );
#if ( HSE_VALUE == 16000000 )
  __HAL_RCC_PLL_CONFIG( RCC_PLLSOURCE_HSE, RCC_PREDIV_DIV1, RCC_PLL_MUL3 );
#elif ( HSE_VALUE == 8000000 )
  __HAL_RCC_PLL_CONFIG( RCC_PLLSOURCE_HSE, RCC_PREDIV_DIV1, RCC_PLL_MUL6 );
#endif
  __HAL_RCC_PLL_ENABLE();
  while( __HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY)  == RESET );

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  HAL_RCC_ClockConfig( &RCC_ClkInitStruct, FLASH_LATENCY_1 );

  __HAL_RCC_USB_CONFIG( RCC_USBCLKSOURCE_PLL );
}
#else
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_CRSInitTypeDef RCC_CRSInitStruct = {0};

  __HAL_RCC_HSI48_ENABLE();
  while(__HAL_RCC_GET_FLAG(RCC_FLAG_HSI48RDY) == RESET );

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
    |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  __HAL_RCC_USB_CONFIG( RCC_USBCLKSOURCE_HSI48 );

  /* CRS */
  __HAL_RCC_CRS_CLK_ENABLE();
  
  RCC_CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;
  RCC_CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_USB;
  RCC_CRSInitStruct.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  RCC_CRSInitStruct.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE( 48000000, 1000 );
  RCC_CRSInitStruct.ErrorLimitValue = RCC_CRS_ERRORLIMIT_DEFAULT;
  RCC_CRSInitStruct.HSI48CalibrationValue = RCC_CRS_HSI48CALIBRATION_DEFAULT;

  HAL_RCCEx_CRSConfig( &RCC_CRSInitStruct );
}
#endif

void SysTick_Handler( void )
{
  HAL_IncTick();
}

int main( void )
{
  HAL_Init();
  HAL_IncTick();

  SystemClock_Config();

  pcan_variant_io_init();

  pcan_usb_init();
  pcan_led_init();
  pcan_timestamp_init();
  pcan_protocol_init();

  pcan_led_set_mode( LED_CH0_RX, LED_MODE_BLINK_SLOW, 0 );
  pcan_led_set_mode( LED_CH0_TX, LED_MODE_BLINK_SLOW, 0 );

  for(;;)
  {
    pcan_usb_poll();
    pcan_led_poll();
    pcan_protocol_poll();
  }
}
