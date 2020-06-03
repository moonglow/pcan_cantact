#include "stm32f0xx_hal.h"
#include "pcan_timestamp.h"
#include "pcan_led.h"
#include "pcan_protocol.h"
#include "pcan_usb.h"

void HAL_MspInit( void )
{
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};


  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;

  /* HSE = 16MHZ */
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
  /* HSE = 8MHZ */
  //RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;

  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  HAL_RCC_ClockConfig( &RCC_ClkInitStruct, FLASH_LATENCY_1 );

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;

  HAL_RCCEx_PeriphCLKConfig( &PeriphClkInit );
}

void SysTick_Handler( void )
{
  HAL_IncTick();
}

int main( void )
{
  HAL_Init();

  SystemClock_Config();
  
  pcan_usb_init();
  pcan_led_init();
  pcan_timestamp_init();
  pcan_protocol_init();

  for(;;)
  {
    pcan_usb_poll();
    pcan_led_poll();
    pcan_protocol_poll();
  }
}
