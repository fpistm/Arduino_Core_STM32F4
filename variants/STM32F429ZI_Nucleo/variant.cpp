/*
  Copyright (c) 2011 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "variant.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Pins descriptions
 */
extern const PinDescription g_APinDescription[]=

//  arduino_id  |     ulPin    |   ulPort | mode               |          configured
{
  { ARDUINO_PIN_A0,   GPIO_PIN_3,  GPIOA, GPIO_PIN_IO|GPIO_PIN_ADC ,                    false },
  { ARDUINO_PIN_A1,   GPIO_PIN_0,  GPIOC, GPIO_PIN_IO|GPIO_PIN_ADC ,                    false },
  { ARDUINO_PIN_A2,   GPIO_PIN_3,  GPIOC, GPIO_PIN_IO|GPIO_PIN_ADC ,                    false },
  { ARDUINO_PIN_A3,   GPIO_PIN_3,  GPIOF, GPIO_PIN_ADC,                                 false },
  { ARDUINO_PIN_A4,   GPIO_PIN_5,  GPIOF, GPIO_PIN_IO|GPIO_PIN_ADC,                     false },
  { ARDUINO_PIN_A5,   GPIO_PIN_10, GPIOF, GPIO_PIN_IO|GPIO_PIN_ADC,                     false },
  { ARDUINO_PIN_D0,   GPIO_PIN_9,  GPIOG, GPIO_PIN_IO|GPIO_PIN_UART_RX ,                false },
  { ARDUINO_PIN_D1,   GPIO_PIN_14, GPIOG, GPIO_PIN_IO|GPIO_PIN_UART_TX ,                false },
  { ARDUINO_PIN_D2,   GPIO_PIN_15, GPIOF, GPIO_PIN_IO,                                  false },
  { ARDUINO_PIN_D3,   GPIO_PIN_13, GPIOE, GPIO_PIN_IO|GPIO_PIN_PWM,                     false },
  { ARDUINO_PIN_D4,   GPIO_PIN_14, GPIOF, GPIO_PIN_IO,                                  false },
  { ARDUINO_PIN_D5,   GPIO_PIN_11, GPIOE, GPIO_PIN_IO|GPIO_PIN_PWM,                     false },
  { ARDUINO_PIN_D6,   GPIO_PIN_9,  GPIOE, GPIO_PIN_IO|GPIO_PIN_PWM,                     false },
  { ARDUINO_PIN_D7,   GPIO_PIN_13, GPIOF, GPIO_PIN_IO,                                  false },
  { ARDUINO_PIN_D8,   GPIO_PIN_12, GPIOF, GPIO_PIN_IO,                                  false },
  { ARDUINO_PIN_D9,   GPIO_PIN_15, GPIOD, GPIO_PIN_IO|GPIO_PIN_PWM,                     false },
  { ARDUINO_PIN_D10,  GPIO_PIN_14, GPIOD, GPIO_PIN_IO|GPIO_PIN_PWM|GPIO_PIN_SPI_CS,     false },
  { ARDUINO_PIN_D11,  GPIO_PIN_7,  GPIOA, GPIO_PIN_IO|GPIO_PIN_SPI_MOSI|GPIO_PIN_PWM,   false },
  { ARDUINO_PIN_D12,  GPIO_PIN_6,  GPIOA, GPIO_PIN_IO|GPIO_PIN_SPI_MISO ,               false },
  { ARDUINO_PIN_D13,  GPIO_PIN_5,  GPIOA, GPIO_PIN_IO|GPIO_PIN_SPI_SCK,                 false },
  { ARDUINO_PIN_D14,  GPIO_PIN_9,  GPIOB, GPIO_PIN_IO|GPIO_PIN_I2C_SDA,                 false },
  { ARDUINO_PIN_D15,  GPIO_PIN_8,  GPIOB, GPIO_PIN_IO|GPIO_PIN_I2C_SCL,                 false },
} ;

#ifdef __cplusplus
}
#endif

/*
 * UART objects
 */

UARTClass Serial(USART3_E);    //available on PD8/PD9
UARTClass Serial1(USART6_E);   //available on PG14/PG9
USARTClass Serial2(USART3_E);  //available on PD8/PD9

void serialEvent() __attribute__((weak));
void serialEvent() { }

void serialEventRun(void)
{
  if (Serial.available()) serialEvent();
}

// ----------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif

void __libc_init_array(void);


void init( void )
{
  hw_config_init();
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 168000000
  *            HCLK(Hz)                       = 168000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 336
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  HAL_PWREx_EnableOverDrive();

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

#ifdef __cplusplus
}
#endif
