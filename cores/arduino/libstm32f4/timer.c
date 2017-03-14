/**
  ******************************************************************************
  * @file    timer.c
  * @author  WI6LABS
  * @version V1.0.0
  * @date    01-August-2016
  * @brief   provide timer services
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup stm32f4xx_system
  * @{
  */

/** @addtogroup STM32F4xx_System_Private_Includes
  * @{
  */
#include "stm32f4xx.h"
#include "hw_config.h"
#include "timer.h"
#include "digital_io.h"
#include "clock.h"
#include "analog.h"

#ifdef __cplusplus
 extern "C" {
#endif

/**
  * @}
  */

/** @addtogroup STM32F4xx_System_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32F4xx_System_Private_Defines
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32F4xx_System_Private_FunctionPrototypes
* @{
*/

static void HAL_TIMx_PeriodElapsedCallback(timer_id_e timer_id);
static timer_id_e get_timer_id_from_handle(TIM_HandleTypeDef *htim);

/**
  * @}
  */

/** @addtogroup STM32F4xx_System_Private_Macros
  * @{
  */

static void tim1_clock_enable(void)       { __TIM1_CLK_ENABLE(); }
static void tim1_clock_reset(void)        { __TIM1_CLK_DISABLE(); }
static uint32_t tim1_clock_source(void)   { return HAL_RCC_GetPCLK2Freq();}
static void tim2_clock_enable(void)       { __TIM2_CLK_ENABLE(); }
static void tim2_clock_reset(void)        { __TIM2_CLK_DISABLE(); }
static uint32_t tim2_clock_source(void)   { return HAL_RCC_GetPCLK1Freq();}
static void tim3_clock_enable(void)       { __TIM3_CLK_ENABLE(); }
static void tim3_clock_reset(void)        { __TIM3_CLK_DISABLE(); }
static uint32_t tim3_clock_source(void)   { return HAL_RCC_GetPCLK1Freq();}
static void tim4_clock_enable(void)       { __TIM4_CLK_ENABLE(); }
static void tim4_clock_reset(void)        { __TIM4_CLK_DISABLE(); }
static uint32_t tim4_clock_source(void)   { return HAL_RCC_GetPCLK1Freq();}
static void tim6_clock_enable(void)       { __TIM6_CLK_ENABLE(); }
static void tim6_clock_reset(void)        { __TIM6_CLK_DISABLE(); }
static uint32_t tim6_clock_source(void)   { return HAL_RCC_GetPCLK1Freq();}
static void tim7_clock_enable(void)       { __TIM7_CLK_ENABLE(); }
static void tim7_clock_reset(void)        { __TIM7_CLK_DISABLE(); }
static uint32_t tim7_clock_source(void)   { return HAL_RCC_GetPCLK1Freq();}
static void tim8_clock_enable(void)       { __TIM8_CLK_ENABLE(); }
static void tim8_clock_reset(void)        { __TIM8_CLK_DISABLE(); }
static uint32_t tim8_clock_source(void)   { return HAL_RCC_GetPCLK2Freq();}
static void tim9_clock_enable(void)       { __TIM9_CLK_ENABLE(); }
static void tim9_clock_reset(void)        { __TIM9_CLK_DISABLE(); }
static uint32_t tim9_clock_source(void)   { return HAL_RCC_GetPCLK2Freq();}
static void tim10_clock_enable(void)      { __TIM10_CLK_ENABLE(); }
static void tim10_clock_reset(void)       { __TIM10_CLK_DISABLE(); }
static uint32_t tim10_clock_source(void)  { return HAL_RCC_GetPCLK2Freq();}
static void tim11_clock_enable(void)      { __TIM11_CLK_ENABLE(); }
static void tim11_clock_reset(void)       { __TIM11_CLK_DISABLE(); }
static uint32_t tim11_clock_source(void)  { return HAL_RCC_GetPCLK2Freq();}
static void tim12_clock_enable(void)      { __TIM12_CLK_ENABLE(); }
static void tim12_clock_reset(void)       { __TIM12_CLK_DISABLE(); }
static uint32_t tim12_clock_source(void)  { return HAL_RCC_GetPCLK1Freq();}

static void tim6_irqHandle(timer_id_e timer_id)   { HAL_TIM6_PeriodElapsedCallback(); }

/**
  * @}
  */

/** @addtogroup STM32F4xx_System_Private_Variables
  * @{
  */

/// @brief timer caracteristics

static TIM_HandleTypeDef g_TimerHandle[NB_TIMER_MANAGED];

static timer_conf_t g_timer_config[NB_TIMER_MANAGED] = {
  {
    //TIMER ID and IRQ
    .timInstance = TIM1, .irqtype = TIM1_UP_TIM10_IRQn,
    .irqHandle = HAL_TIMx_PeriodElapsedCallback,
    .irqHandleOC = NULL,
    .timer_mode = TIMER_OTHER,
    //timer clock init
    .timer_clock_init = tim1_clock_enable,
    //timer clock reset
    .timer_clock_reset = tim1_clock_reset,
    //Timer clock source value
    .timer_clock_source = tim1_clock_source,
    //timer prescaler limit : 8 or 16 bits
    .prescalerLimit = bits_16,
    //toggle pin configuration
    .toggle_pin = { .port = NULL },
    .configured = 0
  },
  {
    //TIMER ID and IRQ
    .timInstance = TIM2, .irqtype = TIM2_IRQn,
    .irqHandle = NULL,
    .irqHandleOC = NULL,
    .timer_mode = TIMER_PWM,
    //timer clock init
    .timer_clock_init = tim2_clock_enable,
    //timer clock reset
    .timer_clock_reset = tim2_clock_reset,
    //Timer clock source value
    .timer_clock_source = tim2_clock_source,
    //timer prescaler limit : 8 or 16 bits
    .prescalerLimit = bits_32,
    //toggle pin configuration
    .toggle_pin = { .port = NULL },
    .configured = 0
  },
  {
    //TIMER ID and IRQ
    .timInstance = TIM3, .irqtype = TIM3_IRQn,
    .irqHandle = NULL,
    .irqHandleOC = NULL,
    .timer_mode = TIMER_PWM,
    //timer clock init
    .timer_clock_init = tim3_clock_enable,
    //timer clock reset
    .timer_clock_reset = tim3_clock_reset,
    //Timer clock source value
    .timer_clock_source = tim3_clock_source,
    //timer prescaler limit : 8 or 16 bits
    .prescalerLimit = bits_16,
    //toggle pin configuration
    .toggle_pin = { .port = NULL },
    .configured = 0
  },
  {
    //TIMER ID and IRQ
    .timInstance = TIM4, .irqtype = TIM4_IRQn,
    .irqHandle = HAL_TIMx_PeriodElapsedCallback,
    .irqHandleOC = NULL,
    .timer_mode = TIMER_PWM,
    //timer clock init
    .timer_clock_init = tim4_clock_enable,
    //timer clock reset
    .timer_clock_reset = tim4_clock_reset,
    //Timer clock source value
    .timer_clock_source = tim4_clock_source,
    //timer prescaler limit : 8 or 16 bits
    .prescalerLimit = bits_16,
    //toggle pin configuration
    .toggle_pin = { .port = NULL },
    .configured = 0
  },
  {
    //TIMER ID and IRQ
    .timInstance = TIM6, .irqtype = TIM6_DAC_IRQn,
    .irqHandle = tim6_irqHandle,
    .irqHandleOC = NULL,
    .timer_mode = TIMER_RESERVED,
    //timer clock init
    .timer_clock_init = tim6_clock_enable,
    //timer clock reset
    .timer_clock_reset = tim6_clock_reset,
    //Timer clock source value
    .timer_clock_source = tim6_clock_source,
    //timer prescaler limit : 8 or 16 bits
    .prescalerLimit = bits_16,
    //toggle pin configuration
    .toggle_pin = { .port = NULL },
    .configured = 0
  },
  {
    //TIMER ID and IRQ
    .timInstance = TIM7, .irqtype = TIM7_IRQn,
    .irqHandle = HAL_TIMx_PeriodElapsedCallback,
    .irqHandleOC = NULL,
    .timer_mode = TIMER_OTHER,
    //timer clock init
    .timer_clock_init = tim7_clock_enable,
    //timer clock reset
    .timer_clock_reset = tim7_clock_reset,
    //Timer clock source value
    .timer_clock_source = tim7_clock_source,
    //timer prescaler limit : 8 or 16 bits
    .prescalerLimit = bits_16,
    //toggle pin configuration
    .toggle_pin = { .port = NULL },
    .configured = 0
  },
  {
    //TIMER ID and IRQ
    .timInstance = TIM8, .irqtype = TIM8_UP_TIM13_IRQn,
    .irqHandle = NULL,
    .irqHandleOC = NULL,
    .timer_mode = TIMER_PWM,
    //timer clock init
    .timer_clock_init = tim8_clock_enable,
    //timer clock reset
    .timer_clock_reset = tim8_clock_reset,
    //Timer clock source value
    .timer_clock_source = tim8_clock_source,
    //timer prescaler limit : 8 or 16 bits
    .prescalerLimit = bits_16,
    //toggle pin configuration
    .toggle_pin = { .port = NULL },
    .configured = 0
  },
  {
    //TIMER ID and IRQ
    .timInstance = TIM9, .irqtype = TIM1_BRK_TIM9_IRQn ,
    .irqHandle = HAL_TIMx_PeriodElapsedCallback,
    .irqHandleOC = NULL,
    .timer_mode = TIMER_OTHER,
    //timer clock init
    .timer_clock_init = tim9_clock_enable,
    //timer clock reset
    .timer_clock_reset = tim9_clock_reset,
    //Timer clock source value
    .timer_clock_source = tim9_clock_source,
    //timer prescaler limit : 8 or 16 bits
    .prescalerLimit = bits_16,
    //toggle pin configuration
    .toggle_pin = { .port = NULL },
    .configured = 0
  },
  {
    //TIMER ID and IRQ
    .timInstance = TIM10, .irqtype = TIM1_UP_TIM10_IRQn,
    .irqHandle = HAL_TIMx_PeriodElapsedCallback,
    .irqHandleOC = NULL,
    .timer_mode = TIMER_OTHER,
    //timer clock init
    .timer_clock_init = tim10_clock_enable,
    //timer clock reset
    .timer_clock_reset = tim10_clock_reset,
    //Timer clock source value
    .timer_clock_source = tim10_clock_source,
    //timer prescaler limit : 8 or 16 bits
    .prescalerLimit = bits_16,
    //toggle pin configuration
    .toggle_pin = { .port = NULL },
    .configured = 0
  },
  {
    //TIMER ID and IRQ
    .timInstance = TIM11, .irqtype = TIM1_TRG_COM_TIM11_IRQn,
    .irqHandle = HAL_TIMx_PeriodElapsedCallback,
    .irqHandleOC = NULL,
    .timer_mode = TIMER_OTHER,
    //timer clock init
    .timer_clock_init = tim11_clock_enable,
    //timer clock reset
    .timer_clock_reset = tim11_clock_reset,
    //Timer clock source value
    .timer_clock_source = tim11_clock_source,
    //timer prescaler limit : 8 or 16 bits
    .prescalerLimit = bits_16,
    //toggle pin configuration
    .toggle_pin = { .port = NULL },
    .configured = 0
  },
  {
    //TIMER ID and IRQ
    .timInstance = TIM12, .irqtype = TIM8_BRK_TIM12_IRQn,
    .irqHandle = HAL_TIMx_PeriodElapsedCallback,
    .irqHandleOC = NULL,
    .timer_mode = TIMER_OTHER,
    //timer clock init
    .timer_clock_init = tim12_clock_enable,
    //timer clock reset
    .timer_clock_reset = tim12_clock_reset,
    //Timer clock source value
    .timer_clock_source = tim12_clock_source,
    //timer prescaler limit : 8 or 16 bits
    .prescalerLimit = bits_16,
    //toggle pin configuration
    .toggle_pin = { .port = NULL },
    .configured = 0
  }
};


/**
  * @}
  */

/**
  * @brief  This function returns the corresponding timer id function of the
  *         handle
  * @param  htim : one of the defined
  * @retval the TIMx id
  */
timer_id_e get_timer_id_from_handle(TIM_HandleTypeDef *htim)
{
  timer_id_e timer_id = NB_TIMER_MANAGED;
  uint8_t i;
  for(i = 0; i<NB_TIMER_MANAGED; i++) {
    if(&g_TimerHandle[i] == htim) {
      timer_id = i;
      break;
    }
  }
  return timer_id;
}

/**
  * @brief  Enable the timer clock
  * @param  htim : one of the defined timer
  * @retval None
  */
void timer_enable_clock(TIM_HandleTypeDef *htim)
{
  uint8_t i = 0;
  for(i = 0; i < NB_TIMER_MANAGED; i++) {
    if(g_timer_config[i].timInstance == htim->Instance) {
      g_timer_config[i].timer_clock_init();
      g_timer_config[i].configured = 1;
      break;
    }
  }
}

/**
  * @brief  Disable the timer clock
  * @param  htim : one of the defined timer
  * @retval None
  */
void timer_disable_clock(TIM_HandleTypeDef *htim)
{
  uint8_t i = 0;
  for(i = 0; i < NB_TIMER_MANAGED; i++) {
    if(g_timer_config[i].timInstance == htim->Instance) {
      g_timer_config[i].timer_clock_reset();
      g_timer_config[i].configured = 0;
      break;
    }
  }
}

/**
  * @brief  Find the first timer not used
  * @param  none
  * @retval The id of the first timer not used if not the number of id.
  */
timer_id_e getInactiveTimer(void)
{
  timer_id_e timer_id = NB_TIMER_MANAGED;
  uint8_t i = 0;
  for(i = 0; i < NB_TIMER_MANAGED; i++) {
    if((g_timer_config[i].configured == 0) &&
       (g_timer_config[i].timer_mode == TIMER_OTHER)){
      timer_id = i;
      break;
    }
  }

  return timer_id;
}

/**
  * @brief  Search the timer associate to a pin
  * @param  port : port pointer
  * @param  pin : pin number
  * @retval The timer id
  */
timer_id_e isPinAssociateToTimer(GPIO_TypeDef *port, uint32_t pin)
{
  uint8_t i = 0;
  timer_id_e timer_id = NB_TIMER_MANAGED;

  for(i = 0; i < NB_TIMER_MANAGED; i++) {
    if((g_timer_config[i].toggle_pin.port == port) &&
       (g_timer_config[i].toggle_pin.pin == pin))  {
       timer_id = i;
       break;
    }
  }

  return timer_id;
}

/**
  * @brief  TIMER Initialization - clock init and nvic init
  * @param  htim_base : one of the defined timer
  * @retval None
  */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim_base)
{
  timer_id_e timer_id;

  timer_id = get_timer_id_from_handle(htim_base);
  if(NB_TIMER_MANAGED == timer_id) {
    return;
  }

  timer_enable_clock(htim_base);

  if(timer_id == TIM6_E) {
    HAL_NVIC_SetPriority(g_timer_config[timer_id].irqtype, 3 ,0);
  } else {
    HAL_NVIC_SetPriority(g_timer_config[timer_id].irqtype, 15 ,0);
  }

  HAL_NVIC_EnableIRQ(g_timer_config[timer_id].irqtype);
}

/**
  * @brief  TIMER Deinitialization - clock and nvic
  * @param  htim_base : one of the defined timer
  * @retval None
  */
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *htim_base)
{
  timer_id_e timer_id;

  timer_id = get_timer_id_from_handle(htim_base);
  if(NB_TIMER_MANAGED == timer_id) {
    return;
  }

  timer_disable_clock(htim_base);
  HAL_NVIC_DisableIRQ(g_timer_config[timer_id].irqtype);
}

/**
  * @brief  This function will set the timer to the required value
  * @param  timer_id : timer_id_e
  * @param  period : Timer period in milliseconds
  * @param  prescaler : clock divider
  * @retval None
  */
void TimerHandleInit(timer_id_e timer_id, uint16_t period, uint16_t prescaler)
{
  if(timer_id >= NB_TIMER_MANAGED) return;

  g_TimerHandle[timer_id].Instance               = g_timer_config[timer_id].timInstance;
  g_TimerHandle[timer_id].Init.Prescaler         = prescaler;
  g_TimerHandle[timer_id].Init.CounterMode       = TIM_COUNTERMODE_UP;
  g_TimerHandle[timer_id].Init.Period            = period;
  g_TimerHandle[timer_id].Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  g_TimerHandle[timer_id].Init.RepetitionCounter = 0x0000;

  if(HAL_TIM_Base_Init(&g_TimerHandle[timer_id]) != HAL_OK){
    return;
  }

  if(HAL_TIM_Base_Start_IT(&g_TimerHandle[timer_id]) != HAL_OK);{
    return;
  }
}

/**
  * @brief  This function will reset the timer
  * @param  timer_id : timer_id_e
  * @retval None
  */
void TimerHandleDeinit(timer_id_e timer_id)
{
  HAL_TIM_Base_DeInit(&g_TimerHandle[timer_id]);
  HAL_TIM_Base_Stop_IT(&g_TimerHandle[timer_id]);
}

/**
  * @brief  This function will set the timer to generate pulse in interrupt mode with a particular duty cycle
  * @param  timer_id : timer_id_e
  * @param  period : timer period in microseconds
  * @param  pulseWidth : pulse width in microseconds
  * @param  irqHandle : interrupt routine to call
  * @retval None
  */
void TimerPulseInit(timer_id_e timer_id, uint16_t period, uint16_t pulseWidth, void (*irqHandle)(timer_id_e, uint32_t))
{
  TIM_OC_InitTypeDef sConfig;

  if(timer_id >= NB_TIMER_MANAGED) return;
  if(g_timer_config[timer_id].configured == 1) return;


  //min pulse = 1us - max pulse = 65535us
  g_TimerHandle[timer_id].Instance               = g_timer_config[timer_id].timInstance;
  g_TimerHandle[timer_id].Init.Period            = period;
  g_TimerHandle[timer_id].Init.Prescaler         = (uint32_t)(g_timer_config[timer_id].timer_clock_source() / (1000000)) - 1;
  g_TimerHandle[timer_id].Init.ClockDivision     = 0;
  g_TimerHandle[timer_id].Init.CounterMode       = TIM_COUNTERMODE_UP;
  g_TimerHandle[timer_id].Init.RepetitionCounter = 0;

  g_timer_config[timer_id].irqHandleOC = irqHandle;

  sConfig.OCMode        = TIM_OCMODE_TIMING;
  sConfig.Pulse         = pulseWidth;
  sConfig.OCPolarity    = TIM_OCPOLARITY_HIGH;
  sConfig.OCNPolarity   = TIM_OCNPOLARITY_HIGH;
  sConfig.OCFastMode    = TIM_OCFAST_DISABLE;
  sConfig.OCIdleState   = TIM_OCIDLESTATE_RESET;
  sConfig.OCNIdleState  = TIM_OCNIDLESTATE_RESET;

  if(HAL_TIM_OC_Init(&g_TimerHandle[timer_id]) != HAL_OK) return;

  if(HAL_TIM_OC_ConfigChannel(&g_TimerHandle[timer_id],&sConfig, TIM_CHANNEL_1) != HAL_OK) return;
  if(HAL_TIM_OC_Start_IT(&g_TimerHandle[timer_id], TIM_CHANNEL_1) != HAL_OK) return;
}

/**
  * @brief  This function will reset the pulse generation
  * @param  timer_id : timer_id_e
  * @retval None
  */
void TimerPulseDeinit(timer_id_e timer_id)
{
  if(timer_id >= NB_TIMER_MANAGED) return;

  g_timer_config[timer_id].irqHandleOC = NULL;

  if(HAL_TIM_OC_DeInit(&g_TimerHandle[timer_id]) != HAL_OK) return;
  if(HAL_TIM_OC_Stop_IT(&g_TimerHandle[timer_id], TIM_CHANNEL_1) != HAL_OK) return;
}

/**
  * @brief  Initializes the TIM Output Compare MSP.
  * @param  htim: TIM handle
  * @retval None
  */
void HAL_TIM_OC_MspInit(TIM_HandleTypeDef *htim)
{
  timer_id_e timer_id;

  timer_id = get_timer_id_from_handle(htim);
  if(NB_TIMER_MANAGED == timer_id) {
    return;
  }

  timer_enable_clock(htim);
  HAL_NVIC_SetPriority(g_timer_config[timer_id].irqtype, 14 ,0);
  HAL_NVIC_EnableIRQ(g_timer_config[timer_id].irqtype);
}

/**
  * @brief  DeInitialize TIM Output Compare MSP.
  * @param  htim: TIM handle
  * @retval None
  */
void HAL_TIM_OC_MspDeInit(TIM_HandleTypeDef *htim)
{
  timer_id_e timer_id;

  timer_id = get_timer_id_from_handle(htim);
  if(NB_TIMER_MANAGED == timer_id) {
    return;
  }

  timer_disable_clock(htim);
  HAL_NVIC_DisableIRQ(g_timer_config[timer_id].irqtype);
}

/**
  * @brief  Output Compare callback in non-blocking mode
  * @param  htim : TIM OC handle
  * @retval None
  */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  uint32_t channel = 0;
  timer_id_e timer_id = get_timer_id_from_handle(htim);
  if(NB_TIMER_MANAGED == timer_id) {
    return;
  }

  if(g_timer_config[timer_id].irqHandleOC != NULL) {
    switch(htim->Channel) {
      case HAL_TIM_ACTIVE_CHANNEL_1:
        channel = TIM_CHANNEL_1 / 4;
      break;
      default:
        return;
      break;
    }
      g_timer_config[timer_id].irqHandleOC(timer_id, channel);
  }
}

/**
  * @brief  Period elapsed callback in non-blocking mode
  * @param  timer_id : id of the timer
  * @retval None
  */
void HAL_TIMx_PeriodElapsedCallback(timer_id_e timer_id)
{
  if(g_timer_config[timer_id].toggle_pin.port != NULL) {
    if(g_timer_config[timer_id].toggle_pin.count > 0){
      g_timer_config[timer_id].toggle_pin.count--;

      if(g_timer_config[timer_id].toggle_pin.state == 0) {
        g_timer_config[timer_id].toggle_pin.state = 1;
        digital_io_write(g_timer_config[timer_id].toggle_pin.port,
                         g_timer_config[timer_id].toggle_pin.pin, 1);
      }
      else {
        g_timer_config[timer_id].toggle_pin.state = 0;
        digital_io_write(g_timer_config[timer_id].toggle_pin.port,
                         g_timer_config[timer_id].toggle_pin.pin, 0);
      }
    }
    else if(g_timer_config[timer_id].toggle_pin.count == -1) {
      if(g_timer_config[timer_id].toggle_pin.state == 0) {
        g_timer_config[timer_id].toggle_pin.state = 1;
        digital_io_write(g_timer_config[timer_id].toggle_pin.port,
                         g_timer_config[timer_id].toggle_pin.pin, 1);
      }
      else {
        g_timer_config[timer_id].toggle_pin.state = 0;
        digital_io_write(g_timer_config[timer_id].toggle_pin.port,
                         g_timer_config[timer_id].toggle_pin.pin, 0);
      }
    }
    else {
      digital_io_write(g_timer_config[timer_id].toggle_pin.port,
                       g_timer_config[timer_id].toggle_pin.pin, 0);
    }
  }
}

/**
  * @brief  Period elapsed callback in non-blocking mode
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  timer_id_e timer_id = get_timer_id_from_handle(htim);
  if(NB_TIMER_MANAGED == timer_id) {
    return;
  }

  if(g_timer_config[timer_id].irqHandle != NULL) {
      g_timer_config[timer_id].irqHandle(timer_id);
  }
}

/**
  * @brief  This function will set the tone timer to the required value and
  *         configure the pin to toggle.
  * @param  port : pointer to GPIO_TypeDef
  * @param  pin : pin number to toggle
  * @param  frequency : toggle frequency (in hertz)
  * @param  duration : toggle time
  * @retval None
  */
void TimerPinInit(PinName pin, uint32_t frequency, uint32_t duration)
{
  uint8_t end = 0;
  uint32_t prescaler = 1;
  uint32_t period = 0;
  timer_id_e timer_id;
  GPIO_TypeDef *port = get_GPIO_Port(STM_PORT(pin));
  uint32_t p = STM_GPIO_PIN(p);

  timer_id = isPinAssociateToTimer(port,p);

  if(timer_id == NB_TIMER_MANAGED) {
    timer_id = getInactiveTimer();
    if(timer_id == NB_TIMER_MANAGED)
      return;
  }

  if(frequency > MAX_FREQ)
    return;

  g_timer_config[timer_id].toggle_pin.port = port;
  g_timer_config[timer_id].toggle_pin.pin = p;
  g_timer_config[timer_id].toggle_pin.state = 0;

  //Calculate the toggle count
  if (duration > 0) {
    g_timer_config[timer_id].toggle_pin.count = ((frequency * duration) / 1000) * 2;
  }
  else {
    g_timer_config[timer_id].toggle_pin.count = -1;
  }

  digital_io_init(pin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL);

  while(end == 0) {
    period = ((uint32_t)(g_timer_config[timer_id].timer_clock_source() / frequency / prescaler)) - 1;

    if((period >= g_timer_config[timer_id].prescalerLimit)
        && (prescaler < g_timer_config[timer_id].prescalerLimit))
      prescaler++; //prescaler *= 2;

    else
      end = 1;
  }

  if((period < g_timer_config[timer_id].prescalerLimit)
      && (prescaler < g_timer_config[timer_id].prescalerLimit)) {
    g_timer_config[timer_id].irqHandle = HAL_TIMx_PeriodElapsedCallback;
    TimerHandleInit(timer_id, period, prescaler-1);
  }
  else {
    TimerHandleDeinit(timer_id);
  }
}

/**
  * @brief  This function will reset the tone timer
  * @param  port : pointer to port
  * @param  pin : pin number to toggle
  * @retval None
  */
void TimerPinDeinit(GPIO_TypeDef *port, uint32_t pin)
{
  timer_id_e timer_id = isPinAssociateToTimer(port,pin);

  if(timer_id < NB_TIMER_MANAGED) {
    TimerHandleDeinit(timer_id);
    g_timer_config[timer_id].toggle_pin.port = NULL;
    g_timer_config[timer_id].toggle_pin.pin = 0;
    g_timer_config[timer_id].toggle_pin.count = 0;
    g_timer_config[timer_id].toggle_pin.state = 0;
  }
}

/**
  * @brief  Get the counter value.
  * @param  timer_id : id of the timer
  * @retval Counter value
  */
uint32_t getTimerCounter(timer_id_e timer_id)
{
  if(timer_id < NB_TIMER_MANAGED)
    return __HAL_TIM_GET_COUNTER(&g_TimerHandle[timer_id]);
  else
    return 0;
}

/**
  * @brief  Set the counter value.
  * @param  timer_id : id of the timer
  * @param  value : counter value
  * @retval None
  */
void setTimerCounter(timer_id_e timer_id, uint32_t value)
{
  if(timer_id < NB_TIMER_MANAGED)
    __HAL_TIM_SET_COUNTER(&g_TimerHandle[timer_id], value);
}

/**
  * @brief  Set the TIM Capture Compare Register value.
  * @param  timer_id : id of the timer
  * @param  channel : TIM Channels to be configured.
  * @param  value : register new register.
  * @retval None
  */
void setCCRRegister(timer_id_e timer_id, uint32_t channel, uint32_t value)
{
  if(timer_id < NB_TIMER_MANAGED) {
    __HAL_TIM_SET_COMPARE(&g_TimerHandle[timer_id], channel*4, value);
  }
}

/**
  * @brief  Set the TIM Capture Compare Register value.
  * @param  timer_id : id of the timer
  * @param  channel : TIM Channels to be configured.
  * @retval CRR value.
  */
uint32_t getCCRRegister(timer_id_e timer_id, uint32_t channel)
{
  if(timer_id < NB_TIMER_MANAGED)
    return __HAL_TIM_GET_COMPARE(&g_TimerHandle[timer_id], channel);
  else
    return 0;
}

/**
  * @brief  Attached an interrupt handler
  * @param  timer_id : id of the timer
  * @param  irqHandle : interrupt handler
  * @retval none
  */
void attachIntHandle(timer_id_e timer_id, void (*irqHandle)(timer_id_e))
{
  if(timer_id < NB_TIMER_MANAGED)
    g_timer_config[timer_id].irqHandle = irqHandle;
}


/******************************************************************************/
/*                            TIMx IRQ HANDLER                                */
/******************************************************************************/

/**
  * @brief  TIM1 & TIM10 irq handler
  * @param  None
  * @retval None
  */
void TIM1_UP_TIM10_IRQHandler(void)
{
  if(g_TimerHandle[TIM1_E].Instance == TIM1) {
    HAL_TIM_IRQHandler(&g_TimerHandle[TIM1_E]);
  }

  if(g_TimerHandle[TIM10_E].Instance == TIM10) {
    HAL_TIM_IRQHandler(&g_TimerHandle[TIM10_E]);
  }
}


/**
  * @brief  TIM1 & TIM9 irq handler
  * @param  None
  * @retval None
  */
void TIM1_BRK_TIM9_IRQHandler(void)
{
  if(g_TimerHandle[TIM1_E].Instance == TIM1) {
    HAL_TIM_IRQHandler(&g_TimerHandle[TIM1_E]);
  }

  if(g_TimerHandle[TIM9_E].Instance == TIM9) {
    HAL_TIM_IRQHandler(&g_TimerHandle[TIM9_E]);
  }
}

/**
  * @brief  TIM1 & TIM11 irq handler
  * @param  None
  * @retval None
  */
void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
  if(g_TimerHandle[TIM1_E].Instance == TIM1) {
    HAL_TIM_IRQHandler(&g_TimerHandle[TIM1_E]);
  }

  if(g_TimerHandle[TIM11_E].Instance == TIM11) {
    HAL_TIM_IRQHandler(&g_TimerHandle[TIM11_E]);
  }
}

/**
  * @brief  TIM1 irq handler
  * @param  None
  * @retval None
  */
void TIM1_CC_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&g_TimerHandle[TIM1_E]);
}

/**
  * @brief  TIM2 irq handler
  * @param  None
  * @retval None
  */
void TIM2_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&g_TimerHandle[TIM2_E]);
}

/**
  * @brief  TIM3 irq handler
  * @param  None
  * @retval None
  */
void TIM3_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&g_TimerHandle[TIM3_E]);
}

/**
  * @brief  TIM4 irq handler
  * @param  None
  * @retval None
  */
void TIM4_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&g_TimerHandle[TIM4_E]);
}

/**
  * @brief  TIM6 irq handler
  * @param  None
  * @retval None
  */
void TIM6_DAC_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&g_TimerHandle[TIM6_E]);
}

/**
  * @brief  TIM7 irq handler
  * @param  None
  * @retval None
  */
void TIM7_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&g_TimerHandle[TIM7_E]);
}

/**
  * @brief  TIM8 irq handler
  * @param  None
  * @retval None
  */
void TIM8_UP_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&g_TimerHandle[TIM8_E]);
}


/**
  * @brief  TIM8 TIM12 irq handler
  * @param  None
  * @retval None
  */
void TIM8_BRK_TIM12_IRQHandler(void)
{
  if(g_TimerHandle[TIM12_E].Instance == TIM12) {
    HAL_TIM_IRQHandler(&g_TimerHandle[TIM12_E]);
  }

  if(g_TimerHandle[TIM8_E].Instance == TIM8) {
    HAL_TIM_IRQHandler(&g_TimerHandle[TIM8_E]);
  }
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
#ifdef __cplusplus
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
