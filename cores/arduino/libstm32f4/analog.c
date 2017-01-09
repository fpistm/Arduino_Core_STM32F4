/**
  ******************************************************************************
  * @file    analog.c
  * @author  WI6LABS
  * @version V1.0.0
  * @date    01-August-2016
  * @brief   provide analog services (ADC + PWM)
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
#include "analog.h"
#include "timer.h"

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

#define SAMPLINGTIME        ADC_SAMPLETIME_3CYCLES  /*!< ADC conversions sampling time. */
#define ADC_REGULAR_RANK_1  1
/**
  * @}
  */

/** @addtogroup STM32F4xx_System_Private_Macros
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32F4xx_System_Private_Variables
  * @{
  */
static uint8_t g_current_init_id = 0;


//Give details about the analog pins.
analog_config_str g_analog_config[NB_ANALOG_CHANNELS] = {
  {
    .port = GPIOA,
    .pin = GPIO_PIN_3,
    .adcInstance = ADC1,
    .adcChannelConf = {
      .Channel = ADC_CHANNEL_3,
      .Rank = ADC_REGULAR_RANK_1,
      .SamplingTime = SAMPLINGTIME,
      .Offset = 0x000
    },
    .dacInstance = NULL,
    .timInstance = NULL
  },
  {
    .port = GPIOC,
    .pin = GPIO_PIN_0,
    .adcInstance = ADC1,
    .adcChannelConf = {
      .Channel = ADC_CHANNEL_10,
      .Rank = ADC_REGULAR_RANK_1,
      .SamplingTime = SAMPLINGTIME,
      .Offset = 0x000
    },
    .dacInstance = NULL,
    .timInstance = NULL
  },
  {
    .port = GPIOC,
    .pin = GPIO_PIN_3,
    .adcInstance = ADC1,
    .adcChannelConf = {
      .Channel = ADC_CHANNEL_13,
      .Rank = ADC_REGULAR_RANK_1,
      .SamplingTime = SAMPLINGTIME,
      .Offset = 0x000
    },
    .dacInstance = DAC,
    .dacChannel = DAC_CHANNEL_1,
    .dacChannelConf = {
      .DAC_Trigger = DAC_TRIGGER_NONE,
      .DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE
    },
    .timInstance = NULL
  },
  {
    .port = GPIOA,
    .pin = GPIO_PIN_5,
    .adcInstance = NULL,
    .dacInstance = DAC,
    .dacChannel = DAC_CHANNEL_2,
    .dacChannelConf = {
      .DAC_Trigger = DAC_TRIGGER_NONE,
      .DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE
    },
    .timInstance = NULL
  },
  {
    .port = GPIOA,
    .pin = GPIO_PIN_7,
    .alFunction = GPIO_AF2_TIM3,
    .adcInstance = NULL,
    .dacInstance = NULL,
    .timInstance = TIM3,
    .timChannel = TIM_CHANNEL_2,
    .useNchannel = 0,
    .timConfig = {
      .OCMode       = TIM_OCMODE_PWM1,
      .OCPolarity   = TIM_OCPOLARITY_HIGH,
      .OCFastMode   = TIM_OCFAST_DISABLE,
      .OCNPolarity  = TIM_OCNPOLARITY_HIGH,
      .OCNIdleState = TIM_OCNIDLESTATE_RESET,
      .OCIdleState  = TIM_OCIDLESTATE_RESET
     },
     .timHandle = {}
  },
  {
    .port = GPIOC,
    .pin = GPIO_PIN_1,
    .adcInstance = ADC1,
    .adcChannelConf = {
      .Channel = ADC_CHANNEL_11,
      .Rank = ADC_REGULAR_RANK_1,
      .SamplingTime = SAMPLINGTIME,
      .Offset = 0x000
    },
    .dacInstance = NULL,
    .timInstance = NULL
  },
  {
    .port = GPIOE,
    .pin = GPIO_PIN_13,
    .alFunction = GPIO_AF1_TIM1,
    .adcInstance = NULL,
    .dacInstance = NULL,
    .timInstance = TIM1,
    .timChannel = TIM_CHANNEL_3,
    .useNchannel = 0,
    .timConfig = {
      .OCMode       = TIM_OCMODE_PWM1,
      .OCPolarity   = TIM_OCPOLARITY_HIGH,
      .OCFastMode   = TIM_OCFAST_DISABLE,
      .OCNPolarity  = TIM_OCNPOLARITY_HIGH,
      .OCNIdleState = TIM_OCNIDLESTATE_RESET,
      .OCIdleState  = TIM_OCIDLESTATE_RESET
     },
     .timHandle = {}
  },
  {
    .port = GPIOE,
    .pin = GPIO_PIN_11,
    .alFunction = GPIO_AF1_TIM1,
    .adcInstance = NULL,
    .dacInstance = NULL,
    .timInstance = TIM1,
    .timChannel = TIM_CHANNEL_2,
    .useNchannel = 0,
    .timConfig = {
      .OCMode       = TIM_OCMODE_PWM1,
      .OCPolarity   = TIM_OCPOLARITY_HIGH,
      .OCFastMode   = TIM_OCFAST_DISABLE,
      .OCNPolarity  = TIM_OCNPOLARITY_HIGH,
      .OCNIdleState = TIM_OCNIDLESTATE_RESET,
      .OCIdleState  = TIM_OCIDLESTATE_RESET
     },
     .timHandle = {}
  },
  {
    .port = GPIOD,
    .pin = GPIO_PIN_14,
    .alFunction = GPIO_AF2_TIM4,
    .adcInstance = NULL,
    .dacInstance = NULL,
    .timInstance = TIM4,
    .timChannel = TIM_CHANNEL_3,
    .useNchannel = 0,
    .timConfig = {
      .OCMode       = TIM_OCMODE_PWM1,
      .OCPolarity   = TIM_OCPOLARITY_HIGH,
      .OCFastMode   = TIM_OCFAST_DISABLE,
      .OCNPolarity  = TIM_OCNPOLARITY_HIGH,
      .OCNIdleState = TIM_OCNIDLESTATE_RESET,
      .OCIdleState  = TIM_OCIDLESTATE_RESET
     },
     .timHandle = {}
  },
  {
    .port = GPIOD,
    .pin = GPIO_PIN_15,
    .alFunction = GPIO_AF2_TIM4,
    .adcInstance = NULL,
    .dacInstance = NULL,
    .timInstance = TIM4,
    .timChannel = TIM_CHANNEL_4,
    .useNchannel = 0,
    .timConfig = {
      .OCMode       = TIM_OCMODE_PWM1,
      .OCPolarity   = TIM_OCPOLARITY_HIGH,
      .OCFastMode   = TIM_OCFAST_DISABLE,
      .OCNPolarity  = TIM_OCNPOLARITY_HIGH,
      .OCNIdleState = TIM_OCNIDLESTATE_RESET,
      .OCIdleState  = TIM_OCIDLESTATE_RESET
     },
     .timHandle = {}
  },
  {
    .port = GPIOE,
    .pin = GPIO_PIN_9,
    .alFunction = GPIO_AF1_TIM1,
    .adcInstance = NULL,
    .dacInstance = NULL,
    .timInstance = TIM1,
    .timChannel = TIM_CHANNEL_1,
    .useNchannel = 0,
    .timConfig = {
      .OCMode       = TIM_OCMODE_PWM1,
      .OCPolarity   = TIM_OCPOLARITY_HIGH,
      .OCFastMode   = TIM_OCFAST_DISABLE,
      .OCNPolarity  = TIM_OCNPOLARITY_HIGH,
      .OCNIdleState = TIM_OCNIDLESTATE_RESET,
      .OCIdleState  = TIM_OCIDLESTATE_RESET
     }
  },
  {
    .port = GPIOF,
    .pin = GPIO_PIN_10,
    .adcInstance = ADC3,
    .adcChannelConf = {
      .Channel = ADC_CHANNEL_8,
      .Rank = ADC_REGULAR_RANK_1,
      .SamplingTime = SAMPLINGTIME,
      .Offset = 0x000
    },
    .dacInstance = NULL,
    .timInstance = NULL
  },
  {
    .port = GPIOF,
    .pin = GPIO_PIN_3,
    .adcInstance = ADC3,
    .adcChannelConf = {
      .Channel = ADC_CHANNEL_9,
      .Rank = ADC_REGULAR_RANK_1,
      .SamplingTime = SAMPLINGTIME,
      .Offset = 0x000
    },
    .dacInstance = NULL,
    .timInstance = NULL
  },
  {
    .port = GPIOF,
    .pin = GPIO_PIN_5,
    .adcInstance = ADC3,
    .adcChannelConf = {
      .Channel = ADC_CHANNEL_15,
      .Rank = ADC_REGULAR_RANK_1,
      .SamplingTime = SAMPLINGTIME,
      .Offset = 0x000
    },
    .dacInstance = NULL,
    .timInstance = NULL
  },
  {
    .port = GPIOC,
    .pin = GPIO_PIN_7,
    .alFunction = GPIO_AF2_TIM4,
    .adcInstance = NULL,
    .dacInstance = NULL,
    .timInstance = TIM4,
    .timChannel = TIM_CHANNEL_4,
    .useNchannel = 0,
    .timConfig = {
      .OCMode       = TIM_OCMODE_PWM1,
      .OCPolarity   = TIM_OCPOLARITY_HIGH,
      .OCFastMode   = TIM_OCFAST_DISABLE,
      .OCNPolarity  = TIM_OCNPOLARITY_HIGH,
      .OCNIdleState = TIM_OCNIDLESTATE_RESET,
      .OCIdleState  = TIM_OCIDLESTATE_RESET
     },
     .timHandle = {}
  },
};

/**
  * @}
  */

/** @addtogroup STM32F4xx_System_Private_FunctionPrototypes
  * @{
  */


/**
  * @brief  This function will return the corresponding adc configuration id
  * @param  port : the gpio port to use
  * @param  pin : the gpio pin to use
  * @retval None
  */
int8_t get_analog_instance(GPIO_TypeDef  *port, uint32_t pin)
{
  int8_t i;

  for(i = 0; i < NB_ANALOG_CHANNELS ; i++) {
    if((g_analog_config[i].port == port)&&(g_analog_config[i].pin == pin)) {
      return i;
    }
  }
  return -1;
}



////////////////////////// DAC INTERFACE FUNCTIONS /////////////////////////////

/**
  * @brief DAC MSP Initialization
  *        This function configures the hardware resources used in this example:
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration
  * @param hdac: DAC handle pointer
  * @retval None
  */
void HAL_DAC_MspInit(DAC_HandleTypeDef *hdac)
{
  GPIO_InitTypeDef          GPIO_InitStruct;

  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* Enable GPIO clock ****************************************/
  if(g_analog_config[g_current_init_id].port == GPIOA) {
    __GPIOA_CLK_ENABLE();
  } else if(g_analog_config[g_current_init_id].port == GPIOB){
    __GPIOB_CLK_ENABLE();
  } else if(g_analog_config[g_current_init_id].port == GPIOC){
    __GPIOC_CLK_ENABLE();
  } else if(g_analog_config[g_current_init_id].port == GPIOD){
    __GPIOD_CLK_ENABLE();
  } else if(g_analog_config[g_current_init_id].port == GPIOE){
    __GPIOE_CLK_ENABLE();
  } else if(g_analog_config[g_current_init_id].port == GPIOF){
    __GPIOF_CLK_ENABLE();
  }

  /* DAC Periph clock enable */
  __HAL_RCC_DAC_CLK_ENABLE();

  /*##-2- Configure peripheral GPIO ##########################################*/
  /* DAC Channel1 GPIO pin configuration */
  GPIO_InitStruct.Pin = g_analog_config[g_current_init_id].pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(g_analog_config[g_current_init_id].port, &GPIO_InitStruct);
}


/**
  * @brief  This function will set the DAC to the required value
  * @param  port : the gpio port to use
  * @param  pin : the gpio pin to use
  * @param  value : the value to push on the adc output
  * @param  do_init : if set to 1 the initialization of the adc is done
  * @retval None
  */
void dac_write_value(GPIO_TypeDef  *port, uint32_t pin, uint32_t value, uint8_t do_init)
{
  DAC_HandleTypeDef    DacHandle;

  //find the instance in the global
  int8_t id = get_analog_instance(port, pin);
  if(id < 0 ) return;

  if(g_analog_config[id].adcInstance == NULL) {
    return;
  }

  DacHandle.Instance = g_analog_config[id].dacInstance;

  if(do_init == 1) {

    if (HAL_DAC_DeInit(&DacHandle) != HAL_OK)
    {
      /* DeInitialization Error */
      return;
    }

    /*##-1- Configure the DAC peripheral #######################################*/
    g_current_init_id = id;
    if (HAL_DAC_Init(&DacHandle) != HAL_OK)
    {
      /* Initialization Error */
      return;
    }

    /*##-2- Configure DAC channel1 #############################################*/
    if (HAL_DAC_ConfigChannel(&DacHandle, &g_analog_config[id].dacChannelConf,
                              g_analog_config[id].dacChannel) != HAL_OK)
    {
      /* Channel configuration Error */
      return;
    }
  }

  /*##-3- Set DAC Channel1 DHR register ######################################*/
  if (HAL_DAC_SetValue(&DacHandle, g_analog_config[id].dacChannel,
                       DAC_ALIGN_12B_R, value) != HAL_OK)
  {
    /* Setting value Error */
    return;
  }

  /*##-4- Enable DAC Channel1 ################################################*/
  HAL_DAC_Start(&DacHandle, g_analog_config[id].dacChannel);
}

/**
  * @brief  DeInitialize the DAC MSP.
  * @param  hdac: pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @retval None
  */
void HAL_DAC_MspDeInit(DAC_HandleTypeDef* hdac)
{
  /* DAC Periph clock disable */
  __HAL_RCC_DAC_CLK_DISABLE();
}

/**
  * @brief  This function will stop the DAC
  * @param  port : the gpio port to use
  * @param  pin : the gpio pin to use
  * @retval None
  */
void dac_stop(GPIO_TypeDef  *port, uint32_t pin)
{
  DAC_HandleTypeDef    DacHandle;

  //find the instance in the global
  int8_t id = get_analog_instance(port, pin);
  if(id < 0 ) return;

  if(g_analog_config[id].adcInstance == NULL) {
    return;
  }

  DacHandle.Instance = g_analog_config[id].dacInstance;

  HAL_DAC_Stop(&DacHandle, g_analog_config[id].dacChannel);

  if (HAL_DAC_DeInit(&DacHandle) != HAL_OK)
  {
    /* DeInitialization Error */
    return;
  }
}


////////////////////////// ADC INTERFACE FUNCTIONS /////////////////////////////

/**
  * @brief ADC MSP Initialization
  *        This function configures the hardware resources used in this example:
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration
  * @param hadc: ADC handle pointer
  * @retval None
  */
void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* ADC Periph clock enable */
  if(hadc->Instance == ADC1) {
    __HAL_RCC_ADC1_CLK_ENABLE();
  } else if(hadc->Instance == ADC3) {
    __HAL_RCC_ADC3_CLK_ENABLE();
  }

  /* Enable GPIO clock ****************************************/
  if(g_analog_config[g_current_init_id].port == GPIOA) {
    __GPIOA_CLK_ENABLE();
  } else if(g_analog_config[g_current_init_id].port == GPIOB){
    __GPIOB_CLK_ENABLE();
  } else if(g_analog_config[g_current_init_id].port == GPIOC){
    __GPIOC_CLK_ENABLE();
  } else if(g_analog_config[g_current_init_id].port == GPIOD){
    __GPIOD_CLK_ENABLE();
  } else if(g_analog_config[g_current_init_id].port == GPIOE){
    __GPIOE_CLK_ENABLE();
  } else if(g_analog_config[g_current_init_id].port == GPIOF){
    __GPIOF_CLK_ENABLE();
  }

  /*##-2- Configure peripheral GPIO ##########################################*/
  /* ADC Channel GPIO pin configuration */
  GPIO_InitStruct.Pin = g_analog_config[g_current_init_id].pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(g_analog_config[g_current_init_id].port, &GPIO_InitStruct);
}

/**
  * @brief  DeInitializes the ADC MSP.
  * @param  hadc: ADC handle
  * @retval None
  */
void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{

  __HAL_RCC_ADC_FORCE_RESET();
  __HAL_RCC_ADC_RELEASE_RESET();

  if(hadc->Instance == ADC1) {
    __HAL_RCC_ADC1_CLK_DISABLE();
  } else if(hadc->Instance == ADC3) {
    __HAL_RCC_ADC3_CLK_DISABLE();
  }
}

/**
  * @brief  This function will set the ADC to the required value
  * @param  port : the gpio port to use
  * @param  pin : the gpio pin to use
  * @param  do_init : if set to 1 the initialization of the adc is done
  * @retval the value of the adc
  */
uint16_t adc_read_value(GPIO_TypeDef  *port, uint32_t pin, uint8_t do_init)
{
  ADC_HandleTypeDef    AdcHandle;

  //find the instance in the global
  int8_t id = get_analog_instance(port, pin);
  if(id < 0 ) return 0;
  __IO uint16_t uhADCxConvertedValue = 0;

  AdcHandle.Instance = g_analog_config[id].adcInstance;

  if(do_init == 1) {
    if(HAL_ADC_DeInit(&AdcHandle) != HAL_OK) {
      return 0;
    }

    AdcHandle.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV2;          /* Asynchronous clock mode, input ADC clock divided */
    AdcHandle.Init.Resolution            = ADC_RESOLUTION_12B;            /* 12-bit resolution for converted data */
    AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;           /* Right-alignment for converted data */
    AdcHandle.Init.ScanConvMode          = DISABLE;                       /* Sequencer disabled (ADC conversion on only 1 channel: channel set on rank 1) */
    AdcHandle.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;           /* EOC flag picked-up to indicate conversion end */
    AdcHandle.Init.ContinuousConvMode    = DISABLE;                       /* Continuous mode disabled to have only 1 conversion at each conversion trig */
    AdcHandle.Init.NbrOfConversion       = 1;                             /* Specifies the number of ranks that will be converted within the regular group sequencer. */
    AdcHandle.Init.DiscontinuousConvMode = DISABLE;                       /* Parameter discarded because sequencer is disabled */
    AdcHandle.Init.NbrOfDiscConversion   = 0;                             /* Parameter discarded because sequencer is disabled */
    AdcHandle.Init.ExternalTrigConv      = ADC_SOFTWARE_START;            /* Software start to trig the 1st conversion manually, without external event */
    AdcHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE; /* Parameter discarded because software trigger chosen */
    AdcHandle.Init.DMAContinuousRequests = DISABLE;                       /* DMA one-shot mode selected (not applied to this example) */

    g_current_init_id = id;
    if (HAL_ADC_Init(&AdcHandle) != HAL_OK) {
      return 0;
    }
  }

  /*##-2- Configure ADC regular channel ######################################*/
  if (HAL_ADC_ConfigChannel(&AdcHandle, &g_analog_config[id].adcChannelConf) != HAL_OK)
  {
    /* Channel Configuration Error */
    return 0;
  }

  /*##-3- Start the conversion process ####################*/
  if (HAL_ADC_Start(&AdcHandle) != HAL_OK)
  {
    /* Start Conversation Error */
    return 0;
  }

  /*##-4- Wait for the end of conversion #####################################*/
  /*  For simplicity reasons, this example is just waiting till the end of the
      conversion, but application may perform other tasks while conversion
      operation is ongoing. */
  if (HAL_ADC_PollForConversion(&AdcHandle, 10) != HAL_OK)
  {
    /* End Of Conversion flag not set on time */
    return 0;
  }

  /* Check if the continous conversion of regular channel is finished */
  if ((HAL_ADC_GetState(&AdcHandle) & HAL_ADC_STATE_REG_EOC) == HAL_ADC_STATE_REG_EOC)
  {
    /*##-5- Get the converted value of regular channel  ########################*/
    uhADCxConvertedValue = HAL_ADC_GetValue(&AdcHandle);
  }

  if (HAL_ADC_Stop(&AdcHandle) != HAL_OK)
  {
    /* Start Conversation Error */
    return 0;
  }

  return uhADCxConvertedValue;
}

////////////////////////// PWM INTERFACE FUNCTIONS /////////////////////////////


/**
  * @brief TIM MSP Initialization
  *        This function configures the hardware resources used in this example:
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration
  * @param htim: TIM handle pointer
  * @retval None
  */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
  GPIO_InitTypeDef   GPIO_InitStruct;
  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* TIMx Peripheral clock enable */
  timer_enable_clock(htim);

  /* Enable GPIO Channels Clock */
  /* Enable GPIO clock ****************************************/
  if(g_analog_config[g_current_init_id].port == GPIOA) {
    __GPIOA_CLK_ENABLE();
  } else if(g_analog_config[g_current_init_id].port == GPIOB){
    __GPIOB_CLK_ENABLE();
  } else if(g_analog_config[g_current_init_id].port == GPIOC){
    __GPIOC_CLK_ENABLE();
  } else if(g_analog_config[g_current_init_id].port == GPIOD){
    __GPIOD_CLK_ENABLE();
  } else if(g_analog_config[g_current_init_id].port == GPIOE){
    __GPIOE_CLK_ENABLE();
  } else if(g_analog_config[g_current_init_id].port == GPIOF){
    __GPIOF_CLK_ENABLE();
  }

  /* Common configuration for all channels */
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = g_analog_config[g_current_init_id].alFunction;
  GPIO_InitStruct.Pin = g_analog_config[g_current_init_id].pin;

  HAL_GPIO_Init(g_analog_config[g_current_init_id].port, &GPIO_InitStruct);
}

/**
  * @brief  DeInitializes TIM PWM MSP.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef *htim)
{
  timer_disable_clock(htim);
}

/**
  * @brief  This function will set the PWM to the required value
  * @param  port : the gpio port to use
  * @param  pin : the gpio pin to use
  * @param  clock_freq : frequency of the tim clock
  * @param  period : period of the tim counter
  * @param  value : the value to push on the PWM output
  * @param  do_init : if set to 1 the initialization of the PWM is done
  * @retval None
  */
void pwm_start(GPIO_TypeDef  *port, uint32_t pin, uint32_t clock_freq,
                uint32_t period, uint32_t value, uint8_t do_init)
{
  //find the instance in the global
  int8_t id = get_analog_instance(port, pin);
  if(id < 0) return;

  /* Compute the prescaler value to have TIM counter clock equal to clock_freq Hz */
  g_analog_config[id].timHandle.Instance               = g_analog_config[id].timInstance;
  g_analog_config[id].timHandle.Init.Prescaler         = (uint32_t)(SystemCoreClock / clock_freq) - 1;
  g_analog_config[id].timHandle.Init.Period            = period;
  g_analog_config[id].timHandle.Init.ClockDivision     = 0;
  g_analog_config[id].timHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  g_analog_config[id].timHandle.Init.RepetitionCounter = 0;

  if(do_init == 1) {
    g_current_init_id = id;
    if (HAL_TIM_PWM_Init(&g_analog_config[id].timHandle) != HAL_OK) {
      return;
    }
  }

  //HAL_TIM_PWM_Stop(&g_analog_config[id].timHandle, g_analog_config[id].timChannel);

  /*##-2- Configure the PWM channels #########################################*/
  /* Common configuration for all channels */
  g_analog_config[id].timConfig.Pulse = value;

  if (HAL_TIM_PWM_ConfigChannel(&g_analog_config[id].timHandle, &g_analog_config[id].timConfig,
                                  g_analog_config[id].timChannel) != HAL_OK)
  {
    /*##-2- Configure the PWM channels #########################################*/
    return;
  }

  if(g_analog_config[id].useNchannel) {
    HAL_TIMEx_PWMN_Start(&g_analog_config[id].timHandle, g_analog_config[id].timChannel);
  } else {
    HAL_TIM_PWM_Start(&g_analog_config[id].timHandle, g_analog_config[id].timChannel);
  }
}

/**
  * @brief  This function will disable the PWM
  * @param  port : the gpio port to use
  * @param  pin : the gpio pin to use
  * @retval None
  */
void pwm_stop(GPIO_TypeDef  *port, uint32_t pin)
{
  //find the instance in the global
  int8_t id = get_analog_instance(port, pin);
  if(id < 0) return;

  if(g_analog_config[id].useNchannel) {
    HAL_TIMEx_PWMN_Stop(&g_analog_config[id].timHandle, g_analog_config[id].timChannel);
  } else {
    HAL_TIM_PWM_Stop(&g_analog_config[id].timHandle, g_analog_config[id].timChannel);
  }

  HAL_TIM_PWM_DeInit(&g_analog_config[id].timHandle);
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

/**
  * @}
  */
#ifdef __cplusplus
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
