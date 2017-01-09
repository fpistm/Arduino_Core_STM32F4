/**
  ******************************************************************************
  * @file    usb_interface.c
  * @author  WI6LABS
  * @version V1.0.0
  * @date    27-October-2016
  * @brief   provide the usb interface
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
#include "usb_interface.h"

/**
  * @}
  */

#ifdef __cplusplus
 extern "C" {
#endif

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

/** @addtogroup STM32F4xx_System_Private_Macros
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32F4xx_System_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32F4xx_System_Private_Variables
  * @{
  */
static USBD_HandleTypeDef g_USBD_Device;

/**
  * @}
  */

/** @addtogroup STM32F4xx_System_Private_FunctionPrototypes
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32F4xx_System_Private_Functions
  * @{
  */

/**
  * @brief  initialize the USB HID interface
  * @param  HID_Desc : usb HID descriptor callbacks
  * @retval none
  */
void usbd_interface_init(void)
{
  /* Init Device Library */
  USBD_Init(&g_USBD_Device, &HID_Desc, 0);

  /* Add Supported Class */
  USBD_RegisterClass(&g_USBD_Device, USBD_COMPOSITE_HID_CLASS);

  /* Start Device Process */
  USBD_Start(&g_USBD_Device);
}

/**
  * @brief  initialize the USB HID interface
  * @param  HID_Desc : usb HID descriptor callbacks
  * @retval none
  */
void usbd_interface_mouse_sendReport(uint8_t *report, uint16_t len)
{
  USBD_HID_MOUSE_SendReport(&g_USBD_Device, report, len);
}

/**
  * @brief  initialize the USB HID interface
  * @param  HID_Desc : usb HID descriptor callbacks
  * @retval none
  */
void usbd_interface_keyboard_sendReport(uint8_t *report, uint16_t len)
{
  USBD_HID_KEYBOARD_SendReport(&g_USBD_Device, report, len);
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
