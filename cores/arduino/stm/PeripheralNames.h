/*
 *******************************************************************************
 * Copyright (c) 2016, STMicroelectronics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
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
 *******************************************************************************
 */
#ifndef _PERIPHERALNAMES_H
#define _PERIPHERALNAMES_H

#ifdef __cplusplus
extern "C" {
#endif

/*typedef enum {
    ADC_1 = (int)ADC1_BASE,
    ADC_2 = (int)ADC2_BASE,
    ADC_3 = (int)ADC3_BASE
} ADCName;*/

/*typedef enum {
    DAC_1 = (int)DAC_BASE
} DACName;*/

typedef enum {
    UART1 = (int)USART1_BASE,
    UART2 = (int)USART2_BASE,
    UART3 = (int)USART3_BASE,
    //UART4 = (int)UART4_BASE,
    //UART5 = (int)UART5_BASE,
    UART6 = (int)USART6_BASE,
    //UART7 = (int)UART7_BASE,
    //UART8 = (int)UART8_BASE
} UARTName;

/*#define STDIO_UART_TX  PA_9
#define STDIO_UART_RX  PA_10
#define STDIO_UART     UART_1*/

/*typedef enum {
    SPI_1 = (int)SPI1_BASE,
    SPI_2 = (int)SPI2_BASE,
    SPI_3 = (int)SPI3_BASE,
    SPI_4 = (int)SPI4_BASE,
    SPI_5 = (int)SPI5_BASE,
    SPI_6 = (int)SPI6_BASE
} SPIName;*/

/*typedef enum {
    I2C_1 = (int)I2C1_BASE,
    I2C_2 = (int)I2C2_BASE,
    I2C_3 = (int)I2C3_BASE
} I2CName;*/

typedef enum {
    PWM1  = (int)TIM1_BASE,
    PWM2  = (int)TIM2_BASE,
    PWM3  = (int)TIM3_BASE,
    PWM4  = (int)TIM4_BASE,
    PWM5  = (int)TIM5_BASE,
    PWM8  = (int)TIM8_BASE,
    PWM9  = (int)TIM9_BASE,
    PWM10 = (int)TIM10_BASE,
    PWM11 = (int)TIM11_BASE,
    PWM12 = (int)TIM12_BASE,
    PWM13 = (int)TIM13_BASE,
    PWM14 = (int)TIM14_BASE
} PWMName;

/*typedef enum {
    CAN_1 = (int)CAN1_BASE,
    CAN_2 = (int)CAN2_BASE
} CANName;*/

#ifdef __cplusplus
}
#endif

#endif
