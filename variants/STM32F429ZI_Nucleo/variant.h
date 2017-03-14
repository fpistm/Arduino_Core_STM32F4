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

#ifndef _VARIANT_ARDUINO_STM32_
#define _VARIANT_ARDUINO_STM32_

/*----------------------------------------------------------------------------
 *        Definitions
 *----------------------------------------------------------------------------*/

/** Frequency of the board main oscillator */
//#define VARIANT_MAINOSC		12000000

/** Master clock frequency */
//#define VARIANT_MCK			84000000

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "Arduino.h"
#ifdef __cplusplus
#include "UARTClass.h"
#include "USARTClass.h"
#endif

#ifdef __cplusplus
extern "C"{
#endif // __cplusplus

/**
 * Libc porting layers
 */
#if defined (  __GNUC__  ) /* GCC CS3 */
#    include <syscalls.h> /** RedHat Newlib minimal stub */
#endif

/*----------------------------------------------------------------------------
 *        Pins
 *----------------------------------------------------------------------------*/
#include "PeripheralPins.h"

#define NUM_DIGITAL_PINS        114
#define NUM_ANALOG_INPUTS       (sizeof(PinMap_ADC)/sizeof(PinMap))
#define MAX_DIGITAL_IOS         NUM_DIGITAL_PINS
#define MAX_ANALOG_IOS          NUM_ANALOG_INPUTS

// D0 to D15
#define digitalToPin(p)         ((p < 16) ? digital_arduino[p] : (STM_VALID_PINNAME(p))? p : NC)
// A0 to A5
#define analogToPin(p)          ((p < 6) ? analog_arduino[p] : digitalToPin(p))

#define digitalPinToPort(P)     ( get_GPIO_Port(P) )
#define digitalPinToBitMask(P)  ( STM_GPIO_PIN(P) )

//ADC resolution is 12bits
#define ADC_RESOLUTION          12
#define DACC_RESOLUTION         12

//PWR resolution
#define PWM_RESOLUTION          8
#define PWM_FREQUENCY           1000
#define PWM_MAX_DUTY_CYCLE      255


//SPI defintions
//define 16 channels. As many channel as digital IOs
#define SPI_CHANNELS_NUM        16

//default chip salect pin
#define BOARD_SPI_DEFAULT_SS    10

//In case SPI CS channel is not used we define a default one
#define BOARD_SPI_OWN_SS        SPI_CHANNELS_NUM

#define SPI_INTERFACES_COUNT    1

static const uint8_t SS   = BOARD_SPI_DEFAULT_SS;
static const uint8_t SS1  = 4;
static const uint8_t SS2  = 7;
static const uint8_t SS3  = 8;
static const uint8_t MOSI = 11;
static const uint8_t MISO = 12;
static const uint8_t SCK  = 13;

//Enable Firmata
#define STM32 1

#ifdef __cplusplus
} // extern "C"
#endif
/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus
extern UARTClass Serial;
extern UARTClass Serial1;
extern USARTClass Serial2;

#endif

#endif /* _VARIANT_ARDUINO_STM32_ */
