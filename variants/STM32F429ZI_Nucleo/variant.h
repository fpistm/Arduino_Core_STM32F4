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

extern const PinName digital_arduino[];

enum {
  D0,  D1,  D2,  D3,  D4,  D5,  D6,  D7,  D8,  D9,
  D10, D11, D12, D13, D14, D15, D16, D17, D18, D19,
  D20, D21, D22, D23, D24, D25, D26, D27, D28, D29,
  D30, D31, D32, D33, D34, D35, D36, D37, D38, D39,
  D40, D41, D42, D43, D44, D45, D46, D47, D48, D49,
  D50, D51, D52, D53, D54, D55, D56, D57, D58, D59,
  D60, D61, D62, D63, D64, D65, D66, D67, D68, D69,
  D70, D71, D72, D73, D74, D75, D76, D77, D78, D79,
  D80, D81,
  DEND
};

#define PIN_A0   (73)
#define PIN_A1   (74)
#define PIN_A2   (75)
#define PIN_A3   (76)
#define PIN_A4   (77)
#define PIN_A5   (78)

//Analog pins
static const uint8_t A0  = PIN_A0;
static const uint8_t A1  = PIN_A1;
static const uint8_t A2  = PIN_A2;
static const uint8_t A3  = PIN_A3;
static const uint8_t A4  = PIN_A4;
static const uint8_t A5  = PIN_A5;

#define NUM_DIGITAL_PINS        DEND
#define NUM_ANALOG_INPUTS       (sizeof(PinMap_ADC)/sizeof(PinMap))
#define MAX_DIGITAL_IOS         NUM_DIGITAL_PINS
#define MAX_ANALOG_IOS          NUM_ANALOG_INPUTS

// Convert a digital pin number Dxx to a PinName Pxy
#define digitalToPinName(p)     ((p < NUM_DIGITAL_PINS) ? digital_arduino[p] : (STM_VALID_PINNAME(p))? (PinName)p : NC)
// Convert an analog pin number Axx to a PinName Pxy
#define analogToPinName(p)      ((p < 6) ? digitalToPinName(p+73) : digitalToPinName(p))
// Convert an analog pin number to a digital pin number
#define analogToDigital(p)      ((p < 6) ? (p+73) : p)
// Convert a PinName Pxy to a pin number
uint32_t pinNametoPinNumber(PinName p);

#define digitalPinToPort(p)     ( get_GPIO_Port(digitalToPinName(p)) )
#define digitalPinToBitMask(p)  ( STM_GPIO_PIN(digitalToPinName(p)) )

//ADC resolution is 12bits
#define ADC_RESOLUTION          12
#define DACC_RESOLUTION         12

//PWR resolution
#define PWM_RESOLUTION          8
#define PWM_FREQUENCY           1000
#define PWM_MAX_DUTY_CYCLE      255

//On-board LED pin number
#define LED_BUILTIN             33
#define LED_GREEN               LED_BUILTIN
#define LED_BLUE                79
#define LED_RED                 80

//On-board user button
#define USER_BTN                81


//SPI definitions
//define 16 channels. As many channel as digital IOs
#define SPI_CHANNELS_NUM        16

//default chip salect pin
#define BOARD_SPI_DEFAULT_SS    10

//In case SPI CS channel is not used we define a default one
#define BOARD_SPI_OWN_SS    SPI_CHANNELS_NUM

#define SS    BOARD_SPI_DEFAULT_SS
#define SS1   4
#define SS2   7
#define SS3   8
#define MOSI  11
#define MISO  12
#define SCLK  13
#define SCK   SCLK

//I2C Definitions
#define SDA   14
#define SCL   15

//Timer Definitions
//Do not use timer used by PWM pins when possible. See PinMap_PWM.
#define TIMER_TONE          TIM6
#define TIMER_UART_EMULATED TIM7

//Do not use basic timer: OC is required
#define TIMER_SERVO         TIM2  //TODO: advanced-control timers don't work

//Enable Firmata
#define STM32 1

#ifdef __cplusplus
} // extern "C"
#endif
/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus
extern HardwareSerial Serial;
extern HardwareSerial Serial1;
extern HardwareSerial Serial2;

// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.
#define SERIAL_PORT_MONITOR   Serial
#define SERIAL_PORT_HARDWARE  Serial

#endif

#endif /* _VARIANT_ARDUINO_STM32_ */
