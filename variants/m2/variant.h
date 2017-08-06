/*
*  variant.h for Macchina M2
*
*	Author:	MACCHINA.cc Tony Doust, Adam Voss
*	Date:	8/5/2017
*	Version: V1.0
*
* Short description:
*	Macchina M2 Arduino_DUE Variant PIN Numbers to PIN definitions 
*/

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

/*  Copyright (c) 2016 Macchina
*
*  Permission is hereby granted, free of charge, to any person obtaining
*  a copy of this software and associated documentation files (the
*  "Software"), to deal in the Software without restriction, including
*  without limitation the rights to use, copy, modify, merge, publish,
*  distribute, sublicense, and/or sell copies of the Software, and to
*  permit persons to whom the Software is furnished to do so, subject to
*  the following conditions:
*
*  The above copyright notice and this permission notice shall be included
*  in all copies or substantial portions of the Software.
*
*  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
*  IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
*  CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
*  TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
*  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef _VARIANT_ARDUINO_DUE_X_
#define _VARIANT_ARDUINO_DUE_X_

/*************************************************************************************
*					WARNING	for use with M2 BETA  Hardware ONLY						 *
* Uncomment the following define to use the BETA version of the GPIOx_B sink OUTPUTS *
*																					 *
*************************************************************************************/

//#define M2_Beta

/************************************************************************************/


/*----------------------------------------------------------------------------
 *        Definitions
 *----------------------------------------------------------------------------*/

/** Frequency of the board main oscillator */
#define VARIANT_MAINOSC		12000000

/** Master clock frequency */
#define VARIANT_MCK			84000000

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

// Number of pins defined in PinDescription array
#define PINS_COUNT           (79u)
#define NUM_DIGITAL_PINS     (66u)
#define NUM_ANALOG_INPUTS    (12u)
#define analogInputToDigitalPin(p)  ((p < 12u) ? (p) + 54u : -1)

#define digitalPinToPort(P)        ( g_APinDescription[P].pPort )
#define digitalPinToBitMask(P)     ( g_APinDescription[P].ulPin )
//#define analogInPinToBit(P)        ( )
#define portOutputRegister(port)   ( &(port->PIO_ODSR) )
#define portInputRegister(port)    ( &(port->PIO_PDSR) )
#define digitalPinHasPWM(P)        ( g_APinDescription[P].ulPWMChannel != NOT_ON_PWM || g_APinDescription[P].ulTCChannel != NOT_ON_TIMER )

/*
 * portModeRegister(..) should return a register to set pin mode
 * INPUT or OUTPUT by setting the corresponding bit to 0 or 1.
 * Unfortunately on SAM architecture the PIO_OSR register is
 * read-only and can be set only through the enable/disable registers
 * pair PIO_OER/PIO_ODR.
 */
// #define portModeRegister(port)   ( &(port->PIO_OSR) )

/*
 * digitalPinToTimer(..) is AVR-specific and is not defined for SAM
 * architecture. If you need to check if a pin supports PWM you must
 * use digitalPinHasPWM(..).
 *
 * https://github.com/arduino/Arduino/issues/1833
 */
// #define digitalPinToTimer(P)

// Interrupts
#define digitalPinToInterrupt(p)  ((p) < NUM_DIGITAL_PINS ? (p) : -1)

// **************************************************************************** //
//							M2 PIN Definitions									//
// **************************************************************************** //

// M2 LEDs
#define DS2				(32u)	// (RED)
#define DS3				(94u)	// (YELLOW)
#define DS4				(27u)	// (YELLOW)
#define	DS5				(24u)	// (YELLOW)
#define	DS6				(23u)	// (GREEN)

#define DS7_RED			(11u)	// RGB Red LED
#define	DS7_GREEN		(12u)	// RGB Green LED
#define	DS7_BLUE		(5u)	// RGB Blue LED

#define RGB_RED	= DS7_RED		// RGB Red LED
#define	RGB_GREEN = DS7_GREEN	// RGB Green LED
#define	RGB_BLUE = DS7_BLUE		// RGB Blue LED


/*
#define PIN_LED_13		(13u)
#define PIN_LED_RXL		(72u)
#define PIN_LED_TXL		(73u)
#define PIN_LED			PIN_LED_13
#define PIN_LED2		PIN_LED_RXL
#define PIN_LED3		PIN_LED_TXL
#define LED_BUILTIN		(13u)
*/

// M2 GPIO
#define	GPIO1			(35u)
#define	GPIO2			(37u)
#define	GPIO3			(39u)
#define	GPIO4			(41u)
#define	GPIO5			(95u)
#define	GPIO6			(44u)

#ifdef M2_Beta	// M2 Beta legacy Hardware Sink Input Pins
	// M2 GPIO_B pins for Sinking INPUT Pins
	#define	GPIO1_B		(34u)
	#define	GPIO2_B		(36u)
	#define	GPIO3_B		(38u)
	#define	GPIO4_B		(40u)
	#define	GPIO5_B		(9u)
	#define	GPIO6_B		(8u)
#endif


// M2 User Buttons
#define Button1			(92u)
#define Button2			(93u)


// M2 Analogue GPIO
#define	ANALOG_1		(64u)
#define	ANALOG_2		(63u)
#define	ANALOG_3		(61u)
#define	ANALOG_4		(59u)
#define	ANALOG_5		(60u)
#define	ANALOG_6		(54u)


// CPU Temperature
#define	CPU_TEMP		(96u)	// CPU on chip Tempeature Fix not working TD 6-8-2017 assigned 2 different pin numbers


// Vehicle Voltage
#define	V_SENSE			(58u)


//CURRENT SENSE Power Supply
#define	I_SENSE_EN		(6u)	// 12VIO_EN enable the Current sensing for 12VIO
#define	I_SENSE			(62u)	// Analogue AD8 Input for 12VIO current sensing
#define	I_SENSE_INT		(26u)	// Interupt from Power supply Overcurrent
#define	I_SENSE_DAC		(67u)	// DAC output from CPU to Comparator for Over Current Sensing


// SD
#define	SD_SW			(72u)
#define	MCCK			(42u)
#define	MCCDA			(43u)
#define	MCDA0			(73u)
#define	MCDA1			(57u)
#define	MCDA2			(56u)
#define	MCDA3			(55u)


// CAN							// ** TODO put all CAN definations here TD 6-8-2017 ** //
#define	CANTX0			(69u)	// ** Check if duplicated TD 6-8-2017 ** //
#define	CANRX0			(68u)	// ** Check if duplicated TD 6-8-2017 ** //
#define	CAN0_CS			(28u)
#define	HS_CS		  CAN0_CS

#define	CANTX1			(53u)	// ** Check if duplicated TD 6-8-2017 ** //
#define	CANRX1			(66u)	// ** Check if duplicated TD 6-8-2017 ** //
#define	CAN1_CS			(25u)
#define	MS_CS		 CAN1_CS


// J1850
#define	J1850_PWM_VPW	(97u)
#define	J1850_PWM_RX	(3u)
#define	J1850_VPW_RX	(4u)
#define	J1850P_TX		(45u)
#define	J1850N_TX		(7u)

// Power Supply
#define	PS_BUCK			(98u)
#define BUCK_DIS		PS_BUCK
#define	PS_J1850_9141	(99u)

// XBEE
#define	XBEE_RX			(0u)
#define	XBEE_TX			(1u)
#define	SPI0_MISO		(74u)
#define	SPI0_MOSI		(75u)
#define	SPI0_CLK		(76u)
#define	SPI0_CS			(77u)
#define	XBEE_RST		(100u)
#define	XBEE_PWM		(101u)
#define	XBEE_MULT1		(51u)
#define	XBEE_MULT2		(48u)
#define	XBEE_MULT3		(46u)
#define	XBEE_MULT4		(29u)
#define	XBEE_MULT5		(30u)
#define	XBEE_MULT6		(31u)
#define XBEE_CTS		(22u)
#define	XBEE_STAT		(50u)
#define	XBEE_VREF		(49u)
#define	XBEE_RTS		(2u)


// 9141/LIN
#define	LIN_KTX			(18u)
#define	LIN_KRX			(19u)
#define	LIN_KSLP		(102u)
#define	LIN_LTX			(16u)
#define	LIN_LRX			(17u)
#define	LIN_LSLP		(103u)


// Single Wire Can SWC
#define	SWC_M0			(13u)
#define	SWC_M1			(104u)
#define	SWC_SOF			(10u)
#define	SWC_CLK			(105u)
#define	SWC_RST			-1
#define	SPI0_CS3		(78u)
#define	SWC_INT			(47u)
#define	SWC_RX0			(106u)
#define	SWC_RX1			(107u)

// **************************************************************************** //

/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 1

#define SPI_INTERFACE		   SPI0
#define SPI_INTERFACE_ID	  ID_SPI0
#define SPI_CHANNELS_NUM		4
#define PIN_SPI_SS0				77
#define PIN_SPI_SS1				87
#define PIN_SPI_SS2				86	// ** Check USART2RX ** TD 6-8-2017//
#define PIN_SPI_SS3				78	// ** Check SPIO_CS3 ** TD 6-8-2017//
#define PIN_SPI_MOSI			75
#define PIN_SPI_MISO			74
#define PIN_SPI_SCK				76	// *** Check SPIO_CLK ** TD 6-8-2017//
#define BOARD_SPI_SS0			10
#define BOARD_SPI_SS1			4
#define BOARD_SPI_SS2			52
#define BOARD_SPI_SS3        PIN_SPI_SS3
#define BOARD_SPI_DEFAULT_SS BOARD_SPI_SS3

#define SPI0_CS1				(87u)		// TODO Move to logical section Function group TD 6-8-2017 ** //

// ** TODO Check all SPI Board to Pin assignments may need to fix this ** TD 6-8-2017//
#define BOARD_PIN_TO_SPI_PIN(x) \
	(x==BOARD_SPI_SS0 ? PIN_SPI_SS0 : \
	(x==BOARD_SPI_SS1 ? PIN_SPI_SS1 : \
	(x==BOARD_SPI_SS2 ? PIN_SPI_SS2 : PIN_SPI_SS3 )))
#define BOARD_PIN_TO_SPI_CHANNEL(x) \
	(x==BOARD_SPI_SS0 ? 0 : \
	(x==BOARD_SPI_SS1 ? 1 : \
	(x==BOARD_SPI_SS2 ? 2 : 3)))

static const uint8_t SS   = BOARD_SPI_SS0;
static const uint8_t SS1  = BOARD_SPI_SS1;
static const uint8_t SS2  = BOARD_SPI_SS2;
static const uint8_t SS3  = BOARD_SPI_SS3;
static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK  = PIN_SPI_SCK;

/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 2

#define PIN_WIRE_SDA         20
#define PIN_WIRE_SCL         21
#define WIRE_INTERFACE       TWI1
#define WIRE_INTERFACE_ID    ID_TWI1
#define WIRE_ISR_HANDLER     TWI1_Handler
#define WIRE_ISR_ID          TWI1_IRQn
static const uint8_t SDA1 = PIN_WIRE_SDA;
static const uint8_t SCL1 = PIN_WIRE_SCL;

#define PIN_WIRE1_SDA        70
#define PIN_WIRE1_SCL        71
#define WIRE1_INTERFACE      TWI0
#define WIRE1_INTERFACE_ID   ID_TWI0
#define WIRE1_ISR_HANDLER    TWI0_Handler
#define WIRE1_ISR_ID         TWI0_IRQn

static const uint8_t SDA0  = PIN_WIRE1_SDA;
static const uint8_t SCL0  = PIN_WIRE1_SCL;

// UART3 PINS
#define TXD3				14
#define RXD3				15

/*
 * UART/USART Interfaces
 */
// Serial
#define PINS_UART            81
// Serial1
#define PINS_USART0          82
// Serial2
#define PINS_USART1          83
// Serial3
#define PINS_USART3          84

/*
 * USB Interfaces
 */
#define PINS_USB             85

/*
 * Analog pins
 */
static const uint8_t A0  =	(54u);
static const uint8_t A1  =	(55u);
static const uint8_t A2  =	(56u);
static const uint8_t A3  =	(57u);
static const uint8_t A4  =	(58u);
static const uint8_t A5  =	(59u);
static const uint8_t A6  =	(60u);
static const uint8_t A7  =	(61u);
static const uint8_t A8  =	(62u);
static const uint8_t A9  =	(63u);
static const uint8_t A10 =	(64u);
static const uint8_t A11 =	(65u);
static const uint8_t A15 =	(96u);	// CPU on chip Tempeature Fix not working TD 6-8-2017 assigned 2 different pin numbers
static const uint8_t DAC0 = (66u);
static const uint8_t DAC1 = (67u);

static const uint8_t CANRX = 68;	// ** Check if duplicated TD 6-8-2017 ** //
static const uint8_t CANTX = 69;	// ** Check if duplicated TD 6-8-2017 ** //

#define ADC_RESOLUTION		12

/*
 * Complementary CAN pins
 */
static const uint8_t CAN1RX = 88;	// ** Check if duplicated TD 6-8-2017 ** //
static const uint8_t CAN1TX = 89;	// ** Check if duplicated TD 6-8-2017 ** //

// CAN0
#define PINS_CAN0            90
// CAN1
#define PINS_CAN1            91


/*
 * DACC
 */
#define DACC_INTERFACE		DACC
#define DACC_INTERFACE_ID	ID_DACC
#define DACC_RESOLUTION		12
#define DACC_ISR_HANDLER    DACC_Handler
#define DACC_ISR_ID         DACC_IRQn

/*
 * PWM
 */
#define PWM_INTERFACE		PWM
#define PWM_INTERFACE_ID	ID_PWM
#define PWM_FREQUENCY		1000
#define PWM_MAX_DUTY_CYCLE	255
#define PWM_MIN_DUTY_CYCLE	0
#define PWM_RESOLUTION		8

/*
 * TC
 */
#define TC_INTERFACE        TC0
#define TC_INTERFACE_ID     ID_TC0
#define TC_FREQUENCY        1000
#define TC_MAX_DUTY_CYCLE   255
#define TC_MIN_DUTY_CYCLE   0
#define TC_RESOLUTION		8

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus

extern UARTClass Serial;
extern USARTClass Serial1;
extern USARTClass Serial2;
extern USARTClass Serial3;

#endif

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
#define SERIAL_PORT_MONITOR         Serial
#define SERIAL_PORT_USBVIRTUAL      SerialUSB
#define SERIAL_PORT_HARDWARE_OPEN   Serial1
#define SERIAL_PORT_HARDWARE_OPEN1  Serial2
#define SERIAL_PORT_HARDWARE_OPEN2  Serial3
#define SERIAL_PORT_HARDWARE        Serial
#define SERIAL_PORT_HARDWARE1       Serial1
#define SERIAL_PORT_HARDWARE2       Serial2
#define SERIAL_PORT_HARDWARE3       Serial3

#endif /* _VARIANT_ARDUINO_DUE_X_ */

