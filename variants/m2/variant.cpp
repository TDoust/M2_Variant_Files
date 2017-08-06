/*
*  variant.cpp for Macchina M2
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

#include "variant.h"

/*
 * DUE Board pin   |  PORT  | Label
 * ----------------+--------+-------
 *   0	  (Serial) |  PA8   | "XBEE_RX/RX0"
 *   1             |  PA9   | "XBEE_TX/TX0"
 *   2       TIOA0 |  PB25  | "XBEE_RTS"
 *   3       TIOA7 |  PC28  | "J1850_PWM_RX"
 *   4       TIOB6 |  PC26  | "J1850_VPW_RX"
 *   5       TIOA6 |  PC25  | "DS7_BLUE/RGB_BLUE"
 *   6       PWML7 |  PC24  | "I_SENSE_EN"
 *   7       PWML6 |  PC23  | "J1850N_TX"
 *   8       PWML5 |  PC22  | "GPIO6_B"			// legacy M2 Beta
 *   9       PWML4 |  PC21  | "GPIO5_B"			// legacy M2 Beta
 *  10       TIOB7 |  PC29  | "SWC_SOF"
 *  11       TIOA8 |  PD7   | "DS7_RED/RGB_RED"
 *  12       TIOB8 |  PD8   | "DS7_Green/RGB_GREEN"
 *  13       TIOB0 |  PB27  | "SWC M0"
 *  14	 (Serial3) |  PD4   | "TXD3/UART3"		// 26 Pin Connector
 *  15			   |  PD5   | "RXD3/UART3"		// 26 Pin Connector
 *  16	 (Serial2) |  PA13  | "9141_LTX"
 *  17			   |  PA12  | "9141_LRX"
 *  18	 (Serial1) |  PA11  | "9141_KTX"
 *  19			   |  PA10  | "9141_KRX"
 *  20             |  PB12  | "SDA1"
 *  21             |  PB13  | "SCL1"
 *  22             |  PB26  | "XBEE_CTS"
 *  23             |  PA14  | "DS6"				// Green LED
 *  24             |  PA15  | "DS5"				// Yellow LED
 *  25             |  PD0   | "CAN1_CS/MS_CS"
 *  26             |  PD1   | "I_SENSE_INT"
 *  27             |  PD2   | "DS4"				// Yellow LED"
 *  28             |  PD3   | "CAN0_CS/HS_CS"
 *  29             |  PD6   | "XBEE_MULT4"
 *  30             |  PD9   | "XBEE_MULT5"
 *  31             |  PA7   | "XBEE_MULT6"
 *  32             |  PD10  | "DS2"				// Red LED
 *  33             |  PC1   | "unconnected!"	// ** unconnected processor pin ** //
 *  34             |  PC2   | "GPIO1_B"			// legacy M2 Beta
 *  35             |  PC3   | "GPIO1"
 *  36             |  PC4   | "GPIO2_B"			// legacy M2 Beta
 *  37             |  PC5   | "GPIO2"
 *  38             |  PC6   | "GPIO3_B"			// legacy M2 Beta
 *  39             |  PC7   | "GPIO3"
 *  40             |  PC8   | "GPIO4_B"			// legacy M2 Beta
 *  41             |  PC9   | "GPIO4"
 *  42             |  PA19  | "MCCK"
 *  43             |  PA20  | "MCCDA"
 *  44             |  PC19  | "GPIO6"
 *  45             |  PC18  | "J1850P_TX"
 *  46             |  PC17  | "XBEE_MULT3"
 *  47             |  PC16  | "SWC_INT"
 *  48             |  PC15  | "XB_MULT2"
 *  49             |  PC14  | "XBEE_VREF"
 *  50             |  PC13  | "XBEE_STAT"
 *  51             |  PC12  | "XBEE_MULT1"
 *  52       NPCS2 |  PB21  | "SS2"						// USART2RX // ** Check TD 6-8-2017 ** //
 *  53             |  PB14  | "CANTX1"					// duplicated // ** TODO add additional pin names TD 6-8-2017 ** //
 *  54             |  PA16  | "ANALOG_6"		// AD0
 *  55             |  PA24  | "MCDA3"			// AD1
 *  56             |  PA23  | "MCDA2"			// AD2
 *  57             |  PA22  | "MCDA1"			// AD3		// ** Check TD 6-8-2017 ** //
 *  58       TIOB2 |  PA6   | "V_SENSE"			// AD3		// ** Check TD 6-8-2017 ** //
 *  59             |  PA4   | "ANALOG_4"		// AD5
 *  60       TIOB1 |  PA3   | "ANALOG_5"		// AD6
 *  61       TIOA1 |  PA2   | "ANALOG_3"		// AD7
 *  62             |  PB17  | "I_SENSE"			// AD8
 *  63             |  PB18  | "ANALOG_2"		// AD9
 *  64             |  PB19  | "ANALOG_1"		// AD10
 *  65             |  PB20  | "USART2TX"		// AD11		// ** Check TD 6-8-2017 ** //
 *  66             |  PB15  | "CANRX1"			// DAC0
 *  67             |  PB16  | "I_SENSE_DAC"		// DAC1
 *  68             |  PA1   | "CANRX0"					// ** TODO add additional pin names TD 6-8-2017 ** //
 *  69             |  PA0   | "CANTX0"					// ** TODO add additional pin names TD 6-8-2017 ** //
 *  70             |  PA17  | "SDA0"
 *  71             |  PA18  | "SCL0"
 *  72             |  PC30  | "SD_SW"
 *  73             |  PA21  | "MCDA0"
 *  74       MISO  |  PA25  | "SPI0_MISO"				// Check SPIO or SPI0 ?? TD 6-8-2017 ** //
 *  75       MOSI  |  PA26  | "SPI0_MOSI"
 *  76       SCLK  |  PA27  | "SPI0_CLK"
 *  77			   |  PA28  | "SPI0_CS"
 *  78			   |  PB23  | "SPI0_CS3"
 *  79				Pin Masks
 *		to
 *  84 				Pin Masks
 *  85             |  PB11  | "USB" (UOTGID)
 *  86             |  PB21  | "USART2RX"
 *  87       NPCS1 |  PA29  | "SPI0_CS1"
 *  88             |  PB15  | "CAN1RX"					// ** TODO add additional pin names TD 6-8-2017 ** //
 *  89             |  PB14  | "CAN1TX"					// duplicated // ** TODO add additional pin names TD 6-8-2017 ** //
 *  90 CAN Pin Mask|  PA1A  | "PINS_CAN0"		// Can0 Pin mask for TX & RX
 *  91 CAN Pin Mask|  PB15A | "PINS_CAN1"		// Can1 Pin mask for TX & RX
 *  92             |  PC27  | "Button1"			// M2_Button_1
 *  93             |  PB6A  | "Button1"			// M2_Button_2
 *  94             |  PA5   | "DS3"				// Yellow LED
 *  95             |  PC20  | "GPIO5"
 *  96             |  PD5   | "CPU_TEMP/A15"	// CPU Temperature
 *  97             |  PB8   | "J1850_PWM_VPW"
 *  98             |  PC10  | "PS_BUCK/PS_BUCK"
 *  99             |  PB5   | "PS_J1850_9141"
 *  100            |  PC11  | "XBEE_RST"
 *  101            |  PB3   | "XBEE_PWM"
 *  102            |  PB4   | "LIN_KSLP"
 *  103            |  PB7   | "LIN_LSLP"
 *  104            |  PB0   | "SWC_M1"
 *  105            |  PB22  | "SWC_CLK"
 *  106            |  PB1   | "SWC_RX0"
 *  107            |  PB2   | "SWC_RX1"
 *
 * USB pin         |  PORT
 * ----------------+--------
 *  ID             |  PB11
 *  VBOF           |  PB10
 *
 */

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Pins descriptions
 */
extern const PinDescription g_APinDescription[]=
{
	// **************************************************************************** //
	//							M2 PIN Definitions									//
	// **************************************************************************** //
// ******************** TODO Finish Not all old pins have been taken out ***********************
  // 0 .. 53 - Digital pins
  // ----------------------
  // 0/1 - UART (Serial)
// pPort,	ulPin,		ulPeripheralId, ulPinType, ulPinConfiguration,ulPinAttribute,	ulAnalogChannel,ulADCChannelNumber,ulPWMChannel,ulTCChannel
  { PIOA, PIO_PA8A_URXD,     ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT,  PIN_ATTR_DIGITAL,                 NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // URXD
  { PIOA, PIO_PA9A_UTXD,     ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT,  PIN_ATTR_DIGITAL,                 NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // UTXD

  // 2
  { PIOB, PIO_PB25B_TIOA0,   ID_PIOB, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_TIMER), NO_ADC, NO_ADC, NOT_ON_PWM,  TC0_CHA0     }, // XBEE_RTS
  { PIOC, PIO_PC28B_TIOA7,   ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_TIMER), NO_ADC, NO_ADC, NOT_ON_PWM,  TC2_CHA7     }, // J1850_PWM_RX
  { PIOC, PIO_PC26B_TIOB6,   ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_TIMER), NO_ADC, NO_ADC, NOT_ON_PWM,  TC2_CHB6     }, // J1850_VPW_RX

  // 5
  { PIOC, PIO_PC25B_TIOA6,   ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_TIMER), NO_ADC, NO_ADC, NOT_ON_PWM,  TC2_CHA6     }, // RGB_BLUE
  { PIOC, PIO_PC24B_PWML7,   ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM),   NO_ADC, NO_ADC, PWM_CH7,     NOT_ON_TIMER }, // I_SENSE_EN
  { PIOC, PIO_PC23B_PWML6,   ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM),   NO_ADC, NO_ADC, PWM_CH6,     NOT_ON_TIMER }, // J1850N_TX
  { PIOC, PIO_PC22,			 ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM),   NO_ADC, NO_ADC, PWM_CH5,     NOT_ON_TIMER }, // GPIO6_B
  { PIOC, PIO_PC21,			 ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM),   NO_ADC, NO_ADC, PWM_CH4,     NOT_ON_TIMER }, // GPIO5_B
  // 10
  { PIOC, PIO_PC29B_TIOB7,   ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_TIMER), NO_ADC, NO_ADC, NOT_ON_PWM,  TC2_CHB7     }, // SWC_SOF
  { PIOD, PIO_PD7B_TIOA8,    ID_PIOD, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_TIMER), NO_ADC, NO_ADC, NOT_ON_PWM,  TC2_CHA8     }, // DS7_RED/RGB_RED
  { PIOD, PIO_PD8B_TIOB8,    ID_PIOD, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_TIMER), NO_ADC, NO_ADC, NOT_ON_PWM,  TC2_CHB8     }, // DS7_Green/RGB_GREEN

  // 13 - SWC_M0
  { PIOB, PIO_PB27B_TIOB0,   ID_PIOB, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_TIMER), NO_ADC, NO_ADC, NOT_ON_PWM,  TC0_CHB0     }, // SWC_M0

  // 14/15 - USART3 (Serial3)
  { PIOD, PIO_PD4B_TXD3,     ID_PIOD, PIO_PERIPH_B, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // UART3/TXD3" // 26 Pin Connector
  { PIOD, PIO_PD5B_RXD3,     ID_PIOD, PIO_PERIPH_B, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // UART3/RXD3" // 26 Pin Connector

  // 16/17 - USART1 (Serial2)
  { PIOA, PIO_PA13A_TXD1,    ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // 9141_LTX
  { PIOA, PIO_PA12A_RXD1,    ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // 9141 LRX

  // 18/19 - USART0 (Serial1)
  { PIOA, PIO_PA11A_TXD0,    ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // 9141_KTX
  { PIOA, PIO_PA10A_RXD0,    ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // 9141_KRX

  // 20/21 - TWI1
  { PIOB, PIO_PB12A_TWD1,    ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // SDA1
  { PIOB, PIO_PB13A_TWCK1,   ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // SCL1

  // 22 - 
  { PIOB, PIO_PB26,          ID_PIOB, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // XBEE_CTS
  { PIOA, PIO_PA14,          ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // DS6 // Green LED
  { PIOA, PIO_PA15,          ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // DS5 // Yelow LED
  { PIOD, PIO_PD0,           ID_PIOD, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // CAN1_CS/MS_CS

  // 26
  { PIOD, PIO_PD1,           ID_PIOD, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // I_SENSE_INT
  { PIOD, PIO_PD2,           ID_PIOD, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // DS4 Yellow LED
  { PIOD, PIO_PD3,           ID_PIOD, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // CAN0_CS/HS_CS
  { PIOD, PIO_PD6,           ID_PIOD, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // XBEE_MULT4

  // 30
  { PIOD, PIO_PD9,           ID_PIOD, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // XBEE_MULT5
  { PIOA, PIO_PA7,           ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // XBEE_MULT6
  { PIOD, PIO_PD10,          ID_PIOD, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // DS2 // Red LED
  { PIOC, PIO_PC1,           ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // unconnected!

  // 34
  { PIOC, PIO_PC2,           ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL | PIN_ATTR_PWM), NO_ADC, NO_ADC, PWM_CH0,	 NOT_ON_TIMER }, // GPIO1_B
  { PIOC, PIO_PC3,           ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL | PIN_ATTR_PWM),	NO_ADC, NO_ADC, PWM_CH0,	 NOT_ON_TIMER }, // GPIO1
  { PIOC, PIO_PC4,           ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL | PIN_ATTR_PWM), NO_ADC, NO_ADC, PWM_CH1,	 NOT_ON_TIMER }, // GPIO2_B
  { PIOC, PIO_PC5,           ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL | PIN_ATTR_PWM),	NO_ADC, NO_ADC, PWM_CH1,	 NOT_ON_TIMER }, // GPIO2

  // 38
  { PIOC, PIO_PC6,           ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL | PIN_ATTR_PWM),	NO_ADC, NO_ADC, PWM_CH2,	 NOT_ON_TIMER }, // GPIO3_B
  { PIOC, PIO_PC7,           ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL | PIN_ATTR_PWM),	NO_ADC, NO_ADC, PWM_CH2,	 NOT_ON_TIMER }, // GPIO3
  { PIOC, PIO_PC8,           ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL | PIN_ATTR_PWM),	NO_ADC, NO_ADC, PWM_CH3,	 NOT_ON_TIMER }, // GPIO4_B
  { PIOC, PIO_PC9,           ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL | PIN_ATTR_PWM),	NO_ADC, NO_ADC, PWM_CH3,	 NOT_ON_TIMER }, // GPIO4

  // 42
  { PIOA, PIO_PA19,          ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // MCCK
  { PIOA, PIO_PA20,          ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // MCCDA
  { PIOC, PIO_PC19,          ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL | PIN_ATTR_PWM),	NO_ADC, NO_ADC, PWM_CH5,	 NOT_ON_TIMER }, // GPIO6
  { PIOC, PIO_PC18,          ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // J1850P_TX

  // 46
  { PIOC, PIO_PC17,          ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // XBEE_MULT3
  { PIOC, PIO_PC16,          ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // SWC_INT
  { PIOC, PIO_PC15,          ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // XBEE_MULT2
  { PIOC, PIO_PC14,          ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // XBEE_VREF

  // 50
  { PIOC, PIO_PC13,          ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // XBEE_STAT
  { PIOC, PIO_PC12,          ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // XBEE_MULT1
  { PIOB, PIO_PB21,          ID_PIOB, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // SS2 // USART2RX
  { PIOB, PIO_PB14,          ID_PIOB, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // CANTX1

  // 54 .. 65 - Analog pins
  // ----------------------
  { PIOA, PIO_PA16X1_AD7,    ID_PIOA, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC0,   ADC7,   NOT_ON_PWM,  NOT_ON_TIMER }, // ANALOG_6 // AD0
  { PIOA, PIO_PA24X1_AD6,    ID_PIOA, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC1,   ADC6,   NOT_ON_PWM,  NOT_ON_TIMER }, // MCDA3 // AD1
  { PIOA, PIO_PA23X1_AD5,    ID_PIOA, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC2,   ADC5,   NOT_ON_PWM,  NOT_ON_TIMER }, // MCDA2 // AD2
  { PIOA, PIO_PA22,			 ID_PIOA, PIO_OUTPUT_0,	PIO_DEFAULT, PIN_ATTR_DIGITAL,					NO_ADC,	NO_ADC,	NOT_ON_PWM,  NOT_ON_TIMER }, // MCDA1 // AD3 ** Check **
	// 58
  { PIOA, PIO_PA6,			 ID_PIOA, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC4,   ADC3,   NOT_ON_PWM,  NOT_ON_TIMER }, // V_SENSE // AD3
  { PIOA, PIO_PA4X1_AD2,     ID_PIOA, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC5,   ADC2,   NOT_ON_PWM,  NOT_ON_TIMER }, // ANALOG_4 // AD5
  { PIOA, PIO_PA3X1_AD1,     ID_PIOA, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC6,   ADC1,   NOT_ON_PWM,  TC0_CHB1     }, // ANALOG_5 // AD6
  { PIOA, PIO_PA2X1_AD0,     ID_PIOA, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC7,   ADC0,   NOT_ON_PWM,  TC0_CHA1     }, // ANALOG_3 // AD7
  // 62
  { PIOB, PIO_PB17X1_AD10,   ID_PIOB, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC8,   ADC10,  NOT_ON_PWM,  NOT_ON_TIMER }, // I_SENSE  // AD8
  { PIOB, PIO_PB18X1_AD11,   ID_PIOB, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC9,   ADC11,  NOT_ON_PWM,  NOT_ON_TIMER }, // ANALOG_2 // AD9
  { PIOB, PIO_PB19X1_AD12,   ID_PIOB, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC10,  ADC12,  NOT_ON_PWM,  NOT_ON_TIMER }, // ANALOG_1 // AD10
  { PIOB, PIO_PB20X1_AD13,   ID_PIOB, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC11,  ADC13,  NOT_ON_PWM,  NOT_ON_TIMER }, // USART2TX // AD11 ** Check **

  // 66/67 - CANRX1/DAC1
  { PIOB, PIO_PB15X1_DAC0,   ID_PIOB, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,					ADC12,  DA0,    NOT_ON_PWM,  NOT_ON_TIMER }, // CANRX1	  // DAC0
  { PIOB, PIO_PB16X1_DAC1,   ID_PIOB, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC13,  DA1,    NOT_ON_PWM,  NOT_ON_TIMER }, // I_SENSE_DAC // DAC1

  // 68/69 - CANRX0/CANTX0
  { PIOA, PIO_PA1A_CANRX0,   ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  ADC14,  NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // CANRX0
  { PIOA, PIO_PA0A_CANTX0,   ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,					ADC15,  NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // CANTX0

  // 70/71 - TWI0
  { PIOA, PIO_PA17A_TWD0,    ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // SDA0
  { PIOA, PIO_PA18A_TWCK0,   ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // SCL0

  // 72/73 SD Card Inserted/SD Card MCDA0
  { PIOC, PIO_PC30,          ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // SD_SW
  { PIOA, PIO_PA21,          ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // MCDA0

  // 74/75/76 - SPI0
  { PIOA, PIO_PA25A_SPI0_MISO,ID_PIOA,PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // SPI0_MISO
  { PIOA, PIO_PA26A_SPI0_MOSI,ID_PIOA,PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // SPI0_MOSI
  { PIOA, PIO_PA27A_SPI0_SPCK,ID_PIOA,PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // SPI0_CLK

  // 77 - SPI0_CS0
  { PIOA, PIO_PA28A_SPI0_NPCS0,ID_PIOA,PIO_PERIPH_A,PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // SPI0_CS

  // 78 - SPI0_CS3
  { PIOB, PIO_PB23B_SPI0_NPCS3,ID_PIOB,PIO_PERIPH_B,PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // SPI0_CS3

  // 79 .. 84 - "All pins" masks

  // 79 - TWI0 all pins
  { PIOA, PIO_PA17A_TWD0|PIO_PA18A_TWCK0, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_COMBO), NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER },
  // 80 - TWI1 all pins
  { PIOB, PIO_PB12A_TWD1|PIO_PB13A_TWCK1, ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_COMBO), NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER },
  // 81 - UART (Serial) all pins
  { PIOA, PIO_PA8A_URXD|PIO_PA9A_UTXD, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_COMBO),	 NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER },
  // 82 - USART0 (Serial1) all pins
  { PIOA, PIO_PA11A_TXD0|PIO_PA10A_RXD0, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_COMBO),	 NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER },
  // 83 - USART1 (Serial2) all pins
  { PIOA, PIO_PA13A_TXD1|PIO_PA12A_RXD1, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_COMBO),	 NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER },
  // 84 - USART3 (Serial3) all pins
  { PIOD, PIO_PD4B_TXD3|PIO_PD5B_RXD3, ID_PIOD, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_COMBO),	 NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER },

  // 85 - USB (UOTGID)
  { PIOB, PIO_PB11A_UOTGID|PIO_PB10A_UOTGVBOF, ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,			 NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // USB (UOTGID)

  // 86 - USART2RX
  { PIOB, PIO_PB21B_SPI0_NPCS2, ID_PIOB, PIO_PERIPH_B, PIO_DEFAULT, PIN_ATTR_DIGITAL,							 NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // USART2RX

  // 87 - SPIO_CS1
  { PIOA, PIO_PA29A_SPI0_NPCS1, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,							 NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // SPIO_CS1

  // 88/89 - CAN1RX/CAN1TX (same physical pin for 66/53)
  { PIOB, PIO_PB15A_CANRX1,     ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,							 NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // CAN1RX
  { PIOB, PIO_PB14A_CANTX1,     ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,							 NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // CANT1X

  // 90 .. 91 - "All CAN pins" masks
  // 90 - CAN0 all pins
  { PIOA, PIO_PA1A_CANRX0|PIO_PA0A_CANTX0, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_COMBO), NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER },
  // 91 - CAN1 all pins
  { PIOB, PIO_PB15A_CANRX1|PIO_PB14A_CANTX1, ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_COMBO), NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER },

	  // TODO check if 92 should be PIO_PERIPH_A or B ??
  // 92 - 93 M2_Button_1 & M2_Button_2 **** Added TD 3/6/2017 ***
  { PIOC, PIO_PC27,			ID_PIOC, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,								 NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // Button1
  { PIOB, PIO_PB6A_ERX1,	ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,								 NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // Button2
  // 94 DS3 (YELLOW) LED NonStandardDue
  { PIOA, PIO_PA5,         ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,								 NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // DS3
  // 95 GPIO5
  { PIOC, PIO_PC20,        ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL | PIN_ATTR_PWM),				 NO_ADC, NO_ADC, PWM_CH4,	 NOT_ON_TIMER },  // GPIO5

	// 96 AD15 CPU Chip Temperature
//  { PIOD, PIO_PD5,		ID_PIOD, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_ANALOG,				ADC15,	 ADC15, NOT_ON_PWM,  NOT_ON_TIMER }, // CPU_TEMP/A15 //ADC15 CPU Temperature
  { PIOD, PIO_PD5,			ID_PIOD, PIO_NOT_A_PIN, PIO_DEFAULT, PIN_ATTR_ANALOG,				ADC15,	 ADC15, NOT_ON_PWM,  NOT_ON_TIMER }, // CPU_TEMP/A15 // AD15 CPU Temperature
	// 97 J1850_PWM_VPW
  { PIOB, PIO_PB8A_EMDC,   ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC,  NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // J1850_PWM_VPW
  // 98 5Volt Power Supply Buck Boost
  { PIOC, PIO_PC10,        ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER },  //PS_BUCK/BUCK_DIS
  // 99 J1850_9141_ON
  { PIOB, PIO_PB5A_ERX0,   ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC,  NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PS_J1850_9141
  // 100 XBE_RST
  { PIOC, PIO_PC11,         ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // XBE_RST
  // 101 XBEE_PWM
  { PIOB, PIO_PB3A_ETX1,   ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC,  NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // XBEE_PWM
  // 102 LIN 9141 K SLP
  { PIOB, PIO_PB4A_ECRSDV,   ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC,  NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // ECRSDV
  // 103 LIN 9141 L SLP
  { PIOB, PIO_PB7A_ERXER,   ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC,  NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // ERXER
  // 104 SCW_M1
  { PIOB, PIO_PB0A_ETXCK,   ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC,  NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // ETXCK
  // 105 SCW_CLK
  { PIOB, PIO_PB22,          ID_PIOB, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // SWC CLK
  // 106 SWC RX0
  { PIOB, PIO_PB1A_ETXEN,   ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC,  NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // ETXEN
  // 107 SWC RX1
  { PIOB, PIO_PB2A_ETX0,   ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC,  NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // ETX0
	// END
  { NULL, 0, 0, PIO_NOT_A_PIN, PIO_DEFAULT, 0, NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }
} ;


uint8_t g_pinStatus[PINS_COUNT] = {0};

#ifdef __cplusplus
}
#endif

/*
 * UART objects
 */
RingBuffer rx_buffer1;
RingBuffer tx_buffer1;

UARTClass Serial(UART, UART_IRQn, ID_UART, &rx_buffer1, &tx_buffer1);
void serialEvent() __attribute__((weak));
void serialEvent() { }

// IT handlers
void UART_Handler(void)
{
  Serial.IrqHandler();
}

// ----------------------------------------------------------------------------
/*
 * USART objects
 */
RingBuffer rx_buffer2;
RingBuffer rx_buffer3;
RingBuffer rx_buffer4;
RingBuffer tx_buffer2;
RingBuffer tx_buffer3;
RingBuffer tx_buffer4;

USARTClass Serial1(USART0, USART0_IRQn, ID_USART0, &rx_buffer2, &tx_buffer2);
void serialEvent1() __attribute__((weak));
void serialEvent1() { }
USARTClass Serial2(USART1, USART1_IRQn, ID_USART1, &rx_buffer3, &tx_buffer3);
void serialEvent2() __attribute__((weak));
void serialEvent2() { }
USARTClass Serial3(USART3, USART3_IRQn, ID_USART3, &rx_buffer4, &tx_buffer4);
void serialEvent3() __attribute__((weak));
void serialEvent3() { }

// IT handlers
void USART0_Handler(void)
{
  Serial1.IrqHandler();
}

void USART1_Handler(void)
{
  Serial2.IrqHandler();
}

void USART3_Handler(void)
{
  Serial3.IrqHandler();
}

// ----------------------------------------------------------------------------

void serialEventRun(void)
{
  if (Serial.available()) serialEvent();
  if (Serial1.available()) serialEvent1();
  if (Serial2.available()) serialEvent2();
  if (Serial3.available()) serialEvent3();
}

// ----------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif

void __libc_init_array(void);

void init( void )
{
  SystemInit();

  // Set Systick to 1ms interval, common to all SAM3 variants
  if (SysTick_Config(SystemCoreClock / 1000))
  {
    // Capture error
    while (true);
  }

  // Initialize C library
  __libc_init_array();

  // Disable pull-up on every pin
  for (unsigned i = 0; i < PINS_COUNT; i++)
	  digitalWrite(i, LOW);

  // Enable parallel access on PIO output data registers
  PIOA->PIO_OWER = 0xFFFFFFFF;
  PIOB->PIO_OWER = 0xFFFFFFFF;
  PIOC->PIO_OWER = 0xFFFFFFFF;
  PIOD->PIO_OWER = 0xFFFFFFFF;

  // Initialize Serial port U(S)ART pins
  PIO_Configure(
    g_APinDescription[PINS_UART].pPort,
    g_APinDescription[PINS_UART].ulPinType,
    g_APinDescription[PINS_UART].ulPin,
    g_APinDescription[PINS_UART].ulPinConfiguration);
  digitalWrite(0, HIGH); // Enable pullup for RX0
  PIO_Configure(
    g_APinDescription[PINS_USART0].pPort,
    g_APinDescription[PINS_USART0].ulPinType,
    g_APinDescription[PINS_USART0].ulPin,
    g_APinDescription[PINS_USART0].ulPinConfiguration);
  PIO_Configure(
    g_APinDescription[PINS_USART1].pPort,
    g_APinDescription[PINS_USART1].ulPinType,
    g_APinDescription[PINS_USART1].ulPin,
    g_APinDescription[PINS_USART1].ulPinConfiguration);
  PIO_Configure(
    g_APinDescription[PINS_USART3].pPort,
    g_APinDescription[PINS_USART3].ulPinType,
    g_APinDescription[PINS_USART3].ulPin,
    g_APinDescription[PINS_USART3].ulPinConfiguration);

  // Initialize USB pins
  PIO_Configure(
    g_APinDescription[PINS_USB].pPort,
    g_APinDescription[PINS_USB].ulPinType,
    g_APinDescription[PINS_USB].ulPin,
    g_APinDescription[PINS_USB].ulPinConfiguration);

  // Initialize CAN pins
  PIO_Configure(
    g_APinDescription[PINS_CAN0].pPort,
    g_APinDescription[PINS_CAN0].ulPinType,
    g_APinDescription[PINS_CAN0].ulPin,
    g_APinDescription[PINS_CAN0].ulPinConfiguration);
  PIO_Configure(
    g_APinDescription[PINS_CAN1].pPort,
    g_APinDescription[PINS_CAN1].ulPinType,
    g_APinDescription[PINS_CAN1].ulPin,
    g_APinDescription[PINS_CAN1].ulPinConfiguration);

  // Initialize Analog Controller
  pmc_enable_periph_clk(ID_ADC);
  adc_init(ADC, SystemCoreClock, ADC_FREQ_MAX, ADC_STARTUP_FAST);
  adc_configure_timing(ADC, 0, ADC_SETTLING_TIME_3, 1);
  adc_configure_trigger(ADC, ADC_TRIG_SW, 0); // Disable hardware trigger.
  adc_disable_interrupt(ADC, 0xFFFFFFFF); // Disable all ADC interrupts.
  adc_disable_all_channel(ADC);

  // Initialize analogOutput module
  analogOutputInit();
}

#ifdef __cplusplus
}
#endif

