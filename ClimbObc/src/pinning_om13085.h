/*
===============================================================================
 Name        : pinning_om13085.h
 Author      : Robert
 Created on	 : 05.09.2021
===============================================================================
*/

/* Dev board in connection with external CLIMB_OBC_EM2TA_V1.0 [on J2-1 ... J2-22] */

#ifndef PINNING_OM13085_H_
#define PINNING_OM13085_H_

#include <chip.h>

enum pinidx {
	PINIDX_UART_TX_C_P_SW = 0,
	PINIDX_UART_RX_C_P_SW,
	PINIDX_UART_TX_D_P_SW,
	PINIDX_UART_RX_D_P_SW,
	PINIDX_UART_TX_B_P_SW,
	PINIDX_UART_RX_B_P_SW,
	PINIDX_UART_TX_A_P_SW,
	PINIDX_UART_RX_A_P_SW,

	PINIDX_I2C0_SDA,
	PINIDX_I2C0_SCL,
	PINIDX_I2C1_SDA,
	PINIDX_I2C1_SCL,

	PINIDX_SSP0_SCK,
	PINIDX_SSP0_MISO,
	PINIDX_SSP0_MOSI,

	PINIDX_SSP1_SCK,
	PINIDX_SSP1_MISO,
	PINIDX_SSP1_MOSI,

	PINIDX_SSP0_MRAM_CS1,
	PINIDX_SSP0_MRAM_CS2,
	PINIDX_SSP0_MRAM_CS3,
	PINIDX_SSP1_MRAM_CS1,
	PINIDX_SSP1_MRAM_CS2,
	PINIDX_SSP1_MRAM_CS3,

	PINIDX_SSP0_CS_SD,
	PINIDX_SSP1_CS_SD,

	PINIDX_LED,
	PINIDX_LED2,
	PINIDX_LED3,

	PINIDX_WATCHDOG_FEED,

	// HW not available -> TODO make code conditional or simulate on other pins !?
	PINIDX_SP_VCC_FAULT,			// input (detecting SPx short circuit)
	PINIDX_RBF,						// input
	PINIDX_SP1_VCC_EN,				// output
	PINIDX_SP2_VCC_EN,				// output
	PINIDX_SP3_VCC_EN,				// output
	PINIDX_SP4_VCC_EN,				// output
	PINIDX_SD_VCC_EN				// output
};

#define P0_PIN_UART3_TX				0
#define P0_PIN_UART3_RX				1
#define P0_PIN_UART0_TX				2	// !!! 'double' connected as TX_RX for RS485 on Uart3 -> Uart0 not usable together with this !!!
#define P0_PIN_UART0_RX				3
#define P0_PIN_NC1_SCOPE1			4
#define P0_PIN_NC2					5
#define P0_PIN_SSP1_SD_CS			6
#define P0_PIN_SSP1_SCK				7
#define P0_PIN_SSP1_MISO			8
#define P0_PIN_SSP1_MOSI			9
#define P0_PIN_UART2_TX			    10
#define P0_PIN_UART2_RX			    11
#define P0_PIN_SSP0_SCK				15
#define P0_PIN_SSP0_SD_CS			16
#define P0_PIN_SSP0_MISO			17
#define P0_PIN_SSP0_MOSI			18
#define P0_PIN_I2C_SDA1				19
#define P0_PIN_I2C_SCL1				20
#define P0_PIN_NC3					21
#define P0_PIN_LED_RED				22
#define P0_PIN_SSP1_MRAM_CS1		23
#define P0_PIN_SSP1_MRAM_CS2		24
#define P0_PIN_SSP1_MRAM_CS3		25
#define P0_PIN_SSP0_MRAM_CS1		26
#define P0_PIN_I2C_SDA0				27
#define P0_PIN_I2C_SCL0				28
#define P0_PIN_USB_DP				29	// not used . initialized
#define P0_PIN_USB_DM				30  // not used . initialized

#define P1_PIN_ENET_1 				0	// not used . initialized
#define P1_PIN_ENET_2 				1	// not used . initialized
#define P1_PIN_ENET_3 				4	// not used . initialized
#define P1_PIN_ENET_4 				8	// not used . initialized
#define P1_PIN_ENET_5				9	// not used . initialized
#define P1_PIN_ENET_6 				10	// not used . initialized
#define P1_PIN_ENET_7    			14	// not used . initialized
#define P1_PIN_ENET_8     			15	// not used . initialized
#define P1_PIN_ENET_9     			16	// not used . initialized
#define P1_PIN_ENET_10    			17	// not used . initialized
#define P1_PIN_NC4 				    18
#define P1_PIN_NC5       			19
#define P1_PIN_NC6       			20
#define P1_PIN_NC7       			21
#define P1_PIN_NC8       			22
#define P1_PIN_NC9       			23
#define P1_PIN_NC10       			24
#define P1_PIN_NC11       			25
#define P1_PIN_NC12       			26
#define P1_PIN_NC13       			27
#define P1_PIN_NC14       			28
#define P1_PIN_NC15       			29
#define P1_PIN_SSP0_MRAM_CS2      	30
#define P1_PIN_SSP0_MRAM_CS3     	31

#define P2_PIN_UART1_TX				0
#define P2_PIN_UART1_RX				1
#define P2_PIN_NC16					2
#define P2_PIN_NC17					3
#define P2_PIN_NC18					4
#define P2_PIN_NC19					5
#define P2_PIN_NC20					6
#define P2_PIN_NC21					7
#define P2_PIN_NC22					8
#define P2_PIN_USB_CON				9	// not used . initialized
#define P2_PIN_ISP_EN			 	10  // not used . initialized
#define P2_PIN_NC23	    			11
#define P2_PIN_NC24					12
#define P2_PIN_NC25					13

#define P3_PIN_LED_GREEN			25
#define P3_PIN_LED_BLUE				26

#define P4_PIN_NC26					28
#define P4_PIN_NC27					29

static const PINMUX_GRP_T2 pinmuxing2[] = {

		// Configure all 4 UARTS to be used
		[PINIDX_UART_TX_D_P_SW] = { 0, P0_PIN_UART0_TX, IOCON_MODE_INACT | IOCON_FUNC1 }, 	/* LPC_UART0 "Uart D" */		// J2-21
		[PINIDX_UART_RX_D_P_SW] = { 0, P0_PIN_UART0_RX, IOCON_MODE_INACT | IOCON_FUNC1 }, 	/* LPC_UART0 "Uart D" */		// J2-22

		[PINIDX_UART_TX_C_P_SW] = { 2, P2_PIN_UART1_TX, IOCON_MODE_INACT | IOCON_FUNC2 }, 	/* LPC_UART1 "Uart C" */		// J2-42
		[PINIDX_UART_RX_C_P_SW] = { 2, P2_PIN_UART1_RX, IOCON_MODE_INACT | IOCON_FUNC2 }, 	/* LPC_UART1 "Uart C" */		// J2-43

		[PINIDX_UART_TX_B_P_SW] = { 0, P0_PIN_UART2_TX, IOCON_MODE_INACT | IOCON_FUNC1 }, 	/* LPC_UART2 "Uart B" */		// J2-40	We use this as command line interface for cli module
		[PINIDX_UART_RX_B_P_SW] = { 0, P0_PIN_UART2_RX, IOCON_MODE_INACT | IOCON_FUNC1 }, 	/* LPC_UART2 "Uart B" */		// J2-41

        [PINIDX_UART_TX_A_P_SW] = { 0, P0_PIN_UART3_TX, IOCON_MODE_INACT | IOCON_FUNC2 }, 	/* LPC_UART3 "Uart A" */		// J2-9
		[PINIDX_UART_RX_A_P_SW] = { 0, P0_PIN_UART3_RX, IOCON_MODE_INACT | IOCON_FUNC2 }, 	/* LPC_UART3 "Uart A" */		// J2-10

		[PINIDX_SSP1_MRAM_CS1] = { 0, P0_PIN_SSP1_MRAM_CS1, IOCON_MODE_INACT | IOCON_FUNC0, IOCON_DIR_OUTPUT, IOCON_OD_DISABLED, IOCON_VAL_HIGH  },	/* MRAM CS  SSP1-CS1 	*/ // J2-15
		[PINIDX_SSP1_MRAM_CS2] = { 0, P0_PIN_SSP1_MRAM_CS2, IOCON_MODE_INACT | IOCON_FUNC0, IOCON_DIR_OUTPUT, IOCON_OD_DISABLED, IOCON_VAL_HIGH  }, /* MRAM CS  SSP1-CS2	*/ // J2-16
		[PINIDX_SSP1_MRAM_CS3] = { 0, P0_PIN_SSP1_MRAM_CS3, IOCON_MODE_INACT | IOCON_FUNC0, IOCON_DIR_OUTPUT, IOCON_OD_DISABLED, IOCON_VAL_HIGH }, 	/* MRAM CS  SSP1-CS3	*/ // J2-17
		[PINIDX_SSP0_MRAM_CS1] = { 0, P0_PIN_SSP0_MRAM_CS1, IOCON_MODE_INACT | IOCON_FUNC0, IOCON_DIR_OUTPUT, IOCON_OD_DISABLED, IOCON_VAL_HIGH }, 	/* MRAM CS  SSP0-CS1	*/ // J2-18
		[PINIDX_SSP0_MRAM_CS2] = { 1, P1_PIN_SSP0_MRAM_CS2, IOCON_MODE_INACT | IOCON_FUNC0, IOCON_DIR_OUTPUT, IOCON_OD_DISABLED, IOCON_VAL_HIGH }, 	/* MRAM CS  SSP0-CS2	*/ // J2-19
		[PINIDX_SSP0_MRAM_CS3] = { 1, P1_PIN_SSP0_MRAM_CS3, IOCON_MODE_INACT | IOCON_FUNC0, IOCON_DIR_OUTPUT, IOCON_OD_DISABLED, IOCON_VAL_HIGH }, 	/* MRAM CS  SSP0-CS3	*/ // J2-20


		// 2x I2C
		[PINIDX_I2C0_SDA] = { 0, P0_PIN_I2C_SDA0, IOCON_MODE_INACT | IOCON_FUNC1 }, 	/* I2C0 SDA 	    */		// J2-25
		[PINIDX_I2C0_SCL] = { 0, P0_PIN_I2C_SCL0, IOCON_MODE_INACT | IOCON_FUNC1 }, 	/* I2C0 SCL 	    */  	// J2-26
		//[PINIDX_I2C0_SDA] = { 0, P0_PIN_I2C_SDA0, IOCON_MODE_INACT | IOCON_FUNC0, IOCON_DIR_OUTPUT, IOCON_OD_ENABLED, IOCON_VAL_HIGH   }, 	// J2-25
		//[PINIDX_I2C0_SCL] = { 0, P0_PIN_I2C_SCL0, IOCON_MODE_INACT | IOCON_FUNC0, IOCON_DIR_OUTPUT, IOCON_OD_ENABLED, IOCON_VAL_LOW   }, 	// J2-26


		[PINIDX_I2C1_SDA] = { 0, P0_PIN_I2C_SDA1, IOCON_MODE_INACT | IOCON_FUNC3 }, 	/* I2C1 SDA			*/	 	// PAD8	this connects to boards e2prom with address 0x50
		[PINIDX_I2C1_SCL] = { 0, P0_PIN_I2C_SCL1, IOCON_MODE_INACT | IOCON_FUNC3 }, 	/* I2C1 SCL      	*/  	// PAD2


		// SSP0 Version on Pin 60-63 P0[15-18]
		[PINIDX_SSP0_SCK] = { 0, 15, IOCON_MODE_INACT | IOCON_FUNC2 }, /* SCK   		 */ // J2-13	we use this to test SSP SD card slot.
		//	//{ 0, P0_PIN_SSP0_SD_CS, IOCON_MODE_INACT | IOCON_FUNC2 }, /* SSL - CS  	 */ // J2-13    the 'recommended' SSL/CS for SSP0
//			{ 0, P0_PIN_SSP0_SD_CS, IOCON_MODE_INACT | IOCON_FUNC0 }, /* SSL - CS       */ // J2-13    the 'recommended' SSL/CS for SSP0 used as GPIO - CS has to be controlled by SW
		[PINIDX_SSP0_MISO] = { 0, 17, IOCON_MODE_INACT | IOCON_FUNC2 }, /* MISO	 		 */	// J2-12
		[PINIDX_SSP0_MOSI] = { 0, 18, IOCON_MODE_INACT | IOCON_FUNC2 }, /* MOSI  		 */	// J2-11

		// SPI Version on Pin 60-63 P0[15-18]
//		{ 0, P0_PIN_SSP0_SCK, 	IOCON_MODE_INACT | IOCON_FUNC3 }, 	/* SCK            	*/ // J2-13    we use this to test SPI SD Card
		[PINIDX_SSP0_CS_SD] = { 0, P0_PIN_SSP0_SD_CS, IOCON_MODE_INACT | IOCON_FUNC0, IOCON_DIR_OUTPUT, IOCON_OD_DISABLED, IOCON_VAL_HIGH  }, 	/* SSL - CS  GPIO!	*/ // J2-13    SPI only uses the SSL pin when in slave mode. To use as CS for Master this is normal IO
//		{ 0, P0_PIN_SSP0_MISO,  IOCON_MODE_INACT | IOCON_FUNC3 }, 	/* MISO  	        */ // J2-12
//		{ 0, P0_PIN_SSP0_MOSI,  IOCON_MODE_INACT | IOCON_FUNC3 }, 	/* MOSI           	*/ // J2-11

		// SSP1
		[PINIDX_SSP1_SCK] = { 0, P0_PIN_SSP1_SCK,   IOCON_MODE_INACT | IOCON_FUNC2 }, 	/* SCK   		 	*/  // J2-7	SSP1
		[PINIDX_SSP1_CS_SD] = { 0, P0_PIN_SSP1_SD_CS, IOCON_MODE_INACT | IOCON_FUNC0,  IOCON_DIR_OUTPUT, IOCON_OD_DISABLED, IOCON_VAL_HIGH  }, 	/* SSL  	        */  // J2-8 SSP1  SSL/CS GPIO controlled by SW.
		[PINIDX_SSP1_MISO] = { 0, P0_PIN_SSP1_MISO,  IOCON_MODE_INACT | IOCON_FUNC2 }, 	/* MISO		 	 	*/	// J2-6	SSP1
		[PINIDX_SSP1_MOSI] = { 0, P0_PIN_SSP1_MOSI,  IOCON_MODE_INACT | IOCON_FUNC2 }, 	/* MOSI  		 	*/	// J2-5	SSP1

		// GPIOs
		[PINIDX_LED] = { 3, P3_PIN_LED_BLUE,      IOCON_MODE_INACT | IOCON_FUNC0,  IOCON_DIR_OUTPUT, IOCON_OD_DISABLED, IOCON_VAL_HIGH 	}, 	/* Led blue      	*/
		[PINIDX_LED2] = { 0, P0_PIN_LED_RED,       IOCON_MODE_INACT | IOCON_FUNC0, IOCON_DIR_OUTPUT, IOCON_OD_DISABLED, IOCON_VAL_HIGH },	/* Led red       	*/
		[PINIDX_LED3] = { 3, P3_PIN_LED_GREEN,     IOCON_MODE_INACT | IOCON_FUNC0,  IOCON_DIR_OUTPUT, IOCON_OD_DISABLED, IOCON_VAL_LOW 		}, 	/* Led green     	*/

		[PINIDX_WATCHDOG_FEED] =
		// Use NC GPIO on J2-38 for debugging with scope.....
		{ 0, P0_PIN_NC1_SCOPE1,    IOCON_MODE_INACT | IOCON_FUNC0,  IOCON_DIR_OUTPUT, IOCON_OD_DISABLED, IOCON_VAL_HIGH  } 	 /* P0[4]		    	 */ // J2-38

};

#endif /* PINNING_OM13085_H_ */
