/*
===============================================================================
 Name        : pinning_climbobc.h
 Author      : Robert
 Created on	 : 05.09.2021
===============================================================================
*/

#ifndef PINNING_CLIMBOBC_H_
#define PINNING_CLIMBOBC_H_

#include <chip.h>

#define P0_PIN_UART_TX_A_P_SW		0
#define P0_PIN_UART_RX_A_P_SW		1
#define P0_PIN_UART_TX_C_P_SW		2
#define P0_PIN_UART_RX_C_P_SW		3
#define P0_PIN_SSP1_MRAM_CS2		4
#define P0_PIN_N_C					5
#define P0_PIN_S_STACIE_IO1			6
#define P0_PIN_SSP1_SCK				7
#define P0_PIN_SSP1_MISO			8
#define P0_PIN_SSP1_MOSI			9
#define P0_PIN_I2C2_SDA			    10
#define P0_PIN_I2C2_SCL			    11
#define P0_PIN_SPI_SCK				15
#define P0_PIN_SPI_CS_SD			16
#define P0_PIN_SPI_MISO				17
#define P0_PIN_SPI_MOSI				18
#define P0_PIN_I2C1_SDA				19
#define P0_PIN_I2C1_SCL				20
#define P0_PIN_RBF					21
#define P0_PIN_SSP0_MRAM_CS1		22
#define P0_PIN_SUPPLY_CURRENT		23
#define P0_PIN_SP_SUPPLY_CURRENT	24
#define P0_PIN_TEMPERATURE			25
#define P0_PIN_SUPPLY				26
#define P0_PIN_I2C0_SDA				27
#define P0_PIN_I2C0_SCL				28
#define P0_PIN_VCC_DISABLE			29
#define P0_PIN_I2CA_EN				30


#define P1_PIN_S_STACIE_IO2     	0
#define P1_PIN_I2C_D_EN         	1
#define P1_PIN_I2C_C_EN         	4
#define P1_PIN_N_C1             	8
#define P1_PIN_CLR_WDT_FLIPFLOP 	9
#define P1_PIN_SSP1_MRAM_CS3    	10
#define P1_PIN_GPIO4_CP             14
#define P1_PIN_SP3_VCC_EN           15
#define P1_PIN_SPC_GPIO3_P          16
#define P1_PIN_STACIE_C_IO1_P       17
#define P1_PIN_WATCHDOG_FEED    	18
#define P1_PIN_THRUSTER_CS_P    	19
#define P1_PIN_SSP0_SCK         	20
#define P1_PIN_STACIE_A_IO1_P   	21
#define P1_PIN_SP4_VCC_EN       	22
#define P1_PIN_SSP0_MISO        	23
#define P1_PIN_SSP0_MOSI        	24
#define P1_PIN_STACIE_A_IO2_P   	25
#define P1_PIN_THRUSTER_LATCHUP_P	26
#define P1_PIN_CLK_32765        	27
#define P1_PIN_SP1_VCC_EN       	28
#define P1_PIN_GPIO3_A_P        	29
#define P1_PIN_N_C2             	30
#define P1_PIN_GPIO4_D_P        	31

#define P2_PIN_UART_TX_D_P_SW		0
#define P2_PIN_UART_RX_D_P_SW		1
#define P2_PIN_SSP1_MRAM_CS1		2
#define P2_PIN_I2C_B_EN				3
#define P2_PIN_SP_VCC_FAULT			4
#define P2_PIN_RS485_TX_RX			5
#define P2_PIN_LED					6
#define P2_PIN_SP2_VCC_EN			7
#define P2_PIN_UART_TX_P_SW			8
#define P2_PIN_UART_RX_P_SW			9
#define P2_PIN_ISP				 	10
#define P2_PIN_SSP0_MRAM_CS2	    11
#define P2_PIN_SSP0_MRAM_CS3		12
#define P2_PIN_N_C					13

#define P3_PIN_SUPPLY_RAIL			25
#define P3_PIN_BL_SEL1				26

#define P4_PIN_SD_VCC_EN			28
#define P4_PIN_EXT_WDT_TRIGGERED	29

/// .....
typedef struct {
	uint8_t  port;
	uint8_t pinNr;
	bool 	output;
	bool    initVal;
} GPIO_INIT_T;

#define GPIO_DIR_OUTPUT true
#define GPIO_DIR_INPUT  false

#define GPIO_VAL_HIGH   true
#define GPIO_VAL_LOW    false

static const PINMUX_GRP_T pinmuxing[] = {
	// UARTS
	#define UART_A	LPC_UART3
	{0, P0_PIN_UART_TX_A_P_SW,  IOCON_MODE_INACT | IOCON_FUNC2},	/* TXD3 - UART SP-A	X+  */
	{0, P0_PIN_UART_RX_A_P_SW,  IOCON_MODE_INACT | IOCON_FUNC2},	/* RXD3 - UART SP-A	X+  */
	#define UART_C	LPC_UART0
	{0, P0_PIN_UART_TX_C_P_SW,  IOCON_MODE_INACT | IOCON_FUNC1},	/* TXD0 - UART SP-C	X-  */
	{0, P0_PIN_UART_RX_C_P_SW,  IOCON_MODE_INACT | IOCON_FUNC1},	/* RXD0 - UART SP-C	X-  */

	{0, P0_PIN_SSP1_MRAM_CS2,   IOCON_MODE_INACT | IOCON_FUNC0},	/* GPIO - SSP1-MRAM-CS2 */
	{0, P0_PIN_N_C,             IOCON_MODE_INACT | IOCON_FUNC0}		/* GPIO - n.c.          */

	// .....

};

static const GPIO_INIT_T gpioinit[] = {
	{0, P0_PIN_SSP1_MRAM_CS2, GPIO_DIR_OUTPUT, GPIO_VAL_HIGH  },
	{0, P0_PIN_S_STACIE_IO1,  GPIO_DIR_INPUT,  GPIO_VAL_HIGH  }
	// .....
};


#endif /* PINNING_CLIMBOBC_H_ */
