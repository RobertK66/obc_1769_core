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

// What to do with NC!? Shall we init them ?
////#define P0_PIN_N_C					5
////#define P1_PIN_N_C1             	8
////#define P1_PIN_N_C2             	30
////#define P2_PIN_ISP				 	10
////#define P2_PIN_N_C					13

// Logical PinNames used as IDX in pinmux array.
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
	PINIDX_I2C2_SDA,
	PINIDX_I2C2_SCL,
	PINIDX_SPI_SCK,
	PINIDX_SPI_MISO,
	PINIDX_SPI_MOSI,
	PINIDX_SSP0_SCK,
	PINIDX_SSP0_MISO,
	PINIDX_SSP0_MOSI,
	PINIDX_SSP1_SCK,
	PINIDX_SSP1_MISO,
	PINIDX_SSP1_MOSI,
	PINIDX_SUPPLY_CURRENT,
	PINIDX_SP_SUPPLY_CURRENT,
	PINIDX_TEMPERATURE,
	PINIDX_SUPPLY,
	PINIDX_SSP1_MRAM_CS2,
	PINIDX_SPI_CS_SD,
	PINIDX_SSP0_MRAM_CS1,
	PINIDX_VCC_DISABLE,
	PINIDX_I2C_A_EN,
	PINIDX_I2C_D_EN,
	PINIDX_I2C_C_EN,
	PINIDX_CLR_WDT_FLPFLP,
	PINIDX_SSP1_MRAM_CS3,
	PINIDX_SP3_VCC_EN,
	PINIDX_WATCHDOG_FEED,
	PINIDX_THRUSTER_CS_P,
	PINIDX_SP4_VCC_EN,
	PINIDX_SP1_VCC_EN,
	PINIDX_SSP1_MRAM_CS1,
	PINIDX_I2C_B_EN,
	PINIDX_RS485_TX_RX,
	PINIDX_LED,
	PINIDX_SP2_VCC_EN,
	PINIDX_SSP0_MRAM_CS2,
	PINIDX_SSP0_MRAM_CS3,
	PINIDX_SUPPLY_RAIL,
	PINIDX_SD_VCC_EN,
    PINIDX_RBF,
    PINIDX_THRUSTER_LATCHUP_P,
    PINIDX_SP_VCC_FAULT,
    PINIDX_BL_SEL1,
    PINIDX_EXT_WDT_TRIGGERED,
    PINIDX_S_STACIE_IO1,
    PINIDX_S_STACIE_IO2,
    PINIDX_GPIO4_CP,
    PINIDX_SPC_GPIO3_P,
    PINIDX_STACIE_C_IO1_P,
    PINIDX_STACIE_A_IO1_P,
    PINIDX_STACIE_A_IO2_P,
    PINIDX_GPIO3_A_P,
    PINIDX_GPIO4_D_P,

	PINIDX_MAXIDX				// Size Marker for Checks of pinidx values.
};

// For each PINIDx there must be an entry in the following structure.
// Order of lines is not relevant, but all IDX-enums must have exactly one entry.
static const PINMUX_GRP_T2 pinmuxing2[] = {

	// 	UART_C	LPC_UART0	(X-)
	[PINIDX_UART_TX_C_P_SW] = { 0, 2,  IOCON_MODE_INACT | IOCON_FUNC1},
	[PINIDX_UART_RX_C_P_SW] = { 0, 3,  IOCON_MODE_INACT | IOCON_FUNC1},

	//	UART_D	LPC_UART1  	(Y+)
	[PINIDX_UART_TX_D_P_SW] = { 2, 0,  IOCON_MODE_INACT | IOCON_FUNC2},
	[PINIDX_UART_RX_D_P_SW] = { 2, 1,  IOCON_MODE_INACT | IOCON_FUNC2},

	//	UART_B	LPC_UART2  	(Y-)
	[PINIDX_UART_TX_B_P_SW] = { 2, 8,  IOCON_MODE_INACT | IOCON_FUNC2},
	[PINIDX_UART_RX_B_P_SW] = { 2, 9,  IOCON_MODE_INACT | IOCON_FUNC2},

	//	UART_A	LPC_UART3  	(X+)
	[PINIDX_UART_TX_A_P_SW] = { 0, 0,  IOCON_MODE_INACT | IOCON_FUNC2},
	[PINIDX_UART_RX_A_P_SW] = { 0, 1,  IOCON_MODE_INACT | IOCON_FUNC2},

	// 	3x I2C
//	[PINIDX_I2C0_SDA] = { 0, 27, IOCON_MODE_INACT | IOCON_FUNC1},	// I2C C/D
//	[PINIDX_I2C0_SCL] = { 0, 28, IOCON_MODE_INACT | IOCON_FUNC1},
	[PINIDX_I2C0_SDA] = { 0, 27, IOCON_MODE_INACT | IOCON_FUNC0, IOCON_DIR_OUTPUT, IOCON_OD_DISABLED, IOCON_VAL_HIGH },	// Use as GPIO Output
	[PINIDX_I2C0_SCL] = { 0, 28, IOCON_MODE_INACT | IOCON_FUNC0, IOCON_DIR_OUTPUT, IOCON_OD_DISABLED, IOCON_VAL_HIGH },

	[PINIDX_I2C1_SDA] = { 0, 19, IOCON_MODE_INACT | IOCON_FUNC3},	// I2C Onboard  (FUNC2 is reserved! -> FUNC3 is I2C !!!)
	[PINIDX_I2C1_SCL] = { 0, 20, IOCON_MODE_INACT | IOCON_FUNC3},	//              (FUNC2 is reserved! -> FUNC3 is I2C !!!)
	[PINIDX_I2C2_SDA] = { 0, 10, IOCON_MODE_INACT | IOCON_FUNC2},	// I2C A/B
	[PINIDX_I2C2_SCL] = { 0, 11, IOCON_MODE_INACT | IOCON_FUNC2},

	// 	SPI on P0[15-18]
	[PINIDX_SPI_SCK] = { 0, 15,	IOCON_MODE_INACT | IOCON_FUNC3 },
	[PINIDX_SPI_MISO]= { 0, 17,	IOCON_MODE_INACT | IOCON_FUNC3 },
	[PINIDX_SPI_MOSI]= { 0, 18,	IOCON_MODE_INACT | IOCON_FUNC3 },

	// 	SSP0 on P1[20,23,24]
	[PINIDX_SSP0_SCK] = { 1, 20, IOCON_MODE_INACT | IOCON_FUNC3 },
	[PINIDX_SSP0_MISO]= { 1, 23, IOCON_MODE_INACT | IOCON_FUNC3 },
	[PINIDX_SSP0_MOSI]= { 1, 24, IOCON_MODE_INACT | IOCON_FUNC3 },

	// 	SSP1
	[PINIDX_SSP1_SCK] = { 0, 7,	IOCON_MODE_INACT | IOCON_FUNC2 },
	[PINIDX_SSP1_MISO]= { 0, 8,	IOCON_MODE_INACT | IOCON_FUNC2 },
	[PINIDX_SSP1_MOSI]= { 0, 9, IOCON_MODE_INACT | IOCON_FUNC2 },

	// 	Analog inputs
	[PINIDX_SUPPLY_CURRENT]   = { 0, 23, IOCON_MODE_INACT | IOCON_FUNC1 },
	[PINIDX_SP_SUPPLY_CURRENT]= { 0, 24, IOCON_MODE_INACT | IOCON_FUNC1 },
	[PINIDX_TEMPERATURE]      = { 0, 25, IOCON_MODE_INACT | IOCON_FUNC1 },
	[PINIDX_SUPPLY]           = { 0, 26, IOCON_MODE_INACT | IOCON_FUNC1 },

	// 	GPIOs also define the direction, OpenDrain and initial value bits of the PINMUX_GRP_T2 structure.
	// 	Outputs
	[PINIDX_SSP1_MRAM_CS2] 	= { 0, 4,  IOCON_MODE_INACT | IOCON_FUNC0, IOCON_DIR_OUTPUT, IOCON_OD_DISABLED, IOCON_VAL_HIGH },
	[PINIDX_SPI_CS_SD]     	= { 0, 16, IOCON_MODE_INACT | IOCON_FUNC0, IOCON_DIR_OUTPUT, IOCON_OD_ENABLED,  IOCON_VAL_HIGH },
	[PINIDX_SSP0_MRAM_CS1] 	= { 0, 22, IOCON_MODE_INACT | IOCON_FUNC0, IOCON_DIR_OUTPUT, IOCON_OD_DISABLED, IOCON_VAL_HIGH },
	[PINIDX_VCC_DISABLE]   	= { 0, 29, IOCON_MODE_INACT | IOCON_FUNC0, IOCON_DIR_OUTPUT, IOCON_OD_DISABLED, IOCON_VAL_LOW  },
	[PINIDX_I2C_A_EN]	  	= { 0, 30, IOCON_MODE_INACT | IOCON_FUNC0, IOCON_DIR_OUTPUT, IOCON_OD_DISABLED, IOCON_VAL_HIGH },
	[PINIDX_I2C_D_EN]	  	= { 1, 1,  IOCON_MODE_INACT | IOCON_FUNC0, IOCON_DIR_OUTPUT, IOCON_OD_DISABLED, IOCON_VAL_HIGH },
	[PINIDX_I2C_C_EN]	  	= { 1, 4,  IOCON_MODE_INACT | IOCON_FUNC0, IOCON_DIR_OUTPUT, IOCON_OD_DISABLED, IOCON_VAL_HIGH },
	// to avoid unintended Clear while power on the PINIDX_CLR_WDT_FLPFLP output is initialized as input here!
	[PINIDX_CLR_WDT_FLPFLP]	= { 1, 9,  IOCON_MODE_INACT | IOCON_FUNC0, IOCON_DIR_INPUT,  IOCON_OD_ENABLED,  IOCON_VAL_HIGH },
	[PINIDX_SSP1_MRAM_CS3] 	= { 1, 10, IOCON_MODE_INACT | IOCON_FUNC0, IOCON_DIR_OUTPUT, IOCON_OD_DISABLED, IOCON_VAL_HIGH },
	[PINIDX_SP3_VCC_EN]    	= { 1, 15, IOCON_MODE_INACT | IOCON_FUNC0, IOCON_DIR_OUTPUT, IOCON_OD_DISABLED, IOCON_VAL_HIGH },
	[PINIDX_WATCHDOG_FEED] 	= { 1, 18, IOCON_MODE_INACT | IOCON_FUNC0, IOCON_DIR_OUTPUT, IOCON_OD_DISABLED, IOCON_VAL_HIGH },
	[PINIDX_THRUSTER_CS_P] 	= { 1, 19, IOCON_MODE_INACT | IOCON_FUNC0, IOCON_DIR_OUTPUT, IOCON_OD_DISABLED, IOCON_VAL_HIGH },
	[PINIDX_SP4_VCC_EN]    	= { 1, 22, IOCON_MODE_INACT | IOCON_FUNC0, IOCON_DIR_OUTPUT, IOCON_OD_DISABLED, IOCON_VAL_HIGH },
	[PINIDX_SP1_VCC_EN]    	= { 1, 28, IOCON_MODE_INACT | IOCON_FUNC0, IOCON_DIR_OUTPUT, IOCON_OD_DISABLED, IOCON_VAL_HIGH },
	[PINIDX_SSP1_MRAM_CS1] 	= { 2, 2,  IOCON_MODE_INACT | IOCON_FUNC0, IOCON_DIR_OUTPUT, IOCON_OD_DISABLED, IOCON_VAL_HIGH },
	[PINIDX_I2C_B_EN]      	= { 2, 3,  IOCON_MODE_INACT | IOCON_FUNC0, IOCON_DIR_OUTPUT, IOCON_OD_DISABLED, IOCON_VAL_HIGH }, // ENABLE I2C on  Y- side
	[PINIDX_RS485_TX_RX]   	= { 2, 5,  IOCON_MODE_INACT | IOCON_FUNC2, IOCON_DIR_OUTPUT, IOCON_OD_DISABLED, IOCON_VAL_LOW },//HIGH - TRANSMIT LOW - RECEIVE

	[PINIDX_LED]           = { 2, 6,	IOCON_MODE_INACT | IOCON_FUNC0, IOCON_DIR_OUTPUT, IOCON_OD_DISABLED, IOCON_VAL_LOW },
	[PINIDX_SP2_VCC_EN]    = { 2, 7,	IOCON_MODE_INACT | IOCON_FUNC0, IOCON_DIR_OUTPUT, IOCON_OD_DISABLED, IOCON_VAL_HIGH },
	[PINIDX_SSP0_MRAM_CS2] = { 2, 11,	IOCON_MODE_INACT | IOCON_FUNC0, IOCON_DIR_OUTPUT, IOCON_OD_DISABLED, IOCON_VAL_HIGH },
	[PINIDX_SSP0_MRAM_CS3] = { 2, 12,	IOCON_MODE_INACT | IOCON_FUNC0, IOCON_DIR_OUTPUT, IOCON_OD_DISABLED, IOCON_VAL_HIGH },
	[PINIDX_SUPPLY_RAIL]   = { 3, 25,	IOCON_MODE_INACT | IOCON_FUNC0, IOCON_DIR_OUTPUT, IOCON_OD_DISABLED, IOCON_VAL_HIGH },
	[PINIDX_SD_VCC_EN]     = { 4, 28,	IOCON_MODE_INACT | IOCON_FUNC0, IOCON_DIR_OUTPUT, IOCON_OD_DISABLED, IOCON_VAL_LOW },

	// Inputs
	[PINIDX_RBF]				= { 0, 21, IOCON_MODE_INACT | IOCON_FUNC0, IOCON_DIR_INPUT, IOCON_OD_DISABLED, IOCON_VAL_HIGH },
	[PINIDX_THRUSTER_LATCHUP_P]	= { 1, 26, IOCON_MODE_INACT | IOCON_FUNC0, IOCON_DIR_INPUT, IOCON_OD_DISABLED, IOCON_VAL_HIGH },
	[PINIDX_SP_VCC_FAULT]		= { 2, 4,  IOCON_MODE_INACT | IOCON_FUNC0, IOCON_DIR_INPUT, IOCON_OD_DISABLED, IOCON_VAL_HIGH },
	[PINIDX_BL_SEL1]			= { 3, 26, IOCON_MODE_INACT | IOCON_FUNC0, IOCON_DIR_INPUT, IOCON_OD_DISABLED, IOCON_VAL_HIGH },
	[PINIDX_EXT_WDT_TRIGGERED]	= { 4, 29, IOCON_MODE_INACT | IOCON_FUNC0, IOCON_DIR_INPUT, IOCON_OD_DISABLED, IOCON_VAL_HIGH },

	// not decided yet. to measure correct connections to the outside lets make them outputs for now ....
	[PINIDX_S_STACIE_IO1]	= { 0, 6,  IOCON_MODE_INACT | IOCON_FUNC0, IOCON_DIR_OUTPUT, IOCON_OD_DISABLED, IOCON_VAL_HIGH	},
	[PINIDX_S_STACIE_IO2]	= { 1, 0,  IOCON_MODE_INACT | IOCON_FUNC0, IOCON_DIR_OUTPUT, IOCON_OD_DISABLED, IOCON_VAL_HIGH	},
	[PINIDX_GPIO4_CP]		= { 1, 14, IOCON_MODE_INACT | IOCON_FUNC0, IOCON_DIR_OUTPUT, IOCON_OD_DISABLED, IOCON_VAL_HIGH	},
	[PINIDX_SPC_GPIO3_P]	= { 1, 16, IOCON_MODE_INACT | IOCON_FUNC0, IOCON_DIR_OUTPUT, IOCON_OD_DISABLED, IOCON_VAL_HIGH	},
	[PINIDX_STACIE_C_IO1_P] = { 1, 17, IOCON_MODE_INACT | IOCON_FUNC0, IOCON_DIR_OUTPUT, IOCON_OD_DISABLED, IOCON_VAL_HIGH	},
	[PINIDX_STACIE_A_IO1_P]	= { 1, 21, IOCON_MODE_INACT | IOCON_FUNC0, IOCON_DIR_OUTPUT, IOCON_OD_DISABLED, IOCON_VAL_HIGH	},
	[PINIDX_STACIE_A_IO2_P]	= { 1, 25, IOCON_MODE_INACT | IOCON_FUNC0, IOCON_DIR_OUTPUT, IOCON_OD_DISABLED, IOCON_VAL_HIGH	},
	[PINIDX_GPIO3_A_P]		= { 1, 29, IOCON_MODE_INACT | IOCON_FUNC0, IOCON_DIR_OUTPUT, IOCON_OD_DISABLED, IOCON_VAL_HIGH	},
	[PINIDX_GPIO4_D_P]		= { 1, 31, IOCON_MODE_INACT | IOCON_FUNC0, IOCON_DIR_OUTPUT, IOCON_OD_DISABLED, IOCON_VAL_HIGH	},
};

// This dummy typedef checks if the structure defines a row for each PINIdx enum line. It checks only size it does not detect duplicates !!!
//   ( Failing Check gives compile error ('Negative array size'). Passing check makes no harm - no space needed for typedef. )
typedef char pCheckPins[ (sizeof(pinmuxing2) == PINIDX_MAXIDX*sizeof(PINMUX_GRP_T2)) ? 1 : -1];

#endif /* PINNING_CLIMBOBC_H_ */
