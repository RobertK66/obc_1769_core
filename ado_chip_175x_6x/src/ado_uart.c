#include "ado_uart.h"

void (*irqHandler0)(LPC_USART_T *pUART) = 0;
void (*irqHandler1)(LPC_USART_T *pUART) = 0;
void (*irqHandler2)(LPC_USART_T *pUART) = 0;
void (*irqHandler3)(LPC_USART_T *pUART) = 0;

void DeInitUart(LPC_USART_T *pUart) {
	if (pUart == LPC_UART0) {
		irqHandler0 = 0;
		NVIC_DisableIRQ(UART0_IRQn);
	} else if (pUart == LPC_UART1) {
		irqHandler1 = 0;
		NVIC_DisableIRQ(UART1_IRQn);
	} else if (pUart == LPC_UART2) {
		irqHandler2 = 0;
		NVIC_DisableIRQ(UART2_IRQn);
	} else if (pUart == LPC_UART3) {
		irqHandler3 = 0;
		NVIC_DisableIRQ(UART3_IRQn);
	}
	Chip_UART_DeInit(pUart);
}

void InitUart(LPC_USART_T *pUart, int baud, void(*irqHandler)(LPC_USART_T *pUART)){
	Chip_UART_Init(pUart);
	Chip_UART_SetBaud(pUart, baud);				// Sets the next available 'un-fractioned' baud rate.
	//Chip_UART_ConfigData(pUart, UART_LCR_WLEN8 | UART_LCR_SBS_1BIT | UART_LCR_PARITY_DIS);	//	8N1 ist default
	//Chip_UART_SetupFIFOS(pUart, 0 ); 	//	No FIFOs - no effect on TX FIFO  !?
	Chip_UART_SetupFIFOS(pUart, UART_FCR_FIFO_EN | UART_FCR_TRG_LEV0 | UART_FCR_TX_RS ); 	//	RX IRQ every byte (LEV0)

	if (irqHandler != 0) {
		if (pUart == LPC_UART0) {
			irqHandler0 = irqHandler;
			NVIC_EnableIRQ(UART0_IRQn);
		} else if (pUart == LPC_UART1) {
			irqHandler1 = irqHandler;
			NVIC_EnableIRQ(UART1_IRQn);
		} else if (pUart == LPC_UART2) {
			irqHandler2 = irqHandler;
			NVIC_EnableIRQ(UART2_IRQn);
		} else if (pUart == LPC_UART3) {
			irqHandler3 = irqHandler;
			NVIC_EnableIRQ(UART3_IRQn);
		}
	}
	/* Enable UART IRQs */
	Chip_UART_TXEnable(pUart);
}

void UART0_IRQHandler(void) {
	if (irqHandler0 != 0) {
		irqHandler0(LPC_UART0);
	}
}

void UART1_IRQHandler(void) {
	if (irqHandler1 != 0) {
		irqHandler1(LPC_UART1);
	}
}

void UART2_IRQHandler(void) {
	if (irqHandler2 != 0) {
		irqHandler2(LPC_UART2);
	}
}

void UART3_IRQHandler(void) {
	if (irqHandler3 != 0) {
		irqHandler3(LPC_UART3);
	}
}
