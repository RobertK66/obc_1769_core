/*
 * obc_i2c_int.c
 *
 *  Copied over from Pegasus Flight Software on: 2019-11-21
 *  Copied over from Climb Hwtest on: 2021-12-25
 */

#include <stdio.h>
#include <string.h>

#include "chip.h"

#include "obc_i2c.h"
#include "obc_i2c_rb.h"
#include "obc_i2c_int.h"

#define DEBUG_MODE 0

// module variables
I2C_RB 			I2C_buffer[3];			// This is(are) a(3) ringbuffer(s) holding pointers to job structs
I2C_Data* 		I2C_active[3];			// This is(are) the pointer(s) to one active job per I2CBus.
uint8_t			active_job_done[3];		// A flag (per bus) set by Handler routine if transmission is finished.
i2c_status_t 	i2c_status[3];

// clockrate in kHz
void init_i2c(LPC_I2C_T *I2Cx, uint32_t clockrate) {
	uint8_t busNr;
	int SCL;
	CHIP_SYSCTL_PCLK_T pclk;
	IRQn_Type irq;
	uint32_t irqPrio;
	
	static I2C_Data new_Data;		
	new_Data.job_done = 1;
	new_Data.status = 1;

	if (clockrate > 400) 			/* mehr als 400 kHz wird ned unterstützt */
		clockrate = 400;

	busNr = I2C_getNum(I2Cx);
	if (busNr == -1) {
		// Not a valid I2C device No.
		return;
	}

	i2c_status[busNr].i2c_initialized = 0;
	I2C_RB_init(&I2C_buffer[busNr]);
	I2C_active[busNr] = &new_Data;

	switch (busNr) {
	case 0:
		pclk = SYSCTL_PCLK_I2C0;
		irq = I2C0_IRQn;
		irqPrio = I2C0_INTERRUPT_PRIORITY;
		Chip_IOCON_SetI2CPad(LPC_IOCON, I2CPADCFG_FAST_MODE_PLUS);
		break;
	case 1:
	default:
		pclk = SYSCTL_PCLK_I2C1;
		irq = I2C1_IRQn;
		irqPrio = I2C1_INTERRUPT_PRIORITY;
        Chip_IOCON_EnableOD(LPC_IOCON, 0, 19);
	    Chip_IOCON_EnableOD(LPC_IOCON, 0, 20);
		break;
	case 2:
		pclk = SYSCTL_PCLK_I2C2;
		irq = I2C2_IRQn;
		irqPrio = I2C2_INTERRUPT_PRIORITY;
        Chip_IOCON_EnableOD(LPC_IOCON, 0, 10);
	    Chip_IOCON_EnableOD(LPC_IOCON, 0, 11);
		break;
	}

	active_job_done[busNr] = 1;
	LPC_SYSCTL->PCONP |= (1 << pclk);

	/*--- Clear flags ---*/
	I2Cx->CONCLR = I2C_I2CONCLR_AAC | I2C_I2CONCLR_SIC | I2C_I2CONCLR_STAC | I2C_I2CONCLR_I2ENC;

	/* duty cycle wenn low und high gleich lang ist. (Siehe user manual seite 448) */
	SCL = Chip_Clock_GetPeripheralClockRate(pclk) / (2 * clockrate * 1000);
	if (SCL < 4) {
		SCL = 4; /* SCL darf nicht kleiner als 4 werden (siehe user manual) */
	}
	I2Cx->SCLL = SCL;
	I2Cx->SCLH = SCL;

	/* Install interrupt handler */
	NVIC_EnableIRQ(irq);
	NVIC_SetPriority(irq, irqPrio);
	I2Cx->CONSET = I2C_I2CONSET_I2EN;

	i2c_status[busNr].i2c_error_counter = 0;
	i2c_status[busNr].i2c_interrupt_handler_error = 0;
	i2c_status[busNr].i2c_initialized = 1;
}

uint8_t i2c_add_job(I2C_Data* data) {
	/* Check if I2C hardware was initialized */
	uint8_t busNr = I2C_getNum(data->device);
	if (busNr == -1) {
		return 1;
	}
	if (i2c_status[busNr].i2c_initialized == 0) {
		return 1;
	}

	(*data).job_done = 0;
	(*data).rx_count = 0;
	(*data).tx_count = 0;
	(*data).status = 0;
	(*data).error = I2C_ERROR_NO_ERROR;

	if (data->tx_size == 0) {
		data->dir = 1;
	} else {
		data->dir = 0;
	}

	I2C_RB_put(&I2C_buffer[busNr], (void *) data);

	if (active_job_done[busNr]) {
		I2C_send(data->device, busNr);
	}
	return 0;
}

void I2C0_IRQHandler(void) {
	I2C_Handler(LPC_I2C0);
}

void I2C1_IRQHandler(void) {
	I2C_Handler(LPC_I2C1);
}

void I2C2_IRQHandler(void) {
	I2C_Handler(LPC_I2C2);
}

/**********************************************************************
 * @brief		Convert from I2C peripheral to number
 * @param[in]	I2Cx: I2C peripheral selected, should be:
 * 				- LPC_I2C0
 * 				- LPC_I2C1
 * 				- LPC_I2C2
 * @return 		I2C number, could be: 0..2,
 *              -1 (0xFF) if not detected to be an I2C peripheral
 *********************************************************************/
uint8_t I2C_getNum(LPC_I2C_T *I2Cx) {
	if (I2Cx == LPC_I2C0) {
		return (0);
	} else if (I2Cx == LPC_I2C1) {
		return (1);
	} else if (I2Cx == LPC_I2C2) {
		return (2);
	}
	return (-1);
}

void I2C_Stop(LPC_I2C_T *I2Cx) {
	/* Make sure start bit is not active */
	if (I2Cx->CONSET & I2C_I2CONSET_STA) {
		I2Cx->CONCLR = I2C_I2CONCLR_STAC;
	}
	I2Cx->CONSET = I2C_I2CONSET_STO;
	I2Cx->CONCLR = I2C_I2CONCLR_SIC;
}

void I2C_send(LPC_I2C_T *I2Cx, uint8_t busNr) {
	//uint8_t busNr = I2C_getNum(data->device);
	if (!active_job_done[busNr])
		return;

	active_job_done[busNr] = 0;
	I2C_active[busNr] = I2C_RB_read(&I2C_buffer[busNr]);

	I2Cx->CONCLR = I2C_I2CONCLR_SIC;
	I2Cx->CONSET = I2C_I2CONSET_STA;
}

void I2C_Handler(LPC_I2C_T *I2Cx) {
	uint8_t returnCode;
	uint8_t busNr = I2C_getNum(I2Cx);

	if (busNr == -1) {
		/* I2C not existing */
		return;
	}

	returnCode = (I2Cx->STAT & I2C_STAT_CODE_BITMASK);

	// No Status information available?!
	if (returnCode == 0xf8) {
		if (!I2C_RB_empty(&I2C_buffer[busNr]))
			I2C_send(I2Cx, busNr); /*starte nächsten Job, sofern vorhanden. */

		return;
	}

	switch (returnCode) {
	/* A start/repeat start condition has been transmitted -------------------*/
	case I2C_I2STAT_M_TX_START:
	case I2C_I2STAT_M_TX_RESTART:

		if (DEBUG_MODE == 1)
			I2C_active[busNr]->status = I2C_I2STAT_M_TX_START;

		I2Cx->CONCLR = I2C_I2CONCLR_STAC; /*clear start bit */

		/*dir = 0 bedeutet sende phase, dir muss 1 gesetzt werden wenn beim erstellen der I2C_data keine tx daten angegeben werden!!!! */
		if ((*I2C_active[busNr]).dir == 0) {
			I2Cx->DAT = (I2C_active[busNr]->adress << 1); /*sende SLA+W um übertragung zu starten */
			I2Cx->CONCLR = I2C_I2CONCLR_SIC;
			return;
		} else if (I2C_active[busNr]->dir == 1
				&& I2C_active[busNr]->rx_count
						!= I2C_active[busNr]->rx_size) {
			I2Cx->DAT = (I2C_active[busNr]->adress << 1 | 0x01); /*sende SLA+R um empfang zu starten */
			I2Cx->CONCLR = I2C_I2CONCLR_SIC;
			return;
		} else {
			I2C_Stop(I2Cx); /*wenn er hier her kommt gibt es nichts zu übertragen -> stoppe I2C kommunikation */
			I2C_active[busNr]->job_done = 1; /*wenn es nichts mehr zu übertragen gibt ist der job erledigt */
			active_job_done[busNr] = 1;
			if (!I2C_RB_empty(&I2C_buffer[busNr]))
				I2C_send(I2Cx, busNr);
			return;
		}
		break;

		/* SLA+W has been transmitted, ACK has been received ----------------------*/
	case I2C_I2STAT_M_TX_SLAW_ACK:
		/* Data has been transmitted, ACK has been received */
	case I2C_I2STAT_M_TX_DAT_ACK:

		if (DEBUG_MODE == 1)
			I2C_active[busNr]->status = I2C_I2STAT_M_TX_SLAW_ACK;

		if ((*I2C_active[busNr]).dir == 0) {
			if ((*I2C_active[busNr]).tx_size
					> (*I2C_active[busNr]).tx_count) {
				I2Cx->DAT = *(uint8_t *) (I2C_active[busNr]->tx_data
						+ I2C_active[busNr]->tx_count); /*pointer auf tx daten wird um tx_count verlängert, dann werden die daten ausgelesen */
				I2C_active[busNr]->tx_count++; /*nächsten daten gesendet, daher count erhöhen */
				I2Cx->CONCLR = I2C_I2CONCLR_SIC; /*reset interrupt */
				return;
			} else if ((*I2C_active[busNr]).rx_size
					> (*I2C_active[busNr]).rx_count) {
				I2C_active[busNr]->dir = 1; /*set dir = 1 => change from Transit to receive phase */
				I2Cx->CONSET = I2C_I2CONSET_STA; /*set Start condition for repeated start. */
				I2Cx->CONCLR = I2C_I2CONCLR_AAC | I2C_I2CONCLR_SIC; /*start transmit, reset interrupt */
				return;
			} else {
				I2C_Stop(I2Cx); /*wenn er hier her kommt gibt es nichts mehr zu übertragen -> stoppe I2C kommunikation */
				(*I2C_active[busNr]).job_done = 1; /*wenn es nichts mehr zu übertragen gibt ist der job erledigt */
				active_job_done[busNr] = 1;
				if (!I2C_RB_empty(&I2C_buffer[busNr]))
					I2C_send(I2Cx, busNr); /*starte nächsten Job, sofern vorhanden. */
				return;
			}
		}
		break;
		/* SLA+R has been transmitted, ACK has been received -----------------------------*/
	case I2C_I2STAT_M_RX_SLAR_ACK:

		if (DEBUG_MODE == 1)
			I2C_active[busNr]->status = I2C_I2STAT_M_RX_SLAR_ACK;

		/*if(I2C_active[I2C_num]->rx_count < (I2C_active[I2C_num]->rx_size - 1)){ *//*es werden mehr als 1 Byte erwartet */
		if (I2C_active[busNr]->rx_size > 1) {
			I2Cx->CONSET = I2C_I2CONSET_AA; /*nächstes Byte mit ACK bestätigen damit die nachfolgenden kommen */
		} else {
			I2Cx->CONCLR = I2C_I2CONSET_AA; /*nächstes byte ist das letzte => muss mit NACK bestätigt werden */
		}
		I2Cx->CONCLR = I2C_I2CONCLR_SIC;
		return;

		/* Data has been received, ACK has been returned ----------------------*/
	case I2C_I2STAT_M_RX_DAT_ACK:

		if (DEBUG_MODE == 1)
			I2C_active[busNr]->status = I2C_I2STAT_M_RX_DAT_ACK;

		if (DEBUG_MODE == 0
			|| I2C_active[busNr]->rx_count < I2C_active[busNr]->rx_size) {
			/*check ob noch platz ist um einen overflow zu verhindern */
			*((uint8_t*) (I2C_active[busNr]->rx_data + I2C_active[busNr]->rx_count)) = I2Cx->DAT; /*daten speichern */
			I2C_active[busNr]->rx_count++;
		} else {
			I2C_active[busNr]->error = I2C_ERROR_RX_OVERFLOW;
		}

		if (I2C_active[busNr]->rx_count	< (I2C_active[busNr]->rx_size - 1)) {
			/*es werden mehr als 1 Byte erwartet */
			I2Cx->CONSET = I2C_I2CONSET_AA; /*nächstes Byte mit ACK bestätigen damit die nachfolgenden kommen */
		} else {
			I2Cx->CONCLR = I2C_I2CONSET_AA; /*nächstes byte ist das letzte => muss mit NACK bestätigt werden */
		}
		I2Cx->CONCLR = I2C_I2CONCLR_SIC;
		return;

		/* Data has been received, NACK has been return -------------------------*/
	case I2C_I2STAT_M_RX_DAT_NACK:

		if (DEBUG_MODE == 1)
			I2C_active[busNr]->status = I2C_I2STAT_M_RX_DAT_NACK;

		if (DEBUG_MODE == 0
			|| I2C_active[busNr]->rx_count < I2C_active[busNr]->rx_size) {
			/*check ob noch platz ist um einen overflow zu verhindern */
			*(uint8_t*) (I2C_active[busNr]->rx_data + I2C_active[busNr]->rx_count) = I2Cx->DAT; /*daten speichern */
		} else {
			I2C_active[busNr]->error = I2C_ERROR_RX_OVERFLOW;
		}

		I2C_Stop(I2Cx); /*wenn er hier her kommt gibt es nichts mehr zu empfangen -> stoppe I2C kommunikation */
		I2C_active[busNr]->job_done = 1; /*wenn es nichts mehr zu empfangen gibt ist der job erledigt */
		active_job_done[busNr] = 1;
		if (!I2C_RB_empty(&I2C_buffer[busNr]))
			I2C_send(I2Cx, busNr); /*starte nächsten Job, sofern vorhanden. */

		//read_from_bad_address();
		return;

	case I2C_I2STAT_M_TX_SLAW_NACK:/* SLA+W has been transmitted, NACK has been received */
	case I2C_I2STAT_M_TX_DAT_NACK: /* Data has been transmitted, NACK has been received */
	case I2C_I2STAT_M_RX_SLAR_NACK: /* SLA+R has been transmitted, NACK has been received */
	case I2C_I2STAT_M_RX_ARB_LOST: /* Arbitration lost */

		/* Device antwortet nicht, Job verwerfen */
		I2C_Stop(I2Cx);
		I2C_active[busNr]->error = I2C_ERROR_BUS_ERROR;
		I2C_active[busNr]->status = returnCode;
		I2C_active[busNr]->job_done = 1;
		active_job_done[busNr] = 1;

		i2c_status[busNr].i2c_error_counter++;

		if (!I2C_RB_empty(&I2C_buffer[busNr]))
			I2C_send(I2Cx, busNr);
		return;

	default:
		/* Sollte niemals erreicht werden  */

		i2c_status[busNr].i2c_interrupt_handler_error = 1;
		i2c_status[busNr].i2c_error_counter++;

		I2C_active[busNr]->error = I2C_ERROR_SM_ERROR;
		I2C_Stop(I2Cx);
		I2C_active[busNr]->status = returnCode; /* Eventuell eigener Return-Code für all diese Fälle */
		I2C_active[busNr]->job_done = 1;
		active_job_done[busNr] = 1;
		if (!I2C_RB_empty(&I2C_buffer[busNr]))
			I2C_send(I2Cx, busNr);
		return;
	}

	// Shall never be reached
	I2Cx->CONCLR = I2C_I2CONCLR_SIC; /*wenn er hier her kommt kam es zu einem fehler. Damit der interrupt nicht ständig ausgelöst wird wird er hier resetted */
}

