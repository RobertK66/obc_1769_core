#include <chip.h>

#include "obc_i2c_rb.h"

//
//static inline void __enable_irq() { __asm volatile ("cpsie i"); }
//static inline void __disable_irq() { __asm volatile ("cpsid i"); }
//
//void taskDISABLE_INTERRUPTS() {
//	// in peg here is some ASM inline code called from RTOS
//	__disable_irq();
//}
//
//void taskENABLE_INTERRUPTS() {
//	// in peg here is some ASM inline code called from RTOS
//	__enable_irq();
//}

void I2C_RB_init(I2C_RB *rb)
{
	rb->start = 0;
	rb->end = 0;
	rb->overflow = 0;
}

void I2C_RB_put(I2C_RB *rb, void* daten)
{
	__disable_irq();
	rb->buffer[rb->end] = daten;
	rb->end = (rb->end + 1) & (I2C_RB_Size - 1);

	if (rb->end == rb->start)
	{
		/*rb->overflow = 1; */
//		obc_status_extended.i2c_rb_overflow = 1;
		rb->start = (rb->start + 1) & (I2C_RB_Size - 1);
	}
	__enable_irq();
}

uint8_t I2C_RB_full(I2C_RB *rb)
{
	if (((rb->end + 1) & (I2C_RB_Size - 1)) == rb->start)
	{
		return 1;
	}
	return 0;
}

uint8_t I2C_RB_empty(I2C_RB *rb)
{
	if (rb->end == rb->start)
	{
		return 1;
	}
	return 0;
}

void* I2C_RB_read(I2C_RB *rb)
{
	void *daten;

	__disable_irq();
	daten = rb->buffer[rb->start];
	rb->start = (rb->start + 1) & (I2C_RB_Size - 1);
	__enable_irq();

	return daten;
}
