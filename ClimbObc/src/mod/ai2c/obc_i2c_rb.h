#include <stdint.h>

#define I2C_RB_Size 128

typedef struct
{
	uint32_t start;
	uint32_t end;
	uint32_t overflow;
	void* buffer[I2C_RB_Size];
} I2C_RB;

void I2C_RB_init(I2C_RB *rb);
void I2C_RB_put(I2C_RB *rb, void* daten);
uint8_t I2C_RB_full(I2C_RB *rb);
uint8_t I2C_RB_empty(I2C_RB *rb);
void* I2C_RB_read(I2C_RB *rb);

