/*
 * obc_vram.c
 *
 *  Created on: 21.03.2022
 *      Author: Robert
 */

#include "obc_vram.h"

#include <signal.h>

void vramInit(void *initdata){

}
void vramMain(void){

}

vram_future_t vramWriteAsync(uint8_t sectionId,uint32_t address, uint8_t *data, uint16_t size){
	vram_future_t result = {false, 0};

	return result;
}

vram_future_t vramReadAsync(uint8_t sectionId,uint32_t address,  uint8_t *data, uint16_t size){
	vram_future_t result = {false, 0};

	return result;
}
