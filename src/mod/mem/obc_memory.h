/*
===============================================================================
 Name        : obc_memory.h
 Author      : Robert
 Created on	 : 23.12.2021
===============================================================================
*/

#ifndef MOD_MEM_OBC_MEMORY_H_
#define MOD_MEM_OBC_MEMORY_H_

void memInit(void *dummy);
void memMain(void);

void memChangeInstanceName(char* name);

#define MODULE_ID_MEMORY			0x03
#define EVENT_MEM_OPERATIONAL		1
#define EVENT_MEM_BLOCK0_UPDATED	2



#endif /* MOD_MEM_OBC_MEMORY_H_ */
