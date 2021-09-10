/*
===============================================================================
 Name        : temp-Base.h
 Author      : Robert
 Created on	 : 06.09.2021
===============================================================================
*/
// This file should not be needed in the end. Temporary here we can have common structures and defines which are not ready to go
// to the ado lib. Either they should become useless here or the abstraction is good enough to go to some ado header files/modules....
#ifndef TEMP_BASE_H_
#define TEMP_BASE_H_

#include <chip.h>

// common defines/structures used by board abstraction
#define GPIO_DIR_OUTPUT true
#define GPIO_DIR_INPUT  false
#define GPIO_VAL_HIGH   true
#define GPIO_VAL_LOW    false

typedef struct {
	uint8_t port:3;
	uint8_t pinNr:5;
	bool 	output;
	bool    initVal;
} GPIO_INIT_T;


// common defines/structures used for module composition
typedef struct {
	//void  *initData;
	void  (*init)(void* initData);
	void  (*main)(void);
} MODULE_DEF_T;

#endif /* TEMP_BASE_H_ */
