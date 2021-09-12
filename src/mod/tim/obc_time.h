/*
===============================================================================
 Name        : obc_time.h
 Author      : Robert
 Created on	 : 12.09.2021
===============================================================================
*/

#ifndef MOD_TIM_OBC_TIME_H_
#define MOD_TIM_OBC_TIME_H_

#include "../../temp-Base.h"


void tim_init (void *dummy);
void tim_main (void);

static const MODULE_DEF_T timModuleDesc = {
		(void*)tim_init,
		tim_main
};

#define TimInitModule() {							 	\
	timModuleDesc.init(NULL); 								\
}

#define TimMain() { 		\
	timModuleDesc.main(); 	\
}






#endif /* MOD_TIM_OBC_TIME_H_ */
