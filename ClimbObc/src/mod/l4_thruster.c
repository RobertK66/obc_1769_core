/*
===============================================================================
 Name        : l4_thruster.c
 Author      : Jevgeni
 Created on	 : 30.06.2022

 Higher level logic layer to control thruster
===============================================================================
*/

#include <string.h>
#include <stdio.h>
#include <stdlib.h>


#include <ado_crc.h>



#include "l7_climb_app.h"
#include "l4_thruster.h"

#include <string.h>
#include <stdlib.h>
#include "l2_debug_com.h"
#include <mod/ado_mram.h>
#include <mod/ado_sdcard.h>

#include "mem/obc_memory.h"
#include "tim/obc_time.h"
#include "l3_sensors.h"
#include "hw_check.h"
#include "tim/climb_gps.h"
#include "thr/thr.h"

#include "tim/obc_time.h"

#include <mod/ado_mram.h>
#include "modules_globals.h"

typedef struct  {
	uint8_t sequence_id;
	uint8_t register_index; //access register for which action is taken
	uint32_t wait; // wait between stages [ms]
	double double_arg1;
	double double_arg2;
	double double_arg3;
	//char *args[6];

}l4_stage_arguments_t;



void l4_wait(l4_stage_arguments_t *stage_args);

void l4_GeneralSetRequest(l4_stage_arguments_t *stage_args);
void l4_GeneralSetRequest_sequence(l4_stage_arguments_t *stage_args);
void l4_GeneralSetRequestAndWait_sequence(l4_stage_arguments_t *stage_args);

void l4_GeneralReadRequest(l4_stage_arguments_t *stage_args);
void l4_GeneralReadRequest_sequence(l4_stage_arguments_t *stage_args);
void l4_GeneralReadRequestAndWait_sequence(l4_stage_arguments_t *stage_args);

void l4_ReadAllRegisters(l4_stage_arguments_t *stage_args);
void l4_ReadAllRegistersAndWait_sequence(l4_stage_arguments_t *stage_args);


void thr_execute_sequence();
void l4_void(l4_stage_arguments_t *stage_args);
void l4_value_ramp(l4_stage_arguments_t *stage_args);
//void thr_wait_and_monitor(int argc, char *argv[]);
void l4_wait_and_monitor(int argc, char *argv[]);


void initialize_hardcoded_thr_sequences();



//MEM
void thr_write_mem_callback(uint8_t chipIdx, mram_res_t result, uint32_t adr, uint8_t *data, uint32_t len);
void thr_write_mem();
void thr_read_mem();
void thr_read_mem_callback(uint8_t chipIdx, mram_res_t result, uint32_t adr, uint8_t *data, uint32_t len);
uint8_t MMRAM_READ_BUFFER[6];

#define MAX_EXECUTION_SEQUENCE_DEPTH 10 // Maximum size of execution sequence stack
#define MAX_HARDCODED_SEQUENCES 5 // Maximum number of preprogrammed sequences




typedef struct {
	void (*function)(l4_stage_arguments_t *stage_arguments);
	uint16_t procedure_id;
	l4_stage_arguments_t *stage_args;
} thr_sequences_t;

typedef struct {
	thr_sequences_t sequences[MAX_EXECUTION_SEQUENCE_DEPTH];
	uint8_t length; // length of sequence
	uint16_t execution_index; // current index of execution stack
	uint32_t sequence_execution_begin; // timestamp at which sequence begin
	uint32_t sequence_execution_stage; // timestamp at which stage is completed
	uint32_t sequence_execution_substage; // timestamp at which substage is completed
	bool sequence_trigger;
	bool repeat;
	bool pause;
	bool restart;
	int substage_index;
	double ramp_initial_value;

}l4_hardcoded_sequences_t;
l4_hardcoded_sequences_t L4_HARDCODED_SEQUENCES[MAX_HARDCODED_SEQUENCES];





// TODO: Fill the array with REGISTER_NAMES so that logging messages will contain name of accessed register allong with  [register_index]
//const char* REGISTER_NAME[5] = { "0 Firmware Version (major)","1 Firmware Version (minor)","2 Serial Number","4 None","5 Reset Cycle"	};








void l4_thruster_init (void *dummy) {

	initialize_hardcoded_thr_sequences();



}

void l4_thruster_main (void) {


	for (int i=0;i<=5;i++){ // for all preprogrammed sequences

		if (L4_HARDCODED_SEQUENCES[i].sequence_trigger ){ //if trigger for sequence is set to True - execute sequence
			thr_execute_sequence(i);

		}

	}


}









//void l4_GeneralReadRequest(l4_stage_arguments_t stage_args)
void l4_GeneralSetRequest_sequence(l4_stage_arguments_t *stage_args){
	/*
	 * _sequence is a wrapper arround General  thruster registor Read/Set request functions
	 *
	 * it passes input argv further into original function
	 *
	 * However after execution of original function pointer of execution sequence stack THR_EXECUTION_INDEX
	 * is increased so that next stage can be executed.
	 *
	 * after completion of sequence - timestamp is recorded
	 */
	uint16_t procedure_id = stage_args->sequence_id; // procedure_id is always fist index of argument array
	l4_GeneralSetRequest(stage_args);

	char print_str[200];
	sprintf(print_str, "\nStage SET index= %d completed\n",L4_HARDCODED_SEQUENCES[procedure_id].execution_index);
	//int len = strlen(print_str);
	//deb_print_pure_debug((uint8_t *)print_str, len);
	SysEvent(0, EVENT_INFO, 3, (uint8_t *)print_str, strlen(print_str));

	//*******assume that argv[5] is custom print message
	//len = strlen(argv[5]);
	//deb_print_pure_debug((uint8_t *)argv[5], len);


	L4_HARDCODED_SEQUENCES[procedure_id].execution_index++;
	L4_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage = (uint32_t)timGetSystime();


}

void l4_GeneralSetRequestAndWait_sequence(l4_stage_arguments_t *stage_args){
	uint16_t procedure_id = stage_args->sequence_id;
	uint32_t duration = stage_args->wait;
	char print_str[200];
	uint32_t now_timestamp;
	//int len;




	switch (L4_HARDCODED_SEQUENCES[procedure_id].substage_index){

	case 0:
		l4_GeneralSetRequest(stage_args);
		L4_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage = (uint32_t)timGetSystime();
		L4_HARDCODED_SEQUENCES[procedure_id].substage_index++;
		return;
		break;
	case 1:
		now_timestamp = (uint32_t)timGetSystime();
		if ( (now_timestamp - L4_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage) < duration ){
					// do nothing
				}
				else{
					sprintf(print_str, "\n Set and wait, Stage= %d Complete t = %d\n", L4_HARDCODED_SEQUENCES[procedure_id].execution_index,now_timestamp);
					//len = strlen(print_str);
					//deb_print_pure_debug((uint8_t *)print_str, len);
					SysEvent(0, EVENT_INFO, 3, (uint8_t *)print_str, strlen(print_str));

					L4_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage = (uint32_t)timGetSystime();
					L4_HARDCODED_SEQUENCES[procedure_id].substage_index = 0;
					L4_HARDCODED_SEQUENCES[procedure_id].execution_index++; // increase sequence execution index so that after wait - next module to be executed


					//len = strlen(argv[5]);
					//deb_print_pure_debug((uint8_t *)argv[5], len); // assume that argv[5] is custom print message
				}
		return;
		break;


	}




}


void l4_GeneralReadRequest_sequence(l4_stage_arguments_t *stage_args){
	uint16_t procedure_id = stage_args->sequence_id; // procedure_id is always fist index of argument array
	l4_GeneralReadRequest(stage_args);

	char print_str[200];
	sprintf(print_str, "\nStage READ index= %d completed\n",L4_HARDCODED_SEQUENCES[procedure_id].execution_index);
	//int len = strlen(print_str);
	//deb_print_pure_debug((uint8_t *)print_str, len);
	SysEvent(0, EVENT_INFO, 3, (uint8_t *)print_str, strlen(print_str));

	//len = strlen(argv[5]);
	//deb_print_pure_debug((uint8_t *)argv[5], len); //assume that argv[5] is custom print message


	L4_HARDCODED_SEQUENCES[procedure_id].execution_index++;
	L4_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage = (uint32_t)timGetSystime();

}


void l4_GeneralReadRequestAndWait_sequence(l4_stage_arguments_t *stage_args){

	uint16_t procedure_id = stage_args->sequence_id;
	uint32_t duration = stage_args->wait;
	char print_str[200];
	uint32_t now_timestamp;
	//int len;




	switch (L4_HARDCODED_SEQUENCES[procedure_id].substage_index){

	case 0:
		l4_GeneralReadRequest(stage_args);
		L4_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage = (uint32_t)timGetSystime();
		L4_HARDCODED_SEQUENCES[procedure_id].substage_index++;
		return;
		break;
	case 1:
		now_timestamp = (uint32_t)timGetSystime();
		if ( (now_timestamp - L4_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage) < duration ){
					// do nothing
				}
				else{
					sprintf(print_str, "\n Read single and wait, Stage= %d Complete t = %d\n", L4_HARDCODED_SEQUENCES[procedure_id].execution_index,now_timestamp);
					//len = strlen(print_str);
					//deb_print_pure_debug((uint8_t *)print_str, len);
					SysEvent(0, EVENT_INFO, 3, (uint8_t *)print_str, strlen(print_str));

					L4_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage = (uint32_t)timGetSystime();
					L4_HARDCODED_SEQUENCES[procedure_id].substage_index = 0;
					L4_HARDCODED_SEQUENCES[procedure_id].execution_index++; // increase sequence execution index so that after wait - next module to be executed


					//len = strlen(argv[5]);
					//deb_print_pure_debug((uint8_t *)argv[5], len); // assume that argv[5] is custom print message
				}
		return;
		break;


	}





}



//////
void l4_wait(l4_stage_arguments_t *stage_args){

	/*
	 * thr_wait is a "staging" function incorporated in THR_EXECUTION_SEQUENCE
	 * at end of previous stage - timestep of stage completion is recorded
	 *
	 * duration is required input from argv
	 *
	 * function will do nothing until difference between current and previous timestamps is less then duration
	 *
	 * THR_EXECUTION_INDEX points to next action in the execution sequence stack.
	 * THR_EXECUTION INDEX is increased after current timestamp increased above designed wait duration
	 *
	 */
	uint16_t procedure_id = stage_args->sequence_id; // procedure_id is always fist index of argument array
	uint32_t duration = stage_args->wait;
	uint32_t now_timestamp = (uint32_t)timGetSystime();


	char print_str[200];
	//int len;

	if ( (now_timestamp - L4_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage) < duration ){
		// do nothing
	}
	else{
		L4_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage = (uint32_t)timGetSystime(); //save execution finish time of wait stage
		sprintf(print_str, "\nWait Stage= %d Complete t = %d\n", L4_HARDCODED_SEQUENCES[procedure_id].execution_index,now_timestamp);
		//len = strlen(print_str);
		//deb_print_pure_debug((uint8_t *)print_str, len);
		SysEvent(0, EVENT_INFO, 3, (uint8_t *)print_str, strlen(print_str));
		L4_HARDCODED_SEQUENCES[procedure_id].execution_index++; // increase sequence execution index so that after wait - next module to be executed


		//len = strlen(argv[5]);
		//deb_print_pure_debug((uint8_t *)argv[5], len); // assume that argv[5] is custom print message
	}

}


void l4_wait_and_monitor(int argc, char *argv[]){
	/*
	 *
	 * This module is use for Hot Standby Script Sequence
	 * Module will wait until Reservoir Temperature will heat up
	 * So that thruster is ready to fire
	 *
	 * Module will monitor and log Reservoir Temperature Register 0x63  99
	 *
	 * uint32_t duration - [ms]total wait duration after reservoir heater was turned on
	 * uint32_t logging_dt - [ms] time after which READ requests for temperature monitoring are being sent
	 * uint32_t wait_for_read_request_proccess -[ms] delay to wait until read request is processed
	 *
	 *
	 */

	uint16_t procedure_id = atoi(argv[0]); // procedure_id is always fist index of argument array
	uint32_t duration = atoi(argv[1]);
	uint32_t logging_dt = atoi(argv[2]); //Time after which READ request is sent
	uint32_t now_timestamp = (uint32_t)timGetSystime();
	uint8_t register_index = atoi(argv[3]); // Register that is intended to be monitored
	char print_str[200];
	//int len;
	char *temp_argv[2];
	int waiting_between_logging;
	char argument1[50];

	uint32_t wait_for_read_request_proccess = 3000; // WARNING ! Wait time for processing request [ms] should not be equal to logging dt [ms]
	// Otherwise function never exits


	switch (L4_HARDCODED_SEQUENCES[procedure_id].substage_index){

	case 0:

		if ( ((now_timestamp - L4_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage) % logging_dt ) == 0 ){
				waiting_between_logging = (int)( (now_timestamp - L4_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage)/1000 );

				sprintf(print_str, "\n Waiting %d s \n",waiting_between_logging  );
				//len = strlen(print_str);
				//deb_print_pure_debug((uint8_t *)print_str, len);
				SysEvent(0, EVENT_INFO, 3, (uint8_t *)print_str, strlen(print_str));

				L4_HARDCODED_SEQUENCES[procedure_id].substage_index++; // increase substage to procede with READ request
				L4_HARDCODED_SEQUENCES[procedure_id].sequence_execution_substage = (uint32_t)timGetSystime(); // save timestamp of substage completion
				break;

					}



		if ( (now_timestamp - L4_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage) < duration ){
				// do nothing
			}
		else{
				L4_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage = (uint32_t)timGetSystime(); //save execution finish time of wait stage

				sprintf(print_str, "\nWait and Monitor Stage = %d Complete t = %d\n", L4_HARDCODED_SEQUENCES[procedure_id].execution_index,now_timestamp);
				//len = strlen(print_str);
				//deb_print_pure_debug((uint8_t *)print_str, len);
				SysEvent(0, EVENT_INFO, 3, (uint8_t *)print_str, strlen(print_str));
				L4_HARDCODED_SEQUENCES[procedure_id].execution_index++; // increase sequence execution index so that after wait - next module to be executed
				L4_HARDCODED_SEQUENCES[procedure_id].substage_index = 0;

				// assume that argv[5] is custom print message
				//len = strlen(argv[5]);
				//deb_print_pure_debug((uint8_t *)argv[5], len);
				//SysEvent(0, EVENT_INFO, 3, (uint8_t *)print_str, strlen(print_str));
			}




		break;
	case 1:
		sprintf(print_str, "\n Sending READ [%d] Request %d\n",register_index, now_timestamp );
		//len = strlen(print_str);
		//deb_print_pure_debug((uint8_t *)print_str, len);
		SysEvent(0, EVENT_INFO, 3, (uint8_t *)print_str, strlen(print_str));

		temp_argv[0]= "6";

		sprintf(argument1, "%d",register_index);
		temp_argv[1]= argument1;

		GeneralReadRequest(2, temp_argv);

		L4_HARDCODED_SEQUENCES[procedure_id].substage_index ++;
		L4_HARDCODED_SEQUENCES[procedure_id].sequence_execution_substage = (uint32_t)timGetSystime();

		break;
	case 2:

		if ( (now_timestamp - L4_HARDCODED_SEQUENCES[procedure_id].sequence_execution_substage) < wait_for_read_request_proccess ){ // Wait to proccess READ request
						// do nothing
					}
		else{
			//L4_HARDCODED_SEQUENCES[procedure_id].sequence_execution_substage = (uint32_t)timGetSystime(); //save execution finish time of wait substage


			sprintf(print_str, "\nMonitored [%d] Value = %.6f \n", register_index,ReadThrRegData(register_index));
			//len = strlen(print_str);
			//deb_print_pure_debug((uint8_t *)print_str, len);
			SysEvent(0, EVENT_INFO, 3, (uint8_t *)print_str, strlen(print_str));

			L4_HARDCODED_SEQUENCES[procedure_id].substage_index = 0;
			}


		break;

	}






}


void l4_void(l4_stage_arguments_t *stage_args){
	/*
	 * Incorporates with sequence execution
	 *
	 * void function that does nothing
	 */

	uint16_t procedure_id = stage_args->sequence_id; // procedure_id is always fist index of argument array
	L4_HARDCODED_SEQUENCES[procedure_id].execution_index ++;

}

l4_stage_arguments_t l4_value_ramp_temp_request_args; // this global variable to be tightly coupled with l4_value_ramp function
void l4_value_ramp(l4_stage_arguments_t *stage_args){
	/*
	 * According to enpulsion defination RAMP Frequency should be 1 Hz, Duration of ramp is 30s
	 * Out function allows to specify different dt(frequency) and ramp duration.
	 * When hardcoding sequences - lets stick with enpulsion ramp requirments
	 */

	uint16_t procedure_id = stage_args->sequence_id; // procedure_id is always fist index of argument array
	uint8_t register_index = stage_args->register_index; // first argument is register index at which SET ramp would be implemented
	double goal = stage_args->double_arg1; // goal to which value should be set
	uint32_t ramp_duration = (uint32_t)stage_args->double_arg2; // time through which value should be changed from initial to goal
	uint32_t ramp_dt = stage_args->wait; // wait between SET / Converted to (ms)
	double initial_value;
	uint32_t now_timestamp;

	char print_str[200];
	//int len;

	double ramp_iterations = ramp_duration/ ramp_dt; // WARING - HARDCODE IN A WAY THAT THIS IS ALWAYS INT
	double value_step;




	switch (L4_HARDCODED_SEQUENCES[procedure_id].substage_index){

	case 0:// first we make read request to desired register in order to obtain initial register value
		l4_ReadAllRegisters(stage_args); // arguments are irrelevant for read all registers function

		sprintf(print_str, "\nWInitial read Request Sent\n");
		//len = strlen(print_str);
		//deb_print_pure_debug((uint8_t *)print_str, len);
		SysEvent(0, EVENT_INFO, 3, (uint8_t *)print_str, strlen(print_str));

		// assume that argv[5] is custom print message
		//len = strlen(argv[5]);
		//deb_print_pure_debug((uint8_t *)argv[5], len);

		L4_HARDCODED_SEQUENCES[procedure_id].substage_index++;
		break;
	case 1: // wait for some time while process request is executing
		now_timestamp = (uint32_t)timGetSystime();
		if ( (now_timestamp - L4_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage) <= 5000 ){
					// do nothing
			return;

				}
		else if ( (now_timestamp - L4_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage) > 5000 ){
			L4_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage = (uint32_t)timGetSystime(); //save execution finish time of wait stage
			L4_HARDCODED_SEQUENCES[procedure_id].substage_index++;
			return;

		}
		break;
	case 2: // now we are sure that read request was proccessed - read initial value
		//L4_HARDCODED_SEQUENCES[procedure_id].ramp_initial_value = REGISTER_DATA[register_index];
		L4_HARDCODED_SEQUENCES[procedure_id].ramp_initial_value = ReadThrRegData(register_index);
		L4_HARDCODED_SEQUENCES[procedure_id].substage_index++;
		sprintf(print_str, "\nInitial value save value = %.5f\n",L4_HARDCODED_SEQUENCES[procedure_id].ramp_initial_value);
		//len = strlen(print_str);
		//deb_print_pure_debug((uint8_t *)print_str, len);
		SysEvent(0, EVENT_INFO, 3, (uint8_t *)print_str, strlen(print_str));
		break;
	case 3:

		initial_value  = L4_HARDCODED_SEQUENCES[procedure_id].ramp_initial_value;
		value_step = (goal -initial_value)/ramp_iterations;
		//REGISTER_DATA[register_index] = REGISTER_DATA[register_index] +value_step;
		SetEncodedThrRegValue((ReadThrRegData(register_index)  +value_step ), register_index); // I have some doubts here that it is efficient
		// maybe make global variable instead of running a function two times ?

		//if ((REGISTER_DATA[register_index] >= goal && value_step >0)    || (REGISTER_DATA[register_index] <= goal && value_step <0 )    ){
		if ((ReadThrRegData(register_index) >= goal && value_step >0)    || (ReadThrRegData(register_index) <= goal && value_step <0 )    ){ //TODO again this is stupid
		//two runs of the same function with the same result in if condition - stupid.
			// GOAL REACHED// EXIT FUNCTION

			//sprintf(print_str, "\nWarning ! OVER THE GOAL %.5f \n",REGISTER_DATA[register_index]);
			sprintf(print_str, "\nWarning ! OVER THE GOAL %.5f \n",ReadThrRegData(register_index)); // THIRD RUN OF SAME FUNC WITH SAME RESULT ! STUPID
			//len = strlen(print_str);
			//deb_print_pure_debug((uint8_t *)print_str, len);
			SysEvent(0, EVENT_INFO, 3, (uint8_t *)print_str, strlen(print_str));


			//l4_value_ramp_temp_request_args.sequence_id =stage_args->sequence_id;
			l4_value_ramp_temp_request_args.register_index = register_index;
			l4_value_ramp_temp_request_args.double_arg1= goal;
			l4_GeneralSetRequest(&l4_value_ramp_temp_request_args);
			//REGISTER_DATA[register_index] = goal;
			SetEncodedThrRegValue(goal, register_index); //Todo - check if it is stupid. Instead of assignment operation we run function ......

			//sprintf(print_str, "\nValue corrected according to goal %.5f \n",REGISTER_DATA[register_index]);
			sprintf(print_str, "\nValue corrected according to goal %.5f \n",ReadThrRegData(register_index)); //TODO remove
			//len = strlen(print_str);
			//deb_print_pure_debug((uint8_t *)print_str, len);
			SysEvent(0, EVENT_INFO, 3, (uint8_t *)print_str, strlen(print_str));

			L4_HARDCODED_SEQUENCES[procedure_id].substage_index++;
			L4_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage = (uint32_t)timGetSystime();
			break;
		}



		sprintf(print_str, "\nRestore saved initial value = %.5f\n",initial_value );
		//len = strlen(print_str);
		//deb_print_pure_debug((uint8_t *)print_str, len);
		SysEvent(0, EVENT_INFO, 3, (uint8_t *)print_str, strlen(print_str));


		sprintf(print_str, "\nSTEP = %.2f\n",value_step);
		if (register_index == 16){sprintf(print_str, "\nSTEP = %.7f\n",value_step);    } // for Thrust ramp we need to print goal with 5 decimal precession
		//len = strlen(print_str);
		//deb_print_pure_debug((uint8_t *)print_str, len);
		SysEvent(0, EVENT_INFO, 3, (uint8_t *)print_str, strlen(print_str));


		sprintf(print_str, "\nSET RAMP value = %.5f\n",ReadThrRegData(register_index));
		//len = strlen(print_str);
		//deb_print_pure_debug((uint8_t *)print_str, len);
		SysEvent(0, EVENT_INFO, 3, (uint8_t *)print_str, strlen(print_str));

		l4_value_ramp_temp_request_args.register_index = register_index;
		l4_value_ramp_temp_request_args.double_arg1= ReadThrRegData(register_index)+value_step;
		l4_GeneralSetRequest(&l4_value_ramp_temp_request_args);

		L4_HARDCODED_SEQUENCES[procedure_id].substage_index++;
		L4_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage = (uint32_t)timGetSystime();
		break;

	case 4: // WAIT after RAMP set request done
		now_timestamp = (uint32_t)timGetSystime();
		if ( (now_timestamp - L4_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage) < ramp_dt*1000){ // ramp_dt*1000 converts dt from [s] to [ms]
			// do nothing
		}
		else{
			L4_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage = (uint32_t)timGetSystime(); //save execution finish time of wait stage
			L4_HARDCODED_SEQUENCES[procedure_id].substage_index++;
			sprintf(print_str, "\nWaiting after SET RAMP\n");
			//len = strlen(print_str);
			//deb_print_pure_debug((uint8_t *)print_str, len);
			SysEvent(0, EVENT_INFO, 3, (uint8_t *)print_str, strlen(print_str));
		}

		break;

	case 5:

		// EXIT CONDITIONS

		initial_value  = L4_HARDCODED_SEQUENCES[procedure_id].ramp_initial_value;
		value_step = (goal -initial_value)/ramp_iterations;

		if (value_step > 0){
			//if (REGISTER_DATA[register_index] >= goal){
			if (ReadThrRegData(register_index) >= goal*0.99){ // more then 99% of goal - WARNING THIS IS DANGEROUS

				// GOAL REACHED// EXIT FUNCTION
				L4_HARDCODED_SEQUENCES[procedure_id].execution_index ++;
				L4_HARDCODED_SEQUENCES[procedure_id].substage_index = 0;

			}
			else{
			 	 // IF GOAL VALUE NOT YET REACHED JUMP BACK TO SET REQUEST
				L4_HARDCODED_SEQUENCES[procedure_id].substage_index=3;
				L4_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage = (uint32_t)timGetSystime();
			}
		}

		if(value_step <0){

			//if (REGISTER_DATA[register_index] <= goal){
			if (ReadThrRegData(register_index) <= goal*1.01){ //goal*1.01 (with 101% error) this is dangerous !
							// GOAL REACHED// EXIT FUNCTION
							L4_HARDCODED_SEQUENCES[procedure_id].execution_index ++;
							L4_HARDCODED_SEQUENCES[procedure_id].substage_index = 0;

						}
						else{
						 	 // IF GOAL VALUE NOT YET REACHED JUMP BACK TO SET REQUEST
							L4_HARDCODED_SEQUENCES[procedure_id].substage_index=3;
							L4_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage = (uint32_t)timGetSystime();
						}



		}
		if(value_step == 0){

			//warning unexpected.
			L4_HARDCODED_SEQUENCES[procedure_id].execution_index ++;
			L4_HARDCODED_SEQUENCES[procedure_id].substage_index = 0;
			sprintf(print_str, "\nExit condition valuestep=0\n");
			//len = strlen(print_str);
			//deb_print_pure_debug((uint8_t *)print_str, len);
			SysEvent(0, EVENT_INFO, 3, (uint8_t *)print_str, strlen(print_str));

		}

		break;




	}







}

void thr_execute_sequence_cmd(int argc, char *argv[]){
	/*
	 * This function is meant to trigger execution of SEQUENCE
	 * Triggered function is thr_execute_sequence()

	*/
	uint8_t procedure_id = atoi(argv[1]);
	uint8_t action = atoi(argv[2]);
	char print_str[200];
	//int len;

	switch(action){

	case 0: // start sequence

		L4_HARDCODED_SEQUENCES[procedure_id].sequence_trigger = true;
		L4_HARDCODED_SEQUENCES[procedure_id].execution_index = 0;
		L4_HARDCODED_SEQUENCES[procedure_id].sequence_execution_begin = (uint32_t)timGetSystime();
		L4_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage = (uint32_t)timGetSystime();
		L4_HARDCODED_SEQUENCES[procedure_id].substage_index = 0;
		sprintf(print_str, "\nSequence id=%d start\n",procedure_id);
		//len = strlen(print_str);
		//deb_print_pure_debug((uint8_t *)print_str, len);
		SysEvent(0, EVENT_INFO, 3, (uint8_t *)print_str, strlen(print_str));
		break;
	case 1: // restart sequence
		L4_HARDCODED_SEQUENCES[procedure_id].restart = true;
		sprintf(print_str, "\nSequence id=%d will now restart start\n",procedure_id);
		//len = strlen(print_str);
		//deb_print_pure_debug((uint8_t *)print_str, len);
		SysEvent(0, EVENT_INFO, 3, (uint8_t *)print_str, strlen(print_str));
		break;
	case 2: //repeat sequence
		L4_HARDCODED_SEQUENCES[procedure_id].repeat = true;
		sprintf(print_str, "\nSequence id=%d will now repeat\n",procedure_id);
		//len = strlen(print_str);
		//deb_print_pure_debug((uint8_t *)print_str, len);
		SysEvent(0, EVENT_INFO, 3, (uint8_t *)print_str, strlen(print_str));
		break;
	case 3: //cancel repeat sequence
			L4_HARDCODED_SEQUENCES[procedure_id].repeat = false;
			sprintf(print_str, "\nSequence id=%d stop repeat\n",procedure_id);
			//len = strlen(print_str);
			//deb_print_pure_debug((uint8_t *)print_str, len);
			SysEvent(0, EVENT_INFO, 3, (uint8_t *)print_str, strlen(print_str));
			break;
	case 4: //Pause sequence
			L4_HARDCODED_SEQUENCES[procedure_id].pause = true;
			L4_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage = (uint32_t)timGetSystime();
			sprintf(print_str, "\nSequence id=%d paused\n",procedure_id);
			//len = strlen(print_str);
			//deb_print_pure_debug((uint8_t *)print_str, len);
			SysEvent(0, EVENT_INFO, 3, (uint8_t *)print_str, strlen(print_str));
			break;
	case 5: //resume sequence
			L4_HARDCODED_SEQUENCES[procedure_id].pause = false;
			L4_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage = (uint32_t)timGetSystime();
			sprintf(print_str, "\nSequence id=%d resumed\n",procedure_id);
			//len = strlen(print_str);
			//deb_print_pure_debug((uint8_t *)print_str, len);
			SysEvent(0, EVENT_INFO, 3, (uint8_t *)print_str, strlen(print_str));
			break;
	case 6: //Stop sequence
			L4_HARDCODED_SEQUENCES[procedure_id].sequence_trigger = false;
			L4_HARDCODED_SEQUENCES[procedure_id].execution_index = 0;
			sprintf(print_str, "\nSequence id=%d manual stop\n",procedure_id);
			//len = strlen(print_str);
			//deb_print_pure_debug((uint8_t *)print_str, len);
			SysEvent(0, EVENT_INFO, 3, (uint8_t *)print_str, strlen(print_str));
			break;
	}

	//L4_HARDCODED_SEQUENCES[procedure_id].sequence_trigger = true;
	//L4_HARDCODED_SEQUENCES[procedure_id].execution_index = 0;
	//L4_HARDCODED_SEQUENCES[procedure_id].sequence_execution_begin = (uint32_t)timGetSystime();
	//L4_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage = (uint32_t)timGetSystime();

	//sprintf(print_str, "\nSequence id=%d start\n",procedure_id);
	//len = strlen(print_str);
	//deb_print_pure_debug((uint8_t *)print_str, len);



	//*** THIS IS TEST EXAMPLE TO DEMONSTRATE THAT IT IS POSSIBLE TO CHANGE EXECUTION FUNCTION AND ARGUMENTS IN A SEQUENCE
	//char temp_arg[1];
	//sprintf(temp_arg, "%d",procedure_id);
	//L4_HARDCODED_SEQUENCES[procedure_id].sequences[0].thr_argv[0] = temp_arg;
	//L4_HARDCODED_SEQUENCES[procedure_id].sequences[0].thr_argv[1] = "20";
	//L4_HARDCODED_SEQUENCES[procedure_id].sequences[0].thr_argv[2] = "3000";
	//L4_HARDCODED_SEQUENCES[procedure_id].sequences[0].function = GeneralSetRequest_sequence;
	 //***********************************************************************




}



void thr_execute_sequence(int procedure_id){
	/*
	 *
	 * Executes preprogrammed sequence  defined in THR_EXECUTION_SEQUENCE function pointer array
	 * THR_SEQUENCES array of argv to be input into THR_EXECUTION_SEQUENCE
	 */

	if(L4_HARDCODED_SEQUENCES[procedure_id].pause){
		// if sequence pause set to true
		// do nothing
		return;
		}

	if(L4_HARDCODED_SEQUENCES[procedure_id].restart){
					// Restart sequence
					L4_HARDCODED_SEQUENCES[procedure_id].sequence_trigger = true;
					L4_HARDCODED_SEQUENCES[procedure_id].execution_index =0; // reset execution index to 0
					L4_HARDCODED_SEQUENCES[procedure_id].restart = false;
					return;
			}

	if (L4_HARDCODED_SEQUENCES[procedure_id].execution_index > L4_HARDCODED_SEQUENCES[procedure_id].length){
		char print_str[200];
		sprintf(print_str, "\nSequence complete\n");
		//int len = strlen(print_str);
		//deb_print_pure_debug((uint8_t *)print_str, len);
		SysEvent(0, EVENT_INFO, 3, (uint8_t *)print_str, strlen(print_str));

		if(L4_HARDCODED_SEQUENCES[procedure_id].repeat){
			sprintf(print_str, "\nRepeat sequence\n");
			//len = strlen(print_str);
			//deb_print_pure_debug((uint8_t *)print_str, len);
			SysEvent(0, EVENT_INFO, 3, (uint8_t *)print_str, strlen(print_str));
			L4_HARDCODED_SEQUENCES[procedure_id].sequence_trigger = true;
			L4_HARDCODED_SEQUENCES[procedure_id].execution_index =0;
		}
		else{
			L4_HARDCODED_SEQUENCES[procedure_id].sequence_trigger = false;
			L4_HARDCODED_SEQUENCES[procedure_id].execution_index =0;
			return;

		}


	}

	else{
		void (*f)(l4_stage_arguments_t *stage_arguments);
		f = L4_HARDCODED_SEQUENCES[procedure_id].sequences[L4_HARDCODED_SEQUENCES[procedure_id].execution_index].function;
		//LAST_STARTED_MODULE = L4_HARDCODED_SEQUENCES[procedure_id].execution_index; //DEBUG
		f(L4_HARDCODED_SEQUENCES[procedure_id].sequences[L4_HARDCODED_SEQUENCES[procedure_id].execution_index].stage_args);
	}





}


//****************** THIS IS JUST TEST AND TRIAL********************************

void thr_write_mem(){
	uint8_t testdata[6];
	testdata[0] = 0x68;
	testdata[1] = 0x65;
	testdata[2] = 0x6C;
	testdata[3] = 0x6C;
	testdata[4] = 0x6F;
	testdata[5] = 0x0A;
	MramWriteAsync(0, 5000, (uint8_t*)&testdata, sizeof(testdata), thr_write_mem_callback);

}


void thr_write_mem_callback(uint8_t chipIdx, mram_res_t result, uint32_t adr, uint8_t *data, uint32_t len) {
	if (result == MRAM_RES_SUCCESS) {
		char print_str[200];
		sprintf(print_str, "\nMemory Write Success\n");
		//int len = strlen(print_str);
		//deb_print_pure_debug((uint8_t *)print_str, len);
		SysEvent(0, EVENT_INFO, 3, (uint8_t *)print_str, strlen(print_str));

		thr_read_mem();



	} else {
		// TODO:?? retry counter / timeouts ....
	}
}



void thr_read_mem(){
	MramReadAsync(0, 5000, MMRAM_READ_BUFFER, sizeof(MMRAM_READ_BUFFER), thr_read_mem_callback);
	//void MramReadAsync(uint8_t chipIdx, uint32_t adr,  uint8_t *rx_data,  uint32_t len,
	//void (*finishedHandler)(uint8_t chipIdx,mram_res_t result, uint32_t adr, uint8_t *data, uint32_t len))
}

void thr_read_mem_callback(uint8_t chipIdx, mram_res_t result, uint32_t adr, uint8_t *data, uint32_t len) {

	if (result == MRAM_RES_SUCCESS) {
		char print_str[200];
		sprintf(print_str, "\nMemory Read Success\n");
		//int len = strlen(print_str);
		//deb_print_pure_debug((uint8_t *)print_str, len);
		SysEvent(0, EVENT_INFO, 3, (uint8_t *)print_str, strlen(print_str));

		len = strlen((char*)data);
		//deb_print_pure_debug(data, len);
		SysEvent(0, EVENT_INFO, 3, (uint8_t *)print_str, strlen(print_str));

		sprintf(print_str, "\n-----msg_end--------\n");
		len = strlen(print_str);
		//deb_print_pure_debug((uint8_t *)print_str, len);
		SysEvent(0, EVENT_INFO, 3, (uint8_t *)print_str, strlen(print_str));



	} else {
		// TODO:?? retry counter / timeouts ....
	}
}


void mem_write_cmd(int argc, char *argv[]){

	thr_write_mem(); //debug test

}
// ******************************************************************************

//const l4_stage_arguments_t temp_arg1 =  {0,20,0,3000,0,0 };

//l4_stage_arguments_t HARDCODED_STAGE_ARGS[MAX_HARDCODED_SEQUENCES][MAX_EXECUTION_SEQUENCE_DEPTH];
l4_stage_arguments_t HARDCODED_STAGE_ARGS[3][10];

void initialize_hardcoded_thr_sequences(){

	/// **************** PREPROGRAMM SEQUENCES HERE ****************
	uint8_t exeFunc_index=0; // this is helper index to simplify HARDCODDING sequence manually.
	int sequence_id_int=0;
	uint32_t wait_between_stages = 500;

	// SET
	L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = l4_GeneralSetRequest_sequence;
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].sequence_id =sequence_id_int;
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].register_index = THR_SPECIFIC_IMPULSE_REF_REG;
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].double_arg1 = 1000;
	L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].stage_args = &HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index];
	exeFunc_index++;



	 //Wait Between SET requests
	L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = l4_wait;
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].sequence_id =sequence_id_int;
	//HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].register_index = THR_SPECIFIC_IMPULSE_REF_REG;
	//HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].double_arg1 = 3000;
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].wait = wait_between_stages;
	L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].stage_args = &HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index];
	exeFunc_index++;


	// Action SET
	L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = l4_GeneralSetRequest_sequence;
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].sequence_id =sequence_id_int;
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].register_index = THR_SPECIFIC_IMPULSE_REF_REG;
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].double_arg1 = 1500;
	L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].stage_args = &HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index];
	exeFunc_index++;


	 //Wait Between SET requests
	L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = l4_wait;
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].sequence_id =sequence_id_int;
	//HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].register_index = THR_SPECIFIC_IMPULSE_REF_REG;
	//HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].double_arg1 = 3000;
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].wait = wait_between_stages;
	L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].stage_args = &HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index];
	exeFunc_index++;

	// Action SET
	L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = l4_GeneralSetRequest_sequence;
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].sequence_id =sequence_id_int;
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].register_index = THR_SPECIFIC_IMPULSE_REF_REG;
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].double_arg1 = 1500;
	L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].stage_args = &HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index];
	exeFunc_index++;

	 //Wait Between SET requests
	L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = l4_wait;
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].sequence_id =sequence_id_int;
	//HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].register_index = THR_SPECIFIC_IMPULSE_REF_REG;
	//HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].double_arg1 = 3000;
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].wait = wait_between_stages;
	L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].stage_args = &HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index];
	exeFunc_index++;

	// RAMP
	L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = l4_value_ramp;
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].sequence_id =sequence_id_int;
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].register_index = THR_SPECIFIC_IMPULSE_REF_REG;
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].double_arg1 = 3000; // ramp goal
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].double_arg2 = 20; //ramp duration [s]
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].wait = 2; //dt
	L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].stage_args = &HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index];
	exeFunc_index++;

	 //Wait Between SET requests
	L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = l4_wait;
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].sequence_id =sequence_id_int;
	//HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].register_index = THR_SPECIFIC_IMPULSE_REF_REG;
	//HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].double_arg1 = 3000;
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].wait = wait_between_stages;
	L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].stage_args = &HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index];
	exeFunc_index++;

	// RAMP
	L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = l4_value_ramp;
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].sequence_id =sequence_id_int;
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].register_index = THR_SPECIFIC_IMPULSE_REF_REG;
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].double_arg1 = 1500; // ramp goal
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].double_arg2 = 20; //ramp duration [s]
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].wait = 2; //dt [s]
	L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].stage_args = &HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index];
	exeFunc_index++;


	 //Wait Between SET requests
	L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = l4_wait;
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].sequence_id =sequence_id_int;
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].register_index = THR_SPECIFIC_IMPULSE_REF_REG;
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].double_arg1 = 3000;
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].wait = wait_between_stages;
	L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].stage_args = &HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index];
	//exeFunc_index++;


	L4_HARDCODED_SEQUENCES[sequence_id_int].length = exeFunc_index; // MANUALLY DEFINE LENGTH OF SEQUENCE //
	L4_HARDCODED_SEQUENCES[sequence_id_int].sequence_trigger = false;
	L4_HARDCODED_SEQUENCES[sequence_id_int].repeat = false;
	L4_HARDCODED_SEQUENCES[sequence_id_int].substage_index = 0; //DEFAULT SUBSTAGE INDEX


	// ******* SEQUENCE 1 ************* STABILITY TEST

	exeFunc_index=0; // at the beggining of sequence hardcodding set it to 0
	sequence_id_int = 1;
	wait_between_stages = 500;
	// SET
	L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = l4_GeneralSetRequest_sequence;
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].sequence_id =sequence_id_int;
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].register_index = THR_SPECIFIC_IMPULSE_REF_REG;
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].double_arg1 = 1000;
	L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].stage_args = &HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index];
	exeFunc_index++;



	 //Wait Between SET requests
	L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = l4_wait;
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].sequence_id =sequence_id_int;
	//HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].register_index = THR_SPECIFIC_IMPULSE_REF_REG;
	//HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].double_arg1 = 3000;
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].wait = wait_between_stages;
	L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].stage_args = &HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index];
	exeFunc_index++;


	// Action SET
	L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = l4_GeneralSetRequest_sequence;
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].sequence_id =sequence_id_int;
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].register_index = THR_SPECIFIC_IMPULSE_REF_REG;
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].double_arg1 = 1500;
	L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].stage_args = &HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index];
	exeFunc_index++;


	 //Wait Between SET requests
	L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = l4_wait;
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].sequence_id =sequence_id_int;
	//HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].register_index = THR_SPECIFIC_IMPULSE_REF_REG;
	//HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].double_arg1 = 3000;
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].wait = wait_between_stages;
	L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].stage_args = &HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index];
	exeFunc_index++;

	// Action SET
	L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = l4_GeneralSetRequest_sequence;
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].sequence_id =sequence_id_int;
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].register_index = THR_SPECIFIC_IMPULSE_REF_REG;
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].double_arg1 = 1500;
	L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].stage_args = &HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index];
	exeFunc_index++;

	 //Wait Between SET requests
	L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = l4_wait;
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].sequence_id =sequence_id_int;
	//HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].register_index = THR_SPECIFIC_IMPULSE_REF_REG;
	//HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].double_arg1 = 3000;
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].wait = wait_between_stages;
	L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].stage_args = &HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index];
	//exeFunc_index++;




	L4_HARDCODED_SEQUENCES[sequence_id_int].length = exeFunc_index; // MANUALLY DEFINE LENGTH OF SEQUENCE //
	L4_HARDCODED_SEQUENCES[sequence_id_int].sequence_trigger = false;
	L4_HARDCODED_SEQUENCES[sequence_id_int].repeat = false;
	L4_HARDCODED_SEQUENCES[sequence_id_int].substage_index = 0; //DEFAULT SUBSTAGE INDEX



	// ******* SEQUENCE 2 ************* SETAND WAIT  TEST

	exeFunc_index=0; // at the beggining of sequence hardcodding set it to 0
	sequence_id_int = 2;
	wait_between_stages =2000;


	// SET AND WAIT
	L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = l4_GeneralSetRequestAndWait_sequence;
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].sequence_id =sequence_id_int;
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].register_index = THR_SPECIFIC_IMPULSE_REF_REG;
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].double_arg1 = 1000;
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].wait = wait_between_stages;
	L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].stage_args = &HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index];
	exeFunc_index++;


	// SET AND WAIT
	L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = l4_GeneralSetRequestAndWait_sequence;
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].sequence_id =sequence_id_int;
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].register_index = THR_SPECIFIC_IMPULSE_REF_REG;
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].double_arg1 = 1500;
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].wait = wait_between_stages;
	L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].stage_args = &HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index];
	exeFunc_index++;


	// READ SINGLE REGISTER AND WAIT
	L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = l4_GeneralReadRequestAndWait_sequence;
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].sequence_id =sequence_id_int;
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].register_index = THR_SPECIFIC_IMPULSE_REF_REG;
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].double_arg1 = 1500;
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].wait = wait_between_stages;
	L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].stage_args = &HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index];
	exeFunc_index++;





	// READ ALL REGISTERS AND WAIT
	L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = l4_ReadAllRegistersAndWait_sequence;
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].sequence_id =sequence_id_int;
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].register_index = THR_SPECIFIC_IMPULSE_REF_REG;
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].double_arg1 = 1500;
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].wait = wait_between_stages;
	L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].stage_args = &HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index];
	exeFunc_index++;

	// READ ALL REGISTERS AND WAIT
	L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = l4_ReadAllRegistersAndWait_sequence;
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].sequence_id =sequence_id_int;
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].register_index = THR_SPECIFIC_IMPULSE_REF_REG;
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].double_arg1 = 1500;
	HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index].wait = wait_between_stages;
	L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].stage_args = &HARDCODED_STAGE_ARGS[sequence_id_int][exeFunc_index];
	//exeFunc_index++;




	L4_HARDCODED_SEQUENCES[sequence_id_int].length = exeFunc_index; // MANUALLY DEFINE LENGTH OF SEQUENCE //
	L4_HARDCODED_SEQUENCES[sequence_id_int].sequence_trigger = false;
	L4_HARDCODED_SEQUENCES[sequence_id_int].repeat = false;
	L4_HARDCODED_SEQUENCES[sequence_id_int].substage_index = 0; //DEFAULT SUBSTAGE INDEX

		/*
		// ******* SEQUENCE 0 ************* OPERATIONAL SCRIPTS /Hot Standby Script / 10-1

		exeFunc_index=0; // at the beggining of sequence hardcodding set it to 0
		sequenc_id_char = "0";
		sequence_id_int = 0;
		wait_between_stages_str = "2000";
		//sprintf(sequenc_id_char, "%d",sequence_id_int); // for this to work sequence_id_char[50] have to be initialized as array instead of pointer

		// Action 31001:Set Operational Mode 0 / Register 0x0E  14
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = "14";
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2] = "0";
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nAction 31001:Set Operational Mode 0\n";
		exeFunc_index++;


		// Wait Between SET requests
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
		exeFunc_index++;

		// Action 31002: Set Emitter Mode 0 / Register 0x1E  30
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = "30";
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2] = "0";
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nAction 31002: Set Emitter Mode 0\n";
		exeFunc_index++;

		// Wait Between SET requests
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
		exeFunc_index++;

		// Action 31003:Set Extractor Mode 0 / Register 0x2D  45
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = "45";
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2] = "0";
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nAction 31003:Set Extractor Mode 0\n";
		exeFunc_index++;

		// Wait Between SET requests
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
		exeFunc_index++;

		// Action 31004: Set Neutralizer Mode 0 / Register 0x4B  75
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = "75";
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2] = "0";
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nAction 31004: Set Neutralizer Mode 0\n";
		exeFunc_index++;


		// Wait Between SET requests
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
		exeFunc_index++;

		// Action 31005: Set Temperature Mode 0 / Register 0x60  96
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = "75";
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2] = "0";
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nAction 31005: Set Temperature Mode 0\n";
		exeFunc_index++;

		// Wait Between SET requests
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
		exeFunc_index++;

		// Action 31006: Set Reservoire Heater Mode 0 / Register 0x3C  60
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = "60";
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2] = "0";
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nAction 31006: Set Reservoire Heater Mode 0\n";
		exeFunc_index++;

		// Wait Between SET requests
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
		exeFunc_index++;

		// Action 31007: Set Thrust Ref 0 N / Register 0x10  16
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = "16";
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2] = "0";
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nAction 31007: Set Thrust Ref 0 N\n";
		exeFunc_index++;


		// Wait Between SET requests
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
		exeFunc_index++;

		// Action 31008: Set Reservoir Heater Power Ref 6 W / Register 0x45  69
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = "69";
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2] = "6";
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nAction 31008: Set Reservoir Heater Power Ref 6 W\n";
		exeFunc_index++;


		// Wait Between SET requests
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
		exeFunc_index++;

		// Action 31009: Set Operational Mode 1 / Register 0x0E  14
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = "14";
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2] = "1";
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nAction 31008: Reservoir Heater Power Ref 6 W\n";
		exeFunc_index++;

		// Action 31010: Wait AND MONITOR / Register (Reservoir Temperature) 0x63 99
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait_and_monitor;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = "3600000"; // Total wait duration [ms] 2.5h = 9 000 000ms
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2] = "10000"; // dt between READ request for Reservoir temperature
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[3] = "97"; // Register that is intended to be monitored Reservoir temp Ref
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nAction 31010: Wait AND MONITOR\n";
		//exeFunc_index++;



		L4_HARDCODED_SEQUENCES[sequence_id_int].length = exeFunc_index; // MANUALLY DEFINE LENGTH OF SEQUENCE //
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequence_trigger = false;
		L4_HARDCODED_SEQUENCES[sequence_id_int].repeat = false;
		L4_HARDCODED_SEQUENCES[sequence_id_int].substage_index = 0; //DEFAULT SUBSTAGE INDEX
		exeFunc_index= 0;




		// ******* SEQUENCE 2 ************* Operational Scripts / Thrust Maneuver /10-3

		exeFunc_index=0; // at the beggining of sequence hardcodding set it to 0
		sequenc_id_char = "1";
		sequence_id_int = 1;
		//sprintf(sequenc_id_char, "%d",sequence_id_int);

		// Action 31301: Set Extractor Mode 0 / Register 0x2D  45
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = "45";
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2] = "0";
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nAction 31301: Set Extractor Mode 0\n";
		exeFunc_index++;


		// Wait Between SET requests
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
		//exeFunc_index++;

		// Action 31302: Set Emitter Mode 0 / Register 0x1E  30
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = "30";
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2] = "0";
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nAction 31302: Set Emitter Mode 0\n";
		exeFunc_index++;



		// Wait Between SET requests
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
		exeFunc_index++;


		// Action 31303: Set Neutralizer Mode 0 / Register 0x4B  75
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = "75";
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2] = "0";
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nAction 31304: Set Neutralizer Mode 0\n";
		exeFunc_index++;

		// Wait Between SET requests
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
		exeFunc_index++;

		// Action 31304: Set Emitter Power Ref 30 W / Register 0x27  39
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = "39";
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2] = "30"; // [W]
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nAction 31304: Set Emitter Power Ref 30 W\n";
		exeFunc_index++;

		// Wait Between SET requests
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
		exeFunc_index++;



		//5 Action 31305: Set Neutralizer Filament Ref 1/2 // Register 0x4d 77
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "77";
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "1";
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5]= "\nAction 31305: Set Neutralizer Filament Ref 1\n";
		exeFunc_index++;



		// Wait Between SET requests
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
		exeFunc_index++;

		//5 Action 31306: Set Specific Impulse Ref 3000 s // Register 0x14 20
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "20";
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "3000";
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5]= "\nAction 31306: Set Specific Impulse Ref 3000 s\n";
		exeFunc_index++;



		// Wait Between SET requests
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
		exeFunc_index++;




		//17 Action 31307: Thrust Ref ramp (30s,1Hz) from INITIAL VALUE to 300 microN // Register 0x10 16 dec
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_value_ramp;  // RAMP DOWN
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "16"; // Neutralizer Heater Power Ref register
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "0.0003"; // GOAL of RAMP - 300 microN = 0.3 mN = 0.0003N
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[3]= "30"; // ramp duration 30s
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[4]= "2"; // 2 Secons between set requests = 0.5Hz
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5]= "\nAction 13009: Neutralizer Heater Power Ref ramp (30s,1Hz) from 2W to 0W\n ";
		exeFunc_index++;

		// Wait Between SET requests
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
		exeFunc_index++;


		// Action 31308: Wait AND MONITOR Thrust/ Register 0x12  18
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait_and_monitor;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = "60000"; // Total wait duration [ms] 2.5h = 9 000 000ms
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2] = "10000"; // dt between READ request
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[3] = "18"; // Register that is intended to be monitored
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nAction 31308: Wait AND MONITOR Thrust\n";
		exeFunc_index++;

		// Wait Between SET requests
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
		exeFunc_index++;

		//5 Action 31309: Set Thrust Ref 0 N // Register 0x10 16
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "16";
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "0";
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5]= "\nAction 31309: Set Thrust Ref 0 N\n";
		//exeFunc_index++;

		L4_HARDCODED_SEQUENCES[sequence_id_int].length = exeFunc_index; // MANUALLY DEFINE LENGTH OF SEQUENCE //
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequence_trigger = false;
		L4_HARDCODED_SEQUENCES[sequence_id_int].repeat = false;
		L4_HARDCODED_SEQUENCES[sequence_id_int].substage_index = 0; //DEFAULT SUBSTAGE INDEX
		exeFunc_index= 0;

		// ******* SEQUENCE 3 / Operational Scripts /Cooldown Script / 10-4

		exeFunc_index=0; // at the beggining of sequence hardcodding set it to 0
		sequenc_id_char = "2";
		sequence_id_int = 2;
		wait_between_stages_str = "2000";

		// Action 31401:Set Operational Mode 0 / Register 0x0E  14
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = "14";
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2] = "0";
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nAction 31401:Set Operational Mode 0\n";
		exeFunc_index++;


		// Wait Between SET requests
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
		exeFunc_index++;

		// Action 31402: Set Emitter Mode 0 / Register 0x1E  30
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = "30";
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2] = "0";
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nAction 31402: Set Emitter Mode 0\n";
		exeFunc_index++;

		// Wait Between SET requests
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
		exeFunc_index++;


		// Action 31404: Set Neutralizer Mode 0 / Register 0x4B  75
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = "75";
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2] = "0";
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nAction 31404: Set Neutralizer Mode 0\n";
		exeFunc_index++;



		// Wait Between SET requests
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
		exeFunc_index++;




		// Action 31404: Wait AND MONITOR Reservoir Temperature/ Register 0x63  99
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait_and_monitor;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = "3600000"; // Total wait duration [ms] 1h = 3600 000[ms]
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2] = "10000"; // dt between READ request
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[3] = "99"; // Register that is intended to be monitored
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nAction 31404: Wait AND MONITOR Reservoir Temperature\n";
		exeFunc_index++;


		L4_HARDCODED_SEQUENCES[sequence_id_int].length = exeFunc_index; // MANUALLY DEFINE LENGTH OF SEQUENCE //
		L4_HARDCODED_SEQUENCES[sequence_id_int].sequence_trigger = false;
		L4_HARDCODED_SEQUENCES[sequence_id_int].repeat = false;
		L4_HARDCODED_SEQUENCES[sequence_id_int].substage_index = 0; //DEFAULT SUBSTAGE INDEX
		exeFunc_index= 0;


		// ******   SEQUENCE 4  *******System Cold Test/ Neutralizer Bias Test/ Table 9-3

		exeFunc_index=0; // at the beggining of sequence hardcodding set it to 0
		sequenc_id_char = "3";
		sequence_id_int = 3;
		wait_between_stages_str = "2000";




			//1 Action 12001: Set Neutralizer Mode 0 // Register 0x4B hex 75 dec
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "75"; //Neutralizer Mode
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "0";
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5]= "\nAction 12001: Set Neutralizer Mode 0\n";
			exeFunc_index++;

			// Wait Between SET requests
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
			exeFunc_index++;

			//3 Action 12002: Set Neutralizer Bias Ref 1 // Register 0x4E hex 78 dec
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "78";
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "1";
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5]= "\nAction 12002: Set Neutralizer Bias Ref 1\n";
			exeFunc_index++;

			// Wait Between SET requests
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
			exeFunc_index++;

			//5 Action 12003: Set Neutralizer Mode 1 // Register 0x4B hex 75 dec
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "75"; //Neutralizer Mode
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "1";
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5]= "\nAction 12003: Set Neutralizer Mode 0\n";
			exeFunc_index++;



			//6 Action 12004: Wait for 60s
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "10000";
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5]= "\nAction 12004: Waiting 10s\n";
			exeFunc_index++;

			//7 wait
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "10000";
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5]= "\nAction 12004: Waiting 20s\n";
			exeFunc_index++;


			//8 wait
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "10000";
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5]= "\nAction 12004: Waiting 30s\n";
			exeFunc_index++;

			//9 wait
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "10000";
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5]= "\nAction 12004: Waiting 40s\n";
			exeFunc_index++;

			//10 wait
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "10000";
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5]= "\nAction 12004: Waiting 50s\n";
			exeFunc_index++;

			//11 wait
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "10000";
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5]= "\nAction 12004: Waiting 60s\n";
			exeFunc_index++;

					//12 Action 12005: Set Neutralizer Bias Ref 1 // Register 0x4E hex 78 dec
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "78";
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "0";
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5]= "\nAction 12005: Set Neutralizer Bias Ref 0\n";
			exeFunc_index++;


			//13 Wait between SET requests
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "2000";
			exeFunc_index++;


			//14 Action 12006: Set Neutralizer Mode 0 // Register 0x4B hex 75 dec
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "75"; //Neutralizer Mode
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "0";
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5]= "\nAction 12001: Set Neutralizer Mode 0\n";
			//exeFunc_index++;

			L4_HARDCODED_SEQUENCES[sequence_id_int].length = exeFunc_index; // MANUALLY DEFINE LENGTH OF SEQUENCE //
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequence_trigger = false;
			L4_HARDCODED_SEQUENCES[sequence_id_int].repeat = false;
			L4_HARDCODED_SEQUENCES[sequence_id_int].substage_index = 0; //DEFAULT SUBSTAGE INDEX
			exeFunc_index= 0;


			//////////// ******** SEQUENCE 3*************** SYSTEM COLD TEST/ Heater Ramp Test/ Table 9-2

			exeFunc_index=0; // at the beggining of sequence hardcodding set it to 0
			sequenc_id_char = "4";
			sequence_id_int = 4;
			wait_between_stages_str = "2000";

			//0 Action 11001: Set Heater Mode 0  / Register 0x3C hex   60 dec
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "60";
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "0";
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5]= "\nAction 11001: Heater Mode set 0 \n";
			exeFunc_index++;

			// Wait Between SET requests
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
			exeFunc_index++;

			//2 Action 1102: Set Heater Voltage Ref 12v / Register 0x3D 61 dec
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;  // Set request
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char; //procedure id HARDCODED
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "61"; // Reservoir Heater Voltage Ref register
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "12"; //  Set to 12V
			exeFunc_index++;


			// Wait Between SET requests
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
			exeFunc_index++;

			//4 Action 1103: Set Heater Current Ref 1.5A / Register 0x41 65 dec
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;  // Set request
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char; //procedure id HARDCODED
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "65"; // Reservoir Heater Current Ref register
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "1.5"; //  Set to 1.5 A
			exeFunc_index++;


			// Wait Between SET requests
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
			exeFunc_index++;

			//6 Action 1104: Set Heater Power Ref 0W / Register 0x45 69 dec
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;  // Set request
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char; //procedure id HARDCODED
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "69"; // Reservoir Heater Power Ref register
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "0"; //  Set to 0 W
			exeFunc_index++;

			// Wait Between SET requests
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
			exeFunc_index++;

			//8 Action 11005: Set Heater Mode 1  / Register 0x3C hex   60 dec
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char; // OBC sequence id
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "60"; // Reservoir heater mode register
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "1"; // Heater mode value

			// Wait Between SET requests
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
			exeFunc_index++;

			//10 Action 11006: Heater Power Ref ramp (30s,1Hz) from 0W to 10W // Register 0x45 69 dec
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_value_ramp;  // RAMP UP
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char; //procedure id HARDCODED
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "69"; // Heater Power Ref register
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "10"; // GOAL of RAMP - manually set to 3000s
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[3]= "30"; // ramp duration 30s
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[4]= "2"; // 1 Secons between set requests = 1Hz
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5]= "\nAction 11006: Heater Power Ref ramp (30s,1Hz) from 0W to 10W\n ";
			exeFunc_index++;

			// Wait Between SET requests
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
			exeFunc_index++;

			//11 Action 11007: Heater Power Ref ramp (30s,1Hz) from 10W to 0W // Register 0x45 69 dec
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_value_ramp;  // RAMP UP
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char; //procedure id HARDCODED
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "69"; // Heater Power Ref register
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "0"; // GOAL of RAMP - manually set to 3000s
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[3]= "30"; // ramp duration 30s
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[4]= "2"; // 1 Secons between set requests = 1Hz
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5]= "\nAction 11007: Heater Power Ref ramp (30s,1Hz) from 10W to 0W\n ";
			exeFunc_index++;

			// Wait Between SET requests
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
			exeFunc_index++;

			//13 Action 1108: Set Heater Voltage Ref 0V / Register 0x3D 61 dec
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;  // Set request
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char; //procedure id HARDCODED
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "61"; // Reservoir Heater Voltage Ref register
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "0"; //  Set to 0V
			exeFunc_index++;

			// Wait Between SET requests
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
			exeFunc_index++;

			//15 Action 1109: Set Heater Current Ref 0A / Register 0x41 65 dec
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;  // Set request
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char; //procedure id HARDCODED
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "65"; // Reservoir Heater Current Ref register
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "0"; //  Set to 0 A
			exeFunc_index++;

			// Wait Between SET requests
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
			exeFunc_index++;

			//17 Action 11010: Set Heater Mode 0  / Register 0x3C hex   60 dec
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char; // OBC sequence id
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "60"; // Reservoir heater mode register
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "0"; // Heater mode value
			exeFunc_index++;

			// Wait Between SET requests
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
			exeFunc_index++;

			//19 Action 11011: Set Heater Power Ref 6 W / Register 0x45 65 dec
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;  // Set request
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char; //procedure id HARDCODED
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "69"; // Reservoir Heater Power Ref register
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "6"; //  Set to 6 W
			//exeFunc_index++;


			L4_HARDCODED_SEQUENCES[sequence_id_int].length = exeFunc_index; // MANUALLY DEFINE LENGTH OF SEQUENCE //
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequence_trigger = false;
			L4_HARDCODED_SEQUENCES[sequence_id_int].repeat = false;
			L4_HARDCODED_SEQUENCES[sequence_id_int].substage_index = 0; //DEFAULT SUBSTAGE INDEX
			exeFunc_index= 0;

			// ******   SEQUENCE 6  *******System Cold Test / Neutralizer Heating Test /Table 9-4

			exeFunc_index=0; // at the beggining of sequence hardcodding set it to 0
			sequenc_id_char = "5";
			sequence_id_int = 5;
			wait_between_stages_str = "2000";




			//1 Action 13001: Set Neutralizer Mode 0 // Register 0x4B hex 75 dec
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "75"; //Neutralizer Mode
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "0";
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5]= "\nAction 12001: Set Neutralizer Mode 0\n";
			exeFunc_index++;

			// Wait Between SET requests
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
			exeFunc_index++;

			//3 Action 13002: Set Neutralizer Heater Current Ref 0.7A // Register 0x52 82
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "82"; //Neutralizer Mode
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "0.7";
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5]= "\nAction 13002: Set Heutralizer Heater Current Ref 0.7A\n";
			exeFunc_index++;

			// Wait Between SET requests
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
			exeFunc_index++;

			//5 Action 13003: Set Neutralizer Filament Ref 1/2 // Register 0x4d 77
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "77"; //Neutralizer Mode
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "1";
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5]= "\nAction 13003: Set Neutralizer Filament Ref 1\n";
			exeFunc_index++;

			// Wait Between SET requests
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
			exeFunc_index++;

			//7 Action 13004: Set Neutralizer Heater Power Ref 0W // Register 0x56 86
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "86";
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "0";
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5]= "\nAction 13004: Set Neutralizer Heater Power Ref 0W\n";
			exeFunc_index++;

			// Wait Between SET requests
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
			exeFunc_index++;

			//9 Action 13005: Set Neutralizer Beam Current Ref 1mA // Register 0x5C 92
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "92";
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "0.001"; // 0.001 A
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5]= "\nAction 13004: Set Neutralizer Heater Power Ref 0W\n";
			exeFunc_index++;

			// Wait Between SET requests
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
			exeFunc_index++;


			//11 Action 13006: Set Neutralizer Bias Ref 1 // Register 0x4E hex 78 dec
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "78";
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "1";
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5]= "\nAction 13006: Set Neutralizer Bias Ref 1\n";
			exeFunc_index++;

			// Wait Between SET requests
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
			exeFunc_index++;



			//13 Action 13007: Set Neutralizer Mode 1 // Register 0x4B hex 75 dec
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "75"; //Neutralizer Mode
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "1";
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5]= "\nAction 13007: Set Neutralizer Mode 1\n";
			exeFunc_index++;


			// Wait Between SET requests
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
			exeFunc_index++;



			//15 Action 13008: Neutralizer Heater Power Ref ramp (30s,1Hz) from 0W to 2W // Register 0x56 86 dec
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_value_ramp;  // RAMP UP
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "86"; // Neutralizer Heater Power Ref register
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "2"; // GOAL of RAMP - 2w
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[3]= "30"; // ramp duration 30s
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[4]= "2"; // 2 Secons between set requests = 0.5Hz
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5]= "\nAction 13008: Neutralizer Heater Power Ref ramp (30s,1Hz) from 0W to 2W\n ";
			exeFunc_index++;


			// Wait Between SET requests
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
			exeFunc_index++;




			//17 Action 13009: Neutralizer Heater Power Ref ramp (30s,1Hz) from 2W to 0W // Register 0x56 86 dec
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_value_ramp;  // RAMP DOWN
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "86"; // Neutralizer Heater Power Ref register
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "0"; // GOAL of RAMP - 0W
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[3]= "30"; // ramp duration 30s
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[4]= "2"; // 2 Secons between set requests = 0.5Hz
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5]= "\nAction 13009: Neutralizer Heater Power Ref ramp (30s,1Hz) from 2W to 0W\n ";
			exeFunc_index++;



			// Wait Between SET requests
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
			exeFunc_index++;



			//19 Action 13010: Set Neutralizer Heater Current Ref 0A // Register 0x52 82
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "82"; //Neutralizer Mode
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "0";
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5]= "\nAction 13010: Set Heutralizer Heater Current Ref 0A\n";
			exeFunc_index++;


			// Wait Between SET requests
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
			exeFunc_index++;



			//21 Action 13011: Set Neutralizer Beam Current Ref 0mA // Register 0x5C 92
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "92";
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "0"; // 0.001 A
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5]= "\nAction 13011: Set Neutralizer Beam Current Ref 0mA\n";
			exeFunc_index++;


			// Wait Between SET requests
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
			exeFunc_index++;




			//23 Action 13012: Set Neutralizer Heater Power Ref 0W // Register 0x56 86
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "86";
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "0";
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5]= "\nAction 13012: Set Neutralizer Heater Power Ref 0W\n";
			exeFunc_index++;


			// Wait Between SET requests
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
			exeFunc_index++;


			//25 Action 13013: Set Neutralizer Bias Ref 0 // Register 0x4E hex 78 dec
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "78";
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "0";
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5]= "\nAction 13013: Set Neutralizer Bias Ref 0\n";
			exeFunc_index++;


			// Wait Between SET requests
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
			exeFunc_index++;


			//27 Action 13014: Set Neutralizer Mode 0 // Register 0x4B hex 75 dec
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "75"; //Neutralizer Mode
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "0";
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5]= "\nAction 13014: Set Neutralizer Mode 0\n";
			//exeFunc_index++;


			L4_HARDCODED_SEQUENCES[sequence_id_int].length = exeFunc_index; // MANUALLY DEFINE LENGTH OF SEQUENCE //
			L4_HARDCODED_SEQUENCES[sequence_id_int].sequence_trigger = false;
			L4_HARDCODED_SEQUENCES[sequence_id_int].repeat = false;
			L4_HARDCODED_SEQUENCES[sequence_id_int].substage_index = 0; //DEFAULT SUBSTAGE INDEX
			exeFunc_index= 0;
			*/

}



//////////////****************************** EXPERIMENTAL FUNCS ///////////////////////



void l4_GeneralSetRequest(l4_stage_arguments_t *stage_args){

	uint8_t len; // will carry total length of request array

	uint8_t access_register = stage_args->register_index;

	uint8_t length_of_register = REGISTER_LENGTH[access_register];

	//Check if valid/existing register is accessed
	if(length_of_register ==0){
		return;
		// It means that SET request is attempted with mismatch to actual origin of data register
		// Writing to wrong register position should be avoided.
		// For now lets assume that user never attempts to set wrong register.
		// TODO: Validate is register are allowed to be written/set to. (Based on enpulsion documentation)
	}

	//Request array initialisation
	if(length_of_register ==1){
		// if length of register is 1 byte then total length of request array is
		len=8;
		}

	if(length_of_register ==2){
		len=9;
		}

	if(length_of_register ==4){
		len=11;
		}


	uint8_t request[len];


	//Message header
	request[0] = SENDER_ADRESS;
	request[1] = DEVICE;
	request[2] = MSGTYPE[3]; // WRITE -3
	request[3] = 0x00; //checksum


	//////Payload length
	if (length_of_register ==1){
		// if register that is intended to be set is one byte
		// then length of payload is register address(1) + data(1)= 2 bytes
		request[4] = 2; //
	}

	if (length_of_register ==2){
		//if length of register is 2 bytes then
		//length of payload is address(1)+data(2) = 3 bytes
		request[4] =3;

	}

	if (length_of_register ==4){
			/// 4 bytes only for FUSE REGISTERS
			request[4] =5;

		}


	request[5] = 0x00;
	request[6]= access_register;



	if (length_of_register ==1){
		uint8_t input_uint8 = (uint8_t) stage_args->double_arg1;
		input_uint8 = input_uint8* (uint8_t) CONVERSION_DOUBLE[access_register];
		request[7]= input_uint8;

	}

	if (length_of_register ==2){

		// parse argument as double
		// WARNING : Set Project-Settings-Manager linker script - Redlib (nohost) to use atof()
		double input = stage_args->double_arg1;
		// in cases where data stored in register has length of 2 bytes. uint16_t would be used
		// to store this data in register. For some registers input may be float
		// example 3.3V  or 3.0 or 3
		// Function should be able to handle both float and integer inputs from user

		// Apply conversion multiplier
		input = input* CONVERSION_DOUBLE[access_register];
		// Convert input to uint16_t
		uint16_t value = (uint16_t) input;
		// Represent uint16_t as two uint8_t bytes to fill the request array.
		request[7]= value & 0xff;
		request[8] = (value >> 8) & 0xff;
	}

	request[3] = CRC8(request, len); // calculate checksum after whole request array is sent
	l4_thr_ExpectedReceiveBuffer = 6;// change expected receive buffer accordingly
	thrSendBytes(request, len);
}


void l4_ReadAllRegisters(l4_stage_arguments_t *stage_args){
	uint8_t request[8];


	request[0]= 0x00;
	request[1]= 0xFF;
	request[2]= 0x03;
	request[3]= 0x69;
	request[4]= 0x02;
	request[5]= 0x00;
	request[6]= 0x00;
	request[7]= 0x7F;

	int len = sizeof(request);

	// every request function should manually set expected RX buffer size and reset byte counter !!!!!!!!!!
	l4_thr_ExpectedReceiveBuffer = 133;
	l4_thr_counter =0;

	thrSendBytes(request, len);
	LATEST_ACCESSED_REGISTER = 0;




}

void l4_ReadAllRegistersAndWait_sequence(l4_stage_arguments_t *stage_args){
	uint16_t procedure_id = stage_args->sequence_id;
	uint32_t duration = stage_args->wait;
	char print_str[200];
	uint32_t now_timestamp;
	//int len;




	switch (L4_HARDCODED_SEQUENCES[procedure_id].substage_index){

	case 0:
		l4_ReadAllRegisters(stage_args);
		L4_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage = (uint32_t)timGetSystime();
		L4_HARDCODED_SEQUENCES[procedure_id].substage_index++;
		return;
		break;
	case 1:
		now_timestamp = (uint32_t)timGetSystime();
		if ( (now_timestamp - L4_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage) < duration ){
					// do nothing
				}
				else{
					sprintf(print_str, "\n Read all registers and wait, Stage= %d Complete t = %d\n", L4_HARDCODED_SEQUENCES[procedure_id].execution_index,now_timestamp);
					//len = strlen(print_str);
					//deb_print_pure_debug((uint8_t *)print_str, len);
					SysEvent(0, EVENT_INFO, 3, (uint8_t *)print_str, strlen(print_str));

					L4_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage = (uint32_t)timGetSystime();
					L4_HARDCODED_SEQUENCES[procedure_id].substage_index = 0;
					L4_HARDCODED_SEQUENCES[procedure_id].execution_index++; // increase sequence execution index so that after wait - next module to be executed


					//len = strlen(argv[5]);
					//deb_print_pure_debug((uint8_t *)argv[5], len); // assume that argv[5] is custom print message
				}
		return;
		break;


	}






}


//General read request to any register
void l4_GeneralReadRequest(l4_stage_arguments_t *stage_args){
		// FIRST ARGUMENT SHOUD BE uint8_t VALUE OF REGISTER THAT WOULD BE READ FROM
		uint8_t access_register = stage_args->register_index;
		uint8_t length_of_register = REGISTER_LENGTH[access_register];

		// TODO : Check if valid/existing register is accessed
		if(length_of_register ==0){
			return;
		}

		uint8_t request[8];
		request[0] = SENDER_ADRESS;
		request[1] = DEVICE;
		request[2] = MSGTYPE[2]; // READ -3
		request[3] = 0x00; // checksum
		request[4] = 0x02; // LENGTH of payload REGISTER and length
		request[5] = 0x00; // Hardcoded because length of payload is defined with two bytes of uint16
		request[6]= access_register; // Access register at address requested by used
		request[7]= length_of_register;

		uint8_t len = sizeof(request);
		request[3] = CRC8(request, len);

		// Reply is n bytes long.
		//Therefore we set global variable that should be used to process the RX buffer to corresponding length.
		l4_thr_ExpectedReceiveBuffer = 6+REGISTER_LENGTH[access_register];
		l4_thr_counter =0;

		thrSendBytes(request, len);
		TYPE_OF_LAST_REQUEST = 0x03;
		LATEST_ACCESSED_REGISTER = access_register;
}
