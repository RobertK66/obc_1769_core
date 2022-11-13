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


void thr_wait(int argc, char *argv[]);
void GeneralSetRequest_sequence(int argc, char *argv[]);
void GeneralReadRequest_sequence(int argc, char *argv[]);
void thr_execute_sequence();
void thr_void(int argc, char *argv[]);
void thr_value_ramp(int argc, char *argv[]);
void thr_wait_and_monitor(int argc, char *argv[]);

void initialize_hardcoded_thr_sequences();


//MEM
void thr_write_mem_callback(uint8_t chipIdx, mram_res_t result, uint32_t adr, uint8_t *data, uint32_t len);
void thr_write_mem();
void thr_read_mem();
void thr_read_mem_callback(uint8_t chipIdx, mram_res_t result, uint32_t adr, uint8_t *data, uint32_t len);
uint8_t MMRAM_READ_BUFFER[6];

#define MAX_EXECUTION_SEQUENCE_DEPTH 50 // Maximum size of execution sequence stack
#define MAX_HARDCODED_SEQUENCES 8 // Maximum number of preprogrammed sequences

typedef struct {
	char *thr_argv[6];
	void (*function)(int argc, char *argv[]);
	uint16_t procedure_id;
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

}thr_hardcoded_sequences_t;
thr_hardcoded_sequences_t THR_HARDCODED_SEQUENCES[MAX_HARDCODED_SEQUENCES];






// TODO: Fill the array with REGISTER_NAMES so that logging messages will contain name of accessed register allong with  [register_index]
//const char* REGISTER_NAME[5] = { "0 Firmware Version (major)","1 Firmware Version (minor)","2 Serial Number","4 None","5 Reset Cycle"	};








void l4_thruster_init (void *dummy) {

	initialize_hardcoded_thr_sequences();



}

void l4_thruster_main (void) {
	LAST_STARTED_MODULE = 11;




	for (int i=0;i<=5;i++){ // for all preprogrammed sequences

		if (THR_HARDCODED_SEQUENCES[i].sequence_trigger ){ //if trigger for sequence is set to True - execute sequence
			thr_execute_sequence(i);

		}

	}


}










void GeneralSetRequest_sequence(int argc, char *argv[]){
	LAST_STARTED_MODULE = 1104;
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
	uint16_t procedure_id = atoi(argv[0]); // procedure_id is always fist index of argument array
	GeneralSetRequest(3,argv);

	char print_str[200];
	sprintf(print_str, "\nStage SET index= %d completed\n",THR_HARDCODED_SEQUENCES[procedure_id].execution_index);
	int len = strlen(print_str);
	deb_print_pure_debug((uint8_t *)print_str, len);

	//*******assume that argv[5] is custom print message
	len = strlen(argv[5]);
	deb_print_pure_debug((uint8_t *)argv[5], len);


	THR_HARDCODED_SEQUENCES[procedure_id].execution_index++;
	THR_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage = (uint32_t)timGetSystime();


}





void GeneralReadRequest_sequence(int argc, char *argv[]){
	LAST_STARTED_MODULE = 1106;
	uint16_t procedure_id = atoi(argv[0]); // procedure_id is always fist index of argument array
	GeneralReadRequest(3,argv);

	char print_str[200];
	sprintf(print_str, "\nStage READ index= %d completed\n",THR_HARDCODED_SEQUENCES[procedure_id].execution_index);
	int len = strlen(print_str);
	deb_print_pure_debug((uint8_t *)print_str, len);

	len = strlen(argv[5]);
	deb_print_pure_debug((uint8_t *)argv[5], len); //assume that argv[5] is custom print message


	THR_HARDCODED_SEQUENCES[procedure_id].execution_index++;
	THR_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage = (uint32_t)timGetSystime();

}






//////
void thr_wait(int argc, char *argv[]){
	LAST_STARTED_MODULE = 1108;

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
	uint16_t procedure_id = atoi(argv[0]); // procedure_id is always fist index of argument array
	uint32_t duration = atoi(argv[1]);
	uint32_t now_timestamp = (uint32_t)timGetSystime();


	char print_str[200];
	int len;

	if ( (now_timestamp - THR_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage) < duration ){
		// do nothing
	}
	else{
		THR_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage = (uint32_t)timGetSystime(); //save execution finish time of wait stage
		sprintf(print_str, "\nWait Stage= %d Complete t = %d\n", THR_HARDCODED_SEQUENCES[procedure_id].execution_index,now_timestamp);
		len = strlen(print_str);
		deb_print_pure_debug((uint8_t *)print_str, len);
		THR_HARDCODED_SEQUENCES[procedure_id].execution_index++; // increase sequence execution index so that after wait - next module to be executed


		len = strlen(argv[5]);
		deb_print_pure_debug((uint8_t *)argv[5], len); // assume that argv[5] is custom print message
	}

}


void thr_wait_and_monitor(int argc, char *argv[]){
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


	LAST_STARTED_MODULE = 1108;
	uint16_t procedure_id = atoi(argv[0]); // procedure_id is always fist index of argument array
	uint32_t duration = atoi(argv[1]);
	uint32_t logging_dt = atoi(argv[2]); //Time after which READ request is sent
	uint32_t now_timestamp = (uint32_t)timGetSystime();
	uint8_t register_index = atoi(argv[3]); // Register that is intended to be monitored
	char print_str[200];
	int len;
	char *temp_argv[2];
	int waiting_between_logging;
	char argument1[50];

	uint32_t wait_for_read_request_proccess = 3000; // WARNING ! Wait time for processing request [ms] should not be equal to logging dt [ms]
	// Otherwise function never exits


	switch (THR_HARDCODED_SEQUENCES[procedure_id].substage_index){

	case 0:

		if ( ((now_timestamp - THR_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage) % logging_dt ) == 0 ){
				waiting_between_logging = (int)( (now_timestamp - THR_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage)/1000 );

				sprintf(print_str, "\n Waiting %d s \n",waiting_between_logging  );
				len = strlen(print_str);
				deb_print_pure_debug((uint8_t *)print_str, len);

				THR_HARDCODED_SEQUENCES[procedure_id].substage_index++; // increase substage to procede with READ request
				THR_HARDCODED_SEQUENCES[procedure_id].sequence_execution_substage = (uint32_t)timGetSystime(); // save timestamp of substage completion
				break;

					}



		if ( (now_timestamp - THR_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage) < duration ){
				// do nothing
			}
		else{
				THR_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage = (uint32_t)timGetSystime(); //save execution finish time of wait stage

				sprintf(print_str, "\nWait and Monitor Stage = %d Complete t = %d\n", THR_HARDCODED_SEQUENCES[procedure_id].execution_index,now_timestamp);
				len = strlen(print_str);
				deb_print_pure_debug((uint8_t *)print_str, len);
				THR_HARDCODED_SEQUENCES[procedure_id].execution_index++; // increase sequence execution index so that after wait - next module to be executed
				THR_HARDCODED_SEQUENCES[procedure_id].substage_index = 0;

				// assume that argv[5] is custom print message
				len = strlen(argv[5]);
				deb_print_pure_debug((uint8_t *)argv[5], len);
			}




		break;
	case 1:
		sprintf(print_str, "\n Sending READ [%d] Request %d\n",register_index, now_timestamp );
		len = strlen(print_str);
		deb_print_pure_debug((uint8_t *)print_str, len);

		temp_argv[0]= "6";

		sprintf(argument1, "%d",register_index);
		temp_argv[1]= argument1;

		GeneralReadRequest(2, temp_argv);

		THR_HARDCODED_SEQUENCES[procedure_id].substage_index ++;
		THR_HARDCODED_SEQUENCES[procedure_id].sequence_execution_substage = (uint32_t)timGetSystime();

		break;
	case 2:

		if ( (now_timestamp - THR_HARDCODED_SEQUENCES[procedure_id].sequence_execution_substage) < wait_for_read_request_proccess ){ // Wait to proccess READ request
						// do nothing
					}
		else{
			//THR_HARDCODED_SEQUENCES[procedure_id].sequence_execution_substage = (uint32_t)timGetSystime(); //save execution finish time of wait substage

			//sprintf(print_str, "\nMonitored [%d] Value = %.6f \n", register_index,REGISTER_DATA[register_index]);
			//len = strlen(print_str);
			//deb_print_pure_debug((uint8_t *)print_str, len);

			sprintf(print_str, "\nMonitored 2 [%d] Value = %.6f \n", register_index,ReadThrRegData(register_index));
			len = strlen(print_str);
			deb_print_pure_debug((uint8_t *)print_str, len);

			THR_HARDCODED_SEQUENCES[procedure_id].substage_index = 0;
			}


		break;

	}






}


void thr_void(int argc, char *argv[]){
	LAST_STARTED_MODULE = 1109;

	/*
	 * Incorporates with sequence execution
	 *
	 * void function that does nothing
	 */

	uint16_t procedure_id = atoi(argv[0]); // procedure_id is always fist index of argument array
	THR_HARDCODED_SEQUENCES[procedure_id].execution_index ++;

}

void thr_value_ramp(int argc, char *argv[]){
	/*
	 * According to enpulsion defination RAMP Frequency should be 1 Hz, Duration of ramp is 30s
	 * Out function allows to specify different dt(frequency) and ramp duration.
	 * When hardcoding sequences - lets stick with enpulsion ramp requirments
	 */

	uint16_t procedure_id = atoi(argv[0]); // procedure_id is always fist index of argument array
	uint8_t register_index = atoi(argv[1]); // first argument is register index at which SET ramp would be implemented
	double goal = atof(argv[2]); // goal to which value should be set
	uint32_t ramp_duration = atoi(argv[3]); // time through which value should be changed from initial to goal
	uint32_t ramp_dt = atoi(argv[4]); // wait between SET / Converted to (ms)
	double initial_value;
	uint32_t now_timestamp;

	char print_str[200];
	int len;

	double ramp_iterations = ramp_duration/ ramp_dt; // WARING - HARDCODE IN A WAY THAT THIS IS ALWAYS INT
	double value_step;

	char *temp_argv[3];
	char argument2[50];
	char argument3[50]; //7 //length of array should always be more then maximum characters for set value !!!!!



	switch (THR_HARDCODED_SEQUENCES[procedure_id].substage_index){

	case 0:// first we make read request to desired register in order to obtain initial register value
		//GeneralReadRequest(argc,argv); // make sure that argv[1] is register index
		ReadAllRegisters(argc,argv);

		sprintf(print_str, "\nWInitial read Request Sent\n");
		len = strlen(print_str);
		deb_print_pure_debug((uint8_t *)print_str, len);

		// assume that argv[5] is custom print message
		len = strlen(argv[5]);
		deb_print_pure_debug((uint8_t *)argv[5], len);

		THR_HARDCODED_SEQUENCES[procedure_id].substage_index++;
		break;
	case 1: // wait for some time while process request is executing
		now_timestamp = (uint32_t)timGetSystime();
		if ( (now_timestamp - THR_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage) <= 5000 ){
					// do nothing
			return;

				}
		else if ( (now_timestamp - THR_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage) > 5000 ){
			THR_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage = (uint32_t)timGetSystime(); //save execution finish time of wait stage
			THR_HARDCODED_SEQUENCES[procedure_id].substage_index++;
			return;

		}
		break;
	case 2: // now we are sure that read request was proccessed - read initial value
		//THR_HARDCODED_SEQUENCES[procedure_id].ramp_initial_value = REGISTER_DATA[register_index];
		THR_HARDCODED_SEQUENCES[procedure_id].ramp_initial_value = ReadThrRegData(register_index);
		THR_HARDCODED_SEQUENCES[procedure_id].substage_index++;
		sprintf(print_str, "\nInitial value save value = %.5f\n",THR_HARDCODED_SEQUENCES[procedure_id].ramp_initial_value);
		len = strlen(print_str);
		deb_print_pure_debug((uint8_t *)print_str, len);
		break;
	case 3:

		initial_value  = THR_HARDCODED_SEQUENCES[procedure_id].ramp_initial_value;
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
			len = strlen(print_str);
			deb_print_pure_debug((uint8_t *)print_str, len);


			temp_argv[0]= "7";

			sprintf(argument2, "%d",register_index);
			temp_argv[1]= argument2;

			sprintf(argument3, "%.2f",goal);
			temp_argv[2]= argument3;

			if (register_index == 16){sprintf(argument3, "%.5f",goal); temp_argv[2]= argument3;     } // for Thrust ramp we need to print goal with 5 decimal precesion

			//sprintf(temp_argv[2], "%.2f",REGISTER_DATA[register_index]+value_step);
			GeneralSetRequest(3, temp_argv);
			//REGISTER_DATA[register_index] = goal;
			SetEncodedThrRegValue(goal, register_index); //Todo - check if it is stupid. Instead of assignment operation we run function ......

			//sprintf(print_str, "\nValue corrected according to goal %.5f \n",REGISTER_DATA[register_index]);
			sprintf(print_str, "\nValue corrected according to goal %.5f \n",ReadThrRegData(register_index)); //TODO remove
			len = strlen(print_str);
			deb_print_pure_debug((uint8_t *)print_str, len);

			THR_HARDCODED_SEQUENCES[procedure_id].substage_index++;
			THR_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage = (uint32_t)timGetSystime();
			break;
		}



		sprintf(print_str, "\nRestore saved initial value = %.5f\n",initial_value );
		len = strlen(print_str);
		deb_print_pure_debug((uint8_t *)print_str, len);


		sprintf(print_str, "\nSTEP = %.2f\n",value_step);
		if (register_index == 16){sprintf(print_str, "\nSTEP = %.7f\n",value_step);    } // for Thrust ramp we need to print goal with 5 decimal precession
		len = strlen(print_str);
		deb_print_pure_debug((uint8_t *)print_str, len);


		temp_argv[0]= "7";

		sprintf(argument2, "%d",register_index);
		temp_argv[1]= argument2;

		//sprintf(argument3, "%.2f",goal);
		//temp_argv[2]= argument3;

		//sprintf(argument3, "%.2f",REGISTER_DATA[register_index]+value_step);
		sprintf(argument3, "%.2f",ReadThrRegData(register_index)+value_step);
		temp_argv[2]= argument3;

		//if (register_index == 16){sprintf(argument3, "%.5f",REGISTER_DATA[register_index]+value_step); temp_argv[2]= argument3;     } // for Thrust ramp we need to print goal with 5 decimal precesion
		if (register_index == 16){sprintf(argument3, "%.5f",ReadThrRegData(register_index)+value_step); temp_argv[2]= argument3;     } // for Thrust ramp we need to print goal with 5 decimal precesion

		//sprintf(temp_argv[2], "%.2f",REGISTER_DATA[register_index]+value_step); // this also works

		//sprintf(print_str, "\nSET RAMP value = %.5f\n",REGISTER_DATA[register_index]);
		sprintf(print_str, "\nSET RAMP value = %.5f\n",ReadThrRegData(register_index));
		len = strlen(print_str);
		deb_print_pure_debug((uint8_t *)print_str, len);

		GeneralSetRequest(3, temp_argv);

		THR_HARDCODED_SEQUENCES[procedure_id].substage_index++;
		THR_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage = (uint32_t)timGetSystime();

		break;
	case 4: // WAIT after RAMP set request done
		now_timestamp = (uint32_t)timGetSystime();
		if ( (now_timestamp - THR_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage) < ramp_dt*1000){ // ramp_dt*1000 converts dt from [s] to [ms]
			// do nothing
		}
		else{
			THR_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage = (uint32_t)timGetSystime(); //save execution finish time of wait stage
			THR_HARDCODED_SEQUENCES[procedure_id].substage_index++;
			sprintf(print_str, "\nWaiting after SET RAMP\n");
			len = strlen(print_str);
			deb_print_pure_debug((uint8_t *)print_str, len);
		}

		break;

	case 5:

		// EXIT CONDITIONS

		initial_value  = THR_HARDCODED_SEQUENCES[procedure_id].ramp_initial_value;
		value_step = (goal -initial_value)/ramp_iterations;

		if (value_step > 0){
			//if (REGISTER_DATA[register_index] >= goal){
			if (ReadThrRegData(register_index) >= goal*0.99){ // more then 99% of goal - WARNING THIS IS DANGEROUS

				// GOAL REACHED// EXIT FUNCTION
				THR_HARDCODED_SEQUENCES[procedure_id].execution_index ++;
				THR_HARDCODED_SEQUENCES[procedure_id].substage_index = 0;

			}
			else{
			 	 // IF GOAL VALUE NOT YET REACHED JUMP BACK TO SET REQUEST
				THR_HARDCODED_SEQUENCES[procedure_id].substage_index=3;
				THR_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage = (uint32_t)timGetSystime();
			}
		}

		if(value_step <0){

			//if (REGISTER_DATA[register_index] <= goal){
			if (ReadThrRegData(register_index) <= goal*1.01){ //goal*1.01 (with 101% error) this is dangerous !
							// GOAL REACHED// EXIT FUNCTION
							THR_HARDCODED_SEQUENCES[procedure_id].execution_index ++;
							THR_HARDCODED_SEQUENCES[procedure_id].substage_index = 0;

						}
						else{
						 	 // IF GOAL VALUE NOT YET REACHED JUMP BACK TO SET REQUEST
							THR_HARDCODED_SEQUENCES[procedure_id].substage_index=3;
							THR_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage = (uint32_t)timGetSystime();
						}



		}
		if(value_step == 0){

			//warning unexpected.
			THR_HARDCODED_SEQUENCES[procedure_id].execution_index ++;
			THR_HARDCODED_SEQUENCES[procedure_id].substage_index = 0;
			sprintf(print_str, "\nExit condition valuestep=0\n");
			len = strlen(print_str);
			deb_print_pure_debug((uint8_t *)print_str, len);

		}

		break;




	}







}

void thr_execute_sequence_cmd(int argc, char *argv[]){
	LAST_STARTED_MODULE = 1110;
	/*
	 * This function is meant to trigger execution of SEQUENCE
	 * Triggered function is thr_execute_sequence()

	*/
	uint8_t procedure_id = atoi(argv[1]);
	uint8_t action = atoi(argv[2]);
	char print_str[200];
	int len;

	switch(action){

	case 0: // start sequence

		THR_HARDCODED_SEQUENCES[procedure_id].sequence_trigger = true;
		THR_HARDCODED_SEQUENCES[procedure_id].execution_index = 0;
		THR_HARDCODED_SEQUENCES[procedure_id].sequence_execution_begin = (uint32_t)timGetSystime();
		THR_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage = (uint32_t)timGetSystime();
		THR_HARDCODED_SEQUENCES[procedure_id].substage_index = 0;
		sprintf(print_str, "\nSequence id=%d start\n",procedure_id);
		len = strlen(print_str);
		deb_print_pure_debug((uint8_t *)print_str, len);
		break;
	case 1: // restart sequence
		THR_HARDCODED_SEQUENCES[procedure_id].restart = true;
		sprintf(print_str, "\nSequence id=%d will now restart start\n",procedure_id);
		len = strlen(print_str);
		deb_print_pure_debug((uint8_t *)print_str, len);
		break;
	case 2: //repeat sequence
		THR_HARDCODED_SEQUENCES[procedure_id].repeat = true;
		sprintf(print_str, "\nSequence id=%d will now repeat\n",procedure_id);
		len = strlen(print_str);
		deb_print_pure_debug((uint8_t *)print_str, len);
		break;
	case 3: //cancel repeat sequence
			THR_HARDCODED_SEQUENCES[procedure_id].repeat = false;
			sprintf(print_str, "\nSequence id=%d stop repeat\n",procedure_id);
			len = strlen(print_str);
			deb_print_pure_debug((uint8_t *)print_str, len);
			break;
	case 4: //Pause sequence
			THR_HARDCODED_SEQUENCES[procedure_id].pause = true;
			THR_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage = (uint32_t)timGetSystime();
			sprintf(print_str, "\nSequence id=%d paused\n",procedure_id);
			len = strlen(print_str);
			deb_print_pure_debug((uint8_t *)print_str, len);
			break;
	case 5: //resume sequence
			THR_HARDCODED_SEQUENCES[procedure_id].pause = false;
			THR_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage = (uint32_t)timGetSystime();
			sprintf(print_str, "\nSequence id=%d resumed\n",procedure_id);
			len = strlen(print_str);
			deb_print_pure_debug((uint8_t *)print_str, len);
			break;
	case 6: //Stop sequence
			THR_HARDCODED_SEQUENCES[procedure_id].sequence_trigger = false;
			THR_HARDCODED_SEQUENCES[procedure_id].execution_index = 0;
			sprintf(print_str, "\nSequence id=%d manual stop\n",procedure_id);
			len = strlen(print_str);
			deb_print_pure_debug((uint8_t *)print_str, len);
			break;
	}

	//THR_HARDCODED_SEQUENCES[procedure_id].sequence_trigger = true;
	//THR_HARDCODED_SEQUENCES[procedure_id].execution_index = 0;
	//THR_HARDCODED_SEQUENCES[procedure_id].sequence_execution_begin = (uint32_t)timGetSystime();
	//THR_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage = (uint32_t)timGetSystime();

	//sprintf(print_str, "\nSequence id=%d start\n",procedure_id);
	//len = strlen(print_str);
	//deb_print_pure_debug((uint8_t *)print_str, len);



	//*** THIS IS TEST EXAMPLE TO DEMONSTRATE THAT IT IS POSSIBLE TO CHANGE EXECUTION FUNCTION AND ARGUMENTS IN A SEQUENCE
	//char temp_arg[1];
	//sprintf(temp_arg, "%d",procedure_id);
	//THR_HARDCODED_SEQUENCES[procedure_id].sequences[0].thr_argv[0] = temp_arg;
	//THR_HARDCODED_SEQUENCES[procedure_id].sequences[0].thr_argv[1] = "20";
	//THR_HARDCODED_SEQUENCES[procedure_id].sequences[0].thr_argv[2] = "3000";
	//THR_HARDCODED_SEQUENCES[procedure_id].sequences[0].function = GeneralSetRequest_sequence;
	 //***********************************************************************




}



void thr_execute_sequence(int procedure_id){
	LAST_STARTED_MODULE = 1111;
	/*
	 *
	 * Executes preprogrammed sequence  defined in THR_EXECUTION_SEQUENCE function pointer array
	 * THR_SEQUENCES array of argv to be input into THR_EXECUTION_SEQUENCE
	 */

	if(THR_HARDCODED_SEQUENCES[procedure_id].pause){
		// if sequence pause set to true
		// do nothing
		return;
		}

	if(THR_HARDCODED_SEQUENCES[procedure_id].restart){
					// Restart sequence
					THR_HARDCODED_SEQUENCES[procedure_id].sequence_trigger = true;
					THR_HARDCODED_SEQUENCES[procedure_id].execution_index =0; // reset execution index to 0
					THR_HARDCODED_SEQUENCES[procedure_id].restart = false;
					return;
			}

	if (THR_HARDCODED_SEQUENCES[procedure_id].execution_index > THR_HARDCODED_SEQUENCES[procedure_id].length){
		char print_str[200];
		sprintf(print_str, "\nSequence complete\n");
		int len = strlen(print_str);
		deb_print_pure_debug((uint8_t *)print_str, len);

		if(THR_HARDCODED_SEQUENCES[procedure_id].repeat){
			sprintf(print_str, "\nRepeat sequence\n");
			len = strlen(print_str);
			deb_print_pure_debug((uint8_t *)print_str, len);
			THR_HARDCODED_SEQUENCES[procedure_id].sequence_trigger = true;
			THR_HARDCODED_SEQUENCES[procedure_id].execution_index =0;
		}
		else{
			THR_HARDCODED_SEQUENCES[procedure_id].sequence_trigger = false;
			THR_HARDCODED_SEQUENCES[procedure_id].execution_index =0;
			return;

		}


	}

	else{
		void (*f)(int argc, char *argv[]);
		f = THR_HARDCODED_SEQUENCES[procedure_id].sequences[THR_HARDCODED_SEQUENCES[procedure_id].execution_index].function;
		LAST_STARTED_MODULE = THR_HARDCODED_SEQUENCES[procedure_id].execution_index; //DEBUG
		f(3,THR_HARDCODED_SEQUENCES[procedure_id].sequences[THR_HARDCODED_SEQUENCES[procedure_id].execution_index].thr_argv);
	}





}


//****************** THIS IS JUST TEST AND TRIAL********************************

void thr_write_mem(){
	LAST_STARTED_MODULE = 1112;
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
	LAST_STARTED_MODULE = 1113;

	if (result == MRAM_RES_SUCCESS) {
		char print_str[200];
		sprintf(print_str, "\nMemory Write Success\n");
		int len = strlen(print_str);
		deb_print_pure_debug((uint8_t *)print_str, len);

		thr_read_mem();



	} else {
		// TODO:?? retry counter / timeouts ....
	}
}



void thr_read_mem(){
	LAST_STARTED_MODULE = 1114;
	MramReadAsync(0, 5000, MMRAM_READ_BUFFER, sizeof(MMRAM_READ_BUFFER), thr_read_mem_callback);
	//void MramReadAsync(uint8_t chipIdx, uint32_t adr,  uint8_t *rx_data,  uint32_t len,
	//void (*finishedHandler)(uint8_t chipIdx,mram_res_t result, uint32_t adr, uint8_t *data, uint32_t len))
}

void thr_read_mem_callback(uint8_t chipIdx, mram_res_t result, uint32_t adr, uint8_t *data, uint32_t len) {
	LAST_STARTED_MODULE = 1115;

	if (result == MRAM_RES_SUCCESS) {
		char print_str[200];
		sprintf(print_str, "\nMemory Read Success\n");
		int len = strlen(print_str);
		deb_print_pure_debug((uint8_t *)print_str, len);

		len = strlen((char*)data);
		deb_print_pure_debug(data, len);

		sprintf(print_str, "\n-----msg_end--------\n");
		len = strlen(print_str);
		deb_print_pure_debug((uint8_t *)print_str, len);



	} else {
		// TODO:?? retry counter / timeouts ....
	}
}


void mem_write_cmd(int argc, char *argv[]){

	thr_write_mem(); //debug test

}
// ******************************************************************************



void initialize_hardcoded_thr_sequences(){

	/// **************** PREPROGRAMM SEQUENCES HERE ****************
		uint8_t exeFunc_index; // this is helper index to simplify HARDCODDING sequence manually.
		int sequence_id_int;
		char *wait_between_stages_str ="5000";
		char* sequenc_id_char;


		// ******* SEQUENCE 0 ************* OPERATIONAL SCRIPTS /Hot Standby Script / 10-1

		exeFunc_index=0; // at the beggining of sequence hardcodding set it to 0
		sequenc_id_char = "0";
		sequence_id_int = 0;
		wait_between_stages_str = "2000";
		//sprintf(sequenc_id_char, "%d",sequence_id_int); // for this to work sequence_id_char[50] have to be initialized as array instead of pointer

		// Action 31001:Set Operational Mode 0 / Register 0x0E  14
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = "14";
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2] = "0";
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nAction 31001:Set Operational Mode 0\n";
		exeFunc_index++;


		// Wait Between SET requests
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
		exeFunc_index++;

		// Action 31002: Set Emitter Mode 0 / Register 0x1E  30
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = "30";
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2] = "0";
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nAction 31002: Set Emitter Mode 0\n";
		exeFunc_index++;

		// Wait Between SET requests
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
		exeFunc_index++;

		// Action 31003:Set Extractor Mode 0 / Register 0x2D  45
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = "45";
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2] = "0";
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nAction 31003:Set Extractor Mode 0\n";
		exeFunc_index++;

		// Wait Between SET requests
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
		exeFunc_index++;

		// Action 31004: Set Neutralizer Mode 0 / Register 0x4B  75
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = "75";
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2] = "0";
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nAction 31004: Set Neutralizer Mode 0\n";
		exeFunc_index++;


		// Wait Between SET requests
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
		exeFunc_index++;

		// Action 31005: Set Temperature Mode 0 / Register 0x60  96
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = "75";
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2] = "0";
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nAction 31005: Set Temperature Mode 0\n";
		exeFunc_index++;

		// Wait Between SET requests
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
		exeFunc_index++;

		// Action 31006: Set Reservoire Heater Mode 0 / Register 0x3C  60
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = "60";
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2] = "0";
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nAction 31006: Set Reservoire Heater Mode 0\n";
		exeFunc_index++;

		// Wait Between SET requests
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
		exeFunc_index++;

		// Action 31007: Set Thrust Ref 0 N / Register 0x10  16
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = "16";
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2] = "0";
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nAction 31007: Set Thrust Ref 0 N\n";
		exeFunc_index++;


		// Wait Between SET requests
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
		exeFunc_index++;

		// Action 31008: Set Reservoir Heater Power Ref 6 W / Register 0x45  69
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = "69";
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2] = "6";
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nAction 31008: Set Reservoir Heater Power Ref 6 W\n";
		exeFunc_index++;


		// Wait Between SET requests
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
		exeFunc_index++;

		// Action 31009: Set Operational Mode 1 / Register 0x0E  14
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = "14";
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2] = "1";
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nAction 31008: Reservoir Heater Power Ref 6 W\n";
		exeFunc_index++;

		// Action 31010: Wait AND MONITOR / Register (Reservoir Temperature) 0x63 99
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait_and_monitor;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = "3600000"; // Total wait duration [ms] 2.5h = 9 000 000ms
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2] = "10000"; // dt between READ request for Reservoir temperature
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[3] = "97"; // Register that is intended to be monitored Reservoir temp Ref
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nAction 31010: Wait AND MONITOR\n";
		//exeFunc_index++;



		THR_HARDCODED_SEQUENCES[sequence_id_int].length = exeFunc_index; // MANUALLY DEFINE LENGTH OF SEQUENCE //
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequence_trigger = false;
		THR_HARDCODED_SEQUENCES[sequence_id_int].repeat = false;
		THR_HARDCODED_SEQUENCES[sequence_id_int].substage_index = 0; //DEFAULT SUBSTAGE INDEX
		exeFunc_index= 0;




		// ******* SEQUENCE 2 ************* Operational Scripts / Thrust Maneuver /10-3

		exeFunc_index=0; // at the beggining of sequence hardcodding set it to 0
		sequenc_id_char = "1";
		sequence_id_int = 1;
		//sprintf(sequenc_id_char, "%d",sequence_id_int);

		// Action 31301: Set Extractor Mode 0 / Register 0x2D  45
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = "45";
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2] = "0";
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nAction 31301: Set Extractor Mode 0\n";
		exeFunc_index++;


		// Wait Between SET requests
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
		//exeFunc_index++;

		// Action 31302: Set Emitter Mode 0 / Register 0x1E  30
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = "30";
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2] = "0";
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nAction 31302: Set Emitter Mode 0\n";
		exeFunc_index++;



		// Wait Between SET requests
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
		exeFunc_index++;


		// Action 31303: Set Neutralizer Mode 0 / Register 0x4B  75
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = "75";
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2] = "0";
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nAction 31304: Set Neutralizer Mode 0\n";
		exeFunc_index++;

		// Wait Between SET requests
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
		exeFunc_index++;

		// Action 31304: Set Emitter Power Ref 30 W / Register 0x27  39
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = "39";
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2] = "30"; // [W]
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nAction 31304: Set Emitter Power Ref 30 W\n";
		exeFunc_index++;

		// Wait Between SET requests
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
		exeFunc_index++;



		//5 Action 31305: Set Neutralizer Filament Ref 1/2 // Register 0x4d 77
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "77";
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "1";
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5]= "\nAction 31305: Set Neutralizer Filament Ref 1\n";
		exeFunc_index++;



		// Wait Between SET requests
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
		exeFunc_index++;

		//5 Action 31306: Set Specific Impulse Ref 3000 s // Register 0x14 20
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "20";
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "3000";
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5]= "\nAction 31306: Set Specific Impulse Ref 3000 s\n";
		exeFunc_index++;



		// Wait Between SET requests
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
		exeFunc_index++;




		//17 Action 31307: Thrust Ref ramp (30s,1Hz) from INITIAL VALUE to 300 microN // Register 0x10 16 dec
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_value_ramp;  // RAMP DOWN
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "16"; // Neutralizer Heater Power Ref register
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "0.0003"; // GOAL of RAMP - 300 microN = 0.3 mN = 0.0003N
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[3]= "30"; // ramp duration 30s
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[4]= "2"; // 2 Secons between set requests = 0.5Hz
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5]= "\nAction 13009: Neutralizer Heater Power Ref ramp (30s,1Hz) from 2W to 0W\n ";
		exeFunc_index++;

		// Wait Between SET requests
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
		exeFunc_index++;


		// Action 31308: Wait AND MONITOR Thrust/ Register 0x12  18
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait_and_monitor;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = "60000"; // Total wait duration [ms] 2.5h = 9 000 000ms
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2] = "10000"; // dt between READ request
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[3] = "18"; // Register that is intended to be monitored
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nAction 31308: Wait AND MONITOR Thrust\n";
		exeFunc_index++;

		// Wait Between SET requests
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
		exeFunc_index++;

		//5 Action 31309: Set Thrust Ref 0 N // Register 0x10 16
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "16";
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "0";
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5]= "\nAction 31309: Set Thrust Ref 0 N\n";
		//exeFunc_index++;

		THR_HARDCODED_SEQUENCES[sequence_id_int].length = exeFunc_index; // MANUALLY DEFINE LENGTH OF SEQUENCE //
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequence_trigger = false;
		THR_HARDCODED_SEQUENCES[sequence_id_int].repeat = false;
		THR_HARDCODED_SEQUENCES[sequence_id_int].substage_index = 0; //DEFAULT SUBSTAGE INDEX
		exeFunc_index= 0;

		// ******* SEQUENCE 3 / Operational Scripts /Cooldown Script / 10-4

		exeFunc_index=0; // at the beggining of sequence hardcodding set it to 0
		sequenc_id_char = "2";
		sequence_id_int = 2;
		wait_between_stages_str = "2000";

		// Action 31401:Set Operational Mode 0 / Register 0x0E  14
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = "14";
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2] = "0";
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nAction 31401:Set Operational Mode 0\n";
		exeFunc_index++;


		// Wait Between SET requests
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
		exeFunc_index++;

		// Action 31402: Set Emitter Mode 0 / Register 0x1E  30
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = "30";
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2] = "0";
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nAction 31402: Set Emitter Mode 0\n";
		exeFunc_index++;

		// Wait Between SET requests
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
		exeFunc_index++;


		// Action 31404: Set Neutralizer Mode 0 / Register 0x4B  75
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = "75";
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2] = "0";
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nAction 31404: Set Neutralizer Mode 0\n";
		exeFunc_index++;



		// Wait Between SET requests
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
		exeFunc_index++;




		// Action 31404: Wait AND MONITOR Reservoir Temperature/ Register 0x63  99
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait_and_monitor;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = "3600000"; // Total wait duration [ms] 1h = 3600 000[ms]
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2] = "10000"; // dt between READ request
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[3] = "99"; // Register that is intended to be monitored
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nAction 31404: Wait AND MONITOR Reservoir Temperature\n";
		exeFunc_index++;


		THR_HARDCODED_SEQUENCES[sequence_id_int].length = exeFunc_index; // MANUALLY DEFINE LENGTH OF SEQUENCE //
		THR_HARDCODED_SEQUENCES[sequence_id_int].sequence_trigger = false;
		THR_HARDCODED_SEQUENCES[sequence_id_int].repeat = false;
		THR_HARDCODED_SEQUENCES[sequence_id_int].substage_index = 0; //DEFAULT SUBSTAGE INDEX
		exeFunc_index= 0;


		// ******   SEQUENCE 4  *******System Cold Test/ Neutralizer Bias Test/ Table 9-3

		exeFunc_index=0; // at the beggining of sequence hardcodding set it to 0
		sequenc_id_char = "3";
		sequence_id_int = 3;
		wait_between_stages_str = "2000";




			//1 Action 12001: Set Neutralizer Mode 0 // Register 0x4B hex 75 dec
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "75"; //Neutralizer Mode
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "0";
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5]= "\nAction 12001: Set Neutralizer Mode 0\n";
			exeFunc_index++;

			// Wait Between SET requests
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
			exeFunc_index++;

			//3 Action 12002: Set Neutralizer Bias Ref 1 // Register 0x4E hex 78 dec
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "78";
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "1";
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5]= "\nAction 12002: Set Neutralizer Bias Ref 1\n";
			exeFunc_index++;

			// Wait Between SET requests
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
			exeFunc_index++;

			//5 Action 12003: Set Neutralizer Mode 1 // Register 0x4B hex 75 dec
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "75"; //Neutralizer Mode
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "1";
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5]= "\nAction 12003: Set Neutralizer Mode 0\n";
			exeFunc_index++;



			//6 Action 12004: Wait for 60s
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "10000";
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5]= "\nAction 12004: Waiting 10s\n";
			exeFunc_index++;

			//7 wait
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "10000";
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5]= "\nAction 12004: Waiting 20s\n";
			exeFunc_index++;


			//8 wait
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "10000";
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5]= "\nAction 12004: Waiting 30s\n";
			exeFunc_index++;

			//9 wait
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "10000";
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5]= "\nAction 12004: Waiting 40s\n";
			exeFunc_index++;

			//10 wait
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "10000";
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5]= "\nAction 12004: Waiting 50s\n";
			exeFunc_index++;

			//11 wait
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "10000";
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5]= "\nAction 12004: Waiting 60s\n";
			exeFunc_index++;

					//12 Action 12005: Set Neutralizer Bias Ref 1 // Register 0x4E hex 78 dec
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "78";
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "0";
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5]= "\nAction 12005: Set Neutralizer Bias Ref 0\n";
			exeFunc_index++;


			//13 Wait between SET requests
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "2000";
			exeFunc_index++;


			//14 Action 12006: Set Neutralizer Mode 0 // Register 0x4B hex 75 dec
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "75"; //Neutralizer Mode
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "0";
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5]= "\nAction 12001: Set Neutralizer Mode 0\n";
			//exeFunc_index++;

			THR_HARDCODED_SEQUENCES[sequence_id_int].length = exeFunc_index; // MANUALLY DEFINE LENGTH OF SEQUENCE //
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequence_trigger = false;
			THR_HARDCODED_SEQUENCES[sequence_id_int].repeat = false;
			THR_HARDCODED_SEQUENCES[sequence_id_int].substage_index = 0; //DEFAULT SUBSTAGE INDEX
			exeFunc_index= 0;


			//////////// ******** SEQUENCE 3*************** SYSTEM COLD TEST/ Heater Ramp Test/ Table 9-2

			exeFunc_index=0; // at the beggining of sequence hardcodding set it to 0
			sequenc_id_char = "4";
			sequence_id_int = 4;
			wait_between_stages_str = "2000";

			//0 Action 11001: Set Heater Mode 0  / Register 0x3C hex   60 dec
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "60";
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "0";
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5]= "\nAction 11001: Heater Mode set 0 \n";
			exeFunc_index++;

			// Wait Between SET requests
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
			exeFunc_index++;

			//2 Action 1102: Set Heater Voltage Ref 12v / Register 0x3D 61 dec
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;  // Set request
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char; //procedure id HARDCODED
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "61"; // Reservoir Heater Voltage Ref register
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "12"; //  Set to 12V
			exeFunc_index++;


			// Wait Between SET requests
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
			exeFunc_index++;

			//4 Action 1103: Set Heater Current Ref 1.5A / Register 0x41 65 dec
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;  // Set request
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char; //procedure id HARDCODED
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "65"; // Reservoir Heater Current Ref register
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "1.5"; //  Set to 1.5 A
			exeFunc_index++;


			// Wait Between SET requests
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
			exeFunc_index++;

			//6 Action 1104: Set Heater Power Ref 0W / Register 0x45 69 dec
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;  // Set request
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char; //procedure id HARDCODED
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "69"; // Reservoir Heater Power Ref register
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "0"; //  Set to 0 W
			exeFunc_index++;

			// Wait Between SET requests
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
			exeFunc_index++;

			//8 Action 11005: Set Heater Mode 1  / Register 0x3C hex   60 dec
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char; // OBC sequence id
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "60"; // Reservoir heater mode register
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "1"; // Heater mode value

			// Wait Between SET requests
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
			exeFunc_index++;

			//10 Action 11006: Heater Power Ref ramp (30s,1Hz) from 0W to 10W // Register 0x45 69 dec
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_value_ramp;  // RAMP UP
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char; //procedure id HARDCODED
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "69"; // Heater Power Ref register
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "10"; // GOAL of RAMP - manually set to 3000s
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[3]= "30"; // ramp duration 30s
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[4]= "2"; // 1 Secons between set requests = 1Hz
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5]= "\nAction 11006: Heater Power Ref ramp (30s,1Hz) from 0W to 10W\n ";
			exeFunc_index++;

			// Wait Between SET requests
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
			exeFunc_index++;

			//11 Action 11007: Heater Power Ref ramp (30s,1Hz) from 10W to 0W // Register 0x45 69 dec
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_value_ramp;  // RAMP UP
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char; //procedure id HARDCODED
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "69"; // Heater Power Ref register
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "0"; // GOAL of RAMP - manually set to 3000s
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[3]= "30"; // ramp duration 30s
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[4]= "2"; // 1 Secons between set requests = 1Hz
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5]= "\nAction 11007: Heater Power Ref ramp (30s,1Hz) from 10W to 0W\n ";
			exeFunc_index++;

			// Wait Between SET requests
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
			exeFunc_index++;

			//13 Action 1108: Set Heater Voltage Ref 0V / Register 0x3D 61 dec
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;  // Set request
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char; //procedure id HARDCODED
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "61"; // Reservoir Heater Voltage Ref register
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "0"; //  Set to 0V
			exeFunc_index++;

			// Wait Between SET requests
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
			exeFunc_index++;

			//15 Action 1109: Set Heater Current Ref 0A / Register 0x41 65 dec
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;  // Set request
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char; //procedure id HARDCODED
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "65"; // Reservoir Heater Current Ref register
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "0"; //  Set to 0 A
			exeFunc_index++;

			// Wait Between SET requests
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
			exeFunc_index++;

			//17 Action 11010: Set Heater Mode 0  / Register 0x3C hex   60 dec
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char; // OBC sequence id
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "60"; // Reservoir heater mode register
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "0"; // Heater mode value
			exeFunc_index++;

			// Wait Between SET requests
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
			exeFunc_index++;

			//19 Action 11011: Set Heater Power Ref 6 W / Register 0x45 65 dec
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;  // Set request
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char; //procedure id HARDCODED
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "69"; // Reservoir Heater Power Ref register
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "6"; //  Set to 6 W
			//exeFunc_index++;


			THR_HARDCODED_SEQUENCES[sequence_id_int].length = exeFunc_index; // MANUALLY DEFINE LENGTH OF SEQUENCE //
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequence_trigger = false;
			THR_HARDCODED_SEQUENCES[sequence_id_int].repeat = false;
			THR_HARDCODED_SEQUENCES[sequence_id_int].substage_index = 0; //DEFAULT SUBSTAGE INDEX
			exeFunc_index= 0;

			// ******   SEQUENCE 6  *******System Cold Test / Neutralizer Heating Test /Table 9-4

			exeFunc_index=0; // at the beggining of sequence hardcodding set it to 0
			sequenc_id_char = "5";
			sequence_id_int = 5;
			wait_between_stages_str = "2000";




			//1 Action 13001: Set Neutralizer Mode 0 // Register 0x4B hex 75 dec
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "75"; //Neutralizer Mode
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "0";
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5]= "\nAction 12001: Set Neutralizer Mode 0\n";
			exeFunc_index++;

			// Wait Between SET requests
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
			exeFunc_index++;

			//3 Action 13002: Set Neutralizer Heater Current Ref 0.7A // Register 0x52 82
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "82"; //Neutralizer Mode
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "0.7";
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5]= "\nAction 13002: Set Heutralizer Heater Current Ref 0.7A\n";
			exeFunc_index++;

			// Wait Between SET requests
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
			exeFunc_index++;

			//5 Action 13003: Set Neutralizer Filament Ref 1/2 // Register 0x4d 77
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "77"; //Neutralizer Mode
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "1";
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5]= "\nAction 13003: Set Neutralizer Filament Ref 1\n";
			exeFunc_index++;

			// Wait Between SET requests
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
			exeFunc_index++;

			//7 Action 13004: Set Neutralizer Heater Power Ref 0W // Register 0x56 86
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "86";
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "0";
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5]= "\nAction 13004: Set Neutralizer Heater Power Ref 0W\n";
			exeFunc_index++;

			// Wait Between SET requests
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
			exeFunc_index++;

			//9 Action 13005: Set Neutralizer Beam Current Ref 1mA // Register 0x5C 92
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "92";
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "0.001"; // 0.001 A
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5]= "\nAction 13004: Set Neutralizer Heater Power Ref 0W\n";
			exeFunc_index++;

			// Wait Between SET requests
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
			exeFunc_index++;


			//11 Action 13006: Set Neutralizer Bias Ref 1 // Register 0x4E hex 78 dec
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "78";
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "1";
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5]= "\nAction 13006: Set Neutralizer Bias Ref 1\n";
			exeFunc_index++;

			// Wait Between SET requests
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
			exeFunc_index++;



			//13 Action 13007: Set Neutralizer Mode 1 // Register 0x4B hex 75 dec
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "75"; //Neutralizer Mode
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "1";
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5]= "\nAction 13007: Set Neutralizer Mode 1\n";
			exeFunc_index++;


			// Wait Between SET requests
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
			exeFunc_index++;



			//15 Action 13008: Neutralizer Heater Power Ref ramp (30s,1Hz) from 0W to 2W // Register 0x56 86 dec
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_value_ramp;  // RAMP UP
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "86"; // Neutralizer Heater Power Ref register
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "2"; // GOAL of RAMP - 2w
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[3]= "30"; // ramp duration 30s
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[4]= "2"; // 2 Secons between set requests = 0.5Hz
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5]= "\nAction 13008: Neutralizer Heater Power Ref ramp (30s,1Hz) from 0W to 2W\n ";
			exeFunc_index++;


			// Wait Between SET requests
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
			exeFunc_index++;




			//17 Action 13009: Neutralizer Heater Power Ref ramp (30s,1Hz) from 2W to 0W // Register 0x56 86 dec
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_value_ramp;  // RAMP DOWN
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "86"; // Neutralizer Heater Power Ref register
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "0"; // GOAL of RAMP - 0W
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[3]= "30"; // ramp duration 30s
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[4]= "2"; // 2 Secons between set requests = 0.5Hz
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5]= "\nAction 13009: Neutralizer Heater Power Ref ramp (30s,1Hz) from 2W to 0W\n ";
			exeFunc_index++;



			// Wait Between SET requests
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
			exeFunc_index++;



			//19 Action 13010: Set Neutralizer Heater Current Ref 0A // Register 0x52 82
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "82"; //Neutralizer Mode
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "0";
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5]= "\nAction 13010: Set Heutralizer Heater Current Ref 0A\n";
			exeFunc_index++;


			// Wait Between SET requests
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
			exeFunc_index++;



			//21 Action 13011: Set Neutralizer Beam Current Ref 0mA // Register 0x5C 92
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "92";
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "0"; // 0.001 A
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5]= "\nAction 13011: Set Neutralizer Beam Current Ref 0mA\n";
			exeFunc_index++;


			// Wait Between SET requests
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
			exeFunc_index++;




			//23 Action 13012: Set Neutralizer Heater Power Ref 0W // Register 0x56 86
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "86";
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "0";
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5]= "\nAction 13012: Set Neutralizer Heater Power Ref 0W\n";
			exeFunc_index++;


			// Wait Between SET requests
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
			exeFunc_index++;


			//25 Action 13013: Set Neutralizer Bias Ref 0 // Register 0x4E hex 78 dec
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "78";
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "0";
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5]= "\nAction 13013: Set Neutralizer Bias Ref 0\n";
			exeFunc_index++;


			// Wait Between SET requests
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = thr_wait;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1] = wait_between_stages_str; // Wait [ms]
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5] = "\nWaiting between SET Requests\n";
			exeFunc_index++;


			//27 Action 13014: Set Neutralizer Mode 0 // Register 0x4B hex 75 dec
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].function = GeneralSetRequest_sequence;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[0]= sequenc_id_char;
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[1]= "75"; //Neutralizer Mode
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[2]= "0";
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequences[exeFunc_index].thr_argv[5]= "\nAction 13014: Set Neutralizer Mode 0\n";
			//exeFunc_index++;


			THR_HARDCODED_SEQUENCES[sequence_id_int].length = exeFunc_index; // MANUALLY DEFINE LENGTH OF SEQUENCE //
			THR_HARDCODED_SEQUENCES[sequence_id_int].sequence_trigger = false;
			THR_HARDCODED_SEQUENCES[sequence_id_int].repeat = false;
			THR_HARDCODED_SEQUENCES[sequence_id_int].substage_index = 0; //DEFAULT SUBSTAGE INDEX
			exeFunc_index= 0;


}
