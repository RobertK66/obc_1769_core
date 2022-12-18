/*
===============================================================================
 Copy paste from l3_sensor by       : Jevgeni Potulov
 Created on	 : 07.06.2022
===============================================================================
*/

#ifndef MOD_I2C_ARDUINO_H_
#define MOD_I2C_ARDUINO_H_

#include <chip.h>

#include "../ai2c/obc_i2c.h"
#include "../l7_climb_app.h"
#include "../l2_debug_com.h"
#include "../../ClimbObc.h"
#include "../libfixmath/fix16.h"

// init data needed. Choose UARt and 2 GPIO Pins to be used
typedef struct {
	LPC_I2C_T 	*pI2C;
	uint16_t frequency;
} psu_i2c_initdata_t;


//////
static bool readInProgress = false;
///////

// Module base
void psu_init(void *);
void psu_main();


#define I2C_ADR_EPS  0b1010101

/* EPS housekeeping data IDs/register numbers */
#define 	EPS_HK_I_PV2_5V 		0	/*	Current through FET3-2 between PV2-bus and 5V converter, low byte	*/
#define 	EPS_HK_I_PV1_5V 		2	/*	Current through FET3-1 between PV1-bus and 5V converter, low byte	*/
#define 	EPS_HK_V_PV2 			4	/*	Voltage at PV2-bus, low byte	*/
#define 	EPS_HK_V_5V_IN 			6	/*	Voltage at the input of the 5V converter measured at FET3-1, low byte	*/
#define 	EPS_HK_I_PV1_3V3 		8	/*	Current through FET5-1 between PV1-bus and 3V3 converter, low byte	*/
#define 	EPS_HK_I_PV2_3V3 		10	/*	Current through FET5-2 between PV2-bus and 3V3 converter, low byte	*/
#define 	EPS_HK_V_PV1 			12	/*	Voltage at PV1-bus, low byte	*/
#define 	EPS_HK_V_3V3_IN 		14	/*	Voltage at the input of the 3V3 converter measured at FET5-2, low byte	*/
#define 	EPS_HK_TEMP_BAT1SW 		16	/*	Temp near BAT1 switches low byte	*/
#define 	EPS_HK_TEMP_5V 			18	/*	Temp near 5V converter low byte	*/
#define 	EPS_HK_I_PV1_HV 		20	/*	Current through FET4-1 between PV1-bus and HV supply, low byte	*/
#define 	EPS_HK_I_PV2_HV 		22	/*	Current through FET4-2 between PV2-bus and HV supply, low byte	*/
#define 	EPS_HK_V_3V3_OUT 		24	/*	Voltage at the output of the 3V3 converter, low byte	*/
#define 	EPS_HK_V_HV 			26	/*	Voltage at the output of the HV supply to the PPTs measured at FET4-2, low byte	*/
#define 	EPS_HK_I_PV2_BAT1 		28	/*	Current through FET1-2 between PV2-bus and battery 1, low byte	*/
#define 	EPS_HK_I_PV1_BAT1 		30	/*	Current through FET1-1 between PV1-bus and  battery 1, low byte	*/
#define 	EPS_HK_V_5V_OUT 		32	/*	Voltage at the output of the 5V converter, low byte	*/
#define 	EPS_HK_V_BAT1 			34	/*	Voltage of the battery 1, low byte	*/
#define 	EPS_HK_I_PV2_BAT2 		36	/*	Current through FET2-2 between PV2-bus and battery 2, low byte	*/
#define 	EPS_HK_I_PV1_BAT2 		38	/*	Current through FET2-1 between PV1-bus and  battery 2, low byte	*/
#define 	EPS_HK_VCC_MC			40	/*	Supply voltage of the MC	*/
#define 	EPS_HK_TEMP_MC			41	/*	Temperature of the internal sensor of the MC	*/
#define 	EPS_HK_V_BAT2 			42	/*	Voltage of the battery 2, low byte	*/
#define 	EPS_HK_TEMP_BAT1 		44	/*	Temp of BAT1 on the battery holder, low byte	*/
#define 	EPS_HK_TEMP_BAT2 		46	/*	Temp of BAT2 on the battery holder, low byte	*/
#define 	EPS_HK_STATUS_1			48	/*	B7 (MSB): 3V3-1 on, B6: 3V3-2 on, B5: 3V3-3 on, B4: 3V3-Backup on, B3: 5V-1 on, B2: 5V-2 on, B1: 5V-3 on, B0 (LSB): 5V-4 on	*/
#define 	EPS_HK_STATUS_2			49	/*	B7 (MSB): Power Low Warning (EPS will enter in Power Down Mode soon after this warning), B6: Bat1 connected to PV1, B5: Bat2 connected to PV2, B4: 3V3 on, B3: 5V on, B2-B0 (LSB): Mode: 000 Debug Mode, 001 Boot Mode, 010 Flight Mode, 011 Power Down Mode, 100 Safe Mode,	*/
#define 	EPS_HK_STATUS_3			50	/*	B7 (MSB): 3V3 Burst Mode on, B6: 5V Burst Mode on, B5: Bat1 connected to PV2, B4: Bat2 connected to PV1, ,Bit 2 = CC1 Communication Status: OKAY = 1, LOST = 0,Bit 1 = CC2 Communication Status: OKAY = 1, LOST = 0,Bit 0 = RBF pin*/
#define 	EPS_HK_STATUS_BAT1		51	/*	Estimation of the remaining capacity of the Battery 1	*/
#define 	EPS_HK_STATUS_BAT2		52	/*	Estimation of the remaining capacity of the Battery 2	*/
#define 	EPS_HK_REBOOT_MC		53	/*	Number of reboots since RBF of the main controller	*/
#define 	EPS_HK_REBOOT_CC1		54	/*	Number of reboots since RBF of the first communication controller	*/
#define 	EPS_HK_REBOOT_CC2		55	/*	Number of reboots since RBF of the second communication controller	*/
#define 	EPS_HK_VCC_CC1			56	/*	Supply voltage of CC1	*/
#define 	EPS_HK_TEMP_CC1			57	/*	Temperature of the internal sensor of CC1	*/
#define 	EPS_HK_VCC_CC2			58	/*	Supply voltage of CC2	*/
#define 	EPS_HK_TEMP_CC2			59	/*	Temperature of the internal sensor of CC2	*/
#define 	EPS_HK_STATUS_CC1		60	/*	B7 (MSB)-B6: CC Mode: 00 Boot Mode, 01 Flight Mode, 10 Safe Mode, 11 CC1 unavailable, B5: mcTimeoutFlag, B4: RBF (CC1 only), B3: EN_I2C, B2: Bat1 connected to PV1, B1: Bat2 connected to PV2, B0 (LSB): 3V3-Backup on	*/
#define 	EPS_HK_STATUS_CC2		61	/*	B7 (MSB)-B6: CC Mode: 00 Boot Mode, 01 Flight Mode, 10 Safe Mode, 11 CC2 unavailable, B5: mcTimeoutFlag, B4: TBD B3: EN_I2C, B2: Bat1 connected to PV1, B1: TBD, B0 (LSB): 3V3-Backup on	*/
#define 	EPS_HK_CC_ID			62	/*	0xAB	*/
#define 	EPS_HK_TBD				63	/*	Not Used	*/

/* Struct for PSU housekeeping data */
typedef struct eps_settings_s
{
	uint8_t current_lim_hv;
	uint8_t current_lim_3v3_1;
	uint8_t current_lim_3v3_2;
	uint8_t current_lim_3v3_3;
	uint8_t current_lim_3v3_backup;
	uint8_t current_lim_1;
	uint8_t current_lim_2;
	uint8_t current_lim_3;
	uint8_t current_lim_4;
	uint8_t mc_force_out_val_1;
	uint8_t mc_force_out_val_2;
	uint8_t mc_force_out_val_3;
	uint8_t mc_out_val_1;
	uint8_t mc_out_val_2;
	uint8_t mc_out_val_3;
	uint8_t cc1_force_out_val;
	uint8_t cc1_output;
	uint8_t cc2_force_out_val;
	uint8_t cc2_output;
	uint8_t mc_sensor_disable;
	uint8_t error_flag;
	uint8_t tbd1;
	uint8_t obc_watchdog;
	uint8_t repetition;

	uint8_t data_valid;
} eps_settings_t;

typedef union eps_hk_status_u
{
	struct
	{
		uint8_t status1;
		uint8_t status2;
		uint8_t status3;
		uint8_t status4;
	} bytes;

	struct
	{
		// Status 1: B7 (MSB): 3V3-1 on, B6: 3V3-2 on, B5: 3V3-3 on, B4: 3V3-Backup on, B3: 5V-1 on, B2: 5V-2 on, B1: 5V-3 on, B0 (LSB): 5V-4 on
		uint8_t rail_5v_d :1;
		uint8_t rail_5v_c :1;
		uint8_t rail_5v_b :1;
		uint8_t rail_5v_a :1;
		uint8_t rail_3v3_backup :1;
		uint8_t rail_3v3_d :1;
		uint8_t rail_3v3_b :1;
		uint8_t rail_3v3_a :1;

		// Status 2: B7 (MSB): Power Low Warning , B6: Bat1 connected to PV1, B5: Bat2 connected to PV2, B4: 3V3 on, B3: 5V on, B2-B0 (LSB): Mode
		uint8_t mode :3; 				// 000 Debug Mode, 001 Boot Mode, 010 Flight Mode, 011 Power Down Mode, 100 Safe Mode
		uint8_t vreg_5v_on :1;
		uint8_t vreg_3v3_on :1;
		uint8_t bat2_con_pv2 :1;
		uint8_t bat1_con_pv1 :1;		// B6: Bat1 connected to PV1
		uint8_t power_low_warning :1; 	// B7: Power Low Warning (EPS will enter in Power Down Mode soon after this warning)

		// Status 3: B7 (MSB): 3V3 Burst Mode on, B6: 5V Burst Mode on, B5: Bat1 connected to PV2, B4: Bat2 connected to PV1, B3: Temperatur warning flag, B2: CC1 connection okay flag, B1: CC2 connection okay flag, B0 (LSB): RBF
		uint8_t rbf :1; 				// B0 (LSB): RBF
		uint8_t cc2_con_ok :1; 			// B1: CC2 connection okay flag
		uint8_t cc1_con_ok :1; 			// B2: CC1 connection okay flag
		uint8_t temperature_warning :1; // B3: Temperatur warning flag
		uint8_t bat2_con_pv1 :1; 		// B4: Bat2 connected to PV1
		uint8_t bat1_con_pv2 :1; 		// B5: Bat1 connected to PV2
		uint8_t burst_mode_5v :1; 		// B6: 5V Burst Mode on
		uint8_t burst_mode_3v3 :1; 		// B7 (MSB): 3V3 Burst Mode on

		uint8_t z_not_used :8;
	} bits;
} eps_hk_status_t;

typedef struct eps_hk_data_s
{
	//BLOCK 1 :  size 16 bytes
	int16_t i_pv2_5v;
	int16_t i_pv1_5v;
	uint16_t v_pv2;
	uint16_t v_5v_in;
	int16_t i_pv1_3v3;
	int16_t i_pv2_3v3;
	uint16_t v_pv1;
	uint16_t v_3v3_in;

	// BLOCK 2 : size 16 bytes
	int16_t temp_bat1_sw;
	int16_t temp_5v;
	int16_t i_pv1_hv;
	int16_t i_pv2_hv;
	uint16_t v_3v3_out;
	uint16_t v_hv;
	int16_t i_pv2_bat1;
	int16_t i_pv1_bat1;

	// BLOCK 3: size 16
	uint16_t v_5v_out;
	uint16_t v_bat1;
	int16_t i_pv2_bat2;
	int16_t i_pv1_bat2;
	uint8_t vcc_mc;
	int8_t temp_mc;
	uint16_t v_bat2;
	int16_t temp_bat1;
	int16_t temp_bat2;

	// BLOCK 4 : size 16
	uint8_t status_1;
	uint8_t status_2;
	uint8_t status_3;
	uint8_t status_bat1;
	uint8_t status_bat2;
	uint8_t reboot_mc;
	uint8_t reboot_cc1;
	uint8_t reboot_cc2;
	uint8_t vcc_cc1;
	int8_t temp_cc1;
	uint8_t vcc_cc2;
	int8_t temp_cc2;
	uint8_t status_cc1;
	uint8_t status_cc2;
	uint8_t cc_id;
	uint8_t empty2;

	// Note : 4 Blocks - each 16 bytes length

	uint8_t data_valid;

} eps_hk_data_t;

typedef enum
{
    MV_VALUE_GOOD = 0, MV_SENSOR_DEAD, MV_VALUE_OUT_OF_RANGE, MV_VALUE_CRITICAL, MV_VALUE_ID_NOT_DEFINED
} quality_en;


typedef struct mval_s
{
    quality_en quality;
    fix16_t val;
}
mval_t;


enum mval_id
{
	/*

    // OBC MVAL IDs /
    OBC_MVAL_MAG_X = 1, // per setter and getter
    OBC_MVAL_MAG_Y,
    OBC_MVAL_MAG_Z,

    OBC_MVAL_TEMP,

    OBC_MVAL_GYRO_X, //per setter and getter
    OBC_MVAL_GYRO_Y,
    OBC_MVAL_GYRO_Z,
    OBC_MVAL_GYRO_Y1,

    OBC_MVAL_MPU_GYRO_X, //per setter and getter
    OBC_MVAL_MPU_GYRO_Y,
    OBC_MVAL_MPU_GYRO_Z,

    OBC_MVAL_MPU_TEMP,

    OBC_MVAL_RTC_TIME_S,
    OBC_MVAL_RTC_TIME,
    OBC_MVAL_RTC_DATE,
    OBC_MVAL_RTC_UTC,


    // Bottom panel MVAL IDs /
    OBC_MVAL_MAG_BP_X, // per setter and getter //
    OBC_MVAL_MAG_BP_Y,
    OBC_MVAL_MAG_BP_Z,

    OBC_MVAL_MAG_BOOM_X, // per setter and getter //
    OBC_MVAL_MAG_BOOM_Y,
    OBC_MVAL_MAG_BOOM_Z,

    // Light sensor IDs //
    OBC_MVAL_LS_A, // per setter and getter //
    OBC_MVAL_LS_B,
    OBC_MVAL_LS_C,
    OBC_MVAL_LS_D,
    OBC_MVAL_LS_BP,
    OBC_MVAL_LS_TP,

    // Temperature sensor IDs //
    OBC_MVAL_TEMP_A,
    OBC_MVAL_TEMP_B,
    OBC_MVAL_TEMP_C,
    OBC_MVAL_TEMP_D,
    OBC_MVAL_TEMP_BP,
    OBC_MVAL_TEMP_TP,

    // MPPT sensor IDs //
    OBC_MVAL_MPPT1_VOLTAGE_A,
    OBC_MVAL_MPPT2_VOLTAGE_A,
    OBC_MVAL_MPPT1_VOLTAGE_B,
    OBC_MVAL_MPPT2_VOLTAGE_B,
    OBC_MVAL_MPPT1_VOLTAGE_C,
    OBC_MVAL_MPPT2_VOLTAGE_C,
    OBC_MVAL_MPPT1_VOLTAGE_D,
    OBC_MVAL_MPPT2_VOLTAGE_D,

    OBC_MVAL_MPPT1_CURRENT_A,
    OBC_MVAL_MPPT2_CURRENT_A,
    OBC_MVAL_MPPT1_CURRENT_B,
    OBC_MVAL_MPPT2_CURRENT_B,
    OBC_MVAL_MPPT1_CURRENT_C,
    OBC_MVAL_MPPT2_CURRENT_C,
    OBC_MVAL_MPPT1_CURRENT_D,
    OBC_MVAL_MPPT2_CURRENT_D,

    // Reset counters //
    OBC_MVAL_RC_A,
    OBC_MVAL_RC_B,
    OBC_MVAL_RC_C,
    OBC_MVAL_RC_D,
    OBC_MVAL_RC_BP,
    OBC_MVAL_RC_SA,

    // GPS IDs //
    OBC_MVAL_GPS_LAT,
    OBC_MVAL_GPS_LAT_DIR,
    OBC_MVAL_GPS_LON,
    OBC_MVAL_GPS_LON_DIR,
    OBC_MVAL_GPS_HEIGHT,
    OBC_MVAL_GPS_HDOP,
    OBC_MVAL_GPS_UTC_TIME,
    OBC_MVAL_GPS_UTC_DATE,

    // Merged values //
    OBC_MVAL_MAG_MERGED_X,
    OBC_MVAL_MAG_MERGED_Y,
    OBC_MVAL_MAG_MERGED_Z,

    OBC_MVAL_GYRO_MERGED_X,
    OBC_MVAL_GYRO_MERGED_Y,
    OBC_MVAL_GYRO_MERGED_Z,
    */

    // EPS IDs //
    EPS_MVAL_HK_I_PV2_5V,
    EPS_MVAL_HK_I_PV1_5V,
    EPS_MVAL_HK_V_PV2,
    EPS_MVAL_HK_V_5V_IN,
    EPS_MVAL_HK_I_PV1_3V3,
    EPS_MVAL_HK_I_PV2_3V3,
    EPS_MVAL_HK_V_PV1,
    EPS_MVAL_HK_V_3V3_IN,
    EPS_MVAL_HK_TEMP_BAT1SW,
    EPS_MVAL_HK_TEMP_5V,
    EPS_MVAL_HK_I_PV1_HV,
    EPS_MVAL_HK_I_PV2_HV,
    EPS_MVAL_HK_V_3V3_OUT,
    EPS_MVAL_HK_V_HV,
    EPS_MVAL_HK_I_PV2_BAT1,
    EPS_MVAL_HK_I_PV1_BAT1,
    EPS_MVAL_HK_V_5V_OUT,
    EPS_MVAL_HK_V_BAT1,
    EPS_MVAL_HK_I_PV2_BAT2,
    EPS_MVAL_HK_I_PV1_BAT2,
    EPS_MVAL_HK_VCC_MC,
    EPS_MVAL_HK_TEMP_MC,
    EPS_MVAL_HK_V_BAT2,
    EPS_MVAL_HK_TEMP_BAT1,
    EPS_MVAL_HK_TEMP_BAT2,
    EPS_MVAL_HK_STATUS_1,
    EPS_MVAL_HK_STATUS_2,
    EPS_MVAL_HK_STATUS_3,
    EPS_MVAL_HK_STATUS_BAT1,
    EPS_MVAL_HK_STATUS_BAT2,
    EPS_MVAL_HK_REBOOT_MC,
    EPS_MVAL_HK_REBOOT_CC1,
    EPS_MVAL_HK_REBOOT_CC2,
    EPS_MVAL_HK_VCC_CC1,
    EPS_MVAL_HK_TEMP_CC1,
    EPS_MVAL_HK_VCC_CC2,
    EPS_MVAL_HK_TEMP_CC2,
    EPS_MVAL_HK_STATUS_CC1,
    EPS_MVAL_HK_STATUS_CC2,
    EPS_MVAL_HK_CC_ID,
    EPS_MVAL_HK_TBD,

    // EPS settings IDs //

    EPS_MVAL_SETTING_CURRENT_LIMIT_HV,
    EPS_MVAL_SETTING_CURRENT_LIMIT_3V3_1,
    EPS_MVAL_SETTING_CURRENT_LIMIT_3V3_2,
    EPS_MVAL_SETTING_CURRENT_LIMIT_3V3_3,
    EPS_MVAL_SETTING_CURRENT_LIMIT_3V3_BACKUP,
    EPS_MVAL_SETTING_CURRENT_LIMIT_5V_1,
    EPS_MVAL_SETTING_CURRENT_LIMIT_5V_2,
    EPS_MVAL_SETTING_CURRENT_LIMIT_5V_3,
    EPS_MVAL_SETTING_CURRENT_LIMIT_5V_4,
    EPS_MVAL_SETTING_MC_FORCE_OUTPUT_VALUE_REGISTER_1,
    EPS_MVAL_SETTING_MC_FORCE_OUTPUT_VALUE_REGISTER_2,
    EPS_MVAL_SETTING_MC_FORCE_OUTPUT_VALUE_REGISTER_3,
    EPS_MVAL_SETTING_MC_OUTPUT_VALUE_REGISTER_1,
    EPS_MVAL_SETTING_MC_OUTPUT_VALUE_REGISTER_2,
    EPS_MVAL_SETTING_MC_OUTPUT_VALUE_REGISTER_3,
    EPS_MVAL_SETTING_CC1_FORCE_OUTPUT_VALUE_REGISTER,
    EPS_MVAL_SETTING_CC1_OUTPUT_VALUE_REGISTER,
    EPS_MVAL_SETTING_CC2_FORCE_OUTPUT_VALUE_REGISTER,
    EPS_MVAL_SETTING_CC2_OUTPUT_VALUE_REGISTER,


	/*
    // SP IDs //

    // TOP LAYER IDs //
    STACIE_TEMP_1, // per setter and getter //
    STACIE_TEMP_2, // per setter and getter //
    OBC_STATE, // per setter and getter //
    ACDS_STATE, // per setter and getter //

    // STATE MASCHINE COMMANDS//

    ACDS_SHUTDOWN, // per setter and getter //
    SA_SHUTDOWN, // per setter and getter //
    FLUSH_BUFFER, // per setter and getter //
    ALLOW_TRASMISSION, // per setter and getter //

	OBC_MVAL_MAG_BOOM_TEMPERATURE,
	OBC_MVAL_HEAP_FREE_BYTES_REMAINING,
	OBC_MVAL_HEAP_MINIMUM_EVER_FREE_BYTES_REMAINING,
	OBC_MVAL_OBC_RESET_COUNTER,
	EPS_MVAL_SETTING_OBC_WATCHDOG,
	EPS_MVAL_EPS_SOFTWARE_VERSION,
	OBC_MVAL_STACIE_RSSI_A,
	OBC_MVAL_STACIE_RSSI_C,
	OBC_MVAL_STACIE_MODE_A,
	OBC_MVAL_STACIE_MODE_C,
	OBC_MVAL_STACIE_VERSION,

	MVAL_MAX_ENUMVAL
	*/
};


void i2c_Proccess_Received_Buffer(I2C_Data i2cJob, uint8_t *i2c_buffer,uint8_t i2c_buffer_len);
void PSU_datavector_request(int argc, char *argv[]);
void eps_housekeeping_data_read( uint8_t block);
void old_pegasys_PSU_request_cmd(int argc, char *argv[]);

#endif /* MOD_L3_SENSORS_H_ */
