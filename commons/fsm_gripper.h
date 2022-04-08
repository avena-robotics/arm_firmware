/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __COMMONS_H
#define __COMMONS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <main.h>

#define GEAR_RATIO												(uint16_t) 121
#define JOINT_SPEED_LIMIT									(float) 1.5
#define CALIBRATION_TORQUE_LIMIT					1000
#define CALIBRATION_SPEED_LIMIT						10
#define CALIBRATION_ZERO_POSITION_OFFSET	10
#define MAX_READABLE_CURRENT							(float) 33.0
#define MAX_TORQUE_THROUGH_CAN						(float) 360.0
#define MAX_SPEED_THROUGH_CAN							(float) 2.0

#define CURRENT_TORQUE_DATA_SIZE 					(uint8_t) 10
#define CURRENT_SPEED_DATA_SIZE 					(uint8_t) 10

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef M_TWOPI
#define M_TWOPI 2.0 * 3.14159265358979323846
#endif

typedef enum Joint_Error {
	JOINT_NO_ERROR = 0,
//	JOINT_POSITION_ENCODER_FAILED = 1,
	JOINT_MC_ERROR = 2,
	JOINT_SPEED_TO_HIGH = 4,
	JOINT_HW_ERROR = 8 // brak MA730, brak temp, GD fault
} Joint_Error_t;

typedef enum Joint_Warning {
	JOINT_NO_WARNING = 0,
	JOINT_POSITION_NOT_ACCURATE = 1,
	JOINT_OUTSIDE_WORKING_AREA = 2,
	JOINT_MA730_NOT_PROPER_MAGNETOC_FIELD = 4
} Joint_Warning_t;


typedef enum FSM_State {
	FSM_START = 0, /**< @brief Starting uC.*/
	FSM_INIT = 1,
	FSM_READY_TO_OPERATE = 2,
	FSM_OPERATION_ENABLE = 3, /**< @brief Enable power.*/

	FSM_CALIBRATION_PHASE_0 = 100,
	FSM_CALIBRATION_PHASE_1 = 101,
	FSM_CALIBRATION_PHASE_2 = 102,
	FSM_CALIBRATION_PHASE_3 = 103,
	FSM_CALIBRATION_PHASE_4 = 104,
	FSM_CALIBRATION_PHASE_5 = 105,
	FSM_CALIBRATION_PHASE_6 = 106,

	FSM_TRANSITION_START_TO_INIT = 10,
	FSM_TRANSITION_INIT_TO_READY_TO_OPERATE = 11,
	FSM_TRANSITION_READY_TO_OPERATE_TO_OPERATION_ENABLE = 12,
	FSM_TRANSITION_OPERATION_ENABLE_TO_READY_TO_OPERATE = 13,
	FSM_TRANSITION_OPERATION_ENABLE_TO_INIT = 14,
	FSM_TRANSITION_FAULT_REACTION_ACTIVE_TO_FAULT = 15,
	FSM_TRANSITION_FAULT_TO_INIT = 16,

	FSM_TRANSITION_INIT_TO_CALIBRATION_PHASE_0 = 110,
	FSM_TRANSITION_CALIBRATION_PHASE_0_TO_CALIBRATION_PHASE_1 = 111,
	FSM_TRANSITION_CALIBRATION_PHASE_1_TO_CALIBRATION_PHASE_2 = 112,
	FSM_TRANSITION_CALIBRATION_PHASE_2_TO_CALIBRATION_PHASE_3 = 113,
	FSM_TRANSITION_CALIBRATION_PHASE_3_TO_CALIBRATION_PHASE_4 = 114,
	FSM_TRANSITION_CALIBRATION_PHASE_4_TO_CALIBRATION_PHASE_5 = 115,
	FSM_TRANSITION_CALIBRATION_PHASE_5_TO_CALIBRATION_PHASE_6 = 116,
	FSM_TRANSITION_CALIBRATION_PHASE_6_TO_INIT = 117,

	FSM_FAULT_REACTION_ACTIVE = 254,
	FSM_FAULT = 255
} FSM_State_t;


typedef enum Calibration_State {
	CALIBRATION_NOT_FINISHED = 0, /**< @brief Starting uC.*/
	CALIBRATION_TABLE_NOT_FILLED = 1, // No difference between hi and low value of the sector
	SECTORS_NARROW = 2, // No difference between hi and low value of the sector
	SECTORS_INTERCECTION = 3, // intersection of the sectors > 0
	SECTORS_NOT_SEPARETED = 3, // intersection of the sectors > 0
	CALIBRATION_TABLE_CONTAINS_ZEROES = 4,
	MISSED_CENTER_POSITION = 100,
	FLASH_DOESNT_WRITE_PROPERLY = 100,
	CALIBRATION_OK = 255
} Calibration_State_t;


typedef enum Joint_Calibratation_State {
	JOINT_NOT_CALIBRATED = 0, /**< @brief Starting uC.*/
	JOINT_CALIBRATED = 1
} Joint_Calibratation_State_t;

typedef enum Working_Mode {
	WORKING_MODE_NOT_SELECTED = 0, /**< @brief Starting uC.*/
	TORQUE_MODE,
	SPEED_MODE,
	POSITION_MODE
} Working_Mode_t;

typedef enum Motor {
	MOTOR_TYPE_NOT_SELECTED = 0, /**< @brief Starting uC.*/
	RI70,
	RI80
} Motor_t;

typedef struct MA730_Handle {
	bool started;
	uint16_t angle;
	uint8_t z0;
	uint8_t z1;
	uint8_t bct;
	uint8_t ety;
	uint8_t etx;
	uint8_t ppt0;
	uint8_t ppt1;
	uint8_t ilip;
	uint8_t mglt;
	uint8_t mght;
	uint8_t rd;
	uint8_t mgh;
	uint8_t mgl;
} __attribute__ ((packed)) MA730_Handle_t;

typedef struct Counters_Handle {
	uint32_t main_loop;
	uint32_t timer6;
	uint32_t timer7;
	uint32_t can_rx_counter;
	uint32_t can_tx_counter;
	uint32_t spi_rx_counter;
	uint32_t spi_tx_counter;
	uint32_t spi_txrx_counter;
} __attribute__ ((packed)) Counters_Handle_t;

typedef struct Joint_Configuration_Handle {
	uint8_t can_node_id;
	uint8_t gear_ratio;
	uint8_t pole_pairs;
	Motor_t motor_type;
//	uint16_t encoder_resolution;
	float electric_rotation_width;

	bool ma730_enabled;
	bool motor_temperature_enabled;
	bool bearing_temperature_enabled;
	bool safety_enabled;
//	bool canbus_enabled;
//	bool canbus_watchdog_enabled;
	bool working_area_constrain_enabled; /**< @brief settings to enable/disable constrain */

	Joint_Calibratation_State_t calibration_state;

	uint16_t maximum_electrical_rotations;
	uint16_t reachable_electrical_rotations;

	uint16_t number_of_sectors;
	int16_t  zero_electric_position;
	uint16_t zero_electric_rotation;
	float joint_working_area;
	uint16_t calibration_sector_size; // in electrical rotations
	uint16_t calibration_table_size;
	uint16_t calibration_table_1[1694];
	uint16_t calibration_table_2[1694];

} __attribute__ ((packed)) Joint_Configuration_Handle_t;

typedef struct App_Command_Handle {
	Working_Mode_t working_mode;
	float joint_torque;				// Nm
	float motor_torque;				// Nm
	float joint_speed;				// rads
	float motor_speed;				// rads
	float joint_position;			// rad
	float motor_position;			// rad
	int16_t _motor_torque;			//
	int16_t _motor_speed;			//
	int32_t _motor_position;		//
} __attribute__ ((packed)) App_Command_Handle_t;

typedef enum Joint_Position_State {
	POSITION_UNDER_WORKING_AREA = -1,
	POSITION_IN_WORKING_AREA = 0, /**< @brief Starting uC.*/
	POSITION_OVER_WORKING_AREA = 1
} Joint_Position_State_t;

typedef enum Encoder_Position_State {
	POSITION_UNKNOWN = 0, /**< @brief Starting uC.*/
	POSITION_APROXIMATED = 1,
	POSITION_ACCURATE = 2,
	POSITION_ESTIMATION_FAILED = 255
} Encoder_Position_State_t;

typedef struct FSMStatus {
	FSM_State_t state;
	bool state_is_running;
	bool transition_is_running;
} __attribute__ ((packed)) FSMStatus_t;

typedef struct Joint_Status_Handle {
	Joint_Position_State_t current_joint_position;
	Encoder_Position_State_t encoder_position_state;

	float f_current_encoder_position_offset;	// rad

	float f_current_motor_position; 					// rad
	float f_current_joint_position_multiturn; // rad
	float f_current_joint_position; 					// rad
	float f_current_motor_speed; 							// rad/s
	float f_current_joint_speed; 							// rad/s
	float f_current_motor_torque; 						// Nm
	float f_current_joint_torque; 						// Nm
	float f_current_voltage;									// V

	int16_t current_motor_temperature;				// C
	int16_t current_bearing_temperature;			// C

	int16_t mc_current_motor_torque;
	int16_t mc_current_motor_speed;
	int32_t mc_current_motor_rotation;
	int64_t mc_current_motor_position_multiturn;
	uint16_t mc_current_motor_position;
	uint16_t mc_previous_motor_position;
	int32_t mc_current_electric_rotation;
	int16_t mc_current_electric_position;
//	int16_t mc_previous_electric_position;

	bool gd_nfault;
	bool gd_ready;

	uint8_t errors;
	uint8_t warnings;

//	PosCtrlStatus_t mc_position_control_status;
//	AlignStatus_t mc_encoder_align_status;
	State_t stm_state_motor;
	uint8_t mc_current_faults_motor;
	uint8_t mc_occured_faults_motor;

	int16_t _current_torque_data[CURRENT_TORQUE_DATA_SIZE];
	uint8_t _current_torque_index;

	int16_t _current_speed_data[CURRENT_SPEED_DATA_SIZE];
	uint8_t _current_speed_index;
} __attribute__ ((packed)) Joint_Status_Handle_t;


// FUNCTION DEFINITION
void UJ_Init();

// EXTERN VARIABLES
extern volatile int16_t g_current_electrical_rotation;
extern volatile int16_t g_current_electrical_position;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern FDCAN_HandleTypeDef hfdcan1;
extern SPI_HandleTypeDef hspi1;

#ifdef __cplusplus
}
#endif

#endif /* __COMMONS_H */