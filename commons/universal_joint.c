#include "universal_joint.h"
#include "pz_1sf_driver.h"
#include "fsm.h"
//#include "mc_configuration_registers.h"
#include <stdlib.h>
#include <string.h>
//#pragma GCC push_options
//#pragma GCC optimize ("O0")

// SDK_VERSION
// 5.4.7 // 0x05040700
// 5.Y.3 // 0x055a0300
// 5.Y.4 // 0x055a0400

union register_data_flash_t {
	 uint16_t data[4];
	 uint64_t flash;
};

union register_data_flash_t g_registers_flash[64] = {0}; // 256 baitow

FDCAN_RxHeaderTypeDef can_rx_header; // CAN Bus Transmit Header
FDCAN_TxHeaderTypeDef can_tx_header; // CAN Bus Transmit Header
uint8_t can_rx_data[64] = {0};  //CAN Bus Receive Buffer
uint8_t can_tx_data[64] = {0};  //CAN Bus Send Buffer

volatile bool     g_wchdg_ok 				= false;

// FLASH
// 0x018000 -  98304 (32kB z tylu na konfiguracje)
uint32_t g_flash_address_configuration 			= 0x08018000;
//uint32_t g_flash_address_calibration_table 	= 0x08018100;

//uint16_t g_calibration_config[20] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//uint32_t g_data[2] = {0, 0};

#define CALIBRATION_ZERO_POSITION_COUNTER	15
uint16_t g_encoder_position[2][CALIBRATION_ZERO_POSITION_COUNTER] = {0};
float g_encoder_position_summary[2][3] = {0.0};

// PZ FIXME
volatile uint32_t g_pz2656_angle = 0;
volatile float g_pz2656_angle_deg = 0;
volatile uint8_t buf_tx[0x0F + 3] = {0};
volatile uint8_t buf_rx[0x0F + 3] = {0};
volatile uint16_t bufsize = 0;

// Calibration variables
volatile Calibration_State_t g_calibration_state = CALIBRATION_NOT_FINISHED;

Counters_Handle_t volatile g_counters = {
	.main_loop = 0,
	.timer6 = 0,
	.timer7 = 0,
//	.spi_rx_counter = 0,
//	.spi_tx_counter = 0,
	.spi_txrx_counter = 0,
};

Joint_Command_Handle_t g_joint_command = {
	.working_mode = TORQUE_MODE,
	.motor_torque = 0.0,
};

volatile Joint_Configuration_Handle_t g_joint_configuration = {
	.absolute_encoder_enabled = false,
	.safety_enabled = false,
	.working_area_constrain_enabled = false,
	.canbus_watchdog_enabled = false,
	.speed_limit_enabled = true,
	.motor_type = MOTOR_TYPE,
	.can_node_id = 0x00,
	.gear_ratio = 121,
};

Joint_Status_Handle_t g_joint_status = {
	.mc_current_motor_position = 0,
	.encoder_position_state = POSITION_UNKNOWN,
	.b_safety_input = true,
};

volatile PZ2656_Handle_t	g_pz2656 = {
	.started = false,
	.angle = 0x00,
	.diag = 0x00000000,
	.revert_direction = 0,
};

volatile FSMStatus_t 	g_fsm_status = {
	.current_state = FSM_START,
	.new_state 	   = FSM_START,
//	.state_is_running = false,
//	.transition_is_running = false,
};

NTC_Handle_t   g_TempBearingSensorParamsM1 = {
  .bSensorType = REAL_SENSOR,
  .TempRegConv =
  {
    .regADC = ADC2,
    .channel = MC_ADC_CHANNEL_17,
    .samplingTime = M1_TEMP_SAMPLING_TIME,
  },
  .hLowPassFilterBW        = M1_TEMP_SW_FILTER_BW_FACTOR,
  .hOverTempThreshold      = (uint16_t)(OV_TEMPERATURE_THRESHOLD_d),
  .hOverTempDeactThreshold = (uint16_t)(OV_TEMPERATURE_THRESHOLD_d - OV_TEMPERATURE_HYSTERESIS_d),
  .hSensitivity            = (uint16_t)(ADC_REFERENCE_VOLTAGE/dV_dT),
  .wV0                     = (uint16_t)(V0_V *65536/ ADC_REFERENCE_VOLTAGE),
  .hT0                     = T0_C,
};

// LOCAL FUNCTIONS DEFINITIONS
float Calculate_ADC_to_Temperature(uint16_t adc_data, uint16_t beta, uint8_t nominal_temperature, uint16_t ntc_nominal_resistance, uint16_t series_resistance);
void FLASH_Configuration_Load();
uint32_t FLASH_Configuration_Save();

void REG_Set(uint8_t rejestr, uint16_t * data);
void REG_Get(uint8_t rejestr, uint16_t * data);
uint8_t REG_Get_uint8(uint8_t rejestr);
uint16_t REG_Get_uint16(uint8_t rejestr);
uint16_t * REG_Get_pointer(uint8_t rejestr);
float REG_Get_float(uint8_t rejestr);
void REG_Update(uint8_t poczatek, uint8_t koniec);

#ifdef ENCODER_PZ2656
void PZ2656_ReadAngle(void);
#endif

//void motor_stop(void);
uint16_t NTC_SetFaultState(NTC_Handle_t * pHandle);
void pz_spi_transfer(uint8_t *data_tx, uint8_t *data_rx, uint16_t datasize);

// FUNCTIONS BODIES
int cmpfunc (const void * a, const void * b) {
   return ( * (int * ) a - * (int * ) b );
}

void UJ_Init() 
{
	
	bool flash_need_to_be_saved = false;
	// NTC
	NTC_Init(&g_TempBearingSensorParamsM1);

#if PCB_VERSION >= 0x0300
	FLASH_Configuration_Load(); // Read configuration from FLASH
#endif

#if PCB_VERSION >= 0x0304
	HAL_GPIO_WritePin(BC_ENABLE_GPIO_Port, BC_ENABLE_Pin, GPIO_PIN_SET);
#endif

#if PCB_VERSION >= 0x0303
	if (g_registers_flash[0].data[0] == 0xFFFF) // pusta konfiguracja
	{
		uint16_t revert_direction = 0x01;
		uint16_t offset = 0x00;
	
		REG_Set(REG_NV_RW_REVERT_DIRECTION, &revert_direction); // revert_direction
		REG_Set(REG_NV_RW_PZ_OFFSET, &offset); // offset

		flash_need_to_be_saved = true;
	}
#endif

  // Ustawienie wartosci rejestrów RO
	uint16_t pcb_version 		= (PCB_VERSION & 0x0000FFFF);
	uint16_t app_version_1 	= (APP_VERSION & 0xFFFF0000) >> 16;
	uint16_t app_version_2 	= (APP_VERSION & 0x0000FFFF);
	uint16_t motor_type 		= MOTOR_TYPE;
	uint16_t gear_ratio 		= 121;
	
	REG_Set(0x03, &pcb_version); // PCB
	REG_Set(0x04, &app_version_1); // APP_1
	REG_Set(0x05, &app_version_2); // APP_2
	REG_Set(0x06, &motor_type); // MOTOR_TYPE
	REG_Set(0x07, &gear_ratio); // 121

	// Przeniesienie rejestrów ------------------------------------
	uint16_t revert_direction = REG_Get_uint16(0x21);
	uint16_t offset = REG_Get_uint16(0x22);
	REG_Set(0x57, &revert_direction); // revert_direction
	REG_Set(REG_NV_RW_PZ_OFFSET, &offset); // offset

	if (REG_Get_uint16(REG_NV_RW_CAN_ID) == 0xFFFF)	
	{
		uint16_t can_id = 0x00;
		REG_Set(REG_NV_RW_CAN_ID, &can_id); // default can_id = 0

		flash_need_to_be_saved = true;
	}

//	if ((REG_Get_uint16(REG_NV_RW_PID_ID_KP_GAIN) == 0xFFFF)) // Zapisanie domyslnych pidow we flash - jak wczesniej nie byly
//	{

//		uint16_t pid_id_kp_gain = PIDIdHandle_M1.hKpGain;
//		uint16_t pid_id_ki_gain = PIDIdHandle_M1.hKiGain;
//		uint16_t pid_id_kp_div_pow2 = PIDIdHandle_M1.hKpDivisorPOW2;
//		uint16_t pid_id_ki_div_pow2 = PIDIdHandle_M1.hKiDivisorPOW2;
//		
//		uint16_t pid_iq_kp_gain = PIDIqHandle_M1.hKpGain;
//		uint16_t pid_iq_ki_gain = PIDIqHandle_M1.hKiGain;
//		uint16_t pid_iq_kp_div_pow2 = PIDIqHandle_M1.hKpDivisorPOW2;
//		uint16_t pid_iq_ki_div_pow2 = PIDIqHandle_M1.hKiDivisorPOW2;
//	
//		REG_Set(REG_NV_RW_PID_ID_KP_GAIN, &pid_id_kp_gain);
//		REG_Set(REG_NV_RW_PID_ID_KI_GAIN, &pid_id_ki_gain);
//		REG_Set(REG_NV_RW_PID_ID_KP_DIV_POW2, &pid_id_kp_div_pow2);
//		REG_Set(REG_NV_RW_PID_ID_KI_DIV_POW2, &pid_id_ki_div_pow2);

//		REG_Set(REG_NV_RW_PID_IQ_KP_GAIN, &pid_iq_kp_gain);
//		REG_Set(REG_NV_RW_PID_IQ_KI_GAIN, &pid_iq_ki_gain);
//		REG_Set(REG_NV_RW_PID_IQ_KP_DIV_POW2, &pid_iq_kp_div_pow2);
//		REG_Set(REG_NV_RW_PID_IQ_KI_DIV_POW2, &pid_iq_ki_div_pow2);
//		
//		flash_need_to_be_saved = true;
//	}
	
#ifdef ENCODER_PZ2656
	HAL_Delay(100);
	// Konfiguracja stala
	pz_write_param(&PZ_FCL, 			0x00);
	pz_write_param(&PZ_FCS, 			0x00);
	pz_write_param(&PZ_SPI_EXT,		0x01);
	pz_write_param(&PZ_MT_PDL, 		0x00);
	pz_write_param(&PZ_ST_PDL, 		0x10);
	pz_write_param(&PZ_SPI_MT_DL, 0x00);
	pz_write_param(&PZ_SPI_ST_DL, 0x10);	

	pz_write_command(PZ_COMMAND_CONF_READ_ALL);
	g_pz2656.diag	= pz_read_param(&PZ_CMD_STAT);
	if (g_pz2656.diag != 0x00)
	{
		
	}
	g_pz2656.reg_ai_phase 	= pz_read_param(&PZ_AI_PHASE);
	g_pz2656.reg_ai_scale 	= pz_read_param(&PZ_AI_SCALE);
	g_pz2656.reg_cos_off 		= pz_read_param(&PZ_COS_OFF); // PZ_COS_OFF
	g_pz2656.reg_sin_off 		= pz_read_param(&PZ_SIN_OFF); // PZ_SIN_OFF
	g_pz2656.reg_sc_gain 		= pz_read_param(&PZ_SC_GAIN); // PZ_SC_GAIN
	g_pz2656.reg_sc_phase		= pz_read_param(&PZ_SC_PHASE); // PZ_SC_PHASE

	g_pz2656.reg_fcs 				= pz_read_param(&PZ_FCL);
	g_pz2656.reg_fcs 				= pz_read_param(&PZ_FCS);
	g_pz2656.reg_spi_ext 		= pz_read_param(&PZ_SPI_EXT);
	g_pz2656.reg_mt_pdl 		= pz_read_param(&PZ_MT_PDL);
	g_pz2656.reg_st_pdl 		= pz_read_param(&PZ_ST_PDL);
	g_pz2656.reg_spi_mt_dl 	= pz_read_param(&PZ_SPI_MT_DL);
	g_pz2656.reg_spi_st_dl 	= pz_read_param(&PZ_SPI_ST_DL);
#endif

	if (flash_need_to_be_saved)
	{
		FLASH_Configuration_Save();
	}
	
	HAL_Delay(100); // Wait to initialize all pheripherals
	
	REG_Update(REG_NV_RW_CAN_ID, REG_NV_RW_PID_IQ_KI_DIV_POW2);

	// TIMERS Init
	HAL_TIM_Base_Start_IT(&htim6); 	// Enable 10 kHz timer for fast calculation
	HAL_TIM_Base_Start_IT(&htim7);  	// Enable  1 kHz timer for FSM
}

// MOTOR
void motor_start(Working_Mode_t mode, int16_t goal) 
{
	g_joint_command.working_mode = mode;

	switch (g_joint_command.working_mode)
	{
		case TORQUE_MODE:
			g_joint_command._motor_torque = goal;
			MC_ProgramTorqueRampMotor1(g_joint_command._motor_torque, 0);
			MC_StartMotor1();
			break;

		case SPEED_MODE:
			g_joint_command._motor_speed = goal;
			MC_ProgramSpeedRampMotor1(g_joint_command._motor_speed, 0);
			MC_StartMotor1();
			break;

		default:
			motor_stop();
			break;
	}
}

void motor_stop() 
{
//	if (g_joint_status.stm_state_motor == RUN)
	if (g_joint_status.stm_state_motor != IDLE)
	{
		// clear torque readings
		for (int i = 0; i < CURRENT_TORQUE_DATA_SIZE; i++)
		{
			g_joint_status._current_torque_data[i] = 0;
		}

		g_joint_command._motor_torque = 0;
		g_joint_command.motor_torque = 0;
		g_joint_command.joint_torque = 0;

		g_joint_command._motor_speed = 0;
		g_joint_command.motor_speed = 0;
		g_joint_command.joint_speed = 0;

		MC_StopMotor1();
	}
}

void Read_MC_Encoder_1kHz() 
{
	// READ POSITION
	g_joint_status.mc_previous_motor_position = g_joint_status.mc_current_motor_position; // store old position
	g_joint_status.mc_current_motor_position = ENCODER_M1.PreviousCapture; // 0 ... 14336 - 1 mechanical motor rotation

	if (g_joint_status.mc_previous_motor_position > ENCODER_M1.PulseNumber - ENCODER_M1.PulseNumber / 4 && g_joint_status.mc_current_motor_position  < ENCODER_M1.PulseNumber / 4)
	{
		g_joint_status.mc_current_motor_rotation++;
	}
	if (g_joint_status.mc_current_motor_position  > ENCODER_M1.PulseNumber - ENCODER_M1.PulseNumber / 4 && g_joint_status.mc_previous_motor_position < ENCODER_M1.PulseNumber / 4)
	{
		g_joint_status.mc_current_motor_rotation--;
	}

	g_joint_status.mc_current_motor_position_multiturn = g_joint_status.mc_current_motor_position + g_joint_status.mc_current_motor_rotation * ENCODER_M1.PulseNumber;

	// READ SPEED
	if (g_joint_status.mc_current_motor_position - g_joint_status.mc_previous_motor_position > ENCODER_M1.PulseNumber / 2)
	{
		g_joint_status._current_speed_data[g_joint_status._current_speed_index++] = g_joint_status.mc_current_motor_position - g_joint_status.mc_previous_motor_position - ENCODER_M1.PulseNumber;
	}
	else if (g_joint_status.mc_current_motor_position - g_joint_status.mc_previous_motor_position < -1 * ENCODER_M1.PulseNumber / 2)
	{
		g_joint_status._current_speed_data[g_joint_status._current_speed_index++] = g_joint_status.mc_current_motor_position - g_joint_status.mc_previous_motor_position + ENCODER_M1.PulseNumber;
	}
	else
	{
		g_joint_status._current_speed_data[g_joint_status._current_speed_index++] = g_joint_status.mc_current_motor_position - g_joint_status.mc_previous_motor_position;
	}
	g_joint_status._current_speed_index %= CURRENT_SPEED_DATA_SIZE;
}

void Read_MC_Torque() 
{
	// READ TORQUE
	g_joint_status._current_torque_data[g_joint_status._current_torque_index++] = MC_GetPhaseCurrentAmplitudeMotor1();
	g_joint_status._current_torque_index %= CURRENT_TORQUE_DATA_SIZE;
}

void Read_MC_State() 
{
	g_joint_status.stm_state_motor 				= MC_GetSTMStateMotor1();
	g_joint_status.mc_current_faults_motor 		= (uint8_t) MC_GetCurrentFaultsMotor1();
	g_joint_status.mc_occured_faults_motor 		= (uint8_t) MC_GetOccurredFaultsMotor1();

	g_joint_status.current_motor_temperature 		= NTC_GetAvTemp_C(&TempSensor_M1);
	g_joint_status.current_bearing_temperature 	= NTC_GetAvTemp_C(&g_TempBearingSensorParamsM1);

#if PCB_VERSION >= 0x0300
	g_joint_status.gd_nfault = (HAL_GPIO_ReadPin(GPIOE, GD_NFAULT_Pin) == GPIO_PIN_RESET) ? (0) : (1);
	g_joint_status.gd_ready = (HAL_GPIO_ReadPin(GPIOE, GD_READY_Pin) == GPIO_PIN_RESET) ? (0) : (1);
#endif
}

void Update_Data_From_MC() 
{
	// POSITION
	g_joint_status.f_current_motor_position = ((double) g_joint_status.mc_current_motor_position_multiturn / ENCODER_M1.PulseNumber) * M_TWOPI;
	g_joint_status.f_current_joint_position_multiturn = (double) g_joint_status.f_current_motor_position / g_joint_configuration.gear_ratio + g_joint_status.f_current_encoder_position_offset;
	if (g_joint_status.f_current_joint_position_multiturn > 0)
	{
		g_joint_status.f_current_joint_position = fmod((double) g_joint_status.f_current_joint_position_multiturn + M_PI, (double) M_TWOPI) - M_PI;
	}
	else
	{
		g_joint_status.f_current_joint_position = fmod((double) g_joint_status.f_current_joint_position_multiturn - M_PI, (double) M_TWOPI) + M_PI;
	}

	// SPEED
	float temp_speed = 0;
	for (int i = 0; i < CURRENT_SPEED_DATA_SIZE; i++)
	{
		temp_speed += (float) g_joint_status._current_speed_data[i];
	}

	g_joint_status.mc_current_motor_speed	= temp_speed / CURRENT_SPEED_DATA_SIZE; // impulses per 1ms
	g_joint_status.f_current_motor_speed 	= ((float) temp_speed / CURRENT_SPEED_DATA_SIZE * 1000.0 / ENCODER_M1.PulseNumber) * M_TWOPI;
	g_joint_status.f_current_joint_speed 	= g_joint_status.f_current_motor_speed / g_joint_configuration.gear_ratio;

	// TORQUE
	// recalculate_torques
	float temp_torque = 0;
	for (int i = 0; i < CURRENT_TORQUE_DATA_SIZE; i++)
	{
		temp_torque += (float) g_joint_status._current_torque_data[i];
	}

	g_joint_status.mc_current_motor_torque			= temp_torque / CURRENT_TORQUE_DATA_SIZE;
	g_joint_status.f_current_motor_torque			= (((float) (temp_torque / CURRENT_TORQUE_DATA_SIZE)) / INT16_MAX) * MAX_READABLE_CURRENT * KT;
	g_joint_status.f_current_joint_torque			= g_joint_status.f_current_motor_torque * g_joint_configuration.gear_ratio;

}

void CheckErrorsAndWarnings() 
{
	bool error = false;

	// MOTOR CONTROL ERROR REACTION
	if ((g_joint_status.mc_current_faults_motor > 0 || g_joint_status.mc_occured_faults_motor > 0) && FSM_Get_State() != FSM_TRANSITION_FAULT_TO_INIT) {
		g_joint_status.errors =  g_joint_status.errors | JOINT_MC_ERROR;
		error = true;
	}
	else
	{
		g_joint_status.errors = g_joint_status.errors & (0xFF ^ JOINT_MC_ERROR);
	}

	// OVERSPEED ERROR REACTION
	if ((fabs(g_joint_status.f_current_joint_speed) > JOINT_SPEED_LIMIT) && FSM_Get_State() != FSM_TRANSITION_FAULT_TO_INIT && g_joint_configuration.speed_limit_enabled == true) {
		g_joint_status.errors =  g_joint_status.errors | JOINT_SPEED_TO_HIGH;
		error = true;
	}

	// HARDWARE ERROR REACTION
	#if PCB_VERSION >= 0x0300
	if (g_joint_status.gd_nfault == 0 && FSM_Get_State() != FSM_TRANSITION_FAULT_TO_INIT) {
		g_joint_status.errors =  g_joint_status.errors | JOINT_HW_ERROR;
		error = true;
	}
	#endif

	// WARNINGS:
	// WORKING AREA
	if (g_joint_status.f_current_joint_position < -1 * g_joint_configuration.joint_working_area && g_joint_configuration.working_area_constrain_enabled)
	{
		g_joint_status.current_joint_position = POSITION_UNDER_WORKING_AREA;
		g_joint_status.warnings = g_joint_status.warnings | JOINT_OUTSIDE_WORKING_AREA;
	}
	else if(g_joint_status.f_current_joint_position > g_joint_configuration.joint_working_area && g_joint_configuration.working_area_constrain_enabled)
	{
		g_joint_status.current_joint_position = POSITION_OVER_WORKING_AREA;
		g_joint_status.warnings = g_joint_status.warnings | JOINT_OUTSIDE_WORKING_AREA;
	}
	else
	{
		g_joint_status.current_joint_position = POSITION_IN_WORKING_AREA;
		g_joint_status.warnings = g_joint_status.warnings & (0xFF ^ JOINT_OUTSIDE_WORKING_AREA);
	}

	// SAFETY
	if (g_joint_status.b_safety_input == 0)
	{
		g_joint_status.warnings = g_joint_status.warnings | JOINT_SAFETY;
	}
	else
	{
		g_joint_status.warnings = g_joint_status.warnings & (0xFF ^ JOINT_SAFETY);
	}

	if (error == true && FSM_Get_State() != FSM_FAULT && FSM_Get_State() != FSM_FAULT_REACTION_ACTIVE && FSM_Get_State() != FSM_TRANSITION_FAULT_REACTION_ACTIVE_TO_FAULT)
	{
		FSM_Activate_State(FSM_FAULT_REACTION_ACTIVE);
	}
}

bool WCHDG_CheckTasks() 
{

	if (g_joint_configuration.canbus_watchdog_enabled == true && g_counters.can_rx_counter == 0 && g_counters.can_tx_counter == 0) {
		return false;
	}
	
	if (g_high_frequency_task_running == 0 && FSM_Get_State() == FSM_OPERATION_ENABLE) {
		return false;
	}

	if (g_medium_frequency_task_running == 0) {
		return false;
	}

	if (g_safety_task == 0) {
		return false;
	}
	
	return true;
}

void WCHDG_ResetCounters() 
{
	g_high_frequency_task_running = 0; // only running in OPERATION_ENABLE
	g_medium_frequency_task_running = 0;
	g_safety_task = 0;
}
// CAN FD
void FDCAN_Set_Filters() 
{
	// CAN FILTERS
	FDCAN_FilterTypeDef   can_filter_config_0; // BROADCAST
	can_filter_config_0.IdType = FDCAN_STANDARD_ID;
	can_filter_config_0.FilterIndex = 0;
	can_filter_config_0.FilterType = FDCAN_FILTER_MASK;
	can_filter_config_0.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	can_filter_config_0.FilterID1 = 0x0F0;
	can_filter_config_0.FilterID2 = 0x70F ;
	HAL_FDCAN_ConfigFilter(&hfdcan1, &can_filter_config_0); //Initialize CAN Filter

	FDCAN_FilterTypeDef   can_filter_config_1; // UNICAST
	can_filter_config_1.IdType = FDCAN_STANDARD_ID;
	can_filter_config_1.FilterIndex = 1;
	can_filter_config_1.FilterType = FDCAN_FILTER_MASK;
	can_filter_config_1.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	can_filter_config_1.FilterID1 = 0x100 + REG_Get_uint8(REG_NV_RW_CAN_ID);
	can_filter_config_1.FilterID2 = 0x70F;
	HAL_FDCAN_ConfigFilter(&hfdcan1, &can_filter_config_1); //Initialize CAN Filter

}

// FLASH
void Flash_Read_Data(uint32_t StartPageAddress, uint32_t *RxBuf, uint16_t numberofwords) 
{
	while (1)
	{
		*RxBuf = *(__IO uint32_t *) StartPageAddress;
		StartPageAddress += 4;
		RxBuf++;
		if (!(numberofwords--)) break;
	}
}

bool FLASH_Configuration_Check()
{
	uint32_t _config = 0;
	Flash_Read_Data(g_flash_address_configuration, (uint32_t *) _config, 1);
	if (_config == 0xFFFFFFFF) return false;
	else return true;
}	

void FLASH_Configuration_Load() 
{

#ifdef ENCODER_PZ2656
	Flash_Read_Data(g_flash_address_configuration, (uint32_t *) g_registers_flash, 64 * 2);
#endif

	g_joint_configuration.joint_working_area 								= M_PI * (180.0 - 7.655 - 1.0) / 180.0;
}

uint32_t FLASH_Configuration_Save() 
{
	// wylaczenie enkoder absolutnego
	g_joint_configuration.absolute_encoder_enabled = false;
	g_pz2656.started = false;
	volatile uint32_t error = 0;
	HAL_TIM_Base_Stop_IT(&htim6); 	// Enable 10 kHz timer for fast calculation
	HAL_TIM_Base_Stop_IT(&htim7);  	// Enable  1 kHz timer for FSM

	HAL_FLASH_Unlock();

	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
	
#ifdef ENCODER_PZ2656

	FLASH_EraseInitTypeDef EraseInitStruct;
	EraseInitStruct.TypeErase 	= FLASH_TYPEERASE_PAGES;
	EraseInitStruct.Banks 			= FLASH_BANK_1;
	EraseInitStruct.Page 				= (g_flash_address_configuration & 0x07FFFFFF) / FLASH_PAGE_SIZE;
	EraseInitStruct.NbPages 		= 1; // 1 - 2kB
	uint32_t PageError;

	if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
	{
		error = HAL_FLASH_GetError ();
	}

	// RW
	for( int i = 0; i < 32; i++) 
	{
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, g_flash_address_configuration + i * 8, g_registers_flash[i].flash) != HAL_OK)
		{
			error = HAL_FLASH_GetError ();
		}
	}

#endif

	HAL_FLASH_Lock();

	HAL_TIM_Base_Start_IT(&htim6); 	// Enable 10 kHz timer for fast calculation
	HAL_TIM_Base_Start_IT(&htim7);  	// Enable  1 kHz timer for FSM

	return error;
}


// Temperature
float Calculate_ADC_to_Temperature(uint16_t adc_data, uint16_t beta, uint8_t ntc_nominal_temperature, uint16_t ntc_nominal_resistance, uint16_t series_resistance) 
{
	double vo = (double) adc_data / 65536.0;

	uint16_t ntc_resistance = series_resistance * (1.0 - vo) / vo ;

//	g_joint_status.current_bearing_temperature = ntc_resistance;

	float steinhart;

	steinhart = (double) ntc_resistance / ntc_nominal_resistance; // (R/Ro)
	steinhart = log(steinhart); // ln(R/Ro)
	steinhart /= beta; // 1/B * ln(R/Ro)
	steinhart += 1.0 / (ntc_nominal_temperature + 273.15); // + (1/To)
	steinhart = 1.0 / steinhart; // Invert
	steinhart -= 273.15; // convert to C

	return steinhart;
}


// FSM
bool FSM_START_Callback() 
{
	HAL_TIM_Base_Stop_IT(&htim6); // Disable 10 kHz timer

	g_joint_configuration.gear_ratio = GEAR_RATIO;
	
	// FDCAN
	HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1, 10, 0);
	HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1);

	FDCAN_Set_Filters();

	HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, 3, 3, FDCAN_FILTER_REMOTE, FDCAN_REJECT_REMOTE);
	HAL_FDCAN_Start(&hfdcan1); //Initialize CAN Bus
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);// Initialize CAN Bus Rx Interrupt
	HAL_FDCAN_EnableISOMode(&hfdcan1);

	// FSM_Activate_Transition(FSM_TRANSITION_START_TO_INIT);

	HAL_TIM_Base_Start_IT(&htim6); // Enable 10 kHz timer
	
	return true;
}

bool FSM_READY_TO_OPERATE_Callback() 
{
	motor_stop();

	return true;
}

bool FSM_OPERATION_ENABLE_Callback() 
{
	int16_t goal = 0;

	switch (g_joint_command.working_mode)
	{
		case TORQUE_MODE:
		{
			goal = g_joint_command._motor_torque;
			break;
		}

		case SPEED_MODE:
		{
			goal = g_joint_command._motor_speed;
			break;
		}

		default:
		{
			goal = 0;
			g_joint_command._motor_speed = 0;
			g_joint_command._motor_torque = 0;
			break;
		}
	}

	if (g_joint_configuration.working_area_constrain_enabled)
	{
		switch (g_joint_status.current_joint_position)
		{
			case POSITION_UNDER_WORKING_AREA: // Accept only positive torque
				if (goal < 0)
				{
					motor_stop();
				}
				else
				{
					motor_start(g_joint_command.working_mode, goal);
				}
				break;

			case POSITION_IN_WORKING_AREA:
				motor_start(g_joint_command.working_mode, goal);
				break;

			case POSITION_OVER_WORKING_AREA: // Accept only negative torque
				if (goal > 0)
				{
					motor_stop();
				}
				else
				{
					motor_start(g_joint_command.working_mode, goal);
				}
				break;
		}
	}
	else
	{
		motor_start(g_joint_command.working_mode, goal);
	}

	return true;
	
}

bool FSM_TRANSITION_OPERATION_ENABLE_TO_READY_TO_OPERATE_Callback()
{
	motor_stop();

	return true;
}

bool FSM_TRANSITION_READY_TO_OPERATE_TO_OPERATION_ENABLE_Callback()
{
	motor_start(g_joint_command.working_mode, 0);
	// sprawdzenie czy MC jest w RUN
	if ( g_joint_status.stm_state_motor == RUN)
		return true;	
	else
		return false;
}

bool FSM_FAULT_REACTION_ACTIVE_Callback()
{
	motor_stop();

	return true;	
}

bool FSM_TRANSITION_FAULT_REACTION_ACTIVE_TO_FAULT_Callback() 
{
	motor_stop();

	return true;	
}

bool FSM_FAULT_Callback() 
{
	motor_stop();

	return true;
}

bool FSM_TRANSITION_FAULT_TO_INIT_Callback() 
{
	MC_AcknowledgeFaultMotor1();
	g_joint_status.errors = 0;

	return true;	
}

void FSM_Tick_Callback() 
{
	
	// FSM
	switch (FSM_Get_State()) {

#ifdef ENCODER_PZ2656
		// ADJUSTMENT DIGITAL
		case FSM_CALIBRATION_PZ_PHASE_1_STEP_1:

			g_joint_configuration.absolute_encoder_enabled = false; 

			uint16_t offset = 0;
			REG_Set(0x22, &offset);
		
			pz_write_param(&PZ_FCL, 			0x00);
			pz_write_param(&PZ_FCS, 			0x00);
			pz_write_param(&PZ_MT_PDL, 		0x00);
			pz_write_param(&PZ_ST_PDL, 		0x10);
			pz_write_param(&PZ_SPI_MT_DL, 0x00);
			pz_write_param(&PZ_SPI_ST_DL, 0x10);	
		
			// Precise Mechanical Alignment
			pz_write_param(&PZ_AI_P_SEL, 	0x07); // Set <Dynamic Adjustment Phase Select> to ’7’ AI_P_SEL = 0x07.
			pz_write_param(&PZ_AI_S_SEL, 	0x07); // Set <Dynamic Adjustment Scale Select> to ’7’ AI_S_SEL = 0x07
		
			pz_write_param(&PZ_AC_ETO, 		0x01);
			pz_write_param(&PZ_AC_COUNT, 	0x06);
			pz_write_param(&PZ_AC_SEL1, 	0x0F);
			pz_write_param(&PZ_AC_SEL2, 	0x00);
		
			motor_start(TORQUE_MODE, 0);
			FSM_Activate_State(FSM_CALIBRATION_PZ_PHASE_1_STEP_2);
			break;

		case FSM_CALIBRATION_PZ_PHASE_1_STEP_2: // encoder align process - waits to finish
			if (g_joint_status.stm_state_motor != RUN) 
			{
				motor_start(TORQUE_MODE, 0);
			}
			else
			{
				FSM_Activate_State(FSM_CALIBRATION_PZ_PHASE_1_STEP_3);
			}
			break;
			
		case FSM_CALIBRATION_PZ_PHASE_1_STEP_3:
			motor_start(SPEED_MODE, 280);
		
			pz_write_command(PZ_COMMAND_AUTO_ADJ_DIG); // AUTO_ADJ_DIG
			g_pz2656.diag = pz_read_param(&PZ_CMD_STAT); // CMD_STAT
		
			static uint16_t fsm_calib_pz_p1_s3_counter;
			fsm_calib_pz_p1_s3_counter++;
			if (fsm_calib_pz_p1_s3_counter > 25000 && g_pz2656.diag == 0x00) { // run 25s
				FSM_Activate_State(FSM_CALIBRATION_PZ_PHASE_1_STEP_4);
			}
			break;

		case FSM_CALIBRATION_PZ_PHASE_1_STEP_4:
			{
				// Koniec Precise Mechanical Alignment
				motor_start(TORQUE_MODE, 0);
			
				g_pz2656.reg_ai_scale = pz_read_param(&PZ_AI_SCALES); // AI_SCALES
				g_pz2656.reg_ai_phase = pz_read_param(&PZ_AI_PHASES); // AI_PHASES
				uint16_t reg_ai_scale = g_pz2656.reg_ai_scale;
				uint16_t reg_ai_phase = g_pz2656.reg_ai_phase;
				REG_Set(0x18, (uint16_t *) &reg_ai_scale);
				REG_Set(0x19, (uint16_t *) &reg_ai_phase);

				// disable dynamic adjustment by setting AI_S_SEL = 0x00 and AI_P_SEL = 0x00.
				pz_write_param(&PZ_AI_P_SEL, 0x00);
				pz_write_param(&PZ_AI_S_SEL, 0x00);

				pz_write_param(&PZ_AI_SCALE, REG_Get_uint16(0x18));
				pz_write_param(&PZ_AI_PHASE, REG_Get_uint16(0x19));

				FSM_Activate_State(FSM_CALIBRATION_PZ_PHASE_2_STEP_1);
			}
			break;

		// ADJUSTMENT ANALOG
		case FSM_CALIBRATION_PZ_PHASE_2_STEP_1:
			// Filter Configuration before Analog Adjustment
			pz_write_param(&PZ_IPO_FILT1, 0x6E); // Set <Filter Parameter 1> to ’Before analog...’ IPO_FILT1 = 0x6E = 0d110
			pz_write_param(&PZ_IPO_FILT2, 0x04); // Set <Filter Parameter 2> to ’Suitable for any...’ IPO_FILT2 = 0x04
		
			// Analog Autocalibration
			pz_write_param(&PZ_AC_ETO, 		0x01); // Low speed operation
			pz_write_param(&PZ_AC_COUNT, 	0x06); // Autocalibration count
			pz_write_param(&PZ_AC_SEL1, 	0x0F); // Autocalibration select count
			pz_write_param(&PZ_AC_SEL2, 	0x00); // Autocalibration select end

			motor_start(TORQUE_MODE, 0);
			FSM_Activate_State(FSM_CALIBRATION_PZ_PHASE_2_STEP_2);

			break;

		case FSM_CALIBRATION_PZ_PHASE_2_STEP_2: // encoder align process - waits to finish
			if (g_joint_status.stm_state_motor != RUN) 
			{
				motor_start(TORQUE_MODE, 0);
			}
			else
			{
				FSM_Activate_State(FSM_CALIBRATION_PZ_PHASE_2_STEP_3);
			}
			break;

		case FSM_CALIBRATION_PZ_PHASE_2_STEP_3:
			motor_start(SPEED_MODE, 280);
		
			pz_write_command(PZ_COMMAND_AUTO_ADJ_DIG); // AUTO_ADJ_DIG
			g_pz2656.diag = pz_read_param(&PZ_CMD_STAT); // CMD_STAT
		
			static uint16_t fsm_calib_pz_p2_s3_counter;
			fsm_calib_pz_p2_s3_counter++;
			if (fsm_calib_pz_p2_s3_counter > 25000 && g_pz2656.diag == 0x00) { // run 25s
				FSM_Activate_State(FSM_CALIBRATION_PZ_PHASE_2_STEP_4);
			}
			break;

		case FSM_CALIBRATION_PZ_PHASE_2_STEP_4:
			{
				// Koniec Analog Adjustment
				motor_start(TORQUE_MODE, 0);
			
				uint16_t reg_cos_off 	= pz_read_param(&PZ_COS_OFFS); // PZ_COS_OFFS
				uint16_t reg_sin_off 	= pz_read_param(&PZ_SIN_OFFS); // PZ_SIN_OFFS
				uint16_t reg_sc_gain 	= pz_read_param(&PZ_SC_GAINS); // PZ_SC_GAINS
				uint16_t reg_sc_phase	= pz_read_param(&PZ_SC_PHASES); // PZ_SC_PHASES
			
				REG_Set(0x1A, &reg_cos_off);
				REG_Set(0x1B, &reg_sin_off);
				REG_Set(0x1C, &reg_sc_gain);
				REG_Set(0x1D, &reg_sc_phase);

				pz_write_param(&PZ_IPO_FILT1, 0xEA); // Set <Filter Parameter 1> to ’After analog...’ IPO_FILT1 = 0xEA = 0d234
				pz_write_param(&PZ_IPO_FILT2, 0x04); // Set <Filter Parameter 2> to ’Suitable for any...’ IPO_FILT2 = 0x04
			
				g_joint_configuration.absolute_encoder_enabled = true;
			
				FSM_Activate_State(FSM_CALIBRATION_PZ_FINISH);
			}
			break;

		case FSM_CALIBRATION_PZ_PHASE_4_STEP_1:
			motor_start(SPEED_MODE, 0);

			static uint16_t fsm_calib_pz_p4_s1_counter;
		
			if (fsm_calib_pz_p4_s1_counter > CALIBRATION_ZERO_POSITION_COUNTER) {
				motor_start(SPEED_MODE, 0);
				// wyliczenie mediany z tablicy

				// Kolejny krok
				g_joint_configuration.safety_enabled = false;
				fsm_calib_pz_p4_s1_counter = 0;
				motor_start(SPEED_MODE, -200);

				FSM_Activate_State(FSM_CALIBRATION_PZ_PHASE_4_STEP_4);
			} else {
				g_joint_configuration.absolute_encoder_enabled = true;
				
				fsm_calib_pz_p4_s1_counter++;
				// wylaczenie sprawdzanie safety, wlaczenie ruchu ++
				g_joint_configuration.safety_enabled = false;
				motor_start(SPEED_MODE, 50);
				
				FSM_Activate_State(FSM_CALIBRATION_PZ_PHASE_4_STEP_2);
			}
			break;

		case FSM_CALIBRATION_PZ_PHASE_4_STEP_2:
			// jak zgasnie safety, zalaczenie sprawdzanie safety
			if (g_joint_status.b_safety_input == 1) {
				g_joint_configuration.safety_enabled = true;
				motor_start(SPEED_MODE, -5);
				FSM_Activate_State(FSM_CALIBRATION_PZ_PHASE_4_STEP_3);
			}
			break;

		case FSM_CALIBRATION_PZ_PHASE_4_STEP_3:
			if (g_joint_status.b_safety_input == 0) {
				g_encoder_position[1][fsm_calib_pz_p4_s1_counter - 1] = REG_Get_uint16(0xD8);
				g_joint_configuration.safety_enabled = false;
				FSM_Activate_State(FSM_CALIBRATION_PZ_PHASE_4_STEP_1);
			}
			break;

		case FSM_CALIBRATION_PZ_PHASE_4_STEP_4:
			motor_start(SPEED_MODE, 0);
	
			if (fsm_calib_pz_p4_s1_counter > CALIBRATION_ZERO_POSITION_COUNTER) {
				motor_start(SPEED_MODE, 0);

				// wyliczenie mediany z tablicy
				qsort(g_encoder_position[0], CALIBRATION_ZERO_POSITION_COUNTER, sizeof(uint16_t), cmpfunc);
				qsort(g_encoder_position[1], CALIBRATION_ZERO_POSITION_COUNTER, sizeof(uint16_t), cmpfunc);
				
				g_encoder_position_summary[0][0] = ((float) g_encoder_position[0][0] / UINT16_MAX) * 360.0;
				g_encoder_position_summary[0][1] = ((float) g_encoder_position[0][CALIBRATION_ZERO_POSITION_COUNTER / 2] / UINT16_MAX) * 360.0;
				g_encoder_position_summary[0][2] = ((float) g_encoder_position[0][CALIBRATION_ZERO_POSITION_COUNTER - 1] / UINT16_MAX) * 360.0;
				g_encoder_position_summary[1][0] = ((float) g_encoder_position[1][0] / UINT16_MAX) * 360.0;
				g_encoder_position_summary[1][1] = ((float) g_encoder_position[1][CALIBRATION_ZERO_POSITION_COUNTER / 2] / UINT16_MAX) * 360.0;
				g_encoder_position_summary[1][2] = ((float) g_encoder_position[1][CALIBRATION_ZERO_POSITION_COUNTER - 1] / UINT16_MAX) * 360.0;

//				g_pz2656.offset = (uint16_t) (((uint32_t) g_encoder_position[0][CALIBRATION_ZERO_POSITION_COUNTER / 2] + (uint32_t) g_encoder_position[1][CALIBRATION_ZERO_POSITION_COUNTER / 2]) / 2);
				uint16_t offset = (uint16_t) (((uint32_t) g_encoder_position[0][CALIBRATION_ZERO_POSITION_COUNTER / 2] + (uint32_t) g_encoder_position[1][CALIBRATION_ZERO_POSITION_COUNTER / 2]) / 2);
				REG_Set(0x22, &offset);
				REG_Set(REG_NV_RW_PZ_OFFSET, &offset);
				
				motor_start(TORQUE_MODE, 0);
				FSM_Activate_State(FSM_CALIBRATION_PZ_FINISH);
			} else {
				g_joint_configuration.absolute_encoder_enabled = true;
				
				fsm_calib_pz_p4_s1_counter++;
				// wylaczenie sprawdzanie safety, wlaczenie ruchu ++
				g_joint_configuration.safety_enabled = false;
				motor_start(SPEED_MODE, -50);
				
				FSM_Activate_State(FSM_CALIBRATION_PZ_PHASE_4_STEP_5);
			}
			break;

		case FSM_CALIBRATION_PZ_PHASE_4_STEP_5:
			// jak zgasnie safety, zalaczenie sprawdzanie safety
			if (g_joint_status.b_safety_input == 1) {
				g_joint_configuration.safety_enabled = true;
				motor_start(SPEED_MODE, 5);
				FSM_Activate_State(FSM_CALIBRATION_PZ_PHASE_4_STEP_6);
			}
			break;

		case FSM_CALIBRATION_PZ_PHASE_4_STEP_6:
			if (g_joint_status.b_safety_input == 0) {
				g_encoder_position[0][fsm_calib_pz_p4_s1_counter - 1] = REG_Get_uint16(0xD8);
				g_joint_configuration.safety_enabled = false;
				FSM_Activate_State(FSM_CALIBRATION_PZ_PHASE_4_STEP_4);
			}
			break;
					
#endif

		case FSM_CALIBRATION_PZ_FINISH:
			g_joint_configuration.absolute_encoder_enabled = false;
			motor_stop();
		
			// ustawienie rejestrow

			if (g_joint_status.stm_state_motor == IDLE) {
				// Zapis do eeprom
				pz_write_command(PZ_COMMAND_CONF_WRITE_ALL);
				FSM_Activate_State(FSM_CALIBRATION_PZ_STORE_CONFIGURATION); // CALIBRATION FINISHED - GO TO INIT STATE
			}
			break;

		case FSM_CALIBRATION_PZ_STORE_CONFIGURATION:
			{
				// Sprawdzenie statusu zapisu EEPROM
				uint16_t cmd_stat	= pz_read_param(&PZ_CMD_STAT);
				if (cmd_stat == 0x00)
				{
					FSM_Activate_State(FSM_INIT); // CALIBRATION FINISHED - GO TO INIT STATE
				}
			}
			break;

		default:
			FSM_Activate_State(FSM_FAULT_REACTION_ACTIVE);
			break;
	}
}

bool FSM_Switch_State_Callback() // FIXME running transition should block changing state to new one - add flag transition is running
{
	
	switch (g_fsm_status.new_state)
	{
	
		case FSM_CALIBRATION_PZ_PHASE_1_STEP_1:
		{

			if (FSM_Get_State() == FSM_INIT)
			{
				return FSM_Activate_State(FSM_CALIBRATION_PZ_PHASE_1_STEP_1);
			}

			break;
		}

		case FSM_CALIBRATION_PZ_PHASE_4_STEP_1:
		{

			if (FSM_Get_State() == FSM_INIT)
			{
				return FSM_Activate_State(FSM_CALIBRATION_PZ_PHASE_4_STEP_1);
			}

			break;
		}
		
		default:
		{
			break;
		}
	}

	return false;
}

// HAL Callbacks
//void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef * hspi)
//{
//	//ABSOLUTE_ENCODER_CS_GPIO_Port->BRR = (uint32_t) ABSOLUTE_ENCODER_CS_Pin;
//	HAL_GPIO_WritePin(ABSOLUTE_ENCODER_CS_GPIO_Port, ABSOLUTE_ENCODER_CS_Pin, GPIO_PIN_SET);

//	switch (buf_rx[0]) {
//		case 0xA6: // read position
//			{
//				uint16_t readings = ((uint16_t) buf_rx[1] << 8 | (uint16_t) buf_rx[2]);
//				REG_Set(0xD8, (uint16_t *) &readings);
//				g_pz2656.readings = REG_Get_uint16(0xD8);
//								
//				if (g_pz2656.started == true)
//				{
//					bufsize = 6;
//					buf_tx[0] = 0xA6;

////					for (uint16_t i = 1; i < bufsize; i++) {
////						buf_tx[i] = 0x00;
////					}

//					//ABSOLUTE_ENCODER_CS_GPIO_Port->BSRR = (uint32_t) ABSOLUTE_ENCODER_CS_Pin;
//					HAL_GPIO_WritePin(ABSOLUTE_ENCODER_CS_GPIO_Port, ABSOLUTE_ENCODER_CS_Pin, GPIO_PIN_RESET);
//					HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t *) &buf_tx, (uint8_t *) &buf_rx, bufsize);
//				}

//		//			// next command - send diagnostic
//		//			bufsize = 2;
//		//			buf_tx[0] = 0x68;
//		//			buf_tx[1] = 0x00;
//		//			HAL_GPIO_WritePin(ABSOLUTE_ENCODER_CS_GPIO_Port, ABSOLUTE_ENCODER_CS_Pin, GPIO_PIN_RESET);
//		//			HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t *) &buf_tx, (uint8_t *) &buf_rx, bufsize);
//			}
//			
//			break;

////		case 0x68: // diag
////			g_pz2656.diag = buf_rx[1];
////			// next command - send diagnostic
////			bufsize = 2;
////			buf_tx[0] = 0x69;
////			buf_tx[1] = 0x00;
////			HAL_GPIO_WritePin(ABSOLUTE_ENCODER_CS_GPIO_Port, ABSOLUTE_ENCODER_CS_Pin, GPIO_PIN_RESET);
////			HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t *) &buf_tx, (uint8_t *) &buf_rx, bufsize);
////			break;

////		case 0x69: // diag
////			g_pz2656.diag = buf_rx[1] << 8;
////			// next command - send diagnostic
////			bufsize = 2;
////			buf_tx[0] = 0x6A;
////			buf_tx[1] = 0x00;
////			HAL_GPIO_WritePin(ABSOLUTE_ENCODER_CS_GPIO_Port, ABSOLUTE_ENCODER_CS_Pin, GPIO_PIN_RESET);
////			HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t *) &buf_tx, (uint8_t *) &buf_rx, bufsize);
////			break;

////		case 0x6A: // diag
////			g_pz2656.diag = buf_rx[1] << 16;
////			// next command - send diagnostic
////			bufsize = 2;
////			buf_tx[0] = 0x6B;
////			buf_tx[1] = 0x00;
////			HAL_GPIO_WritePin(ABSOLUTE_ENCODER_CS_GPIO_Port, ABSOLUTE_ENCODER_CS_Pin, GPIO_PIN_RESET);
////			HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t *) &buf_tx, (uint8_t *) &buf_rx, bufsize);
////			break;

////		case 0x6B: // diag
////			g_pz2656.diag = buf_rx[1] << 24;
////			break;
//	}

//	g_counters.spi_txrx_counter++;

//}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) 
{
	if(htim->Instance == TIM6) 	// 10kHz (5,0) - fast recalculation
	{
		Read_MC_Torque();
		
		g_joint_status.b_safety_input = (HAL_GPIO_ReadPin(SEC_IN_GPIO_Port, SEC_IN_Pin) == GPIO_PIN_SET) ? (0) : (1);
				
		g_counters.timer6++;
	}
	
	if(htim->Instance == TIM7) 	// 1kHz (5,1) - 1ms - main loop
	{	
		// Update data
		Read_MC_Encoder_1kHz();
		Update_Data_From_MC(); // Przelicz na jednostki we floatach
		Read_MC_State(); // Sprawdzenie stanu MC SDK

		// Odczyt enkoderow absolutnych

#ifdef ENCODER_PZ2656
		if (g_joint_configuration.absolute_encoder_enabled == true)
		{
			if (REG_Get_uint16(REG_NV_RW_PZ_OFFSET) != 0)
			{
				g_joint_configuration.calibration_state = JOINT_CALIBRATED;
			}
//			if (!g_pz2656.started) {
//				g_pz2656.started = true;
			bufsize = 6;
			buf_tx[0] = 0xA6;

			for (uint16_t i = 1; i < bufsize; i++) {
				buf_tx[i] = 0x00;
			}

			HAL_GPIO_WritePin(ABSOLUTE_ENCODER_CS_GPIO_Port, ABSOLUTE_ENCODER_CS_Pin, GPIO_PIN_RESET);
			//HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t *) &buf_tx, (uint8_t *) &buf_rx, bufsize);
			HAL_SPI_TransmitReceive(&hspi1, (uint8_t *) &buf_tx, (uint8_t *) &buf_rx, bufsize, 1);
			HAL_GPIO_WritePin(ABSOLUTE_ENCODER_CS_GPIO_Port, ABSOLUTE_ENCODER_CS_Pin, GPIO_PIN_SET);
			
			uint16_t readings = ((uint16_t) buf_rx[1] << 8 | (uint16_t) buf_rx[2]);
			REG_Set(0xD8, (uint16_t *) &readings);
			g_pz2656.readings = REG_Get_uint16(0xD8);
			//g_pz2656.readings = ((uint16_t) buf_rx[1] << 8 | (uint16_t) buf_rx[2]);

			//}

			if (REG_Get_uint8(REG_NV_RW_REVERT_DIRECTION)) {
				g_pz2656.angle 			= UINT16_MAX - (uint16_t) ((int32_t) REG_Get_uint16(0xD8) - (int32_t) REG_Get_uint16(REG_NV_RW_PZ_OFFSET));
			} 
			else
			{
				g_pz2656.angle 			= (uint16_t) ((int32_t) REG_Get_uint16(0xD8) - (int32_t) REG_Get_uint16(REG_NV_RW_PZ_OFFSET));
			}
			
			g_joint_status.f_current_joint_position_from_absolute_encoder =  fmod((double) ((double) g_pz2656.angle / UINT16_MAX) * M_TWOPI + M_PI, (double) M_TWOPI) - M_PI ;
		}
#endif

		if ( FSM_Get_State() > 0) // W przypadku, gdy juz CAN uruchomiony, wlaczam obsluge bledów
		{
			CheckErrorsAndWarnings(); // Sprawdzenie czy nie wygenerowaly sie jakies bledy czy ostrzezenia
		}

		g_joint_status.mc_current_electric_rotation = g_current_electrical_rotation; // pobranie informacji aktualnej z MC

		// Reakcja na niski stan safety - tylko gdy opcja wlaczona oraz joint w trybie pracy
		if(g_joint_configuration.safety_enabled == true && g_joint_status.b_safety_input == false && FSM_Get_State() == FSM_OPERATION_ENABLE)
		{
				FSM_Activate_State(FSM_READY_TO_OPERATE);
		}

		// WCHDG - 100Hz
		if (g_counters.timer7 % 10 == 0) {
			g_wchdg_ok = WCHDG_CheckTasks();
			if (!g_wchdg_ok) HAL_NVIC_SystemReset();
			WCHDG_ResetCounters();
		}
		
		// FSM
		FSM_Switch_State();			
		FSM_Tick();

		g_counters.timer7++;
		
	}
}

void REG_Set(uint8_t rejestr, uint16_t * data) 
{
	uint16_t * p_g_registers = (uint16_t *) &g_registers_flash;

	*(p_g_registers + rejestr) = data[0];
}

void REG_Get(uint8_t rejestr, uint16_t * data) 
{
	uint16_t * p_g_registers = (uint16_t *) &g_registers_flash;

	data[0] = *(p_g_registers + rejestr);
}

uint16_t * REG_Get_pointer(uint8_t rejestr)
{
	uint16_t * p_g_registers = (uint16_t *) &g_registers_flash;
	
	return p_g_registers + rejestr;
}

uint8_t REG_Get_uint8(uint8_t rejestr) 
{
	uint16_t * p_g_registers = (uint16_t *) &g_registers_flash;

	uint8_t value = (*(p_g_registers + rejestr)) & 0xFF;

	return value;
}

uint16_t REG_Get_uint16(uint8_t rejestr) 
{
	uint16_t * p_g_registers = (uint16_t *) &g_registers_flash;

	uint16_t value = (*(p_g_registers + rejestr)) & 0xFFFF;

	return value;
}

void REG_Update(uint8_t poczatek, uint8_t koniec) 
{
	for (int i = poczatek; i < koniec; i++)
	{
		switch(i)
		{
			case REG_NV_RW_CAN_ID:
				break;

//			case REG_NV_RW_PID_ID_KP_GAIN:
//				{
//					uint16_t regdata16 = REG_Get_uint16(REG_NV_RW_PID_ID_KP_GAIN);
//					PID_SetKP(pPIDId[0], regdata16);
//				}
//				break;
//			
//			case REG_NV_RW_PID_ID_KI_GAIN:
//				{
//					uint16_t regdata16 = REG_Get_uint16(REG_NV_RW_PID_ID_KI_GAIN);
//					PID_SetKI(pPIDId[0], regdata16);
//				}
//				break;
//			
//			case REG_NV_RW_PID_ID_KP_DIV_POW2:
//				{
//					uint16_t regdata16 = REG_Get_uint16(REG_NV_RW_PID_ID_KP_DIV_POW2);
//					PID_SetKIDivisorPOW2(pPIDId[0], regdata16);
//				}
//				break;
//			
//			case REG_NV_RW_PID_ID_KI_DIV_POW2:
//				{
//					uint16_t regdata16 = REG_Get_uint16(REG_NV_RW_PID_ID_KI_DIV_POW2);
//					PID_SetKIDivisorPOW2(pPIDId[0], regdata16);
//				}
//				break;
//			
//			case REG_NV_RW_PID_IQ_KP_GAIN:
//				{
//					uint16_t regdata16 = REG_Get_uint16(REG_NV_RW_PID_IQ_KP_GAIN);
//					PID_SetKP(pPIDIq[0], regdata16);
//				}
//				break;
//			
//			case REG_NV_RW_PID_IQ_KI_GAIN:
//				{
//					uint16_t regdata16 = REG_Get_uint16(REG_NV_RW_PID_IQ_KI_GAIN);
//					PID_SetKI(pPIDIq[0], regdata16);
//				}
//				break;
//			
//			case REG_NV_RW_PID_IQ_KP_DIV_POW2:
//				{
//					uint16_t regdata16 = REG_Get_uint16(REG_NV_RW_PID_IQ_KP_DIV_POW2);
//					PID_SetKIDivisorPOW2(pPIDIq[0], regdata16);
//				}
//				break;
//			
//			case REG_NV_RW_PID_IQ_KI_DIV_POW2:
//				{
//					uint16_t regdata16 = REG_Get_uint16(REG_NV_RW_PID_IQ_KI_DIV_POW2);
//					PID_SetKIDivisorPOW2(pPIDIq[0], regdata16);
//				}
//				break;
			
			default:
				break;
		}
	}
}

//void REG_Write(uint8_t poczatek, uint8_t koniec, uint8_t * data)
//{
//	uint8_t * p_g_registers = (uint8_t *) &g_registers_flash;
//	// ustalenie poczatku pamieci
////	memcpy ( &g_registers + poczatek, data, sizeof(uint16_t) * (koniec - poczatek) );
//	for (int i = 0; i < (koniec - poczatek); i++)
//	{
//		*(p_g_registers + poczatek * 2 + i * 2 + 1) = data[i * 2];
//		*(p_g_registers + poczatek * 2 + i * 2)     = data[i * 2 + 1];
//	}
//}

void REG_Read(uint8_t poczatek, uint8_t koniec, uint8_t * data) 
{
	uint8_t * p_g_registers = (uint8_t *) &g_registers_flash;

	for (int i = 0; i < koniec - poczatek; i++)
	{
		data[i * 2]    = *(p_g_registers + poczatek * 2 + i * 2 + 1);
		data[i * 2 + 1]= *(p_g_registers + poczatek * 2 + i * 2);
	}
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan1, uint32_t RxFifo0ITs) 
{

	if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0)
//	do
	{
		uint32_t fill_level = HAL_FDCAN_GetRxFifoFillLevel(hfdcan1, FDCAN_RX_FIFO0);
		for (int i = 0; i < fill_level; i++)
		{
		
		// RETRIEVE CAN MESSAGE -------------------------------------------------------------------------
		HAL_FDCAN_GetRxMessage(hfdcan1, FDCAN_RX_FIFO0, &can_rx_header, can_rx_data);
		g_counters.can_rx_counter++;

		// RESET SAFE TIMER -----------------------------------------------------------------------------
//		HAL_TIM_Base_Stop(&htim6);
//		TIM6->CNT = 0;
//		HAL_TIM_Base_Start_IT(&htim6);

		bool l_send_response = true;
		uint8_t l_cmd 	= (can_rx_header.Identifier & 0x0F0) >> 4; // Ustalenie polecenie
		bool l_unicast 	= (can_rx_header.Identifier & 0x100) >> 8; // Ustalenie typu transmisji: BROADCAST|UNICAST

		// CAN TRANSMITION CONFIGURATION
		can_tx_header.Identifier = can_rx_header.Identifier | REG_Get_uint8(REG_NV_RW_CAN_ID) | 0x01 << 9;
		can_tx_header.IdType = FDCAN_STANDARD_ID;
		can_tx_header.TxFrameType = FDCAN_DATA_FRAME;
		can_tx_header.DataLength = FDCAN_DLC_BYTES_12;
		can_tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
		can_tx_header.BitRateSwitch = FDCAN_BRS_ON;
		can_tx_header.FDFormat = FDCAN_FD_CAN;
		can_tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
		can_tx_header.MessageMarker = 0;

		// UPDATE JOINT INFO ----------------------------------------------------------------------------
		uint8_t numer_w_szeregu = 0;
		uint8_t dlugosc_danych_polecenia = 0;
		uint8_t offset = 0;

		if (!l_unicast)
		{
			numer_w_szeregu = REG_Get_uint8(REG_NV_RW_CAN_ID); // ???
		}

		switch (l_cmd) {

			case 0x0: // Wykonaj akcje
				{
					dlugosc_danych_polecenia = 2;
					// int16_t w zaleznosci od trybu pracy - torque, speed
					offset = dlugosc_danych_polecenia * numer_w_szeregu;

					int16_t goal;
					goal  = can_rx_data[offset] << 8;
					goal += can_rx_data[offset + 1];
					
					switch (g_joint_command.working_mode)
					{
						case TORQUE_MODE:
							// GOAL TORQUE
							// -------------------------------------------------------------------------------------------------
							// recalculate torque goal data from CAN to floats
							g_joint_command.joint_torque = ((float) goal * MAX_TORQUE_THROUGH_CAN) / (float) INT16_MAX; // convert from int16 to float
							g_joint_command.joint_speed = 0.0;
							break;

						case SPEED_MODE:
							// GOAL SPEED
							// -------------------------------------------------------------------------------------------------
							// recalculate speed goal data from CAN to floats
							g_joint_command.joint_torque = 0.0;
							g_joint_command.joint_speed = ((float) goal * MAX_SPEED_THROUGH_CAN) / (float) INT16_MAX; // convert from int16 to float
							break;

						default:
							g_joint_command.joint_torque = 0.0;
							g_joint_command.joint_speed = 0.0;
							break;
					}

					// recalculate torque goal data from floats to MC
					g_joint_command.motor_torque = g_joint_command.joint_torque / g_joint_configuration.gear_ratio;
					g_joint_command._motor_torque	= (((float) g_joint_command.motor_torque) * INT16_MAX) / (MAX_READABLE_CURRENT * KT);  // convert from float to s16A

					// recalculate speed goal data from floats to MC
					g_joint_command.motor_speed = g_joint_command.joint_speed * g_joint_configuration.gear_ratio;
					g_joint_command._motor_speed = ((float) g_joint_command.motor_speed) * 1.5867768595;  // convert from float to s16A

					// -------------------------------------------------------------------------------------------------
					// Recalculate data from MC to Floats
					Update_Data_From_MC();
					// -------------------------------------------------------------------------------------------------
					// Recalculate data from floats to CAN
					int32_t l_joint_position_in_s32degree = (int32_t) (g_joint_status.f_current_joint_position_from_absolute_encoder * ( (float) UINT32_MAX / M_TWOPI));

					int16_t l_speed_in_dpp = g_joint_status.f_current_joint_speed * (float) INT16_MAX / MAX_SPEED_THROUGH_CAN;

					int16_t l_current_torque_in_s16a = (g_joint_status.f_current_joint_torque / MAX_TORQUE_THROUGH_CAN) * (float) INT16_MAX;
					if (g_joint_command.motor_torque < 0 || g_joint_command.motor_speed < 0)
					{
						l_current_torque_in_s16a = -l_current_torque_in_s16a;
					}

					// -------------------------------------------------------------------------------------------------
					can_tx_data[0] 	= l_joint_position_in_s32degree >> 24;
					can_tx_data[1] 	= l_joint_position_in_s32degree >> 16;
					can_tx_data[2] 	= l_joint_position_in_s32degree >> 8;
					can_tx_data[3] 	= l_joint_position_in_s32degree;
					can_tx_data[4] 	= l_speed_in_dpp >> 8;
					can_tx_data[5] 	= l_speed_in_dpp;
					can_tx_data[6] 	= l_current_torque_in_s16a >> 8;
					can_tx_data[7] 	= l_current_torque_in_s16a;

					can_tx_data[8] 	= (uint8_t) g_joint_status.current_bearing_temperature;
					can_tx_data[9] 	= FSM_Get_State(); // FSM
					can_tx_data[10]	= g_joint_status.mc_current_faults_motor;
					can_tx_data[11] = g_joint_status.mc_occured_faults_motor;
					can_tx_data[12] = g_joint_status.errors;
					can_tx_data[13] = g_joint_status.warnings;

					can_tx_data[16] = (uint8_t) g_joint_status.current_motor_temperature;
					can_tx_header.DataLength = FDCAN_DLC_BYTES_20;
				}
				break;

			case 0x1: // Zmien stan FSM
				{
					dlugosc_danych_polecenia = 1;
					// uint8_t - FSM
					offset = dlugosc_danych_polecenia * numer_w_szeregu;
					FSM_Set_New_State(can_rx_data[offset]);

					//g_fsm_status.current_state = can_rx_data[offset];

					can_tx_data[0] = FSM_Get_State();

					can_tx_header.DataLength = FDCAN_DLC_BYTES_1;
				}
				break;

			case 0x2: // Odczyt rejestrow
				{
					dlugosc_danych_polecenia = 2;
					// int8_t - rejestr poczatkowy
					// int8_t - dlugosc <= 64
					//offset = dlugosc_danych_polecenia * numer_w_szeregu + 2;
					offset = 0;
					uint8_t poczatek = can_rx_data[offset + 0];     // rejestr poczatkowy
					uint8_t dlugosc  = can_rx_data[offset + 1] * 2; // ilosc rejestrow (16bit)
					uint8_t koniec   = poczatek + dlugosc;
					//poczatek = 0;
					// dlugosc danych can - 0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64
					REG_Read(poczatek, koniec, (uint8_t *) &can_tx_data);
					if (dlugosc > 48) 
					{
						can_tx_header.DataLength = FDCAN_DLC_BYTES_64;
					}
					else if (dlugosc > 32) 
					{
						can_tx_header.DataLength = FDCAN_DLC_BYTES_48;
					}
					else if (dlugosc > 24) 
					{
						can_tx_header.DataLength = FDCAN_DLC_BYTES_32;
					}
					else if (dlugosc > 20) 
					{
						can_tx_header.DataLength = FDCAN_DLC_BYTES_24;
					}
					else if (dlugosc > 16) 
					{
						can_tx_header.DataLength = FDCAN_DLC_BYTES_20;
					}
					else if (dlugosc > 12) 
					{
						can_tx_header.DataLength = FDCAN_DLC_BYTES_16;
					}
					else if (dlugosc > 8) 
					{
						can_tx_header.DataLength = FDCAN_DLC_BYTES_12;
					}
					else
					{
						can_tx_header.DataLength = FDCAN_DLC_BYTES_8;
					}
				}
				break;

			case 0x3: // Zapis rejestrow
				{
					//dlugosc_danych_polecenia = 2;
					// int8_t - rejestr poczatkowy (zapis mozliwy tylko 0-63, 80-191)
					// int8_t - dlugosc
					uint8_t poczatek = can_rx_data[0]; // rejestr poczatkowy
					uint8_t dlugosc  = can_rx_data[1]; // ilosc rejestrow (16bit)
					uint8_t koniec   = poczatek + dlugosc;
					
					offset = dlugosc * numer_w_szeregu + 2;
					// offset + dlugosc nie moze przekroczyc 64 baitow
					
					// sprawdzenie zakresu
//					if ((( (poczatek - 0) | (0 - poczatek) | (koniec - 63) | (63 - koniec) ) >= 0) || (( (poczatek - 80) | (80 - poczatek) | (koniec - 191) | (191 - koniec) ) >= 0) && (offset + dlugosc < 64) )
//					{
					uint16_t register_data;
					for (int i = 0; i < dlugosc; i++) {
						register_data = can_rx_data[2 + i * 2] << 8 | can_rx_data[2 + i * 2 + 1];
						REG_Set(poczatek + i, &register_data);
					}

//						uint8_t * p_data;
//						p_data = (uint8_t *) malloc (dlugosc * 2 * sizeof(uint8_t));
//						memcpy(p_data, (uint8_t * ) &(can_rx_data[offset]), dlugosc * 2 * sizeof(uint8_t));
//						// poprawny zakres
//						REG_Write(poczatek, koniec, (uint8_t *) p_data);
//						free( p_data );

//					}
//					else
//					{
//							// blad zakresu
//					}

					// Aktualizacja wartosci
					REG_Update(poczatek, koniec);
				}
				break;

			case 0x4: // Zapis configuracji na FLASH
				{
					can_tx_data[0] = 0;

					FLASH_Configuration_Save();
//					if (FSM_Get_State() == FSM_INIT)
//					{
//						dlugosc_danych_polecenia = 0;
//						offset = dlugosc_danych_polecenia * numer_w_szeregu;
//						
//						if (FLASH_Configuration_Save() == 0) can_tx_data[0] = 1; // Zapis sie powiódl
//						
//					}
					

					// jak sie powiedzie to 1
					can_tx_data[0] = 1;

					can_tx_header.DataLength = FDCAN_DLC_BYTES_1;
				}
				break;

			case 0xA: // RESET
				{
					l_send_response = false;
					NVIC_SystemReset();
				}
				break;

//			case 0xB: // SET CAN ID
//				{
//					if (FSM_Get_State() == FSM_INIT)
//					{
//						dlugosc_danych_polecenia = 1;
//						// uint8_t - can id
//						uint8_t offset = dlugosc_danych_polecenia * numer_w_szeregu;
//						//g_joint_configuration.can_node_id = can_rx_data[offset]; // moze byc od 0 do F - TODO: zapis do FLASH i restart
//						uint8_t can_node_id = can_rx_data[offset];
//						REG_Set(0x40, ( uint16_t * ) &can_node_id);

//						//FLASH_Configuration_Save(); // flash configuration
//						FDCAN_Set_Filters(); // reload can filters

//						can_tx_data[0] = 1;
//					}
//					else
//					{
//						can_tx_data[0] = 0;
//					}

//					can_tx_header.DataLength = FDCAN_DLC_BYTES_1;
//				}
//				break;

//			case 0xC: // Odczyt rejestrow - EEPROM
//				{
//					dlugosc_danych_polecenia = 2;
//					// int8_t - rejestr poczatkowy
//					// int8_t - dlugosc <= 64
//					offset = dlugosc_danych_polecenia * numer_w_szeregu + 2;
//					uint8_t poczatek = can_rx_data[0]; // rejestr poczatkowy
//					uint8_t dlugosc  = can_rx_data[1] * 2; // ilosc rejestrow (16bit)
//					uint8_t koniec   = poczatek + dlugosc;
//					
//					// dlugosc danych can - 0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64
//					REG_Read(poczatek, koniec, (uint8_t *) &can_tx_data);
//					if (dlugosc > 48) 
//					{
//						can_tx_header.DataLength = FDCAN_DLC_BYTES_64;
//					}
//					else if (dlugosc > 32) 
//					{
//						can_tx_header.DataLength = FDCAN_DLC_BYTES_48;
//					}
//					else if (dlugosc > 24) 
//					{
//						can_tx_header.DataLength = FDCAN_DLC_BYTES_32;
//					}
//					else if (dlugosc > 20) 
//					{
//						can_tx_header.DataLength = FDCAN_DLC_BYTES_24;
//					}
//					else if (dlugosc > 16) 
//					{
//						can_tx_header.DataLength = FDCAN_DLC_BYTES_20;
//					}
//					else if (dlugosc > 12) 
//					{
//						can_tx_header.DataLength = FDCAN_DLC_BYTES_16;
//					}
//					else if (dlugosc > 8) 
//					{
//						can_tx_header.DataLength = FDCAN_DLC_BYTES_12;
//					}
//					else
//					{
//						can_tx_header.DataLength = FDCAN_DLC_BYTES_8;
//					}
//				}
//				break;

//			case 0xD: // Zapis rejestrow - EEPROM - 13 BANKOW, 64 bajty kazdy
//				{
//					//dlugosc_danych_polecenia = 2;
//					// int8_t - rejestr poczatkowy (zapis mozliwy tylko 0-63, 80-191)
//					// int8_t - dlugosc
//					uint8_t poczatek = can_rx_data[0]; // rejestr poczatkowy
//					uint8_t dlugosc  = can_rx_data[1] * 2; // ilosc rejestrow (16bit)
//					uint8_t koniec   = poczatek + dlugosc;
//					
//					offset = dlugosc * numer_w_szeregu + 2;
//					// offset + dlugosc nie moze przekroczyc 64 baitow
//					
//					// sprawdzenie zakresu
////					if ((( (poczatek - 0) | (0 - poczatek) | (koniec - 63) | (63 - koniec) ) >= 0) || (( (poczatek - 80) | (80 - poczatek) | (koniec - 191) | (191 - koniec) ) >= 0) && (offset + dlugosc < 64) )
////					{
//						uint8_t * p_data;
//						p_data = (uint8_t *) malloc (dlugosc * 2 * sizeof(uint8_t));
//						memcpy(p_data, (uint8_t * ) &(can_rx_data[offset]), dlugosc * 2 * sizeof(uint8_t));
//						// poprawny zakres
//						REG_Write(poczatek, koniec, p_data);
//						free( p_data );
////					}
////					else
////					{
////							// blad zakresu
////					}
//				}
//				break;
				
			case 0xF: // Konfiguracja
				{
					dlugosc_danych_polecenia = 2;
					// uint8_t tryb pracy
					offset = dlugosc_danych_polecenia * numer_w_szeregu;
					if (FSM_Get_State() == FSM_INIT || FSM_Get_State() == FSM_READY_TO_OPERATE)
					{
						g_joint_command.working_mode = can_rx_data[offset];
						g_joint_configuration.working_area_constrain_enabled 	= (can_rx_data[offset + 1] & 0x01);
						g_joint_configuration.absolute_encoder_enabled 				= (can_rx_data[offset + 1] & 0x02) >> 1;
						g_joint_configuration.safety_enabled 									= (can_rx_data[offset + 1] & 0x04) >> 2;
						g_joint_configuration.canbus_watchdog_enabled 				= (can_rx_data[offset + 1] & 0x08) >> 3;
						g_joint_configuration.speed_limit_enabled 						= (can_rx_data[offset + 1] & 0x10) >> 4;
						
						can_tx_data[0] = 1;
					}
					else
					{
						can_tx_data[0] = 0;
					}

					can_tx_header.DataLength = FDCAN_DLC_BYTES_1;
				}
				break;

			default:
				l_send_response = false;
				break;

		}

		//	SEND FRAME VIA FD CAN -------------------------------------------------------------------------------
		if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan1, &can_tx_header, can_tx_data) == HAL_OK && l_send_response)
		{
			g_counters.can_tx_counter++;
		}


		// clear global rx and tx buffer
		for (int i = 0; i < 24; i++)
		{
			can_rx_data[i]	= 0;
			can_tx_data[i]	= 0;
		}
		}
	} 
//	while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan1, FDCAN_RX_FIFO0) > 0);
//	}
	
}

//void HAL_FDCAN_TxEventFifoCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t TxEventFifoITs)
//{
//	g_counters.can_tx_counter++;
//}
// PZ
//#pragma GCC push_options
//#pragma GCC optimize ("O0")
#ifdef ENCODER_PZ2656
void pz_spi_transfer(uint8_t *data_tx, uint8_t *data_rx, uint16_t datasize) {

	HAL_GPIO_WritePin(ABSOLUTE_ENCODER_CS_GPIO_Port, ABSOLUTE_ENCODER_CS_Pin, GPIO_PIN_RESET);

#if PCB_VERSION >= 0x0300
		HAL_SPI_TransmitReceive(&hspi1, data_tx, data_rx, datasize, 1);
#else
//		HAL_SPI_TransmitReceive(&hspi2, (uint8_t * ) &send_data, (uint8_t * ) &angle_value, 1, 1);
#endif

	HAL_GPIO_WritePin(ABSOLUTE_ENCODER_CS_GPIO_Port, ABSOLUTE_ENCODER_CS_Pin, GPIO_PIN_SET);
}

#endif

//#pragma GCC pop_options

// MC SDK Override Functions
int16_t NTC_GetAvTemp_C( NTC_Handle_t * pHandle ) 
{
  int32_t wTemp;

  if ( pHandle->bSensorType == REAL_SENSOR )
  {
	  float steinhart = Calculate_ADC_to_Temperature(pHandle->hAvTemp_d, 3924, 25, 10000, 10000);

	  wTemp = steinhart;
  }
  else
  {
    wTemp = pHandle->hExpectedTemp_C;
  }
  return ( ( int16_t )wTemp );
}


uint16_t NTC_CalcAvTemp( NTC_Handle_t * pHandle ) 
{
	
//  uint32_t wtemp;
  uint16_t hAux;

  if ( pHandle->bSensorType == REAL_SENSOR )
  {
    hAux = RCM_ExecRegularConv(pHandle->convHandle);

    if ( hAux != 0xFFFFu )
    {
//      wtemp =  ( uint32_t )( pHandle->hLowPassFilterBW ) - 1u;
//      wtemp *= ( uint32_t ) ( pHandle->hAvTemp_d );
//      wtemp += hAux;
//      wtemp /= ( uint32_t )( pHandle->hLowPassFilterBW );

//      pHandle->hAvTemp_d = ( uint16_t ) wtemp;
		pHandle->hAvTemp_d = ( uint16_t ) hAux;
    }

    pHandle->hFaultState = NTC_SetFaultState( pHandle );
  }
  else  /* case VIRTUAL_SENSOR */
  {
    pHandle->hFaultState = MC_NO_ERROR;
  }

  return ( pHandle->hFaultState );
}

//#if SDK_VERSION >= 0x055a0000

int32_t MCM_Sqrt( int32_t wInput ) 
{
	 int32_t wtemprootnew;

		  if ( wInput > 0 )
		  {
		  uint8_t biter = 0u;
		  int32_t wtemproot;

		    if ( wInput <= ( int32_t )2097152 )
		    {
		      wtemproot = ( int32_t )128;
		    }
		    else
		    {
		      wtemproot = ( int32_t )8192;
		    }

		    do
		    {
		      wtemprootnew = ( wtemproot + wInput / wtemproot ) / ( int32_t )2;
		      if ( wtemprootnew == wtemproot )
		      {
		        biter = 6u;
		      }
		      else
		      {
		        biter ++;
		        wtemproot = wtemprootnew;
		      }
		    }
		    while ( biter < 6u );

		  }
		  else
		  {
		    wtemprootnew = ( int32_t )0;
		  }

		  return ( wtemprootnew );
}
//#endif
