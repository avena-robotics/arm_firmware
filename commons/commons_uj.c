#include "commons_uj.h"
#include "fsm.h"

//#pragma GCC push_options
//#pragma GCC optimize ("O0")

FDCAN_RxHeaderTypeDef can_rx_header; // CAN Bus Transmit Header
FDCAN_TxHeaderTypeDef can_tx_header; // CAN Bus Transmit Header
uint8_t can_rx_data[24] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};  //CAN Bus Receive Buffer
uint8_t can_tx_data[24] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};  //CAN Bus Send Buffer

uint16_t g_spi_rx_data    = 0x0000 ;
uint16_t g_spi_tx_data    = 0x0000 ;

// FLASH
uint32_t g_flash_address_configuration 			= 0x08018000;
uint32_t g_flash_address_calibration_table 	= 0x08018100;
uint16_t g_calibration_config[20] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint32_t g_data[2] = {0, 0};

int32_t g_diff_electric_position;
float g_diff_zero_electric_position;
float g_diff_current_electric_position;
float g_electric_offset_to_zero_in_rad;
float g_encoder_offset_to_current_in_rad;

float g_temp_1;
int16_t g_temp_2;
float g_temp_3;
float g_temp_4;
int16_t g_temp_5;
int16_t g_temp_6;
uint16_t g_temp_7;

uint16_t g_mgh_errors = 0;
uint16_t g_mgl_errors = 0;

volatile int16_t g_temp_current_position_before = 0;
volatile int16_t g_temp_current_position_after = 0;
volatile int32_t g_temp_count_errors = 0;

// Calibration variables
volatile Calibration_State_t g_calibration_state = CALIBRATION_NOT_FINISHED;

volatile uint16_t g_calibration_speed = CALIBRATION_SPEED_LIMIT; // speed of configuration rotations
volatile uint16_t g_calibration_torque_limit = CALIBRATION_TORQUE_LIMIT;

volatile int32_t g_min_encoder_position = 0;
volatile int32_t g_max_encoder_position = 0;
volatile int32_t g_center_encoder_position = 0;
volatile int16_t g_max_electric_rotation_cw  = 0;
volatile int16_t g_max_electric_rotation_ccw = 0;
volatile uint16_t g_calibration_data_1_errors = 0;
volatile uint16_t g_calibration_data_2_errors = 0;

int16_t g_current_sector_number = -1;
int16_t g_previous_sector_number = -1;
int16_t g_current_estimated_electric_rotation = -1;

Counters_Handle_t volatile g_counters = {
	.main_loop = 0,
	.timer6 = 0,
	.timer7 = 0,
	.spi_rx_counter = 0,
	.spi_tx_counter = 0,
	.spi_txrx_counter = 0,
};


App_Command_Handle_t g_joint_command = {
	.working_mode = TORQUE_MODE,
	.motor_torque = 0.0,
};




Joint_Configuration_Handle_t g_joint_configuration = {
	.ma730_enabled = true,
	.safety_enabled = false,
	.working_area_constrain_enabled = false,
	.motor_type = MOTOR_TYPE,
	.can_node_id = 0x00,
	.gear_ratio = 121
};


Joint_Status_Handle_t g_joint_status = {
	.mc_current_motor_position = 0
};

volatile MA730_Handle_t	g_ma730 = {
	.started = false
};

volatile FSMStatus_t 	g_fsm_status = {
	.state = FSM_START,
	.state_is_running = false,
	.transition_is_running = false,
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
void MA730_ReadRegister(uint8_t reg_number);
void MA730_ReadAngle(void);
void MA730_WriteRegister(uint8_t reg_number, uint8_t reg_value);

// FUNCTIONS BODIES
void UJ_Init() {
	NTC_Init(&g_TempBearingSensorParamsM1);

	MA730_WriteRegister(0, 0b00000000);
	MA730_WriteRegister(1, 0b00000000);
//	MA730_WriteRegister(2, 0b00000000);
	MA730_WriteRegister(2, 0b10010000); // BCT=144
	MA730_WriteRegister(3, 0b00000000); // ETX=0, ETY=0
	MA730_WriteRegister(4, 0b11000000);
	MA730_WriteRegister(5, 0b11111111);
	MA730_WriteRegister(6, 0b00011100);
	MA730_WriteRegister(9, 0b00000000);

	MA730_ReadRegister(0);
	MA730_ReadRegister(1);
	MA730_ReadRegister(2);
	MA730_ReadRegister(3);
	MA730_ReadRegister(4);
	MA730_ReadRegister(5);
	MA730_ReadRegister(6);
	MA730_ReadRegister(9);

	// TIMERS
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
		{
			g_joint_command._motor_torque = goal;
			MC_ProgramTorqueRampMotor1(g_joint_command._motor_torque, 0);
			MC_StartMotor1();
			break;
		}

		case SPEED_MODE:
		{
			g_joint_command._motor_speed = goal;
			MC_ProgramSpeedRampMotor1(g_joint_command._motor_speed, 0);
			MC_StartMotor1();
			break;
		}

//			case POSITION_MODE:
//			{
//				MC_ProgramPositionCommandMotor1(g_joint_command.motor_position, 0.1);
////				MC_ProgramSpeedRampMotor1(g_joint_command._motor_position, 0);
////				MC_StartMotor1();
////				MC_ProgramPositionCommandMotor1(g_joint_command.joint_position, 0.1);
////				MC_StartMotor1();
////				HAL_Delay(2000);
//				break;
//			}

		default:
		{
			MC_StopMotor1();
		}
	}
}

void motor_stop() 
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

bool motor_reach_torque_limit() 
{
	if (g_joint_status.mc_current_motor_torque > g_calibration_torque_limit && g_joint_status.stm_state_motor == RUN) return true;

	return false;
}


bool motor_in_position(volatile int32_t position) 
{
	if (abs((int64_t) g_joint_status.mc_current_motor_position_multiturn - position) < CALIBRATION_ZERO_POSITION_OFFSET) return true;

	return false;
}


// CALIBRATION
bool check_calibration_data_cw(int16_t size) 
{
	g_calibration_data_1_errors = 0;

	for (int i = 0; i < size; i++) {
//		if (calibration_data_1[i] == 0) g_calibration_data_1_errors++;
		if (g_joint_configuration.calibration_table_1[i] == 0) g_calibration_data_1_errors++;


	}

	if (g_calibration_data_1_errors > 0) return false;

	return true;
}


bool check_calibration_data_ccw(int16_t size) 
{
	g_calibration_data_2_errors = 0;

	for (int i = 0; i < size; i++) {
//		if (calibration_data_2[i] == 0) g_calibration_data_2_errors++;
		if (g_joint_configuration.calibration_table_2[i] == 0) g_calibration_data_2_errors++;
	}

	if (g_calibration_data_2_errors > 0) return false;

	return true;
}

//int16_t get_sector_number_from_calibration(uint16_t left_index, uint16_t right_index, uint16_t ma730_value, uint16_t offset) 
//{
//    if (right_index >= left_index) {
//    	int16_t mid = left_index + (right_index - left_index) / 2; // srodek

//    	uint32_t mid_left_value  = g_joint_configuration.calibration_table_1[mid];    // wartosc lewego brzegu sektora
//    	uint32_t mid_right_value = g_joint_configuration.calibration_table_2[mid];    // wartosc prawego brzegu sektora
//    	uint32_t searched_value  = ma730_value; // wartosc szukana

//    	// Przesuniecie wartosci o offset, by funkcja byla w calej dlugosci ciagla
//    	if (mid_left_value <= offset)
//    	{
//    		mid_left_value += 16384;
//    	}

//    	if (mid_right_value <= offset)
//    	{
//    		mid_right_value += 16384;
//    	}

//    	if (searched_value <= offset)
//    	{
//    		searched_value += 16384;
//    	}

//        // If the element is present at the middle
//        // itself
//		if (searched_value >= mid_left_value && searched_value <= mid_right_value ) // czy jest w sektorze srodkowym - jezeli tak, to koniec
//		{
//			return mid; // dobry sektor
//		}

//    	if (left_index == right_index) {
//    		return -1;
//    	}

//        // If element is smaller than mid, then
//        // it can only be present in left subarray
//        if (mid_left_value > searched_value) // element jest mniejszsy niz srodkowy
//        {
//			uint16_t index = get_sector_number_from_calibration(left_index, mid - 1, ma730_value, offset); // szukaj z lewej strony
//			return index;
//        }

//        // Else the element can only be present
//        // in right subarray
//        return get_sector_number_from_calibration(mid + 1, right_index, ma730_value, offset);  // szukaj z prawej strony
//    }

//    // We reach here when element is not
//    // present in array
//    return -1; // element poza sektorami
//}
// l - poczatek, r - koniec, x - szukane, arr - lista
// l - lewy sektor, r - prawy sektor, x - ma730, o - offset
int16_t get_sector_number_from_calibration(uint16_t left_index, uint16_t right_index, uint16_t ma730_value, uint16_t offset)
{
    if (right_index >= left_index) {
    	int16_t mid = left_index + (right_index - left_index) / 2; // srodek

    	uint32_t mid_left_value  = g_joint_configuration.calibration_table_1[mid];    // wartosc lewego brzegu sektora
    	uint32_t mid_right_value = g_joint_configuration.calibration_table_2[mid];    // wartosc prawego brzegu sektora
    	uint32_t searched_value  = ma730_value; // wartosc szukana

    	// Przesuniecie wartosci o offset, by funkcja byla w calej dlugosci ciagla
    	if (mid_left_value <= offset)
    	{
    		mid_left_value += 16384;
    	}

    	if (mid_right_value <= offset)
    	{
    		mid_right_value += 16384;
    	}

    	if (searched_value <= offset)
    	{
    		searched_value += 16384;
    	}

        // If the element is present at the middle
        // itself
		if (searched_value >= mid_left_value && searched_value <= mid_right_value ) // czy jest w sektorze srodkowym - jezeli tak, to koniec
		{
			return mid; // dobry sektor
		}

    	if (left_index == right_index) {
    		return -1;
    	}

        // If element is smaller than mid, then
        // it can only be present in left subarray
        if (mid_left_value > searched_value) // element jest mniejszsy niz srodkowy
        {
			uint16_t index = get_sector_number_from_calibration(left_index, mid - 1, ma730_value, offset); // szukaj z lewej strony
			return index;
        }

        // Else the element can only be present
        // in right subarray
        return get_sector_number_from_calibration(mid + 1, right_index, ma730_value, offset);  // szukaj z prawej strony
    }

    // We reach here when element is not
    // present in array
    return -1; // element poza sektorami
}
int16_t get_rotation_number_from_calibration_table(uint16_t left_index, uint16_t right_index, uint16_t ma730_value, uint16_t offset, bool ccw) 
{
    if (right_index >= left_index)
    {
    	int16_t mid = left_index + (right_index - left_index) / 2; // srodek zakresu tablicy

    	uint32_t right_index_value = g_joint_configuration.calibration_table_1[mid - 1];
    	uint32_t mid_index_value   = g_joint_configuration.calibration_table_1[mid];
    	uint32_t left_index_value  = g_joint_configuration.calibration_table_1[mid + 1];
//    	uint32_t mid_left_value  = g_joint_configuration.calibration_table_1[mid + 1]; // wartosc lewego brzegu sektora
////    	uint32_t mid_right_value = g_joint_configuration.calibration_table_2[mid];     // wartosc prawego brzegu sektora
//		uint32_t mid_right_value = g_joint_configuration.calibration_table_1[mid];     // wartosc prawego brzegu sektora
    	uint32_t searched_value    = ma730_value; // wartosc szukana

    	// Przesuniecie wartosci o offset, by funkcja byla w calej dlugosci ciagla
//    	if (mid_left_value <= offset)
//    	{
//    		mid_left_value += 16384;
//    	}
//
//    	if (mid_right_value <= offset)
//    	{
//    		mid_right_value += 16384;
//    	}

    	if (right_index_value <= offset)
    	{
    		right_index_value += 16384;
    	}
    	if (left_index_value <= offset)
    	{
    		left_index_value += 16384;
    	}
    	if (mid_index_value <= offset)
    	{
    		mid_index_value += 16384;
    	}
    	if (searched_value <= offset)
    	{
    		searched_value += 16384;
    	}

    	// ZNALAZLEM
		if (searched_value >= right_index_value && searched_value <= left_index_value) // czy jest w sektorze srodkowym - jezeli tak, to koniec
		{
			if (searched_value >= mid_index_value && searched_value <= left_index_value)
			{
				if (abs(searched_value - mid_index_value) <= abs(searched_value - left_index_value) )
				{
					return mid;
				}
				else
				{
					return mid + 1;
				}
			}
			else
			{
				if (abs(searched_value - mid_index_value) <= abs(searched_value - right_index_value) )
				{
					return mid;
				}
				else
				{
					return mid - 1;
				}

			}
			// Ustalenie blizej ktorej z 3 wartosci znajduje sie
			return mid; // dobry sektor
		}

    	if (left_index == right_index)
    	{
    		return -1;
    	}

        // If element is smaller than mid, then
        // it can only be present in left subarray
    	// SZUKAM Z LEWEJ
        if (mid_index_value > searched_value) // element jest mniejszsy niz srodkowy
        {
			uint16_t index = get_rotation_number_from_calibration_table(left_index, mid - 1, ma730_value, offset, ccw); // szukaj z lewej strony
			return index;
        }

        // Else the element can only be present
        // in right subarray
        // SZUKAM Z PRAWEJ
        return get_rotation_number_from_calibration_table(mid + 1, right_index, ma730_value, offset, ccw);  // szukaj z prawej strony
    }

    // We reach here when element is not
    // present in array
    return -1; // element poza sektorami
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
//	g_joint_status.mc_encoder_align_status  	= MC_GetAlignmentStatusMotor1();
//	g_joint_status.mc_position_control_status 	= MC_GetControlPositionStatusMotor1();
	g_joint_status.current_motor_temperature 	= NTC_GetAvTemp_C(&TempSensor_M1);
	g_joint_status.current_bearing_temperature 	= NTC_GetAvTemp_C(&g_TempBearingSensorParamsM1);

	g_joint_status.gd_nfault = (HAL_GPIO_ReadPin(GPIOE, GD_NFAULT_Pin) == GPIO_PIN_RESET) ? (0) : (1);
	g_joint_status.gd_ready = (HAL_GPIO_ReadPin(GPIOE, GD_READY_Pin) == GPIO_PIN_RESET) ? (0) : (1);

}

void Update_Data_From_MC() 
{
	// POSITION
	g_joint_status.f_current_motor_position = ((double) g_joint_status.mc_current_motor_position_multiturn / ENCODER_M1.PulseNumber) * M_TWOPI;
	g_joint_status.f_current_joint_position_multiturn = (double) g_joint_status.f_current_motor_position / g_joint_configuration.gear_ratio + g_joint_status.f_current_encoder_position_offset;
	g_joint_status.f_current_joint_position = fmod((double) g_joint_status.f_current_joint_position_multiturn + M_PI, (double) M_TWOPI) - M_PI;

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
	if ((fabs(g_joint_status.f_current_joint_speed) > JOINT_SPEED_LIMIT) && FSM_Get_State() != FSM_TRANSITION_FAULT_TO_INIT) {
		g_joint_status.errors =  g_joint_status.errors | JOINT_SPEED_TO_HIGH;
		error = true;
	}

	// HARDWARE ERROR REACTION
	if (g_joint_status.gd_nfault == 0 && FSM_Get_State() != FSM_TRANSITION_FAULT_TO_INIT) {
		g_joint_status.errors =  g_joint_status.errors | JOINT_HW_ERROR;
		error = true;
	}

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

	// ENCODER_NOT_ACCURATE
	if (g_joint_status.encoder_position_state != POSITION_ACCURATE)
	{
		g_joint_status.warnings = g_joint_status.warnings | JOINT_POSITION_NOT_ACCURATE;

	}
	else
	{
		g_joint_status.warnings = g_joint_status.warnings & (0xFF ^ JOINT_POSITION_NOT_ACCURATE);
	}

	if (error == true && FSM_Get_State() != FSM_FAULT && FSM_Get_State() != FSM_FAULT_REACTION_ACTIVE && FSM_Get_State() != FSM_TRANSITION_FAULT_REACTION_ACTIVE_TO_FAULT)
	{
		FSM_Activate_State(FSM_FAULT_REACTION_ACTIVE);
	}

	if (g_ma730.mgl || g_ma730.mgh) // not proper magnetic field strentgh
	{
		g_joint_status.warnings |= JOINT_MA730_NOT_PROPER_MAGNETOC_FIELD;
	}
	else
	{
		g_joint_status.warnings = g_joint_status.warnings & (0xFF ^ JOINT_MA730_NOT_PROPER_MAGNETOC_FIELD);

	}

}

// CAN FD
void FDCAN_Set_Filters() {
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
	can_filter_config_1.FilterID1 = 0x100 + g_joint_configuration.can_node_id;
	can_filter_config_1.FilterID2 = 0x70F;
	HAL_FDCAN_ConfigFilter(&hfdcan1, &can_filter_config_1); //Initialize CAN Filter

}

// FLASH
void Flash_Read_Data(uint32_t StartPageAddress, uint32_t *RxBuf, uint16_t numberofwords) 
{
	while (1)
	{
		*RxBuf = *(__IO uint32_t *)StartPageAddress;
		StartPageAddress += 4;
		RxBuf++;
		if (!(numberofwords--)) break;
	}
}

void FLASH_Configuration_Load() 
{
	Flash_Read_Data(g_flash_address_configuration, (uint32_t *) g_calibration_config, 10);

	g_joint_configuration.calibration_table_size 			= g_calibration_config[8]; // FIXME !!!!

	if (g_joint_configuration.calibration_table_size > 0 && g_joint_configuration.calibration_table_size < 65535) {
		// Read joint configuration from FLASH
		g_joint_configuration.pole_pairs 						= g_calibration_config[0];
		g_joint_configuration.gear_ratio 						= g_calibration_config[1];
		g_joint_configuration.calibration_sector_size 			= g_calibration_config[4];
		g_joint_configuration.reachable_electrical_rotations	= g_calibration_config[5];
		g_joint_configuration.number_of_sectors 				= g_calibration_config[8];
		// 9
		g_joint_configuration.zero_electric_position 			= (int16_t) g_calibration_config[12];
		g_joint_configuration.zero_electric_rotation 			= g_calibration_config[13];

		g_joint_configuration.can_node_id 						= g_calibration_config[16];
		if (g_joint_configuration.can_node_id == 255)
		{
			g_joint_configuration.can_node_id = 0;
		}
		g_joint_configuration.motor_type 						= g_calibration_config[17];

		// Read calibration table from FLASH
		//	for (int i = 0; i <= g_joint_configuration.calibration_table_size / 2 + 1; i++) {
		for (int i = 0; i <= g_joint_configuration.calibration_table_size; i++) {
			Flash_Read_Data (g_flash_address_calibration_table + i * 8, (uint32_t *) g_data, 1);
			g_joint_configuration.calibration_table_1[i]	= g_data[0] >> 16;
			g_joint_configuration.calibration_table_2[i] 	= g_data[0];
		}

		g_joint_configuration.maximum_electrical_rotations = g_joint_configuration.gear_ratio * g_joint_configuration.pole_pairs;

		g_joint_configuration.electric_rotation_width = M_TWOPI / (g_joint_configuration.pole_pairs * g_joint_configuration.gear_ratio); // szerokosc jednego obrotu elektrycznego silnika w stosunku do szerokosci obrotu jointa (2PI)
		g_joint_configuration.calibration_state = JOINT_CALIBRATED;

	}
	g_joint_configuration.joint_working_area = M_PI * 165.0 / 180.0;

}

uint32_t FLASH_Configuration_Save() 
{
	volatile uint32_t error = 0;
	uint64_t data;

	HAL_FLASH_Unlock();

	FLASH_EraseInitTypeDef EraseInitStruct;
	EraseInitStruct.TypeErase 	= FLASH_TYPEERASE_PAGES;
	EraseInitStruct.Banks 		= FLASH_BANK_1;
	EraseInitStruct.Page 		= (g_flash_address_configuration & 0x07FFFFFF) / FLASH_PAGE_SIZE;
	EraseInitStruct.NbPages 	= 4; // 1 - 2kB
	uint32_t PageError;

	if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
	{
		error = HAL_FLASH_GetError ();
	}

	// Configuration
	// 1 - 0 - 3
	data = (g_joint_configuration.gear_ratio << 16) | g_joint_configuration.pole_pairs;
	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, g_flash_address_configuration, data) != HAL_OK)
	{
		error = HAL_FLASH_GetError ();
	}

	// 2 - 4 - 7
	data = (g_joint_configuration.reachable_electrical_rotations << 16) | g_joint_configuration.calibration_sector_size;
	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, g_flash_address_configuration + 8, data) != HAL_OK)
	{
		error = HAL_FLASH_GetError ();
	}

	// 3 - 8 - 11
	data = (g_joint_configuration.calibration_sector_size << 16) | g_joint_configuration.number_of_sectors;
	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, g_flash_address_configuration + 16, data) != HAL_OK)
	{
		error = HAL_FLASH_GetError ();
	}

	// 4 - 12 - 15
	uint16_t * temp = (uint16_t *) &g_joint_configuration.zero_electric_position;
	data = (g_joint_configuration.zero_electric_rotation << 16) | (* temp) ;
	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, g_flash_address_configuration + 24, data) != HAL_OK)
	{
		error = HAL_FLASH_GetError ();
	}

	// 5 - 16 - 19
	data = (g_joint_configuration.motor_type << 16) | g_joint_configuration.can_node_id;
	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, g_flash_address_configuration + 32, data) != HAL_OK)
	{
		error = HAL_FLASH_GetError ();
	}

	for(int i = 0; i < g_joint_configuration.number_of_sectors; i++ )
	{
		data = g_joint_configuration.calibration_table_1[i] << 16 | g_joint_configuration.calibration_table_2[i];
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, g_flash_address_calibration_table + i * 8, data) != HAL_OK)
		{
			error = HAL_FLASH_GetError ();
		}
	}

	HAL_FLASH_Lock();

	return error;

}

// MA730
void MA730_ReadRegister(uint8_t reg_number) 
{
	uint16_t send_data      = 0b010 << 13 | (reg_number & (0b00011111)) << 8 ;

	uint16_t angle_value    = 0;
	uint16_t register_value = 0;

	for (uint16_t i = 0; i < 24; i++) __NOP();  // wait about 150ns

	HAL_GPIO_WritePin(MA730_CS_GPIO_Port, MA730_CS_Pin, GPIO_PIN_RESET);

	for (uint16_t i = 0; i < 130; i++) __NOP();  // wait about 80ns

	// SEND READ REGISTER COMMAND - RECEIVE READ ANGLE RESULT
	HAL_SPI_TransmitReceive(&hspi1, (uint8_t * ) &send_data, (uint8_t * ) &angle_value, 1, 1);

	HAL_GPIO_WritePin(MA730_CS_GPIO_Port, MA730_CS_Pin, GPIO_PIN_SET);

	for (uint16_t i = 0; i < 120; i++) __NOP();  // wait about 750ns

	HAL_GPIO_WritePin(MA730_CS_GPIO_Port, MA730_CS_Pin, GPIO_PIN_RESET);

	for (uint16_t i = 0; i < 13; i++) __NOP();  // wait about 80ns

	send_data      = 0x0000;

	// SEND READ ANGLE COMMAND - RECEIVE READ REGISTER RESULT
	HAL_SPI_TransmitReceive(&hspi1, (uint8_t * ) &send_data, (uint8_t * ) &register_value, 1, 1);

//	g_MA730_read_buffer = register_value >> 8;

	HAL_GPIO_WritePin(MA730_CS_GPIO_Port, MA730_CS_Pin, GPIO_PIN_SET);

	for (uint16_t i = 0; i < 120; i++) __NOP();  // wait about 750ns

	g_ma730.angle = (angle_value >> 2) & 0b0011111111111111;

	register_value = register_value >> 8;
	switch (reg_number)
	{
		case 0x00:
		{
			g_ma730.z0 = (uint8_t) register_value;
			break;
		}
		case 0x01:
		{
			g_ma730.z1 = (uint8_t) register_value;
			break;
		}
		case 0x02:
		{
			g_ma730.bct = (uint8_t) register_value;
			break;
		}
		case 0x03:
		{
			g_ma730.ety = ((uint8_t) register_value & 0b00000010) >> 1;
			g_ma730.etx = ((uint8_t) register_value & 0b00000001);
			break;
		}
		case 0x04:
		{
			g_ma730.ppt0 = ((uint8_t) register_value & 0b11000000) >> 6;
			g_ma730.ilip = ((uint8_t) register_value & 0b00111100) >> 2;
			break;
		}
		case 0x05:
		{
			g_ma730.ppt1 = (uint8_t) register_value;
			break;
		}
		case 0x06:
		{
			g_ma730.mglt = ((uint8_t) register_value & 0b11100000) >> 5;
			g_ma730.mght = ((uint8_t) register_value & 0b00011100) >> 2;
			break;
		}
		case 0x09:
		{
			g_ma730.rd = (uint8_t) register_value >> 7;
			break;
		}
		case 0x1b:
		{
			g_ma730.mgl = ((uint8_t) register_value & 0b01000000) >> 6;
			g_ma730.mgh = ((uint8_t) register_value & 0b10000000) >> 7;
			break;
		}
		default:
		{
			break;
		}
	}

//	if (g_counter_1hz > 1) // wait 1 sec to analyse data
//	{
//		g_motor_status.ma730_is_running = true;
//		g_motor_status.previous_ma730_value = g_motor_status.current_ma730_value;
//		g_motor_status.current_ma730_value = (g_MA730_read_buffer >> 2) & 0b0011111111111111;
//	}


}




void MA730_ReadAngle() 
{
	uint16_t send_data      = 0x0000 ;

	uint16_t angle_value    = 0;

//	for (uint16_t i = 0; i < 24; i++) NOP;  // wait about 150ns

	HAL_GPIO_WritePin(MA730_CS_GPIO_Port, MA730_CS_Pin, GPIO_PIN_RESET);

//	for (uint16_t i = 0; i < 13; i++) NOP;  // wait about 80ns

	// SEND READ REGISTER COMMAND - RECEIVE READ ANGLE RESULT
	HAL_SPI_TransmitReceive(&hspi1, (uint8_t * ) &send_data, (uint8_t * ) &angle_value, 1, 1);

	HAL_GPIO_WritePin(MA730_CS_GPIO_Port, MA730_CS_Pin, GPIO_PIN_SET);

//	for (uint16_t i = 0; i < 12; i++) NOP;  // wait about 80ns

	g_ma730.angle = (angle_value >> 2) & 0b0011111111111111;

}
void MA730_WriteRegister(uint8_t reg_number, uint8_t reg_value) 
{
	uint16_t send_data      = 0b100 << 13 | (reg_number & (0b00011111)) << 8 | reg_value;

	uint16_t angle_value    = 0;
	uint16_t register_value = 0;

	for (uint16_t i = 0; i < 24; i++) __NOP();  // wait about 150ns

	HAL_GPIO_WritePin(MA730_CS_GPIO_Port, MA730_CS_Pin, GPIO_PIN_RESET);

	for (uint16_t i = 0; i < 130; i++) __NOP();  // wait about 80ns

	// SEND READ REGISTER COMMAND - RECEIVE READ ANGLE RESULT
	HAL_SPI_TransmitReceive(&hspi1, (uint8_t * ) &send_data, (uint8_t * ) &angle_value, 1, 1);

	HAL_GPIO_WritePin(MA730_CS_GPIO_Port, MA730_CS_Pin, GPIO_PIN_SET);

	HAL_Delay(20); // Wait 20 ms after write command

	HAL_GPIO_WritePin(MA730_CS_GPIO_Port, MA730_CS_Pin, GPIO_PIN_RESET);

	for (uint16_t i = 0; i < 13; i++) __NOP();  // wait about 80ns

	send_data      = 0x0000;

	// SEND READ ANGLE COMMAND - RECEIVE READ REGISTER RESULT
	HAL_SPI_TransmitReceive(&hspi1, (uint8_t * ) &send_data, (uint8_t * ) &register_value, 1, 1);

	HAL_GPIO_WritePin(MA730_CS_GPIO_Port, MA730_CS_Pin, GPIO_PIN_SET);

	for (uint16_t i = 0; i < 120; i++) __NOP();  // wait about 750ns

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
void FSM_START_Callback() 
{
	HAL_TIM_Base_Stop_IT(&htim6); // Disable 10 kHz timer

	g_joint_configuration.pole_pairs = POLE_PAIRS;
	g_joint_configuration.gear_ratio = GEAR_RATIO;

	FLASH_Configuration_Load(); // Read configuration from FLASH

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
}

void FSM_READY_TO_OPERATE_Callback() 
{
	motor_stop();
}

void FSM_OPERATION_ENABLE_Callback() 
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
}

void FSM_TRANSITION_OPERATION_ENABLE_TO_READY_TO_OPERATE_Callback()
{
	motor_stop();
}

void FSM_FAULT_REACTION_ACTIVE_Callback()
{
	motor_stop();
}

void FSM_TRANSITION_FAULT_REACTION_ACTIVE_TO_FAULT_Callback() 
{
	motor_stop();
}

void FSM_FAULT_Callback() 
{
	motor_stop();
}

void FSM_TRANSITION_FAULT_TO_INIT_Callback() 
{
	MC_AcknowledgeFaultMotor1();
	g_joint_status.errors = 0;
}

void FSM_Tick_Callback() 
{
	
	// FSM
	switch (FSM_Get_State()) {
		
		case FSM_TRANSITION_INIT_TO_CALIBRATION_PHASE_0:
			FSM_Activate_State(FSM_CALIBRATION_PHASE_0);
			break;
		
		case FSM_CALIBRATION_PHASE_0:
			g_joint_configuration.ma730_enabled = true;
			g_joint_configuration.motor_type = RI70;
			g_joint_configuration.calibration_state = JOINT_NOT_CALIBRATED;

			g_current_sector_number = -1;
			g_previous_sector_number = -1;

//				if (g_joint_configuration.ma730_enabled == false)
//				{
//					FSM_Activate_State(FSM_FAULT_REACTION_ACTIVE);
//					break;
//				}

			g_joint_configuration.calibration_sector_size = SECTOR_SIZE;
			g_joint_configuration.pole_pairs = POLE_PAIRS;
			g_joint_configuration.gear_ratio = GEAR_RATIO;
			g_joint_configuration.maximum_electrical_rotations = g_joint_configuration.gear_ratio * g_joint_configuration.pole_pairs;
			g_joint_configuration.calibration_table_size = (uint16_t) (g_joint_configuration.maximum_electrical_rotations / g_joint_configuration.calibration_sector_size);

			g_joint_configuration.reachable_electrical_rotations = 0;
			g_joint_configuration.number_of_sectors = 0;
			g_joint_configuration.zero_electric_position = 0;
			g_joint_configuration.zero_electric_rotation = 0;
			for (int i = 0; i < g_joint_configuration.calibration_table_size; i++)
			{
				g_joint_configuration.calibration_table_1[i] = 0;
				g_joint_configuration.calibration_table_2[i] = 0;
			}

			g_fsm_status.state = FSM_CALIBRATION_PHASE_1;
			break;

		case FSM_CALIBRATION_PHASE_1:
			if (motor_reach_torque_limit()) // REACH MINIMUM EDGE
			{
				motor_stop();
//					g_joint_status.mc_current_electric_rotation = 0; // zeroing electric rotation counter
				g_current_electrical_rotation = 0;
				g_min_encoder_position = g_joint_status.mc_current_motor_position_multiturn; // encoder value

				g_fsm_status.state = FSM_CALIBRATION_PHASE_2;

			}
			else
			{
				motor_start(SPEED_MODE, -1 * g_calibration_speed);
			}
			break;

		case FSM_CALIBRATION_PHASE_2:
		{
			uint16_t sector_number = g_joint_status.mc_current_electric_rotation / g_joint_configuration.calibration_sector_size;
//				uint16_t sector_number = g_joint_status.mc_current_electric_rotation;

			if ((g_joint_status.mc_current_electric_rotation % g_joint_configuration.calibration_sector_size == 0) &&
				(g_joint_configuration.calibration_table_1[sector_number] == 0) &&
				(abs(g_joint_status.mc_current_electric_position) < 2048))
			{
				g_joint_configuration.calibration_table_1[sector_number] = g_ma730.angle;
//					g_joint_configuration.calibration_table_1[g_joint_status.mc_current_electric_rotation / g_joint_configuration.calibration_sector_size] = g_joint_status.mc_current_electric_rotation;

			}

			if (motor_reach_torque_limit()) // REACH MAXIMUM EDGE
			{
				motor_stop();
				g_joint_configuration.calibration_table_2[sector_number] = g_ma730.angle;
//					g_joint_configuration.calibration_table_2[g_joint_status.mc_current_electric_rotation / g_joint_configuration.calibration_sector_size] = g_joint_status.mc_current_electric_rotation;
				g_max_electric_rotation_cw 	= g_joint_status.mc_current_electric_rotation; // max electric rotation counter

				g_max_encoder_position = g_joint_status.mc_current_motor_position_multiturn; // max encoder value
				g_fsm_status.state = FSM_CALIBRATION_PHASE_3;

			}
			else
			{
				motor_start(SPEED_MODE, g_calibration_speed);
			}
			break;
		}
		case FSM_CALIBRATION_PHASE_3:
		{
			uint16_t sector_number = g_joint_status.mc_current_electric_rotation / g_joint_configuration.calibration_sector_size;
//				int16_t sector_number = g_joint_status.mc_current_electric_rotation;

			if (g_joint_status.mc_current_electric_rotation < 0)
			{
				break;
			}

			if ((g_joint_status.mc_current_electric_rotation % g_joint_configuration.calibration_sector_size == g_joint_configuration.calibration_sector_size - 1) &&
				(g_joint_configuration.calibration_table_2[sector_number] == 0) &&
				(abs(g_joint_status.mc_current_electric_position) < 2048))
			{
				g_joint_configuration.calibration_table_2[sector_number] = g_ma730.angle;
//					g_joint_configuration.calibration_table_2[g_joint_status.mc_current_electric_rotation / g_joint_configuration.calibration_sector_size] = g_joint_status.mc_current_electric_rotation;
			}

			if (motor_reach_torque_limit()) // REACH MINIMUM EDGE
			{
				motor_stop();
				g_joint_configuration.calibration_table_1[sector_number] = g_ma730.angle;

				if (g_joint_configuration.calibration_table_2[0] == 0 && g_joint_configuration.calibration_table_2[1] > 0) // center of 0 sector is not reached
				{
					g_joint_configuration.calibration_table_2[0] = g_ma730.angle;
				}

//					g_joint_configuration.calibration_table_1[g_joint_status.mc_current_electric_rotation / g_joint_configuration.calibration_sector_size] = g_joint_status.mc_current_electric_rotation;
				g_fsm_status.state = FSM_CALIBRATION_PHASE_4;

			}
			else
			{
				motor_start(SPEED_MODE, -1 * g_calibration_speed);
			}
			break;
		}
		
		case FSM_CALIBRATION_PHASE_4:
			// Sprawdzenie tablicy kalibracyjnej
			if (!check_calibration_data_cw(g_max_electric_rotation_cw / g_joint_configuration.calibration_sector_size) ||
				!check_calibration_data_ccw(g_max_electric_rotation_cw / g_joint_configuration.calibration_sector_size))
			{
				g_fsm_status.state = FSM_CALIBRATION_PHASE_2;
				g_calibration_state = CALIBRATION_TABLE_CONTAINS_ZEROES;
			}
			else
			{
				g_max_electric_rotation_ccw = g_joint_status.mc_current_electric_rotation; // Should be 0 right now
				g_center_encoder_position = (g_max_encoder_position - g_min_encoder_position + 1) / 2 + g_min_encoder_position;

				g_joint_configuration.number_of_sectors = (uint16_t) (g_max_electric_rotation_cw / g_joint_configuration.calibration_sector_size);
				g_joint_configuration.reachable_electrical_rotations = g_max_electric_rotation_cw;
				g_calibration_state = CALIBRATION_OK;
				g_fsm_status.state = FSM_CALIBRATION_PHASE_5;
			}
			break;

		case FSM_CALIBRATION_PHASE_5:
			if (motor_in_position(g_center_encoder_position))
			{
				g_fsm_status.state = FSM_CALIBRATION_PHASE_6;
				motor_start(SPEED_MODE, 0);
				motor_stop();

				g_joint_configuration.zero_electric_rotation = g_joint_status.mc_current_electric_rotation;
//					g_joint_configuration.zero_electric_position = (uint16_t) (g_motor_status.current_electric_position);
				g_joint_configuration.zero_electric_position = (int16_t) (g_joint_status.mc_current_electric_position);

				// Correct calibration table
//					for (int i = 1; i <= g_joint_configuration.number_of_sectors; i++)
//					{
//						if (g_joint_configuration.calibration_table_1[i] < g_joint_configuration.calibration_table_2[i - 1])
//						{
//							uint16_t temp = g_joint_configuration.calibration_table_1[i];
//							g_joint_configuration.calibration_table_1[i] = g_joint_configuration.calibration_table_2[i - 1];
//							g_joint_configuration.calibration_table_2[i - 1] = temp;
//
//						}
//
//						if (g_joint_configuration.calibration_table_1[i] == g_joint_configuration.calibration_table_2[i - 1])
//						{
//							g_joint_configuration.calibration_table_2[i - 1] -= 1;
//							g_joint_configuration.calibration_table_1[i] 	 += 1;
//						}
//					}
			}
			else
			{
				motor_start(SPEED_MODE, g_calibration_speed);
			}

			if (motor_reach_torque_limit()) // REACH EDGE - FAILURE
			{
				motor_start(SPEED_MODE, 0);
				motor_stop();
				g_calibration_state = MISSED_CENTER_POSITION;
//						g_fsm_status.state = FSM_STOPPED_WITH_ERRORS;
				FSM_Activate_State(FSM_FAULT_REACTION_ACTIVE);

			}
			break;
		
		case FSM_CALIBRATION_PHASE_6:
			HAL_TIM_Base_Stop_IT(&htim6); // Disable 10 kHz timer

			if (FLASH_Configuration_Save() == 0)
			{
				g_joint_configuration.calibration_state = JOINT_CALIBRATED;
				FDCAN_Set_Filters();
				FSM_Activate_State(FSM_INIT); // CALIBRATION FINISHED - GO TO INIT STATE
			}
			else
			{
				FSM_Activate_State(FSM_FAULT_REACTION_ACTIVE); // CALIBRATION FINISHED - GO TO INIT STATE
			}

			HAL_TIM_Base_Start_IT(&htim6); // Enable 10 kHz timer
			break;

//		case FSM_START:
//		{
//			HAL_TIM_Base_Stop_IT(&htim6); // Disable 10 kHz timer

//			g_joint_configuration.pole_pairs = POLE_PAIRS;
//			g_joint_configuration.gear_ratio = GEAR_RATIO;

//			FLASH_Configuration_Load(); // Read configuration from FLASH

//			// FDCAN
//			HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1, 10, 0);
//			HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1);

//			FDCAN_Set_Filters();

//			HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, 3, 3, FDCAN_FILTER_REMOTE, FDCAN_REJECT_REMOTE);
//			HAL_FDCAN_Start(&hfdcan1); //Initialize CAN Bus
//			HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);// Initialize CAN Bus Rx Interrupt
//			HAL_FDCAN_EnableISOMode(&hfdcan1);

//			FSM_Activate_Transition(FSM_TRANSITION_START_TO_INIT);

//			HAL_TIM_Base_Start_IT(&htim6); // Enable 10 kHz timer

//			break;
//		}

//		case FSM_TRANSITION_START_TO_INIT:
//		{
//			FSM_Activate_State(FSM_INIT);
//			break;
//		}

//		case FSM_INIT:
//		{
//			// Wlacz odczyty MA730
////			if (g_joint_configuration.ma730_enabled == true && g_ma730.started == false)
////			{
////				g_ma730.started = true;

////				HAL_GPIO_WritePin(MA730_CS_GPIO_Port, MA730_CS_Pin, GPIO_PIN_RESET);

////				HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t * ) &g_spi_tx_data, (uint8_t * ) &g_spi_rx_data, 1);
////			}
//			// Wylacz odczyty MA730
////				if (g_joint_configuration.ma730_enabled == false && g_ma730.started == true)
////				{
////					g_ma730.started = false;
////
////				}

//			break;
//		}

//		case FSM_TRANSITION_INIT_TO_READY_TO_OPERATE:
//		{
//			// Check the calibration
//			// If joint is not calibrated go to failure
//			FSM_Activate_State(FSM_READY_TO_OPERATE);
//			break;
//		}

//		case FSM_READY_TO_OPERATE:
//		{
//			motor_stop();
//			break;
//		}

//		case FSM_TRANSITION_READY_TO_OPERATE_TO_OPERATION_ENABLE:
//		{
//			FSM_Activate_State(FSM_OPERATION_ENABLE);
//			break;
//		}

//		case FSM_TRANSITION_OPERATION_ENABLE_TO_READY_TO_OPERATE:
//		{
//			FSM_Activate_State(FSM_READY_TO_OPERATE);
//			motor_stop();
//			break;
//		}

//		case FSM_TRANSITION_OPERATION_ENABLE_TO_INIT:
//		{
//			FSM_Activate_State(FSM_INIT);
//			break;
//		}



//		case FSM_OPERATION_ENABLE:
//		{

//			int16_t goal = 0;

//			switch (g_joint_command.working_mode)
//			{
//				case TORQUE_MODE:
//				{
//					goal = g_joint_command._motor_torque;
//					break;
//				}

//				case SPEED_MODE:
//				{
//					goal = g_joint_command._motor_speed;
//					break;
//				}
//			}

//			if (g_joint_configuration.working_area_constrain_enabled)
//			{
//				switch (g_joint_status.current_joint_position)
//				{
//					case POSITION_UNDER_WORKING_AREA: // Accept only positive torque
//						if (goal < 0)
//						{
//							motor_stop();
//						}
//						else
//						{
//							motor_start(g_joint_command.working_mode, goal);
//						}
//						break;

//					case POSITION_IN_WORKING_AREA:
//						motor_start(g_joint_command.working_mode, goal);
//						break;

//					case POSITION_OVER_WORKING_AREA: // Accept only negative torque
//						if (goal > 0)
//						{
//							motor_stop();
//						}
//						else
//						{
//							motor_start(g_joint_command.working_mode, goal);
//						}
//						break;
//				}

//			}
//			else
//			{
//				motor_start(g_joint_command.working_mode, goal);
//			}

//			break;

//		}

//		case FSM_TRANSITION_FAULT_TO_INIT:
//		{
//			MC_AcknowledgeFaultMotor1();
//			g_joint_status.errors = 0;
//			FSM_Activate_State(FSM_INIT);
//			break;
//		}

//		case FSM_TRANSITION_FAULT_REACTION_ACTIVE_TO_FAULT:
//		{
//			motor_stop();
//			FSM_Activate_State(FSM_FAULT);
//			break;
//		}

//		case FSM_FAULT_REACTION_ACTIVE:
//		{
//			motor_stop();
//			FSM_Activate_Transition(FSM_TRANSITION_FAULT_REACTION_ACTIVE_TO_FAULT);
//			break;
//		}

//		case FSM_FAULT:
//		{
//			motor_stop();
//			break;
//		}

		default:
		{
			FSM_Activate_State(FSM_FAULT_REACTION_ACTIVE);
			break;
		}
	}
}

bool FSM_Set_State_Callback(uint8_t new_state) // FIXME running transition should block changing state to new one - add flag transition is running
{

	switch (new_state)
	{
//#if defined CALIBRATION
//#else
		case FSM_START:
		{
			break;
		}

		case FSM_INIT:
		{
			if (FSM_Get_State() == FSM_START)
			{
				return FSM_Activate_Transition(FSM_TRANSITION_START_TO_INIT);
			}

			if (FSM_Get_State() == FSM_FAULT)
			{
				return FSM_Activate_Transition(FSM_TRANSITION_FAULT_TO_INIT);
			}

			if (FSM_Get_State() == FSM_OPERATION_ENABLE)
			{
				return FSM_Activate_Transition(FSM_TRANSITION_OPERATION_ENABLE_TO_INIT);
			}
			break;
		}

		case FSM_READY_TO_OPERATE:
		{

			if (FSM_Get_State() == FSM_INIT)
			{
				return FSM_Activate_Transition(FSM_TRANSITION_INIT_TO_READY_TO_OPERATE);
			}

			if (FSM_Get_State() == FSM_OPERATION_ENABLE)
			{
				return FSM_Activate_Transition(FSM_TRANSITION_OPERATION_ENABLE_TO_READY_TO_OPERATE);
			}

			break;
		}

		case FSM_OPERATION_ENABLE:
		{
			if (FSM_Get_State() == FSM_READY_TO_OPERATE)
			{
				return FSM_Activate_Transition(FSM_TRANSITION_READY_TO_OPERATE_TO_OPERATION_ENABLE);
			}
			break;
		}

		case FSM_CALIBRATION_PHASE_0:
		{

			if (FSM_Get_State() == FSM_INIT)
			{
				return FSM_Activate_Transition(FSM_TRANSITION_INIT_TO_CALIBRATION_PHASE_0);
			}

			break;
		}

		case FSM_FAULT_REACTION_ACTIVE:
		{
			if (FSM_Get_State() != FSM_START)
			{
				return FSM_Activate_Transition(FSM_TRANSITION_FAULT_REACTION_ACTIVE_TO_FAULT);
			}
			break;
		}

		case FSM_FAULT:
		{
			if (FSM_Get_State() == FSM_READY_TO_OPERATE)
			{
				return FSM_Activate_Transition(FSM_TRANSITION_READY_TO_OPERATE_TO_OPERATION_ENABLE);
			}
			break;
		}

		default:
		{
			break;
		}
//#endif
	}

	return false;
}

// HAL Callbacks
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) 
{
	// SECURITY ALERT
    if(GPIO_Pin == SEC_IN_Pin && g_joint_configuration.safety_enabled == true && g_fsm_status.state == FSM_OPERATION_ENABLE) // If The INT Source Is EXTI Line9 (A9 Pin)
    {
    	motor_stop();
			FSM_Activate_State(FSM_READY_TO_OPERATE);
    }
}


//void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef * hspi) 
//{
//	HAL_GPIO_WritePin(MA730_CS_GPIO_Port, MA730_CS_Pin, GPIO_PIN_SET);

//    // TX-RX Done .. Do Something ...
//	g_counters.spi_txrx_counter++;

//	g_ma730.angle = (g_spi_rx_data >> 2) & 0b0011111111111111;

//	if (g_ma730.started)
//	{
//		HAL_GPIO_WritePin(MA730_CS_GPIO_Port, MA730_CS_Pin, GPIO_PIN_RESET);

//		if (HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t * ) &g_spi_tx_data, (uint8_t * ) &g_spi_rx_data, 1) != HAL_OK)
//		{
//			Error_Handler();
//		}
//	}

//}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) 
{
	if(htim->Instance == TIM6) 	// 10kHz (5,0) - fast recalculation
	{
		g_counters.timer6++;
//		Read_MC_Encoder_10kHz();
		Read_MC_Torque();
	}

	if(htim->Instance == TIM7) 	// 1kHz - FSM_Tasks
	{
		Read_MC_Encoder_1kHz();
		Update_Data_From_MC(); // Przelicz na jednostki we floatach
		Read_MC_State(); // Sprawdzenie stanu MC SDK
		if ( FSM_Get_State() > 0)
		{
			CheckErrorsAndWarnings(); // Sprawdzenie czy nie wygenerowaly sie jakies bledy czy ostrzezenia
		}
		NTC_CalcAvTemp(&g_TempBearingSensorParamsM1); // Odczyt temperatury lozyska

		// Odczyt enkoderow MA730 oraz MC komutacji
		if (g_joint_configuration.ma730_enabled == true)
		{
//			MA730_ReadAngle();
			MA730_ReadRegister(0x1B);
			g_mgh_errors += g_ma730.mgh;
			g_mgl_errors += g_ma730.mgl;
		}

		g_joint_status.mc_current_electric_rotation = g_current_electrical_rotation; // pobranie informacji aktualnej z MC
		g_joint_status.mc_current_electric_position = g_current_electrical_position; // pobranie informacji aktualnej z MC
//		g_temp_current_position_before = g_current_electrical_position;

		// Estimate joint position from absolute encoder
		if (g_joint_configuration.ma730_enabled == true && g_joint_configuration.calibration_state == JOINT_CALIBRATED)
		{
			// JOINT POSITION ESTIMATION
			switch (g_joint_status.encoder_position_state)
			{
				case POSITION_ESTIMATION_FAILED:
					// RAISE ERROR
					break;
//
				case POSITION_ACCURATE:
//					g_current_sector_number = get_rotation_number_from_calibration_table(0, g_joint_configuration.number_of_sectors - 1, g_ma730.angle, g_joint_configuration.calibration_table_1[0], 1);
					break;

				case POSITION_APROXIMATED:
					g_current_sector_number = get_sector_number_from_calibration(0, g_joint_configuration.number_of_sectors - 1, g_ma730.angle, g_joint_configuration.calibration_table_1[0]);

					// If motor running and sector is change update
					if (g_current_sector_number != g_previous_sector_number && g_previous_sector_number != -1 && g_current_sector_number != -1) // pass 1 time, to load previous and current
					{
						float electric_rotation_width = M_TWOPI / (g_joint_configuration.pole_pairs * g_joint_configuration.gear_ratio);
						uint16_t l_current_electric_rotation;

						if (g_current_sector_number - g_previous_sector_number > 0)
						{ // rotation in CCW direction => "+"
							l_current_electric_rotation = (g_current_sector_number) * g_joint_configuration.calibration_sector_size;
						}
						else
						{ // rotation in CW direction ==> "-"
							l_current_electric_rotation = (g_current_sector_number + 1) * g_joint_configuration.calibration_sector_size - 1;
						}
						int32_t l_diff_electric_position = g_joint_configuration.zero_electric_position - g_joint_status.mc_current_motor_position_multiturn;
						g_joint_status.mc_current_electric_rotation = l_current_electric_rotation;

						float l_electric_offset_to_zero_in_rad   = -1 * ((g_joint_configuration.zero_electric_rotation - l_current_electric_rotation) + ((float) l_diff_electric_position / 65536) - 1.0) * electric_rotation_width;
						float l_encoder_offset_to_current_in_rad = -1 * ((float) g_joint_status.mc_current_motor_position_multiturn / (ENCODER_M1.PulseNumber / g_joint_configuration.pole_pairs)) * electric_rotation_width;

//						g_motor_status.current_encoder_position_offset_in_rad = -1 * ((g_joint_configuration.zero_electric_rotation - l_current_electric_rotation) + ((float) l_diff_electric_position / 65536) - 1.0) * electric_rotation_width;
						g_joint_status.f_current_encoder_position_offset = l_electric_offset_to_zero_in_rad + l_encoder_offset_to_current_in_rad;

						g_joint_status.encoder_position_state = POSITION_ACCURATE;
					}

					if (g_current_sector_number != -1)
					{
						g_previous_sector_number = g_current_sector_number;
					}
					break;

				case POSITION_UNKNOWN:
					g_current_sector_number = get_sector_number_from_calibration(0, g_joint_configuration.number_of_sectors - 1, g_ma730.angle, g_joint_configuration.calibration_table_1[0]);
					if (g_current_sector_number != -1)
					{
						float electric_rotation_width = M_TWOPI / (g_joint_configuration.pole_pairs * g_joint_configuration.gear_ratio);

						g_current_estimated_electric_rotation = g_current_sector_number *  g_joint_configuration.calibration_sector_size; // center current sector

						g_joint_status.f_current_encoder_position_offset = -1 * (g_joint_configuration.zero_electric_rotation - g_current_estimated_electric_rotation) * electric_rotation_width;

						// calculate encoder positioon offset
						g_joint_status.encoder_position_state = POSITION_APROXIMATED;
					}
					break;			
			}

		}

//		g_temp_current_position_after = g_current_electrical_position;
//		if (g_temp_current_position_before != g_temp_current_position_after)
//		{
//			g_temp_count_errors++;
//		}


		// FSM
		FSM_Tick() ;
		
		g_counters.timer7++;
	}
}
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan1, uint32_t RxFifo0ITs) 
{

	if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0)
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
		can_tx_header.Identifier = can_rx_header.Identifier | g_joint_configuration.can_node_id | 0x01 << 9;
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

		if (!l_unicast)
		{
			numer_w_szeregu = g_joint_configuration.can_node_id; // ???
		}

		switch (l_cmd) {

			case 0x0: // Wykonaj akcje
			{
				dlugosc_danych_polecenia = 2;
				// int16_t w zaleznosci od trybu pracy - torque, speed
				uint8_t offset = dlugosc_danych_polecenia * numer_w_szeregu;

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
						// GOAL TORQUE
						// -------------------------------------------------------------------------------------------------
						// recalculate torque goal data from CAN to floats
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
				int32_t l_joint_position_in_s32degree = (int32_t) (g_joint_status.f_current_joint_position * ( (float) UINT32_MAX / M_TWOPI));

//				int16_t l_speed_in_dpp = g_joint_status.f_current_joint_speed * (float) INT16_MAX / M_TWOPI;
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

				// -- MA730
				can_tx_data[14] = g_ma730.angle >> 8;
				can_tx_data[15] = g_ma730.angle;
				can_tx_data[16] = (uint8_t) g_joint_status.current_motor_temperature;
				can_tx_data[17] = g_current_sector_number;
				can_tx_data[18] = g_current_electrical_rotation >> 8;
				can_tx_data[19] = g_current_electrical_rotation;
				can_tx_header.DataLength = FDCAN_DLC_BYTES_20;

				break;
			}

			case 0x1: // Zmien stan FSM
			{
				dlugosc_danych_polecenia = 1;
				// uint8_t - FSM
				uint8_t offset = dlugosc_danych_polecenia * numer_w_szeregu;

				FSM_Set_State(can_rx_data[offset]);

				can_tx_data[0] = FSM_Get_State();

				can_tx_header.DataLength = FDCAN_DLC_BYTES_1;
				break;
			}

			case 0x9: // Dane pozycjonujace
			{
				int32_t l_temp_1 = (int32_t) (g_temp_1 * (float) INT32_MAX);
				int32_t l_temp_3 = (int32_t) (g_temp_3 * (float) INT32_MAX);
				int32_t l_temp_4 = (int32_t) (g_temp_4 * (float) INT32_MAX);

				can_tx_data[0] 	= l_temp_1 >> 24;
				can_tx_data[1] 	= l_temp_1 >> 16;
				can_tx_data[2] 	= l_temp_1 >> 8;
				can_tx_data[3] 	= l_temp_1;

				can_tx_data[4] 	= g_temp_2 >> 8;
				can_tx_data[5] 	= g_temp_2;

				can_tx_data[6] 	= l_temp_3 >> 24;
				can_tx_data[7] 	= l_temp_3 >> 16;
				can_tx_data[8] 	= l_temp_3 >> 8;
				can_tx_data[9] 	= l_temp_3;

				can_tx_data[10] = l_temp_4 >> 24;
				can_tx_data[11] = l_temp_4 >> 16;
				can_tx_data[12] = l_temp_4 >> 8;
				can_tx_data[13] = l_temp_4;

				can_tx_data[14] = g_temp_5 >> 8;
				can_tx_data[15] = g_temp_5;

				can_tx_data[16] = g_temp_6 >> 8;
				can_tx_data[17] = g_temp_6;

				can_tx_data[18] = g_temp_7 >> 8;
				can_tx_data[19] = g_temp_7;

//				dlugosc_danych_polecenia = 1;
//				// uint8_t - FSM
//				uint8_t offset = dlugosc_danych_polecenia * numer_w_szeregu;
//
//				FSM_Set_State(can_rx_data[offset]);
//
//				can_tx_data[0] = FSM_Get_State();
//
//				can_tx_header.DataLength = FDCAN_DLC_BYTES_1;
				can_tx_header.DataLength = FDCAN_DLC_BYTES_20;
				break;
			}

			case 0xA: // RESET
			{
				l_send_response = false;
				NVIC_SystemReset();
				break;
			}

			case 0xB: // SET CAN ID
			{
				if (FSM_Get_State() == FSM_INIT)
				{
					dlugosc_danych_polecenia = 1;
					// uint8_t - can id
					uint8_t offset = dlugosc_danych_polecenia * numer_w_szeregu;
					g_joint_configuration.can_node_id = can_rx_data[offset]; // moze byc od 0 do F - TODO: zapis do FLASH i restart

					FLASH_Configuration_Save(); // flash configuration
					FDCAN_Set_Filters(); // reload can filters

					can_tx_data[0] = 1;
				}
				else
				{
					can_tx_data[0] = 0;
				}


				can_tx_header.DataLength = FDCAN_DLC_BYTES_1;
				break;
			}

			case 0xC: // GET MA730 SECTOR VALUE
			{
				dlugosc_danych_polecenia = 2;
				// uint8_t - FSM
				uint8_t offset = dlugosc_danych_polecenia * numer_w_szeregu;

//				uint16_t sector_id = ((uint16_t) can_rx_data[offset]) << 8 + can_rx_data[offset + 1];
				int16_t sector_id;
				sector_id  = can_rx_data[offset] << 8;
				sector_id += can_rx_data[offset + 1];

				can_tx_data[0] 	= g_joint_configuration.calibration_table_1[sector_id] >> 8;
				can_tx_data[1] 	= g_joint_configuration.calibration_table_1[sector_id];
				can_tx_data[2] 	= g_joint_configuration.calibration_table_2[sector_id] >> 8;
				can_tx_data[3] 	= g_joint_configuration.calibration_table_2[sector_id];

				can_tx_header.DataLength = FDCAN_DLC_BYTES_4;
				break;
			}

			case 0xF: // Konfiguracja
			{
				dlugosc_danych_polecenia = 2;
				// uint8_t tryb pracy
				uint8_t offset = dlugosc_danych_polecenia * numer_w_szeregu;
				if (FSM_Get_State() == FSM_INIT || FSM_Get_State() == FSM_READY_TO_OPERATE)
				{
					g_joint_command.working_mode = can_rx_data[offset];
					g_joint_configuration.working_area_constrain_enabled = (can_rx_data[offset + 1] & 0x01);
					g_joint_configuration.ma730_enabled = (can_rx_data[offset + 1] & 0x02) >> 1;
					g_joint_configuration.safety_enabled = (can_rx_data[offset + 1] & 0x04) >> 2;
					can_tx_data[0] = 1;
				}
				else
				{
					can_tx_data[0] = 0;
				}

				can_tx_header.DataLength = FDCAN_DLC_BYTES_1;
				break;
			}

			default:
			{
				l_send_response = false;

			}

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

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) 
{
//	g_counter2++;
}



void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) 
{
//	g_counter2++;

}



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
	
  uint32_t wtemp;
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