#include "universal_joint.h"
#include "pz_1sf_driver.h"
#include "fsm.h"
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

//union register_data_t g_rw_non_volatile[16] = {0};
//union register_data_t g_ro_non_volatile[16] = {0};
//union register_data_t g_rw_volatile[16] = {0};
//union register_data_t g_ro_volatile[16] = {0};
union register_data_flash_t g_registers_flash[64] = {0}; // 256 baitow

//uint16_t g_ro_non_volatile[64] 	= {0};
//uint16_t g_rw_non_volatile[64] 	= {0};
//uint16_t g_rw_volatile[64] 			= {0};
//uint16_t g_ro_volatile[64] 			= {0};

//uint16_t g_registers[256] = {0};

FDCAN_RxHeaderTypeDef can_rx_header; // CAN Bus Transmit Header
FDCAN_TxHeaderTypeDef can_tx_header; // CAN Bus Transmit Header
uint8_t can_rx_data[64] = {0};  //CAN Bus Receive Buffer
uint8_t can_tx_data[64] = {0};  //CAN Bus Send Buffer

volatile uint16_t g_spi_rx_data    	= 0x0000 ;
volatile uint16_t g_spi_tx_data    	= 0x0000 ;
volatile bool     g_spi_finished 		= false;

volatile bool     g_wchdg_ok 				= false;

// FLASH
// 0x018000 -  98304 (32kB z tylu na konfiguracje)
uint32_t g_flash_address_configuration 			= 0x08018000;
uint32_t g_flash_address_calibration_table 	= 0x08018100;

uint16_t g_calibration_config[20] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint32_t g_data[2] = {0, 0};

int32_t g_diff_electric_position;
float g_diff_zero_electric_position;
float g_diff_current_electric_position;
float g_electric_offset_to_zero_in_rad;
float g_encoder_offset_to_current_in_rad;

#define CALIBRATION_ZERO_POSITION_COUNTER	15
uint16_t g_encoder_position[2][CALIBRATION_ZERO_POSITION_COUNTER] = {0};
float g_encoder_position_summary[2][3] = {0.0};

uint16_t g_mgh_errors = 0;
uint16_t g_mgl_errors = 0;
// PZ FIXME
volatile uint32_t g_pz2656_angle = 0;
volatile float g_pz2656_angle_deg = 0;
volatile uint8_t buf_tx[0x0F + 3] = {0};
volatile uint8_t buf_rx[0x0F + 3] = {0};
volatile uint16_t bufsize = 0;

//uint32_t g_pz2656_id = 0;
//uint8_t data1[4] = {0};
//uint8_t data2[4] = {0};
//uint8_t data3[4] = {0};
//uint8_t data4[4] = {0};

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

volatile MA730_Handle_t	g_ma730 = {
	.started = false,
	.angle = 0x00,
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

#ifdef ENCODER_MA730
void MA730_ReadRegister(uint8_t reg_number);
void MA730_ReadAngle(void);
void MA730_WriteRegister(uint8_t reg_number, uint8_t reg_value);
#endif

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
	// NTC
	NTC_Init(&g_TempBearingSensorParamsM1);
#ifdef ENCODER_MA730
	MA730_WriteRegister(0, 0b00000000);
	MA730_WriteRegister(1, 0b00000000);
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
#endif

#if PCB_VERSION >= 0x030000
	FLASH_Configuration_Load(); // Read configuration from FLASH
#endif

#if PCB_VERSION >= 0x030400
	HAL_GPIO_WritePin(BC_ENABLE_GPIO_Port, BC_ENABLE_Pin, GPIO_PIN_SET);
	//HAL_Delay(100);
#endif

#if PCB_VERSION >= 0x030300
	if (g_registers_flash[0].data[0] == 0xFFFF) // pusta konfiguracja
	{
		uint16_t sdk_version_1 = (0x5) << 8 | (0x5a);
		uint16_t sdk_version_2 = (0x3) << 8 | (0x0);
		
		uint16_t pcb_version_1 = (PCB_VERSION & 0xFFFF0000) >> 16;
		uint16_t pcb_version_2 = (PCB_VERSION & 0x0000FFFF);
		
		uint16_t motor_type = MOTOR_TYPE;
		uint16_t gear_ratio = 121;

		uint16_t revert_direction = 0x01;
		uint16_t offset = 0x00;

		REG_Set(0x00, &sdk_version_1); // SDK_1
		REG_Set(0x01, &sdk_version_2); // SDK_2
		REG_Set(0x02, &pcb_version_1); // PCB_1
		REG_Set(0x03, &pcb_version_2); // PCB_2
		REG_Set(0x06, &motor_type); // MOTOR_TYPE
		REG_Set(0x07, &gear_ratio); // 121
		
		REG_Set(0x21, &revert_direction); // revert_direction
		REG_Set(0x22, &offset); // offset

		FLASH_Configuration_Save();
	}
#endif

if (REG_Get_uint16(0x40) == 0xFFFF)	
{
		uint16_t can_id = 0x00;
		REG_Set(0x40, &can_id); // default can_id = 0

		FLASH_Configuration_Save();
}
	
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
//	pz_write_param(&PZ_IPO_FILT1, 0xEA);	
//	pz_write_param(&PZ_IPO_FILT2, 0x04);	

//	// Data readed from FLASH
//	// DIGITAL ALIGNMENT
//	pz_write_param(&PZ_AI_SCALE, REG_Get_uint16(0x18));
//	pz_write_param(&PZ_AI_PHASE, REG_Get_uint16(0x19));
//	// ANALOG ALIGNMENT
//	pz_write_param(&PZ_COS_OFF,  REG_Get_uint16(0x1A));
//	pz_write_param(&PZ_SIN_OFF,  REG_Get_uint16(0x1B));
//	pz_write_param(&PZ_SC_GAIN,  REG_Get_uint16(0x1C));
//	pz_write_param(&PZ_SC_PHASE, REG_Get_uint16(0x1D));

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

//	g_pz2656.revert_direction = 1;
#endif

	HAL_Delay(100); // Wait to initialize all pheripherals

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

#ifdef ENCODER_MA730
bool motor_reach_torque_limit() 
{
	if (g_joint_status.mc_current_motor_torque > g_calibration_torque_limit && g_joint_status.stm_state_motor == RUN) return true;

	return false;
}


bool motor_in_position(volatile int32_t position) 
{
	if ( llabs((int64_t) (g_joint_status.mc_current_motor_position_multiturn - position)) < CALIBRATION_ZERO_POSITION_OFFSET) return true;

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


int16_t get_sector_number_from_calibration(uint16_t left_index, uint16_t right_index, uint16_t ma730_value, uint16_t offset, uint16_t count)
{
	if (left_index == 0 && right_index == 65535)
	{
		return -1;
	}
	
	if (count > 9)
	{
		return -1;
	}
		
	if (right_index >= left_index) 
	{

		volatile int16_t mid = left_index + (right_index - left_index) / 2; // srodek

		volatile uint32_t mid_left_value  = g_joint_configuration.calibration_table_1[mid];    // wartosc lewego brzegu sektora
		volatile uint32_t mid_right_value = g_joint_configuration.calibration_table_2[mid];    // wartosc prawego brzegu sektora
		volatile uint32_t searched_value  = ma730_value; // wartosc szukana

		volatile uint16_t _count = ++count;

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
			volatile uint16_t index = get_sector_number_from_calibration(left_index, mid, ma730_value, offset, _count); // szukaj z lewej strony
			return index;
		}

		// Else the element can only be present
		// in right subarray
		volatile uint16_t index = get_sector_number_from_calibration(mid, right_index, ma730_value, offset, _count);  // szukaj z prawej strony
		return index;
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

		int32_t right_index_value = g_joint_configuration.calibration_table_1[mid - 1];
		int32_t mid_index_value   = g_joint_configuration.calibration_table_1[mid];
		int32_t left_index_value  = g_joint_configuration.calibration_table_1[mid + 1];
		int32_t searched_value    = ma730_value; // wartosc szukana

		// Przesuniecie wartosci o offset, by funkcja byla w calej dlugosci ciagla
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
				if ( abs(searched_value - mid_index_value) <= abs(searched_value - left_index_value) )
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
				if ( abs(searched_value - mid_index_value) <= abs(searched_value - right_index_value) )
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

#endif
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
	
//#if SDK_VERSION >= 0x055a0000
	g_joint_status.current_motor_temperature 		= NTC_GetAvTemp_C(&TempSensor_M1);
	g_joint_status.current_bearing_temperature 	= NTC_GetAvTemp_C(&g_TempBearingSensorParamsM1);
//#else
//	g_joint_status.current_motor_temperature 		= 25;
//	g_joint_status.current_bearing_temperature 	= 25;
//#endif


#if PCB_VERSION >= 0x030000
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
//	g_joint_status.f_current_joint_position = fmod((double) g_joint_status.f_current_joint_position_multiturn + M_PI, (double) M_TWOPI) - M_PI;
	//g_joint_status.f_current_joint_position = fmod((double) g_joint_status.f_current_joint_position_multiturn, (double) M_TWOPI);

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
	#if PCB_VERSION >= 0x030000
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

	// ENCODER_NOT_ACCURATE
//	if (g_joint_status.encoder_position_state != POSITION_ACCURATE)
//	{
//		g_joint_status.warnings = g_joint_status.warnings | JOINT_POSITION_NOT_ACCURATE;
//	}
//	else
//	{
//		g_joint_status.warnings = g_joint_status.warnings & (0xFF ^ JOINT_POSITION_NOT_ACCURATE);
//	}

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

	if (g_ma730.mgl || g_ma730.mgh) // not proper magnetic field strentgh
	{
		g_joint_status.warnings |= JOINT_MA730_NOT_PROPER_MAGNETOC_FIELD;
		if (g_ma730.mgh) g_mgh_errors++;
		if (g_ma730.mgl) g_mgl_errors++;
	}
	else
	{
		g_joint_status.warnings = g_joint_status.warnings & (0xFF ^ JOINT_MA730_NOT_PROPER_MAGNETOC_FIELD);

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
	can_filter_config_1.FilterID1 = 0x100 + REG_Get_uint8(0x40);
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

//#ifdef ENCODER_MA730
//	Flash_Read_Data(g_flash_address_configuration, (uint32_t *) g_calibration_config, 10);

//	g_joint_configuration.calibration_table_size 						= g_calibration_config[8]; // FIXME !!!!

//	if (g_joint_configuration.calibration_table_size > 0 && g_joint_configuration.calibration_table_size < 65535) {
//		// Read joint configuration from FLASH
//		g_joint_configuration.pole_pairs 											= g_calibration_config[0];
//		g_joint_configuration.gear_ratio 											= g_calibration_config[1];
//		g_joint_configuration.calibration_sector_size 				= g_calibration_config[4];
//		g_joint_configuration.reachable_electrical_rotations	= g_calibration_config[5];
//		g_joint_configuration.number_of_sectors 							= g_calibration_config[8];
//		// 9
//		g_joint_configuration.zero_electric_position 					= (int16_t) g_calibration_config[12];
//		g_joint_configuration.zero_electric_rotation 					= g_calibration_config[13];

//		g_joint_configuration.can_node_id 										= g_calibration_config[16];
//		if (g_joint_configuration.can_node_id == 255)
//		{
//			g_joint_configuration.can_node_id 									= 0;
//		}
//		g_joint_configuration.motor_type 											= g_calibration_config[17];

//		// Read calibration table from FLASH
//		//	for (int i = 0; i <= g_joint_configuration.calibration_table_size / 2 + 1; i++) {
//		for (int i = 0; i <= g_joint_configuration.calibration_table_size; i++) {
//			Flash_Read_Data (g_flash_address_calibration_table + i * 8, (uint32_t *) g_data, 1);
//			g_joint_configuration.calibration_table_1[i]				= g_data[0] >> 16;
//			g_joint_configuration.calibration_table_2[i] 				= g_data[0];
//		}

//		g_joint_configuration.maximum_electrical_rotations = g_joint_configuration.gear_ratio * g_joint_configuration.pole_pairs;

//		g_joint_configuration.electric_rotation_width 				= M_TWOPI / (g_joint_configuration.pole_pairs * g_joint_configuration.gear_ratio); // szerokosc jednego obrotu elektrycznego silnika w stosunku do szerokosci obrotu jointa (2PI)
//		g_joint_configuration.calibration_state 							= JOINT_CALIBRATED;

//	}
//#endif

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
	
#ifdef ENCODER_MA730

	FLASH_EraseInitTypeDef EraseInitStruct;
	EraseInitStruct.TypeErase 	= FLASH_TYPEERASE_PAGES;
	EraseInitStruct.Banks 			= FLASH_BANK_1;
	EraseInitStruct.Page 				= (g_flash_address_configuration & 0x07FFFFFF) / FLASH_PAGE_SIZE;
	EraseInitStruct.NbPages 		= 4; // 1 - 2kB
	uint32_t PageError;

	if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
	{
		error = HAL_FLASH_GetError ();
	}

	uint64_t data;

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

#endif

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

	// Configuration
	// NON VOLATILE RW
//	g_registers[0].data[0] 			= g_joint_configuration.can_node_id;
//	g_registers[15].data[3]			= g_joint_configuration.can_node_id;

//	// NON VOLATILE RO
//	g_registers[3 + 16].data[0] = ((uint32_t) PCB_VERSION >> 16) & 0xFFFF;
//	g_registers[3 + 16].data[1] = (uint32_t) PCB_VERSION & 0xFFFF;

//	g_registers[0 + 16].data[0] = g_joint_configuration.motor_type;
//	g_registers[0 + 16].data[1] = g_joint_configuration.gear_ratio;


//	g_registers[6 + 16].data[0] = g_pz2656.offset; // 2B;
//	g_registers[6 + 16].data[1] = g_pz2656.reg_ai_phase; // 1B
//	g_registers[6 + 16].data[2] = g_pz2656.reg_cos_off; 	// 2B
//	g_registers[6 + 16].data[3] = g_pz2656.reg_sin_off; 	// 2B

//	g_registers[7 + 16].data[0] = g_pz2656.reg_sc_gain; // 2B
//	g_registers[7 + 16].data[1] = g_pz2656.reg_sc_phase; // 1B
//	g_registers[7 + 16].data[2] = (g_pz2656.reg_ecc_amp >> 16) & 0xFFFF; // 2B
//	g_registers[7 + 16].data[3] = g_pz2656.reg_ecc_amp & 0xFFFF; // 2B

//	g_registers[8 + 16].data[0] = g_pz2656.reg_ecc_phase; // 2B
//	g_registers[8 + 16].data[1] = 0x00; // 2B
//	g_registers[8 + 16].data[2] = 0x00; // 2B
//	g_registers[8 + 16].data[3] = 0x00; // 2B

//	g_registers[15 + 16].data[3]= 0x01; // 2B
	
//	uint16_t offset = 0;

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

#ifdef ENCODER_MA730
	g_joint_configuration.pole_pairs = POLE_PAIRS;
#endif
	
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

//#ifdef ENCODER_MA730
//		
//		case FSM_TRANSITION_INIT_TO_CALIBRATION_PHASE_0:
//			FSM_Activate_State(FSM_CALIBRATION_PHASE_0);
//			break;
//		
//		case FSM_CALIBRATION_PHASE_0:
//			g_joint_configuration.absolute_encoder_enabled = true;
//			g_joint_configuration.motor_type = RI70;
//			g_joint_configuration.calibration_state = JOINT_NOT_CALIBRATED;

//			g_current_sector_number = -1;
//			g_previous_sector_number = -1;

////				if (g_joint_configuration.ma730_enabled == false)
////				{
////					FSM_Activate_State(FSM_FAULT_REACTION_ACTIVE);
////					break;
////				}

//			g_joint_configuration.calibration_sector_size = SECTOR_SIZE;
//			g_joint_configuration.pole_pairs = POLE_PAIRS;
//			g_joint_configuration.gear_ratio = GEAR_RATIO;
//			g_joint_configuration.maximum_electrical_rotations = g_joint_configuration.gear_ratio * g_joint_configuration.pole_pairs;
//			g_joint_configuration.calibration_table_size = (uint16_t) (g_joint_configuration.maximum_electrical_rotations / g_joint_configuration.calibration_sector_size);

//			g_joint_configuration.reachable_electrical_rotations = 0;
//			g_joint_configuration.number_of_sectors = 0;
//			g_joint_configuration.zero_electric_position = 0;
//			g_joint_configuration.zero_electric_rotation = 0;
//			for (int i = 0; i < g_joint_configuration.calibration_table_size; i++)
//			{
//				g_joint_configuration.calibration_table_1[i] = 0;
//				g_joint_configuration.calibration_table_2[i] = 0;
//			}
//			FSM_Activate_State(FSM_CALIBRATION_PHASE_1);
//			break;

//		case FSM_CALIBRATION_PHASE_1:
//			if (motor_reach_torque_limit()) // REACH MINIMUM EDGE
//			{
//				motor_stop();
////					g_joint_status.mc_current_electric_rotation = 0; // zeroing electric rotation counter
//				g_current_electrical_rotation = 0;
//				g_min_encoder_position = g_joint_status.mc_current_motor_position_multiturn; // encoder value

//				FSM_Activate_State(FSM_CALIBRATION_PHASE_2);

//			}
//			else
//			{
//				motor_start(SPEED_MODE, -1 * g_calibration_speed);
//			}
//			break;

//		case FSM_CALIBRATION_PHASE_2:
//		{
//			uint16_t sector_number = g_joint_status.mc_current_electric_rotation / g_joint_configuration.calibration_sector_size;
////				uint16_t sector_number = g_joint_status.mc_current_electric_rotation;

//			if ((g_joint_status.mc_current_electric_rotation % g_joint_configuration.calibration_sector_size == 0) &&
//				(g_joint_configuration.calibration_table_1[sector_number] == 0) &&
//				(abs(g_joint_status.mc_current_electric_position) < 2048))
//			{
//				g_joint_configuration.calibration_table_1[sector_number] = g_ma730.angle;
////					g_joint_configuration.calibration_table_1[g_joint_status.mc_current_electric_rotation / g_joint_configuration.calibration_sector_size] = g_joint_status.mc_current_electric_rotation;

//			}

//			if (motor_reach_torque_limit()) // REACH MAXIMUM EDGE
//			{
//				motor_stop();
//				g_joint_configuration.calibration_table_2[sector_number] = g_ma730.angle;
////					g_joint_configuration.calibration_table_2[g_joint_status.mc_current_electric_rotation / g_joint_configuration.calibration_sector_size] = g_joint_status.mc_current_electric_rotation;
//				g_max_electric_rotation_cw 	= g_joint_status.mc_current_electric_rotation; // max electric rotation counter

//				g_max_encoder_position = g_joint_status.mc_current_motor_position_multiturn; // max encoder value
//				FSM_Activate_State(FSM_CALIBRATION_PHASE_3);

//			}
//			else
//			{
//				motor_start(SPEED_MODE, g_calibration_speed);
//			}
//			break;
//		}
//		case FSM_CALIBRATION_PHASE_3:
//		{
//			uint16_t sector_number = g_joint_status.mc_current_electric_rotation / g_joint_configuration.calibration_sector_size;
////				int16_t sector_number = g_joint_status.mc_current_electric_rotation;

//			if (g_joint_status.mc_current_electric_rotation < 0)
//			{
//				break;
//			}

//			if ((g_joint_status.mc_current_electric_rotation % g_joint_configuration.calibration_sector_size == g_joint_configuration.calibration_sector_size - 1) &&
//				(g_joint_configuration.calibration_table_2[sector_number] == 0) &&
//				(abs(g_joint_status.mc_current_electric_position) < 2048))
//			{
//				g_joint_configuration.calibration_table_2[sector_number] = g_ma730.angle;
////					g_joint_configuration.calibration_table_2[g_joint_status.mc_current_electric_rotation / g_joint_configuration.calibration_sector_size] = g_joint_status.mc_current_electric_rotation;
//			}

//			if (motor_reach_torque_limit()) // REACH MINIMUM EDGE
//			{
//				motor_stop();
//				g_joint_configuration.calibration_table_1[sector_number] = g_ma730.angle;

//				if (g_joint_configuration.calibration_table_2[0] == 0 && g_joint_configuration.calibration_table_2[1] > 0) // center of 0 sector is not reached
//				{
//					g_joint_configuration.calibration_table_2[0] = g_ma730.angle;
//				}

////					g_joint_configuration.calibration_table_1[g_joint_status.mc_current_electric_rotation / g_joint_configuration.calibration_sector_size] = g_joint_status.mc_current_electric_rotation;
//				FSM_Activate_State(FSM_CALIBRATION_PHASE_4);

//			}
//			else
//			{
//				motor_start(SPEED_MODE, -1 * g_calibration_speed);
//			}
//			break;
//		}
//		
//		case FSM_CALIBRATION_PHASE_4:
//			// Sprawdzenie tablicy kalibracyjnej
//			if (!check_calibration_data_cw(g_max_electric_rotation_cw / g_joint_configuration.calibration_sector_size) ||
//				!check_calibration_data_ccw(g_max_electric_rotation_cw / g_joint_configuration.calibration_sector_size))
//			{
//				FSM_Activate_State(FSM_CALIBRATION_PHASE_2);
//				g_calibration_state = CALIBRATION_TABLE_CONTAINS_ZEROES;
//			}
//			else
//			{
//				g_max_electric_rotation_ccw = g_joint_status.mc_current_electric_rotation; // Should be 0 right now
//				g_center_encoder_position = (g_max_encoder_position - g_min_encoder_position + 1) / 2 + g_min_encoder_position;

//				g_joint_configuration.number_of_sectors = (uint16_t) (g_max_electric_rotation_cw / g_joint_configuration.calibration_sector_size);
//				g_joint_configuration.reachable_electrical_rotations = g_max_electric_rotation_cw;
//				g_calibration_state = CALIBRATION_OK;
//				FSM_Activate_State(FSM_CALIBRATION_PHASE_5);
//			}
//			break;

//		case FSM_CALIBRATION_PHASE_5:
//			if (motor_in_position(g_center_encoder_position))
//			{
//				FSM_Activate_State(FSM_CALIBRATION_PHASE_6);
//				motor_start(SPEED_MODE, 0);
//				motor_stop();

//				g_joint_configuration.zero_electric_rotation = g_joint_status.mc_current_electric_rotation;
////					g_joint_configuration.zero_electric_position = (uint16_t) (g_motor_status.current_electric_position);
//				g_joint_configuration.zero_electric_position = (int16_t) (g_joint_status.mc_current_electric_position);

//				// Correct calibration table
////					for (int i = 1; i <= g_joint_configuration.number_of_sectors; i++)
////					{
////						if (g_joint_configuration.calibration_table_1[i] < g_joint_configuration.calibration_table_2[i - 1])
////						{
////							uint16_t temp = g_joint_configuration.calibration_table_1[i];
////							g_joint_configuration.calibration_table_1[i] = g_joint_configuration.calibration_table_2[i - 1];
////							g_joint_configuration.calibration_table_2[i - 1] = temp;
////
////						}
////
////						if (g_joint_configuration.calibration_table_1[i] == g_joint_configuration.calibration_table_2[i - 1])
////						{
////							g_joint_configuration.calibration_table_2[i - 1] -= 1;
////							g_joint_configuration.calibration_table_1[i] 	 += 1;
////						}
////					}
//			}
//			else
//			{
//				motor_start(SPEED_MODE, g_calibration_speed);
//			}

//			if (motor_reach_torque_limit()) // REACH EDGE - FAILURE
//			{
//				motor_start(SPEED_MODE, 0);
//				motor_stop();
//				g_calibration_state = MISSED_CENTER_POSITION;
////						g_fsm_status.state = FSM_STOPPED_WITH_ERRORS;
//				FSM_Activate_State(FSM_FAULT_REACTION_ACTIVE);

//			}
//			break;
//		
//		case FSM_CALIBRATION_PHASE_6:
//			HAL_TIM_Base_Stop_IT(&htim6); // Disable 10 kHz timer

//			if (FLASH_Configuration_Save() == 0)
//			{
//				g_joint_configuration.calibration_state = JOINT_CALIBRATED;
//				FDCAN_Set_Filters();
//				FSM_Activate_State(FSM_INIT); // CALIBRATION FINISHED - GO TO INIT STATE
//			}
//			else
//			{
//				FSM_Activate_State(FSM_FAULT_REACTION_ACTIVE); // CALIBRATION FINISHED - GO TO INIT STATE
//			}

//			HAL_TIM_Base_Start_IT(&htim6); // Enable 10 kHz timer
//			break;
//#endif

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
			motor_start(SPEED_MODE, 330);
		
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
			motor_start(SPEED_MODE, 330);
		
			pz_write_command(PZ_COMMAND_AUTO_ADJ_DIG); // AUTO_ADJ_DIG
			g_pz2656.diag = pz_read_param(&PZ_CMD_STAT); // CMD_STAT
		
			static uint16_t fsm_calib_pz_p2_s3_counter;
			fsm_calib_pz_p2_s3_counter++;
			if (fsm_calib_pz_p2_s3_counter > 25000 && g_pz2656.diag == 0x00 && g_joint_status.b_safety_input == 0) { // run 10s
				FSM_Activate_State(FSM_CALIBRATION_PZ_PHASE_2_STEP_4);
			}
			break;

		case FSM_CALIBRATION_PZ_PHASE_2_STEP_4:
			{
				// Koniec Analog Adjustment
				motor_start(TORQUE_MODE, 0);
				//motor_stop();
			
				uint16_t reg_cos_off 	= pz_read_param(&PZ_COS_OFFS); // PZ_COS_OFFS
				uint16_t reg_sin_off 	= pz_read_param(&PZ_SIN_OFFS); // PZ_SIN_OFFS
				uint16_t reg_sc_gain 	= pz_read_param(&PZ_SC_GAINS); // PZ_SC_GAINS
				uint16_t reg_sc_phase	= pz_read_param(&PZ_SC_PHASES); // PZ_SC_PHASES
			
				REG_Set(0x1A, &reg_cos_off);
				REG_Set(0x1B, &reg_sin_off);
				REG_Set(0x1C, &reg_sc_gain);
				REG_Set(0x1D, &reg_sc_phase);

//				pz_write_param(&PZ_COS_OFF, reg_cos_off); // PZ_COS_OFF
//				pz_write_param(&PZ_SIN_OFF, reg_sin_off); // PZ_SIN_OFF
//				pz_write_param(&PZ_SC_GAIN, reg_sc_gain); // PZ_SC_GAIN
//				pz_write_param(&PZ_SC_PHASE, reg_sc_phase); // PZ_SC_PHASE

				pz_write_param(&PZ_IPO_FILT1, 0xEA); // Set <Filter Parameter 1> to ’After analog...’ IPO_FILT1 = 0xEA = 0d234
				pz_write_param(&PZ_IPO_FILT2, 0x04); // Set <Filter Parameter 2> to ’Suitable for any...’ IPO_FILT2 = 0x04
			
				g_joint_configuration.absolute_encoder_enabled = true;
			
				FSM_Activate_State(FSM_CALIBRATION_PZ_FINISH);
			}
			break;

//		// ADJUSTMENT ECC
//		case FSM_CALIBRATION_PZ_PHASE_3_STEP_1:
//		
//			pz_write_param(&PZ_AC_ETO, 		0x01);
//			pz_write_param(&PZ_AC_COUNT, 	0x06);
//			pz_write_param(&PZ_AC_SEL1, 	0x0F);
//			pz_write_param(&PZ_AC_SEL2, 	0x00);

//			pz_write_param(&PZ_ECC_EN, 			0x00);

//			motor_start(TORQUE_MODE, 0);
//			FSM_Activate_State(FSM_CALIBRATION_PZ_PHASE_3_STEP_2);

//			break;

//		case FSM_CALIBRATION_PZ_PHASE_3_STEP_2: // encoder align process - waits to finish
//			if (g_joint_status.stm_state_motor != RUN) 
//			{
//				motor_start(TORQUE_MODE, 0);
//			}
//			else
//			{
//				FSM_Activate_State(FSM_CALIBRATION_PZ_PHASE_3_STEP_3);
//			}
//			break;

//		case FSM_CALIBRATION_PZ_PHASE_3_STEP_3:
//			motor_start(SPEED_MODE, 630);
//		
//			pz_write_command(0xB3); // AUTO_ADJ_ECC
//			g_pz2656.diag = pz_read_param(&PZ_CMD_STAT); // CMD_STAT
//		
//			static uint16_t fsm_calib_pz_p3_s3_counter;
//			fsm_calib_pz_p3_s3_counter++;
//			if (fsm_calib_pz_p3_s3_counter > 25000 && g_pz2656.diag == 0x00) { // run 10s
//				FSM_Activate_State(FSM_CALIBRATION_PZ_PHASE_3_STEP_4);
//			}
//			break;

//		case FSM_CALIBRATION_PZ_PHASE_3_STEP_4:
//			motor_stop();

//			g_pz2656.reg_ecc_amp	 	= pz_read_param(&PZ_ECC_AMP); // PZ_ECC_AMP
//			g_pz2656.reg_ecc_phase 	= pz_read_param(&PZ_ECC_PHASE); // PZ_ECC_PHASE
//		
//			pz_write_param(&PZ_ECC_EN, 0x01);
//			
//			FSM_Activate_State(FSM_INIT);
//			break;

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
		
//			REG_Set(0x58, (uint16_t *) &g_pz2656.offset); // offset
//			REG_Set(0x58, (uint16_t *) &g_pz2656.offset); // offset

//			HAL_TIM_Base_Stop_IT(&htim7); // Disable 1 kHz timer
//			HAL_TIM_Base_Stop_IT(&htim6); // Disable 10 kHz timer

//			if (FLASH_Configuration_Save() == 0)
//			{
//				g_joint_configuration.calibration_state = JOINT_CALIBRATED;
//				FDCAN_Set_Filters();
//				FSM_Activate_State(FSM_INIT); // CALIBRATION FINISHED - GO TO INIT STATE
//			}
//			else
//			{
//				FSM_Activate_State(FSM_FAULT_REACTION_ACTIVE); // CALIBRATION FINISHED - GO TO INIT STATE
//			}

//			HAL_TIM_Base_Start_IT(&htim7); // Enable 1 kHz timer
//			HAL_TIM_Base_Start_IT(&htim6); // Enable 10 kHz timer

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

		case FSM_CALIBRATION_PHASE_0:
		{

			if (FSM_Get_State() == FSM_INIT)
			{
				return FSM_Activate_Transition(FSM_TRANSITION_INIT_TO_CALIBRATION_PHASE_0);
			}

			break;
		}
		
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

//		case FSM_INIT:
//		{
//			if (FSM_Get_State() == FSM_CALIBRATION_PZ_STORE_CONFIGURATION)
//			{
//				return FSM_Activate_State(FSM_INIT);
//			}

//			break;
//		}
		
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

#ifdef ENCODER_MA730
		if (g_joint_configuration.absolute_encoder_enabled == true)
		{
			MA730_ReadRegister(0x1B);
		}
#endif

#ifdef ENCODER_PZ2656
		if (g_joint_configuration.absolute_encoder_enabled == true)
		{
			if (REG_Get_uint16(0x22) != 0)
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

			if (REG_Get_uint8(0x21)) {
				//g_pz2656.angle 			= UINT16_MAX - (uint16_t) ((int32_t) REG_Get_uint16(0xD8) - (int32_t) REG_Get_uint16(0x22));
				g_pz2656.angle 			= UINT16_MAX - (uint16_t) ((int32_t) g_pz2656.readings - (int32_t) REG_Get_uint16(0x22));
			} 
			else
			{
				//g_pz2656.angle 			= (uint16_t) ((int32_t) REG_Get_uint16(0xD8) - (int32_t) REG_Get_uint16(0x22));
				g_pz2656.angle 			= (uint16_t) ((int32_t) g_pz2656.readings - (int32_t) REG_Get_uint16(0x22));
			}
			
			g_joint_status.f_current_joint_position_from_absolute_encoder =  fmod((double) ((double) g_pz2656.angle / UINT16_MAX) * M_TWOPI + M_PI, (double) M_TWOPI) - M_PI ;
		}

//		}
//		else
//		{
//			g_pz2656.started = false;
//		}
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

#ifdef ENCODER_MA730
		
		// Estimate joint position from absolute encoder
		if (g_joint_configuration.absolute_encoder_enabled == true && g_joint_configuration.calibration_state == JOINT_CALIBRATED && g_joint_configuration.number_of_sectors > 0)
		{
			// JOINT POSITION ESTIMATION
			switch (g_joint_status.encoder_position_state)
			{
				case POSITION_ESTIMATION_FAILED:
					// RAISE ERROR
					break;
//
				case POSITION_ACCURATE:
//						g_current_sector_number = get_rotation_number_from_calibration_table(0, g_joint_configuration.number_of_sectors - 1, g_ma730.angle, g_joint_configuration.calibration_table_1[0], 0);
					break;

				case POSITION_APROXIMATED:
					g_current_sector_number = get_sector_number_from_calibration(0, g_joint_configuration.number_of_sectors - 1, g_ma730.angle, g_joint_configuration.calibration_table_1[0], 0);

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
					g_current_sector_number = get_sector_number_from_calibration(0, g_joint_configuration.number_of_sectors - 1, g_ma730.angle, g_joint_configuration.calibration_table_1[0], 0);
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

#endif

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

void REG_Write(uint8_t poczatek, uint8_t koniec, uint8_t * data) 
{
	uint8_t * p_g_registers = (uint8_t *) &g_registers_flash;
	// ustalenie poczatku pamieci
//	memcpy ( &g_registers + poczatek, data, sizeof(uint16_t) * (koniec - poczatek) );
	for (int i = 0; i < (koniec - poczatek) / 2; i++)
	{
		*(p_g_registers + poczatek * 2 + i * 2 + 1) = data[i];
		*(p_g_registers + poczatek * 2 + i * 2)     = data[i + 1];
	}
}

void REG_Read(uint8_t poczatek, uint8_t koniec, uint8_t * data) 
{
	uint8_t * p_g_registers = (uint8_t *) &g_registers_flash;

	for (int i = 0; i < koniec - poczatek; i++)
	{
		data[i] = *(p_g_registers + poczatek * 2 + i * 2 + 1);
		data[i + 1]= *(p_g_registers + poczatek * 2 + i * 2);
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
		can_tx_header.Identifier = can_rx_header.Identifier | REG_Get_uint8(0x40) | 0x01 << 9;
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
			numer_w_szeregu = REG_Get_uint8(0x40); // ???
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
//					can_tx_data[14] = g_ma730.angle >> 8;
//					can_tx_data[15] = g_ma730.angle;
					can_tx_data[16] = (uint8_t) g_joint_status.current_motor_temperature;
	//				can_tx_data[17] = g_current_sector_number;
	//				can_tx_data[18] = g_current_electrical_rotation >> 8;
	//				can_tx_data[19] = g_current_electrical_rotation;
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
					offset = dlugosc_danych_polecenia * numer_w_szeregu + 2;
					uint8_t poczatek = can_rx_data[0]; // rejestr poczatkowy
					uint8_t dlugosc  = can_rx_data[1] * 2; // ilosc rejestrow (16bit)
					uint8_t koniec   = poczatek + dlugosc;
					
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
					uint8_t dlugosc  = can_rx_data[1] * 2; // ilosc rejestrow (16bit)
					uint8_t koniec   = poczatek + dlugosc;
					
					offset = dlugosc * numer_w_szeregu + 2;
					// offset + dlugosc nie moze przekroczyc 64 baitow
					
					// sprawdzenie zakresu
//					if ((( (poczatek - 0) | (0 - poczatek) | (koniec - 63) | (63 - koniec) ) >= 0) || (( (poczatek - 80) | (80 - poczatek) | (koniec - 191) | (191 - koniec) ) >= 0) && (offset + dlugosc < 64) )
//					{
						uint8_t * p_data;
						p_data = (uint8_t *) malloc (dlugosc * 2 * sizeof(uint8_t));
						memcpy(p_data, (uint8_t * ) &(can_rx_data[offset]), dlugosc * 2 * sizeof(uint8_t));
						// poprawny zakres
						REG_Write(poczatek, koniec, p_data);
						free( p_data );
//					}
//					else
//					{
//							// blad zakresu
//					}
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

			case 0xB: // SET CAN ID
				{
					if (FSM_Get_State() == FSM_INIT)
					{
						dlugosc_danych_polecenia = 1;
						// uint8_t - can id
						uint8_t offset = dlugosc_danych_polecenia * numer_w_szeregu;
						//g_joint_configuration.can_node_id = can_rx_data[offset]; // moze byc od 0 do F - TODO: zapis do FLASH i restart
						uint8_t can_node_id = can_rx_data[offset];
						REG_Set(0x40, ( uint16_t * ) &can_node_id);

						//FLASH_Configuration_Save(); // flash configuration
						FDCAN_Set_Filters(); // reload can filters

						can_tx_data[0] = 1;
					}
					else
					{
						can_tx_data[0] = 0;
					}

					can_tx_header.DataLength = FDCAN_DLC_BYTES_1;
				}
				break;

			case 0xC: // Odczyt rejestrow - EEPROM
				{
					dlugosc_danych_polecenia = 2;
					// int8_t - rejestr poczatkowy
					// int8_t - dlugosc <= 64
					offset = dlugosc_danych_polecenia * numer_w_szeregu + 2;
					uint8_t poczatek = can_rx_data[0]; // rejestr poczatkowy
					uint8_t dlugosc  = can_rx_data[1] * 2; // ilosc rejestrow (16bit)
					uint8_t koniec   = poczatek + dlugosc;
					
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

			case 0xD: // Zapis rejestrow - EEPROM - 13 BANKOW, 64 bajty kazdy
				{
					//dlugosc_danych_polecenia = 2;
					// int8_t - rejestr poczatkowy (zapis mozliwy tylko 0-63, 80-191)
					// int8_t - dlugosc
					uint8_t poczatek = can_rx_data[0]; // rejestr poczatkowy
					uint8_t dlugosc  = can_rx_data[1] * 2; // ilosc rejestrow (16bit)
					uint8_t koniec   = poczatek + dlugosc;
					
					offset = dlugosc * numer_w_szeregu + 2;
					// offset + dlugosc nie moze przekroczyc 64 baitow
					
					// sprawdzenie zakresu
//					if ((( (poczatek - 0) | (0 - poczatek) | (koniec - 63) | (63 - koniec) ) >= 0) || (( (poczatek - 80) | (80 - poczatek) | (koniec - 191) | (191 - koniec) ) >= 0) && (offset + dlugosc < 64) )
//					{
						uint8_t * p_data;
						p_data = (uint8_t *) malloc (dlugosc * 2 * sizeof(uint8_t));
						memcpy(p_data, (uint8_t * ) &(can_rx_data[offset]), dlugosc * 2 * sizeof(uint8_t));
						// poprawny zakres
						REG_Write(poczatek, koniec, p_data);
						free( p_data );
//					}
//					else
//					{
//							// blad zakresu
//					}
				}
				break;
				
//#ifdef ENCODER_MA730

//			case 0xC: // GET MA730 SECTOR VALUE
//				dlugosc_danych_polecenia = 2;
//				// uint8_t - FSM
//				offset = dlugosc_danych_polecenia * numer_w_szeregu;

////				uint16_t sector_id = ((uint16_t) can_rx_data[offset]) << 8 + can_rx_data[offset + 1];
//				int16_t sector_id;
//				sector_id  = can_rx_data[offset] << 8;
//				sector_id += can_rx_data[offset + 1];

//				can_tx_data[0] 	= g_joint_configuration.calibration_table_1[sector_id] >> 8;
//				can_tx_data[1] 	= g_joint_configuration.calibration_table_1[sector_id];
//				can_tx_data[2] 	= g_joint_configuration.calibration_table_2[sector_id] >> 8;
//				can_tx_data[3] 	= g_joint_configuration.calibration_table_2[sector_id];

//				can_tx_header.DataLength = FDCAN_DLC_BYTES_4;
//				break;
//#endif

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

#if PCB_VERSION >= 0x030000
		HAL_SPI_TransmitReceive(&hspi1, data_tx, data_rx, datasize, 1);
#else
//		HAL_SPI_TransmitReceive(&hspi2, (uint8_t * ) &send_data, (uint8_t * ) &angle_value, 1, 1);
#endif

	HAL_GPIO_WritePin(ABSOLUTE_ENCODER_CS_GPIO_Port, ABSOLUTE_ENCODER_CS_Pin, GPIO_PIN_SET);
}

#endif

//#pragma GCC pop_options

#ifdef ENCODER_MA730
// MA730
void MA730_ReadRegister(uint8_t reg_number) 
{
	uint16_t send_data      = 0b010 << 13 | (reg_number & (0b00011111)) << 8 ;

	uint16_t angle_value    = 0;
	uint16_t register_value = 0;

	for (uint16_t i = 0; i < 24; i++) __NOP();  // wait about 150ns

	HAL_GPIO_WritePin(ABSOLUTE_ENCODER_CS_GPIO_Port, ABSOLUTE_ENCODER_CS_Pin, GPIO_PIN_RESET);

	for (uint16_t i = 0; i < 130; i++) __NOP();  // wait about 80ns

	// SEND READ REGISTER COMMAND - RECEIVE READ ANGLE RESULT
#if PCB_VERSION >= 0x030000
	HAL_SPI_TransmitReceive(&hspi1, (uint8_t * ) &send_data, (uint8_t * ) &angle_value, 1, 1);
#else
	HAL_SPI_TransmitReceive(&hspi2, (uint8_t * ) &send_data, (uint8_t * ) &angle_value, 1, 1);
#endif
	HAL_GPIO_WritePin(ABSOLUTE_ENCODER_CS_GPIO_Port, ABSOLUTE_ENCODER_CS_Pin, GPIO_PIN_SET);

	for (uint16_t i = 0; i < 120; i++) __NOP();  // wait about 750ns

	HAL_GPIO_WritePin(ABSOLUTE_ENCODER_CS_GPIO_Port, ABSOLUTE_ENCODER_CS_Pin, GPIO_PIN_RESET);

	for (uint16_t i = 0; i < 13; i++) __NOP();  // wait about 80ns

	send_data      = 0x0000;

	// SEND READ ANGLE COMMAND - RECEIVE READ REGISTER RESULT
#if PCB_VERSION >= 0x030000
	HAL_SPI_TransmitReceive(&hspi1, (uint8_t * ) &send_data, (uint8_t * ) &register_value, 1, 1);
#else
	HAL_SPI_TransmitReceive(&hspi2, (uint8_t * ) &send_data, (uint8_t * ) &register_value, 1, 1);
#endif

//	g_MA730_read_buffer = register_value >> 8;

	HAL_GPIO_WritePin(ABSOLUTE_ENCODER_CS_GPIO_Port, ABSOLUTE_ENCODER_CS_Pin, GPIO_PIN_SET);

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
}

void MA730_ReadAngle() 
{
	uint16_t send_data      = 0x0000 ;

	uint16_t angle_value    = 0;

//	for (uint16_t i = 0; i < 24; i++) NOP;  // wait about 150ns

	HAL_GPIO_WritePin(ABSOLUTE_ENCODER_CS_GPIO_Port, ABSOLUTE_ENCODER_CS_Pin, GPIO_PIN_RESET);

//	for (uint16_t i = 0; i < 13; i++) NOP;  // wait about 80ns

	// SEND READ REGISTER COMMAND - RECEIVE READ ANGLE RESULT
#if PCB_VERSION >= 0x030000
	HAL_SPI_TransmitReceive(&hspi1, (uint8_t * ) &send_data, (uint8_t * ) &angle_value, 1, 1);
#else
	HAL_SPI_TransmitReceive(&hspi2, (uint8_t * ) &send_data, (uint8_t * ) &angle_value, 1, 1);
#endif
	
	HAL_GPIO_WritePin(ABSOLUTE_ENCODER_CS_GPIO_Port, ABSOLUTE_ENCODER_CS_Pin, GPIO_PIN_SET);

//	for (uint16_t i = 0; i < 12; i++) NOP;  // wait about 80ns

	g_ma730.angle = (angle_value >> 2) & 0b0011111111111111;

}

void MA730_WriteRegister(uint8_t reg_number, uint8_t reg_value) 
{
	uint16_t send_data      = 0b100 << 13 | (reg_number & (0b00011111)) << 8 | reg_value;

	uint16_t angle_value    = 0;
	uint16_t register_value = 0;

	for (uint16_t i = 0; i < 24; i++) __NOP();  // wait about 150ns

	HAL_GPIO_WritePin(ABSOLUTE_ENCODER_CS_GPIO_Port, ABSOLUTE_ENCODER_CS_Pin, GPIO_PIN_RESET);

	for (uint16_t i = 0; i < 130; i++) __NOP();  // wait about 80ns

	// SEND READ REGISTER COMMAND - RECEIVE READ ANGLE RESULT
#if PCB_VERSION >= 0x030000
	HAL_SPI_TransmitReceive(&hspi1, (uint8_t * ) &send_data, (uint8_t * ) &angle_value, 1, 1);
#else
	HAL_SPI_TransmitReceive(&hspi2, (uint8_t * ) &send_data, (uint8_t * ) &angle_value, 1, 1);
#endif

	HAL_GPIO_WritePin(ABSOLUTE_ENCODER_CS_GPIO_Port, ABSOLUTE_ENCODER_CS_Pin, GPIO_PIN_SET);

	HAL_Delay(20); // Wait 20 ms after write command

	HAL_GPIO_WritePin(ABSOLUTE_ENCODER_CS_GPIO_Port, ABSOLUTE_ENCODER_CS_Pin, GPIO_PIN_RESET);

	for (uint16_t i = 0; i < 13; i++) __NOP();  // wait about 80ns

	send_data      = 0x0000;

	// SEND READ ANGLE COMMAND - RECEIVE READ REGISTER RESULT
#if PCB_VERSION >= 0x030000
	HAL_SPI_TransmitReceive(&hspi1, (uint8_t * ) &send_data, (uint8_t * ) &register_value, 1, 1);
#else
	HAL_SPI_TransmitReceive(&hspi2, (uint8_t * ) &send_data, (uint8_t * ) &register_value, 1, 1);
#endif

	HAL_GPIO_WritePin(ABSOLUTE_ENCODER_CS_GPIO_Port, ABSOLUTE_ENCODER_CS_Pin, GPIO_PIN_SET);

	for (uint16_t i = 0; i < 120; i++) __NOP();  // wait about 750ns

}
#endif


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
