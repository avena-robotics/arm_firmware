/**
 ******************************************************************************
 * @file    pz_1sf_driver.h
 * @author  iC-Haus GmbH
 * @version 1.1.0
 * @note Designed according to iC-PZ datasheet release E1 for chip revision X.
 ******************************************************************************
 * @attention
 *
 *	Software and its documentation is provided by iC-Haus GmbH or contributors "AS IS" and is
 *	subject to the ZVEI General Conditions for the Supply of Products and Services with iC-Haus
 *	amendments and the ZVEI Software clause with iC-Haus amendments (http://www.ichaus.de/EULA).
 *
 ******************************************************************************
 */

#ifndef PZ_1SF_DRIVER_H
#define PZ_1SF_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @defgroup Wrapper_Functions
 * @brief Functions declared as `extern` are to be defined by the user according to the individual MCU.
 *
 * @note The definition of those functions is mandatory for proper use of this driver.
 * @{
 */
/**
 * @brief This function is a wrapper to be defined with the hardware-related SPI-transmit-receive-function.
 *
 * @param data_tx is a pointer to the data buffer that is transmitted.
 * @param data_rx is a pointer to the buffer the received data is written to.
 * @param datasize is the length of all bytes transmitted.
 * @retval None
 */
extern void pz_spi_transfer(uint8_t *data_tx, uint8_t *data_rx, uint16_t datasize);
/**
 * @}
 */

/**
 * @defgroup PZ_CMD
 * @brief Commands that can be sent to iC-PZ by calling function @ref pz_write_command.
 * @{
 */
typedef enum {
	PZ_COMMAND_OK = 0x00,

	PZ_COMMAND_REBOOT = 0x10,
	PZ_COMMAND_ADI_RESET = 0x18,

	PZ_COMMAND_SCLEAR = 0x20,
	PZ_COMMAND_DIAG_USER0_RESET = 0x28,
	PZ_COMMAND_DIAG_USER0_SET = 0x29,
	PZ_COMMAND_DIAG_USER1_RESET = 0x2A,
	PZ_COMMAND_DIAG_USER1_SET = 0x2B,
	PZ_COMMAND_DIAG_USER2_RESET = 0x2C,
	PZ_COMMAND_DIAG_USER2_SET = 0x2D,

	PZ_COMMAND_CRC_CALC = 0x30,
	PZ_COMMAND_CRC_CHECK = 0x31,

	PZ_COMMAND_CONF_READ_ALL = 0x40,
	PZ_COMMAND_CONF_WRITE_ALL = 0x41,
	PZ_COMMAND_CONF_READ = 0x42,
	PZ_COMMAND_CONF_WRITE = 0x43,

	PZ_COMMAND_MTST_PRESET = 0x80,
	PZ_COMMAND_MT_PRESET = 0x81,
	PZ_COMMAND_ABZ_PRESET = 0x82,
	PZ_COMMAND_UVW_PRESET = 0x83,

	PZ_COMMAND_MTST_PRESET_STORE = 0x88,
	PZ_COMMAND_MT_PRESET_STORE = 0x89,
	PZ_COMMAND_ABZ_PRESET_STORE = 0x8A,
	PZ_COMMAND_UVW_PRESET_STORE = 0x8B,

	PZ_COMMAND_GPIO_OUT0_SET0 = 0x90,
	PZ_COMMAND_GPIO_OUT0_SET1 = 0x91,
	PZ_COMMAND_GPIO_OUT1_SET0 = 0x92,
	PZ_COMMAND_GPIO_OUT1_SET1 = 0x93,

	PZ_COMMAND_RPL_SET_NA = 0xA0,
	PZ_COMMAND_RPL_SET_RO = 0xA1,
	PZ_COMMAND_RPL_GET = 0xA2,

	PZ_COMMAND_AUTO_ADJ_ANA = 0xB0,
	PZ_COMMAND_AUTO_ADJ_DIG = 0xB1,
	PZ_COMMAND_AUTO_READJ_DIG = 0xB2,
	PZ_COMMAND_AUTO_ADJ_ECC = 0xB3,

	PZ_COMMAND_FORCE_BISS = 0xC1,
	PZ_COMMAND_FORCE_SSI = 0xC2,

	PZ_COMMAND_CHIP_ID = 0xE0,
	PZ_COMMAND_CHIP_REV = 0xE1,

	PZ_COMMAND_FAIL = 0xFF,
} PZ_COMMANDS;
/**
 * @}
 */

/**
 * @defgroup PZ_Parameters
 * @{
 */
/**
 * @addtogroup PZ_Parameters_Struct
 * @brief Structure for representing iC-PZ parameters.
 * @{
 */
struct pz_param {
	uint8_t bank;
	uint8_t addr;
	uint8_t pos;
	uint8_t len;
};
/**
 * @}
 */

/**
 * @addtogroup PZ_Parameters_List
 * @brief List of parameters according to iC-PZ register map represented as @ref PZ_Parameters_Struct.
 * @{
 */
/**
 * @addtogroup PZ_Parameters_Bank_0x0
 * @brief Interface Ports Configuration, Interpolator, System Definition, Position Data Length, Absolute Data Interface (ADI), Singleturn Position Evaluation
 * @{
 */
extern const struct pz_param PZ_PADA_CFG;
extern const struct pz_param PZ_PADB_CFG;
extern const struct pz_param PZ_PADC_CFG;
extern const struct pz_param PZ_PADM_CFG;
extern const struct pz_param PZ_VDDIOSEL;
extern const struct pz_param PZ_PORTA_DIS;
extern const struct pz_param PZ_PORTB_DIS;
extern const struct pz_param PZ_PORTC_DIS;
extern const struct pz_param PZ_IPO_FILT1;
extern const struct pz_param PZ_IPO_FILT2;
extern const struct pz_param PZ_CD_INV;
extern const struct pz_param PZ_CD_FLIP;
extern const struct pz_param PZ_SYS_OVR;
extern const struct pz_param PZ_ST_PDL;
extern const struct pz_param PZ_MT_PDL;
extern const struct pz_param PZ_ADI_SBL;
extern const struct pz_param PZ_ADI_OS;
extern const struct pz_param PZ_ADI_EBL;
extern const struct pz_param PZ_ADI_EBP;
extern const struct pz_param PZ_ADI_MSO;
extern const struct pz_param PZ_ADI_CFG;
extern const struct pz_param PZ_RAN_TOL;
extern const struct pz_param PZ_RAN_FLD;
extern const struct pz_param PZ_CFG_CRC_0;
/**
 * @}
 */

/**
 * @addtogroup PZ_Parameters_Bank_0x1
 * @brief Adjustment Analog, Adjustment Digital
 * @{
 */
extern const struct pz_param PZ_COS_OFF;
extern const struct pz_param PZ_SIN_OFF;
extern const struct pz_param PZ_SC_GAIN;
extern const struct pz_param PZ_SC_PHASE;
extern const struct pz_param PZ_AI_PHASE;
extern const struct pz_param PZ_AI_SCALE;
extern const struct pz_param PZ_CFG_CRC_1;
extern const struct pz_param PZ_COS_OFFS;
extern const struct pz_param PZ_SIN_OFFS;
extern const struct pz_param PZ_SC_GAINS;
extern const struct pz_param PZ_SC_PHASES;
extern const struct pz_param PZ_AI_PHASES;
extern const struct pz_param PZ_AI_SCALES;
/**
 * @}
 */

/**
 * @addtogroup PZ_Parameters_Bank_0x2
 * @brief Adjustment Analog Configuration, Adjustment Digital Configuration, Adjustment Eccentricity
 * @{
 */
extern const struct pz_param PZ_SC_OFF_SEL;
extern const struct pz_param PZ_SC_GAIN_SEL;
extern const struct pz_param PZ_SC_PHASE_SEL;
extern const struct pz_param PZ_AI_P_SEL;
extern const struct pz_param PZ_AI_S_SEL;
extern const struct pz_param PZ_ECC_AMP;
extern const struct pz_param PZ_ECC_PHASE;
extern const struct pz_param PZ_ECC_EN;
extern const struct pz_param PZ_CFG_CRC_2;
/**
 * @}
 */

/**
 * @addtogroup PZ_Parameters_Bank_0x3
 * @brief Analog Output
 * @{
 */
extern const struct pz_param PZ_LED_CTRL;
extern const struct pz_param PZ_LED_CUR;
extern const struct pz_param PZ_LED_CONST;
extern const struct pz_param PZ_ANA_SEL;
extern const struct pz_param PZ_ANA_OS;
extern const struct pz_param PZ_CFG_CRC_3;
/**
 * @}
 */

/**
 * @addtogroup PZ_Parameters_Bank_0x4
 * @brief ABZ Generator
 * @{
 */
extern const struct pz_param PZ_ABZ_PER;
extern const struct pz_param PZ_ABZ_HYS;
extern const struct pz_param PZ_ABZ_MTD;
extern const struct pz_param PZ_ABZ_ZGATE;
extern const struct pz_param PZ_ABZ_CFG;
extern const struct pz_param PZ_ABZ_OFF;
extern const struct pz_param PZ_CFG_CRC_4;
/**
 * @}
 */

/**
 * @addtogroup PZ_Parameters_Bank_0x5
 * @brief UVW Generator
 * @{
 */
extern const struct pz_param PZ_UVW_PP;
extern const struct pz_param PZ_UVW_CFG;
extern const struct pz_param PZ_UVW_OFF;
extern const struct pz_param PZ_CFG_CRC_5;
/**
 * @}
 */

/**
 * @addtogroup PZ_Parameters_Bank_0x6
 * @brief BiSS/SSI Slave
 * @{
 */
extern const struct pz_param PZ_BISS_EM;
extern const struct pz_param PZ_BISS_WM;
extern const struct pz_param PZ_BISS_ST_DL;
extern const struct pz_param PZ_BISS_MT_DL;
extern const struct pz_param PZ_BISS_ENSOL;
extern const struct pz_param PZ_BISS_CRC16;
extern const struct pz_param PZ_BISS_NTOA;
extern const struct pz_param PZ_SSI_GRAY;
extern const struct pz_param PZ_SSI_EXT;
extern const struct pz_param PZ_SSI_EN;
extern const struct pz_param PZ_BISS_CRCS;
extern const struct pz_param PZ_CFG_CRC_6;
/**
 * @}
 */

/**
 * @addtogroup PZ_Parameters_Bank_0x7
 * @brief SPI Slave
 * @{
 */
extern const struct pz_param PZ_SPI_EM;
extern const struct pz_param PZ_SPI_WM;
extern const struct pz_param PZ_SPI_ST_DL;
extern const struct pz_param PZ_SPI_MT_DL;
extern const struct pz_param PZ_SPI_EXT;
extern const struct pz_param PZ_SPI_CRCS;
extern const struct pz_param PZ_CFG_CRC_7;
/**
 * @}
 */

/**
 * @addtogroup PZ_Parameters_Bank_0x8
 * @brief FlexCode
 * @{
 */
extern const struct pz_param PZ_FCL;
extern const struct pz_param PZ_FCS;
extern const struct pz_param PZ_CFG_CRC_8;
/**
 * @}
 */

/**
 * @addtogroup PZ_Parameters_Bank_0x9
 * @brief GPIO
 * @{
 */
extern const struct pz_param PZ_GPIO0_M;
extern const struct pz_param PZ_GPIO1_M;
extern const struct pz_param PZ_GPIO0_CFG;
extern const struct pz_param PZ_GPIO0_DIAG;
extern const struct pz_param PZ_GPIO0_SEL;
extern const struct pz_param PZ_GPIO1_CFG;
extern const struct pz_param PZ_GPIO1_DIAG;
extern const struct pz_param PZ_GPIO1_SEL;
extern const struct pz_param PZ_CFG_CRC_9;
/**
 * @}
 */

/**
 * @addtogroup PZ_Parameters_Bank_0xA
 * @brief I2C Master
 * @{
 */
extern const struct pz_param PZ_I2C_DEV_ID_0;
extern const struct pz_param PZ_I2C_DEV_ID_1;
extern const struct pz_param PZ_I2C_DEV_ID_2;
extern const struct pz_param PZ_I2C_DEV_ID_3;
extern const struct pz_param PZ_I2C_F_0;
extern const struct pz_param PZ_I2C_T_0;
extern const struct pz_param PZ_I2C_F_1;
extern const struct pz_param PZ_I2C_T_1;
extern const struct pz_param PZ_I2C_F_2;
extern const struct pz_param PZ_I2C_T_2;
extern const struct pz_param PZ_I2C_F_3;
extern const struct pz_param PZ_I2C_T_3;
extern const struct pz_param PZ_CFG_CRC_A;
/**
 * @}
 */

/**
 * @addtogroup PZ_Parameters_Bank_0xB
 * @brief Position Offset Singleturn
 * @{
 */
extern const struct pz_param PZ_ST_OFF;
extern const struct pz_param PZ_CFG_CRC_B;
/**
 * @}
 */

/**
 * @addtogroup PZ_Parameters_Bank_0xC
 * @brief Position Offset Multiturn
 * @{
 */
extern const struct pz_param PZ_MT_OFF;
extern const struct pz_param PZ_CFG_CRC_C;
/**
 * @}
 */

/**
 * @addtogroup PZ_Parameters_Bank_0xD
 * @brief Temperature Monitoring
 * @{
 */
extern const struct pz_param PZ_TEMP_L_1;
extern const struct pz_param PZ_TEMP_L_2;
extern const struct pz_param PZ_TEMP_LT_1;
extern const struct pz_param PZ_TEMP_LT_2;
extern const struct pz_param PZ_CFG_CRC_D;
/**
 * @}
 */

/**
 * @addtogroup PZ_Parameters_Bank_0xE
 * @brief Profile, BiSS Identifier
 * @{
 */
extern const struct pz_param PZ_EDS_BANK_X;
extern const struct pz_param PZ_BISS_PROFILE_ID_1_X;
extern const struct pz_param PZ_BISS_PROFILE_ID_0_X;
extern const struct pz_param PZ_SERIAL_3_X;
extern const struct pz_param PZ_SERIAL_2_X;
extern const struct pz_param PZ_SERIAL_1_X;
extern const struct pz_param PZ_SERIAL_0_X;
extern const struct pz_param PZ_DEV_ID_5_X;
extern const struct pz_param PZ_DEV_ID_4_X;
extern const struct pz_param PZ_DEV_ID_3_X;
extern const struct pz_param PZ_DEV_ID_2_X;
extern const struct pz_param PZ_DEV_ID_1_X;
extern const struct pz_param PZ_DEV_ID_0_X;
extern const struct pz_param PZ_MFG_ID_1_X;
extern const struct pz_param PZ_MFG_ID_0_X;
extern const struct pz_param PZ_CFG_CRC_E;
/**
 * @}
 */

/**
 * @addtogroup PZ_Parameters_Direct_Access
 * @brief Bank Selection, Revision & Identification, GPIO, Temperature Sensor, Presets, Autocalibration Configuration, Absolute Data Interface (ADI), I2C Device Data, Diagnosis, Bank CRC Status, Commands, BiSS Identifier
 * @{
 */
extern const struct pz_param PZ_BSEL;
extern const struct pz_param PZ_EDS_BANK;
extern const struct pz_param PZ_BISS_PROFILE_ID_1;
extern const struct pz_param PZ_BISS_PROFILE_ID_0;
extern const struct pz_param PZ_SERIAL_3;
extern const struct pz_param PZ_SERIAL_2;
extern const struct pz_param PZ_SERIAL_1;
extern const struct pz_param PZ_SERIAL_0;
extern const struct pz_param PZ_ID;
extern const struct pz_param PZ_REV;
extern const struct pz_param PZ_SYS;
extern const struct pz_param PZ_GPIO_OUT;
extern const struct pz_param PZ_GPIO_IN;
extern const struct pz_param PZ_TEMP;
extern const struct pz_param PZ_ST_PRE;
extern const struct pz_param PZ_MT_PRE;
extern const struct pz_param PZ_ABZ_PRE;
extern const struct pz_param PZ_UVW_PRE;
extern const struct pz_param PZ_AC_SEL1;
extern const struct pz_param PZ_AC_SEL2;
extern const struct pz_param PZ_AC_COUNT;
extern const struct pz_param PZ_AC_ETO;
extern const struct pz_param PZ_ADI_SB;
extern const struct pz_param PZ_I2C_DATA_0;
extern const struct pz_param PZ_I2C_DATA_1;
extern const struct pz_param PZ_I2C_DATA_2;
extern const struct pz_param PZ_I2C_DATA_3;
extern const struct pz_param PZ_DIAG;
extern const struct pz_param PZ_ERR;
extern const struct pz_param PZ_WARN;
extern const struct pz_param PZ_CRC_STAT;
extern const struct pz_param PZ_CMD_STAT;
extern const struct pz_param PZ_CMD;
extern const struct pz_param PZ_DEV_ID_5;
extern const struct pz_param PZ_DEV_ID_4;
extern const struct pz_param PZ_DEV_ID_3;
extern const struct pz_param PZ_DEV_ID_2;
extern const struct pz_param PZ_DEV_ID_1;
extern const struct pz_param PZ_DEV_ID_0;
extern const struct pz_param PZ_MFG_ID_1;
extern const struct pz_param PZ_MFG_ID_0;
/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @defgroup PZ_Functions
 * @{
 */
/**
 * @addtogroup PZ_Functions_Basic
 * @brief Functions according to iC-PZ SPI opcodes.
 * @{
 */
void pz_read_registers(uint8_t addr, uint8_t *data_rx, uint8_t datasize);
void pz_write_registers(uint8_t addr, const uint8_t *data_tx, uint8_t datasize);
void pz_read_position(uint8_t *data_rx, uint8_t datasize);
void pz_write_command(PZ_COMMANDS command);
void pz_read_diagnosis(uint8_t data_rx[8]);
void pz_request_data_from_i2c_slave(uint8_t addr);
void pz_transmit_data_to_i2c_slave(uint8_t addr, uint8_t data);
void pz_get_transaction_info(uint8_t *status_rx, uint8_t *data_rx);
void pz_activate_slave_in_chain(uint8_t ra_pa_configuration_byte);
/**
 * @}
 */

/**
 * @addtogroup PZ_Functions_Advanced
 * @brief Functions going through a sequence of @ref PZ_Functions_Basic with enhanced data handling.
 * @{
 */
void pz_switch_bank(uint8_t bank);
uint32_t pz_read_param(const struct pz_param *param);
void pz_write_param(const struct pz_param *param, uint32_t param_val);
/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* PZ_1SF_DRIVER_H */
