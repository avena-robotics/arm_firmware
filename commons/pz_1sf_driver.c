/**
 ******************************************************************************
 * @file    pz_1sf_driver.c
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

#include "pz_1sf_driver.h"

/* iC-PZ opcodes */
enum PZ_OPCODES {
	PZ_OPCODE_READ_REGISTERS = 0x81,
	PZ_OPCODE_WRITE_REGISTERS = 0xCF,
	PZ_OPCODE_READ_POSITION = 0xA6,
	PZ_OPCODE_WRITE_COMMAND = 0xD9,
	PZ_OPCODE_READ_DIAGNOSIS = 0x9C,
	PZ_OPCODE_REQUEST_DATA_FROM_I2C_SLAVE = 0x97,
	PZ_OPCODE_TRANSMIT_DATA_TO_I2C_SLAVE = 0xD2,
	PZ_OPCODE_GET_TRANSACTION_INFO = 0xAD,
	PZ_OPCODE_ACTIVATE_SLAVE_IN_CHAIN = 0xB0,
};

/* iC-PZ parameters */
const struct pz_param PZ_PADA_CFG = { .bank = 0x00, .addr = 0x00, .pos = 1, .len = 2, };
const struct pz_param PZ_PADB_CFG = { .bank = 0x00, .addr = 0x00, .pos = 3, .len = 2, };
const struct pz_param PZ_PADC_CFG = { .bank = 0x00, .addr = 0x00, .pos = 5, .len = 2, };
const struct pz_param PZ_PADM_CFG = { .bank = 0x00, .addr = 0x00, .pos = 7, .len = 2, };
const struct pz_param PZ_VDDIOSEL = { .bank = 0x00, .addr = 0x01, .pos = 1, .len = 2, };
const struct pz_param PZ_PORTA_DIS = { .bank = 0x00, .addr = 0x01, .pos = 4, .len = 1, };
const struct pz_param PZ_PORTB_DIS = { .bank = 0x00, .addr = 0x01, .pos = 5, .len = 1, };
const struct pz_param PZ_PORTC_DIS = { .bank = 0x00, .addr = 0x01, .pos = 6, .len = 1, };
const struct pz_param PZ_IPO_FILT1 = { .bank = 0x00, .addr = 0x03, .pos = 7, .len = 8, };
const struct pz_param PZ_IPO_FILT2 = { .bank = 0x00, .addr = 0x04, .pos = 2, .len = 3, };
const struct pz_param PZ_CD_INV = { .bank = 0x00, .addr = 0x07, .pos = 0, .len = 1, };
const struct pz_param PZ_CD_FLIP = { .bank = 0x00, .addr = 0x07, .pos = 1, .len = 1, };
const struct pz_param PZ_SYS_OVR = { .bank = 0x00, .addr = 0x07, .pos = 7, .len = 4, };
const struct pz_param PZ_ST_PDL = { .bank = 0x00, .addr = 0x08, .pos = 5, .len = 6, };
const struct pz_param PZ_MT_PDL = { .bank = 0x00, .addr = 0x09, .pos = 5, .len = 6, };
const struct pz_param PZ_ADI_SBL = { .bank = 0x00, .addr = 0x0A, .pos = 2, .len = 3, };
const struct pz_param PZ_ADI_OS = { .bank = 0x00, .addr = 0x0A, .pos = 7, .len = 5, };
const struct pz_param PZ_ADI_EBL = { .bank = 0x00, .addr = 0x0B, .pos = 3, .len = 4, };
const struct pz_param PZ_ADI_EBP = { .bank = 0x00, .addr = 0x0B, .pos = 4, .len = 1, };
const struct pz_param PZ_ADI_MSO = { .bank = 0x00, .addr = 0x0B, .pos = 7, .len = 3, };
const struct pz_param PZ_ADI_CFG = { .bank = 0x00, .addr = 0x0D, .pos = 0, .len = 9, };
const struct pz_param PZ_RAN_TOL = { .bank = 0x00, .addr = 0x0F, .pos = 3, .len = 4, };
const struct pz_param PZ_RAN_FLD = { .bank = 0x00, .addr = 0x0F, .pos = 7, .len = 1, };
const struct pz_param PZ_CFG_CRC_0 = { .bank = 0x00, .addr = 0x10, .pos = 7, .len = 8, };

const struct pz_param PZ_COS_OFF = { .bank = 0x01, .addr = 0x01, .pos = 7, .len = 10, };
const struct pz_param PZ_SIN_OFF = { .bank = 0x01, .addr = 0x03, .pos = 7, .len = 10, };
const struct pz_param PZ_SC_GAIN = { .bank = 0x01, .addr = 0x05, .pos = 7, .len = 10, };
const struct pz_param PZ_SC_PHASE = { .bank = 0x01, .addr = 0x07, .pos = 7, .len = 10, };
const struct pz_param PZ_AI_PHASE = { .bank = 0x01, .addr = 0x09, .pos = 7, .len = 10, };
const struct pz_param PZ_AI_SCALE = { .bank = 0x01, .addr = 0x0B, .pos = 7, .len = 9, };
const struct pz_param PZ_CFG_CRC_1 = { .bank = 0x01, .addr = 0x10, .pos = 7, .len = 8, };
const struct pz_param PZ_COS_OFFS = { .bank = 0x01, .addr = 0x21, .pos = 7, .len = 10, };
const struct pz_param PZ_SIN_OFFS = { .bank = 0x01, .addr = 0x23, .pos = 7, .len = 10, };
const struct pz_param PZ_SC_GAINS = { .bank = 0x01, .addr = 0x25, .pos = 7, .len = 10, };
const struct pz_param PZ_SC_PHASES = { .bank = 0x01, .addr = 0x27, .pos = 7, .len = 10, };
const struct pz_param PZ_AI_PHASES = { .bank = 0x01, .addr = 0x29, .pos = 7, .len = 10, };
const struct pz_param PZ_AI_SCALES = { .bank = 0x01, .addr = 0x2B, .pos = 7, .len = 9, };

const struct pz_param PZ_SC_OFF_SEL = { .bank = 0x02, .addr = 0x00, .pos = 3, .len = 4, };
const struct pz_param PZ_SC_GAIN_SEL = { .bank = 0x02, .addr = 0x00, .pos = 7, .len = 4, };
const struct pz_param PZ_SC_PHASE_SEL = { .bank = 0x02, .addr = 0x01, .pos = 3, .len = 4, };
const struct pz_param PZ_AI_P_SEL = { .bank = 0x02, .addr = 0x03, .pos = 3, .len = 4, };
const struct pz_param PZ_AI_S_SEL = { .bank = 0x02, .addr = 0x03, .pos = 7, .len = 4, };
const struct pz_param PZ_ECC_AMP = { .bank = 0x02, .addr = 0x07, .pos = 7, .len = 32, };
const struct pz_param PZ_ECC_PHASE = { .bank = 0x02, .addr = 0x09, .pos = 7, .len = 14, };
const struct pz_param PZ_ECC_EN = { .bank = 0x02, .addr = 0x0A, .pos = 0, .len = 1, };
const struct pz_param PZ_CFG_CRC_2 = { .bank = 0x02, .addr = 0x10, .pos = 7, .len = 8, };

const struct pz_param PZ_LED_CTRL = { .bank = 0x03, .addr = 0x00, .pos = 0, .len = 1, };
const struct pz_param PZ_LED_CUR = { .bank = 0x03, .addr = 0x00, .pos = 3, .len = 3, };
const struct pz_param PZ_LED_CONST = { .bank = 0x03, .addr = 0x00, .pos = 4, .len = 1, };
const struct pz_param PZ_ANA_SEL = { .bank = 0x03, .addr = 0x01, .pos = 1, .len = 2, };
const struct pz_param PZ_ANA_OS = { .bank = 0x03, .addr = 0x01, .pos = 2, .len = 1, };
const struct pz_param PZ_CFG_CRC_3 = { .bank = 0x03, .addr = 0x10, .pos = 7, .len = 8, };

const struct pz_param PZ_ABZ_PER = { .bank = 0x04, .addr = 0x03, .pos = 2, .len = 27, };
const struct pz_param PZ_ABZ_HYS = { .bank = 0x04, .addr = 0x04, .pos = 7, .len = 8, };
const struct pz_param PZ_ABZ_MTD = { .bank = 0x04, .addr = 0x05, .pos = 3, .len = 4, };
const struct pz_param PZ_ABZ_ZGATE = { .bank = 0x04, .addr = 0x06, .pos = 3, .len = 4, };
const struct pz_param PZ_ABZ_CFG = { .bank = 0x04, .addr = 0x06, .pos = 6, .len = 3, };
const struct pz_param PZ_ABZ_OFF = { .bank = 0x04, .addr = 0x09, .pos = 7, .len = 16, };
const struct pz_param PZ_CFG_CRC_4 = { .bank = 0x04, .addr = 0x10, .pos = 7, .len = 8, };

const struct pz_param PZ_UVW_PP = { .bank = 0x05, .addr = 0x00, .pos = 4, .len = 5, };
const struct pz_param PZ_UVW_CFG = { .bank = 0x05, .addr = 0x01, .pos = 1, .len = 2, };
const struct pz_param PZ_UVW_OFF = { .bank = 0x05, .addr = 0x03, .pos = 7, .len = 16, };
const struct pz_param PZ_CFG_CRC_5 = { .bank = 0x05, .addr = 0x10, .pos = 7, .len = 8, };

const struct pz_param PZ_BISS_EM = { .bank = 0x06, .addr = 0x03, .pos = 7, .len = 32, };
const struct pz_param PZ_BISS_WM = { .bank = 0x06, .addr = 0x07, .pos = 7, .len = 32, };
const struct pz_param PZ_BISS_ST_DL = { .bank = 0x06, .addr = 0x08, .pos = 5, .len = 6, };
const struct pz_param PZ_BISS_MT_DL = { .bank = 0x06, .addr = 0x09, .pos = 5, .len = 6, };
const struct pz_param PZ_BISS_ENSOL = { .bank = 0x06, .addr = 0x0A, .pos = 0, .len = 1, };
const struct pz_param PZ_BISS_CRC16 = { .bank = 0x06, .addr = 0x0A, .pos = 1, .len = 1, };
const struct pz_param PZ_BISS_NTOA = { .bank = 0x06, .addr = 0x0A, .pos = 2, .len = 1, };
const struct pz_param PZ_SSI_GRAY = { .bank = 0x06, .addr = 0x0A, .pos = 4, .len = 1, };
const struct pz_param PZ_SSI_EXT = { .bank = 0x06, .addr = 0x0A, .pos = 5, .len = 1, };
const struct pz_param PZ_SSI_EN = { .bank = 0x06, .addr = 0x0A, .pos = 6, .len = 1, };
const struct pz_param PZ_BISS_CRCS = { .bank = 0x06, .addr = 0x0B, .pos = 5, .len = 6, };
const struct pz_param PZ_CFG_CRC_6 = { .bank = 0x06, .addr = 0x10, .pos = 7, .len = 8, };

const struct pz_param PZ_SPI_EM = { .bank = 0x07, .addr = 0x03, .pos = 7, .len = 32, };
const struct pz_param PZ_SPI_WM = { .bank = 0x07, .addr = 0x07, .pos = 7, .len = 32, };
const struct pz_param PZ_SPI_ST_DL = { .bank = 0x07, .addr = 0x08, .pos = 5, .len = 6, };
const struct pz_param PZ_SPI_MT_DL = { .bank = 0x07, .addr = 0x09, .pos = 5, .len = 6, };
const struct pz_param PZ_SPI_EXT = { .bank = 0x07, .addr = 0x0A, .pos = 0, .len = 1, };
const struct pz_param PZ_SPI_CRCS = { .bank = 0x07, .addr = 0x0B, .pos = 5, .len = 6, };
const struct pz_param PZ_CFG_CRC_7 = { .bank = 0x07, .addr = 0x10, .pos = 7, .len = 8, };

const struct pz_param PZ_FCL = { .bank = 0x08, .addr = 0x01, .pos = 6, .len = 15, };
const struct pz_param PZ_FCS = { .bank = 0x08, .addr = 0x03, .pos = 6, .len = 15, };
const struct pz_param PZ_CFG_CRC_8 = { .bank = 0x08, .addr = 0x10, .pos = 7, .len = 8, };

const struct pz_param PZ_GPIO0_M = { .bank = 0x09, .addr = 0x03, .pos = 7, .len = 32, };
const struct pz_param PZ_GPIO1_M = { .bank = 0x09, .addr = 0x07, .pos = 7, .len = 32, };
const struct pz_param PZ_GPIO0_CFG = { .bank = 0x09, .addr = 0x08, .pos = 1, .len = 2, };
const struct pz_param PZ_GPIO0_DIAG = { .bank = 0x09, .addr = 0x08, .pos = 2, .len = 1, };
const struct pz_param PZ_GPIO0_SEL = { .bank = 0x09, .addr = 0x08, .pos = 3, .len = 1, };
const struct pz_param PZ_GPIO1_CFG = { .bank = 0x09, .addr = 0x08, .pos = 5, .len = 2, };
const struct pz_param PZ_GPIO1_DIAG = { .bank = 0x09, .addr = 0x08, .pos = 6, .len = 1, };
const struct pz_param PZ_GPIO1_SEL = { .bank = 0x09, .addr = 0x08, .pos = 7, .len = 1, };
const struct pz_param PZ_CFG_CRC_9 = { .bank = 0x09, .addr = 0x10, .pos = 7, .len = 8, };

const struct pz_param PZ_I2C_DEV_ID_0 = { .bank = 0x0A, .addr = 0x00, .pos = 7, .len = 8, };
const struct pz_param PZ_I2C_DEV_ID_1 = { .bank = 0x0A, .addr = 0x01, .pos = 7, .len = 8, };
const struct pz_param PZ_I2C_DEV_ID_2 = { .bank = 0x0A, .addr = 0x02, .pos = 7, .len = 8, };
const struct pz_param PZ_I2C_DEV_ID_3 = { .bank = 0x0A, .addr = 0x03, .pos = 7, .len = 8, };
const struct pz_param PZ_I2C_F_0 = { .bank = 0x0A, .addr = 0x04, .pos = 0, .len = 1, };
const struct pz_param PZ_I2C_T_0 = { .bank = 0x0A, .addr = 0x04, .pos = 7, .len = 7, };
const struct pz_param PZ_I2C_F_1 = { .bank = 0x0A, .addr = 0x05, .pos = 0, .len = 1, };
const struct pz_param PZ_I2C_T_1 = { .bank = 0x0A, .addr = 0x05, .pos = 7, .len = 7, };
const struct pz_param PZ_I2C_F_2 = { .bank = 0x0A, .addr = 0x06, .pos = 0, .len = 1, };
const struct pz_param PZ_I2C_T_2 = { .bank = 0x0A, .addr = 0x06, .pos = 7, .len = 7, };
const struct pz_param PZ_I2C_F_3 = { .bank = 0x0A, .addr = 0x07, .pos = 0, .len = 1, };
const struct pz_param PZ_I2C_T_3 = { .bank = 0x0A, .addr = 0x07, .pos = 7, .len = 7, };
const struct pz_param PZ_CFG_CRC_A = { .bank = 0x0A, .addr = 0x10, .pos = 7, .len = 8, };

const struct pz_param PZ_ST_OFF = { .bank = 0x0B, .addr = 0x03, .pos = 7, .len = 32, };
const struct pz_param PZ_CFG_CRC_B = { .bank = 0x0B, .addr = 0x10, .pos = 7, .len = 8, };

const struct pz_param PZ_MT_OFF = { .bank = 0x0C, .addr = 0x03, .pos = 7, .len = 32, };
const struct pz_param PZ_CFG_CRC_C = { .bank = 0x0C, .addr = 0x10, .pos = 7, .len = 8, };

const struct pz_param PZ_TEMP_L_1 = { .bank = 0x0D, .addr = 0x01, .pos = 3, .len = 12, };
const struct pz_param PZ_TEMP_L_2 = { .bank = 0x0D, .addr = 0x03, .pos = 3, .len = 12, };
const struct pz_param PZ_TEMP_LT_1 = { .bank = 0x0D, .addr = 0x04, .pos = 0, .len = 1, };
const struct pz_param PZ_TEMP_LT_2 = { .bank = 0x0D, .addr = 0x04, .pos = 1, .len = 1, };
const struct pz_param PZ_CFG_CRC_D = { .bank = 0x0D, .addr = 0x10, .pos = 7, .len = 8, };

const struct pz_param PZ_EDS_BANK_X = { .bank = 0x0E, .addr = 0x01, .pos = 7, .len = 8, };
const struct pz_param PZ_BISS_PROFILE_ID_1_X = { .bank = 0x0E, .addr = 0x02, .pos = 7, .len = 8, };
const struct pz_param PZ_BISS_PROFILE_ID_0_X = { .bank = 0x0E, .addr = 0x03, .pos = 7, .len = 8, };
const struct pz_param PZ_SERIAL_3_X = { .bank = 0x0E, .addr = 0x04, .pos = 7, .len = 8, };
const struct pz_param PZ_SERIAL_2_X = { .bank = 0x0E, .addr = 0x05, .pos = 7, .len = 8, };
const struct pz_param PZ_SERIAL_1_X = { .bank = 0x0E, .addr = 0x06, .pos = 7, .len = 8, };
const struct pz_param PZ_SERIAL_0_X = { .bank = 0x0E, .addr = 0x07, .pos = 7, .len = 8, };
const struct pz_param PZ_DEV_ID_5_X = { .bank = 0x0E, .addr = 0x08, .pos = 7, .len = 8, };
const struct pz_param PZ_DEV_ID_4_X = { .bank = 0x0E, .addr = 0x09, .pos = 7, .len = 8, };
const struct pz_param PZ_DEV_ID_3_X = { .bank = 0x0E, .addr = 0x0A, .pos = 7, .len = 8, };
const struct pz_param PZ_DEV_ID_2_X = { .bank = 0x0E, .addr = 0x0B, .pos = 7, .len = 8, };
const struct pz_param PZ_DEV_ID_1_X = { .bank = 0x0E, .addr = 0x0C, .pos = 7, .len = 8, };
const struct pz_param PZ_DEV_ID_0_X = { .bank = 0x0E, .addr = 0x0D, .pos = 7, .len = 8, };
const struct pz_param PZ_MFG_ID_1_X = { .bank = 0x0E, .addr = 0x0E, .pos = 7, .len = 8, };
const struct pz_param PZ_MFG_ID_0_X = { .bank = 0x0E, .addr = 0x0F, .pos = 7, .len = 8, };
const struct pz_param PZ_CFG_CRC_E = { .bank = 0x0E, .addr = 0x10, .pos = 7, .len = 8, };

const struct pz_param PZ_BSEL = { .addr = 0x40, .pos = 7, .len = 8, };
const struct pz_param PZ_EDS_BANK = { .addr = 0x41, .pos = 7, .len = 8, };
const struct pz_param PZ_BISS_PROFILE_ID_1 = { .addr = 0x42, .pos = 7, .len = 8, };
const struct pz_param PZ_BISS_PROFILE_ID_0 = { .addr = 0x43, .pos = 7, .len = 8, };
const struct pz_param PZ_SERIAL_3 = { .addr = 0x44, .pos = 7, .len = 8, };
const struct pz_param PZ_SERIAL_2 = { .addr = 0x45, .pos = 7, .len = 8, };
const struct pz_param PZ_SERIAL_1 = { .addr = 0x46, .pos = 7, .len = 8, };
const struct pz_param PZ_SERIAL_0 = { .addr = 0x47, .pos = 7, .len = 8, };
const struct pz_param PZ_ID = { .addr = 0x4B, .pos = 7, .len = 32, };
const struct pz_param PZ_REV = { .addr = 0x4C, .pos = 3, .len = 4, };
const struct pz_param PZ_SYS = { .addr = 0x4C, .pos = 7, .len = 4, };
const struct pz_param PZ_GPIO_OUT = { .addr = 0x4D, .pos = 1, .len = 2, };
const struct pz_param PZ_GPIO_IN = { .addr = 0x4D, .pos = 3, .len = 2, };
const struct pz_param PZ_TEMP = { .addr = 0x4F, .pos = 7, .len = 16, };
const struct pz_param PZ_ST_PRE = { .addr = 0x53, .pos = 7, .len = 32, };
const struct pz_param PZ_MT_PRE = { .addr = 0x57, .pos = 7, .len = 32, };
const struct pz_param PZ_ABZ_PRE = { .addr = 0x59, .pos = 7, .len = 16, };
const struct pz_param PZ_UVW_PRE = { .addr = 0x5B, .pos = 7, .len = 16, };
const struct pz_param PZ_AC_SEL1 = { .addr = 0x5C, .pos = 3, .len = 4, };
const struct pz_param PZ_AC_SEL2 = { .addr = 0x5C, .pos = 7, .len = 4, };
const struct pz_param PZ_AC_COUNT = { .addr = 0x5D, .pos = 3, .len = 4, };
const struct pz_param PZ_AC_ETO = { .addr = 0x5D, .pos = 7, .len = 1, };
const struct pz_param PZ_ADI_SB = { .addr = 0x5E, .pos = 3, .len = 4, };
const struct pz_param PZ_I2C_DATA_0 = { .addr = 0x61, .pos = 7, .len = 16, };
const struct pz_param PZ_I2C_DATA_1 = { .addr = 0x63, .pos = 7, .len = 16, };
const struct pz_param PZ_I2C_DATA_2 = { .addr = 0x65, .pos = 7, .len = 16, };
const struct pz_param PZ_I2C_DATA_3 = { .addr = 0x67, .pos = 7, .len = 16, };
const struct pz_param PZ_DIAG = { .addr = 0x6B, .pos = 7, .len = 32, };
const struct pz_param PZ_ERR = { .addr = 0x6F, .pos = 7, .len = 32, };
const struct pz_param PZ_WARN = { .addr = 0x73, .pos = 7, .len = 32, };
const struct pz_param PZ_CRC_STAT = { .addr = 0x75, .pos = 7, .len = 16, };
const struct pz_param PZ_CMD_STAT = { .addr = 0x76, .pos = 7, .len = 8, };
const struct pz_param PZ_CMD = { .addr = 0x77, .pos = 7, .len = 8, };
const struct pz_param PZ_DEV_ID_5 = { .addr = 0x78, .pos = 7, .len = 8, };
const struct pz_param PZ_DEV_ID_4 = { .addr = 0x79, .pos = 7, .len = 8, };
const struct pz_param PZ_DEV_ID_3 = { .addr = 0x7A, .pos = 7, .len = 8, };
const struct pz_param PZ_DEV_ID_2 = { .addr = 0x7B, .pos = 7, .len = 8, };
const struct pz_param PZ_DEV_ID_1 = { .addr = 0x7C, .pos = 7, .len = 8, };
const struct pz_param PZ_DEV_ID_0 = { .addr = 0x7D, .pos = 7, .len = 8, };
const struct pz_param PZ_MFG_ID_1 = { .addr = 0x7E, .pos = 7, .len = 8, };
const struct pz_param PZ_MFG_ID_0 = { .addr = 0x7F, .pos = 7, .len = 8, };

/* globals */
static uint8_t buf_tx[0xFF + 3];
static uint8_t buf_rx[0xFF + 3];
static uint16_t bufsize;

/**
 * @brief This function is used to read data from consecutive registers in the on-chip RAM.
 *
 * @param addr is the address of the first register to start reading from.
 * @param data_rx is a pointer to a buffer the received data is stored.
 * @param datasize is the number of consecutive registers to be read.
 * @retval None
 */
void pz_read_registers(uint8_t addr, uint8_t *data_rx, uint8_t datasize) {
	bufsize = datasize + 3;
	buf_tx[0] = PZ_OPCODE_READ_REGISTERS;
	buf_tx[1] = addr;
	buf_tx[2] = 0x00;

	for (uint16_t i = 3; i < bufsize; i++) {
		buf_tx[i] = 0x00;
	}

	pz_spi_transfer(buf_tx, buf_rx, bufsize);

	for (uint8_t j = 0; j < datasize; j++) {
		data_rx[j] = buf_rx[j + 3];
	}
}

/**
 * @brief This function is used to write data to consecutive registers in the on-chip RAM.
 *
 * @param addr is the address of the first register to start writing to.
 * @param data_tx is a pointer to a buffer the transmitted data is stored.
 * @param datasize is the number of consecutive registers to be written.
 * @retval None
 */
void pz_write_registers(uint8_t addr, const uint8_t *data_tx, uint8_t datasize) {
	bufsize = datasize + 2;
	buf_tx[0] = PZ_OPCODE_WRITE_REGISTERS;
	buf_tx[1] = addr;

	for (uint16_t i = 0; i < datasize; i++) {
		buf_tx[i + 2] = data_tx[i];
	}

	pz_spi_transfer(buf_tx, buf_rx, bufsize);
}

/**
 * @brief This function is used to read position data from iC-PZ.
 *
 * @note Transmission of full bytes is supported. Surplus bits are padded with zeros.
 * @note Position data format can be configured via SPI_EXT, SPI_ST_DL, SPI_MT_DL, ST_PDL and MT_PDL.
 * @note Position data is latched on the first rising edge of SCLK.
 *
 * @param data_rx is a pointer to a buffer the received data is stored.
 * @param datasize is the number of full bytes according to the configured position data format.
 * @retval None
 */
void pz_read_position(uint8_t *data_rx, uint8_t datasize) {
	bufsize = datasize + 1;
	buf_tx[0] = PZ_OPCODE_READ_POSITION;

	for (uint16_t i = 1; i < bufsize; i++) {
		buf_tx[i] = 0x00;
	}

	pz_spi_transfer(buf_tx, buf_rx, bufsize);

	for (uint8_t j = 0; j < datasize; j++) {
		data_rx[j] = buf_rx[j + 1];
	}
}

/**
 * @brief This function is used to execute a command.
 *
 * @note Commands do require processing time until completed. Completion can be detected by polling the CMD register.
 *
 * @param command according to @ref PZ_CMD.
 * @retval None
 */
void pz_write_command(PZ_COMMANDS command) {
	bufsize = 2;
	buf_tx[0] = PZ_OPCODE_WRITE_COMMAND;
	buf_tx[1] = command;

	pz_spi_transfer(buf_tx, buf_rx, bufsize);
}

/**
 * @brief This function is used to read chip diagnosis.
 *
 * @param data_rx is a pointer to a buffer of size 8 byte the received data is stored.
 * @retval None
 */
void pz_read_diagnosis(uint8_t data_rx[8]) {
	bufsize = 10;
	buf_tx[0] = PZ_OPCODE_READ_DIAGNOSIS;

	for (uint16_t i = 1; i < bufsize; i++) {
		buf_tx[i] = 0x00;
	}

	pz_spi_transfer(buf_tx, buf_rx, bufsize);

	for (uint16_t j = 0; j < bufsize - 2; j++) {
		data_rx[j] = buf_rx[j + 2];
	}
}

/**
 * @brief This function is used to request one data byte from a register of an external device connected to the I2C master of iC-PZ.
 *
 * @note After the opcode has been received, the initiated I2C communication will take additional time until completed.
 * @note Function @ref pz_get_i2c_transaction_info can be used to poll for current communication status and new data.
 *
 * @param addr is the address of the external register data is requested from.
 * @retval None
 */
void pz_request_data_from_i2c_slave(uint8_t addr) {
	bufsize = 2;
	buf_tx[0] = PZ_OPCODE_REQUEST_DATA_FROM_I2C_SLAVE;
	buf_tx[1] = addr;

	pz_spi_transfer(buf_tx, buf_rx, bufsize);
}

/**
 * @brief This function is used to transmit one data byte to a register of an external device connected to the I2C master of iC-PZ.
 *
 * @note After the opcode has been received, the initiated I2C communication will take additional time until completed.
 * @note Function @ref pz_get_i2c_transaction_info can be used to poll for current communication status and new data.
 *
 * @param addr is the address of the external register to transmit data to.
 * @param data is the byte transmitted to the external registers.
 * @retval None
 */
void pz_transmit_data_to_i2c_slave(uint8_t addr, uint8_t data) {
	bufsize = 3;
	buf_tx[0] = PZ_OPCODE_TRANSMIT_DATA_TO_I2C_SLAVE;
	buf_tx[1] = addr;
	buf_tx[2] = data;

	pz_spi_transfer(buf_tx, buf_rx, bufsize);
}

/**
 * @brief This function is used to get the status of the last initiated SPI transaction.
 *
 * @note The data byte is only defined, if function @ref pz_request_data_from_i2c_slave has been called before.
 *
 * @param status_rx is a pointer to an integer the received status is stored.
 * @param data_rx is a pointer to an integer the received data is stored.
 * @retval None
 */
void pz_get_transaction_info(uint8_t *status_rx, uint8_t *data_rx) {
	bufsize = 3;
	buf_tx[0] = PZ_OPCODE_GET_TRANSACTION_INFO;
	buf_tx[1] = 0x00;
	buf_tx[2] = 0x00;

	pz_spi_transfer(buf_tx, buf_rx, bufsize);

	*status_rx = buf_rx[1];
	*data_rx = buf_rx[2];
}

/**
 * @brief This function is used to activate iC-PZ slaves connected in an SPI daisy chain.
 *
 * @note Not meant to be used in point-to-point or bus topologies.
 * @note Usage of this function is limited to 4 slaves maximum.
 *
 * @param rapa_configuration_byte contains the channels that are set active via their corresponding bit.
 * @retval None
 */
void pz_activate_slave_in_chain(uint8_t ra_pa_configuration_byte) {
	bufsize = 2;
	buf_tx[0] = PZ_OPCODE_ACTIVATE_SLAVE_IN_CHAIN;
	buf_tx[1] = ra_pa_configuration_byte;

	pz_spi_transfer(buf_tx, buf_rx, bufsize);
}

/**
 * @brief This function switches the active bank.
 *
 * @param bank to be switched to.
 * @retval None
 */
void pz_switch_bank(uint8_t bank) {
	pz_write_registers(PZ_BSEL.addr, &bank, 1);
}

/**
 * @brief This function reads a specific chip parameter.
 *
 * @param param has to be one of the parameters defined in @ref PZ_Parameters_List.
 * @retval Value of the parameter read.
 */
uint32_t pz_read_param(const struct pz_param *param) {
	uint8_t datasize = 0;

	if (param->len <= 8) {
		datasize = 1;
	}
	else if (param->len <= 16) {
		datasize = 2;
	}
	else if (param->len <= 24) {
		datasize = 3;
	}
	else {
		datasize = 4;
	}

	if (param->addr < PZ_BSEL.addr) {
		pz_switch_bank(param->bank);
	}

	uint8_t param_buf_rx[4] = { 0, 0, 0, 0 };
	pz_read_registers(param->addr - (datasize - 1), param_buf_rx, datasize);

	uint32_t param_buf_rx_to_uint32 = 0;
	for (uint8_t i = 0; i < datasize; i++) {
		param_buf_rx_to_uint32 |= (uint32_t)param_buf_rx[i] << (i * 8);
	}

	uint8_t start_pos_lsb = 0;
	if (datasize == 1) {
		start_pos_lsb = param->pos - (param->len - 1);
	}
	else if (param->pos == 7) {
		start_pos_lsb = datasize * 8 - param->len; // e.g. COS_OFF
	}
	else {
		start_pos_lsb = 0; // e.g. ABZ_PER
	}

	param_buf_rx_to_uint32 >>= start_pos_lsb;

	uint32_t param_mask = 0;
	for (uint8_t i = 0; i < param->len; i++) {
		param_mask |= 1 << i;
	}

	param_buf_rx_to_uint32 &= param_mask;

	return param_buf_rx_to_uint32;
}

/**
 * @brief This function writes a specific chip parameter.
 *
 * @param param has to be one of the parameters defined in @ref PZ_Parameters_List.
 * @param param_val is the value to be written to the parameter.
 * @retval None
 */
void pz_write_param(const struct pz_param *param, uint32_t param_val) {
	uint8_t datasize = 0;

	if (param->len <= 8) {
		datasize = 1;
	}
	else if (param->len <= 16) {
		datasize = 2;
	}
	else if (param->len <= 24) {
		datasize = 3;
	}
	else {
		datasize = 4;
	}

	if (param->addr < PZ_BSEL.addr) {
		pz_switch_bank(param->bank);
	}

	uint8_t param_buf_tx[4] = { 0, 0, 0, 0 };

	if (param->len % 8 == 0 && param->pos == 7) {
		for (uint8_t i = 0; i < datasize; i++) {
			param_buf_tx[i] = (param_val >> (i * 8)) & 0xFF;
		}
	} else {
		uint8_t param_buf_rx[4] = { 0, 0, 0, 0 };

		pz_read_registers(param->addr - (datasize - 1), param_buf_rx, datasize);

		uint32_t param_buf_rx_to_uint32 = 0;
		for (uint8_t i = 0; i < datasize; i++) {
			param_buf_rx_to_uint32 |= (uint32_t)param_buf_rx[i] << (i * 8);
		}

		uint8_t start_pos_lsb = 0;
		if (datasize == 1) {
			start_pos_lsb = param->pos - (param->len - 1);
		}
		else if (param->pos == 7) {
			start_pos_lsb = datasize * 8 - param->len; // e.g. COS_OFF
		}
		else {
			start_pos_lsb = 0; // e.g. ABZ_PER
		}

		uint32_t param_mask = 0;
		for (uint8_t i = 0; i < param->len; i++) {
			param_mask |= 1 << (start_pos_lsb + i);
		}

		param_val <<= start_pos_lsb;
		param_val &= param_mask;
		param_buf_rx_to_uint32 &= ~param_mask;
		param_buf_rx_to_uint32 |= param_val;

		for (uint8_t i = 0; i < datasize; i++) {
			param_buf_tx[i] = (param_buf_rx_to_uint32 >> (i * 8)) & 0xFF;
		}
	}

	pz_write_registers(param->addr - (datasize - 1), param_buf_tx, datasize);
}
