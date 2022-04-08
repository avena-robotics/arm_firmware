/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FSM_COMMONS_H
#define __FSM_COMMONS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

//typedef enum FSM_State {
#define	FSM_START  																					0 /**< @brief Starting uC.*/
#define	FSM_INIT 																						1
#define	FSM_READY_TO_OPERATE 																2
#define	FSM_OPERATION_ENABLE 																3 /**< @brief Enable power.*/
#define	FSM_TRANSITION_START_TO_INIT 												10
#define	FSM_TRANSITION_INIT_TO_READY_TO_OPERATE 						11
#define	FSM_TRANSITION_READY_TO_OPERATE_TO_OPERATION_ENABLE 12
#define	FSM_TRANSITION_OPERATION_ENABLE_TO_READY_TO_OPERATE 13
#define	FSM_TRANSITION_OPERATION_ENABLE_TO_INIT  						14
#define	FSM_TRANSITION_FAULT_REACTION_ACTIVE_TO_FAULT 			15
#define	FSM_TRANSITION_FAULT_TO_INIT 												16

#define	FSM_FAULT_REACTION_ACTIVE 													254
#define	FSM_FAULT  																					255
//} FSM_State_t;

typedef struct FSMStatus {
//	FSM_State_t state;   
	uint8_t state;   
	bool state_is_running;
	bool transition_is_running;
} __attribute__ ((packed)) FSMStatus_t;

// FUNCTION DEFINITION
void FSM_Tick(void);
//bool FSM_Set_State_Callback(FSM_State_t new_state);
uint8_t FSM_Get_State(void);
bool FSM_Set_State(uint8_t new_state);
bool FSM_Activate_State(uint8_t new_state);
bool FSM_Activate_Transition(uint8_t new_transition);
bool FSM_Set_State_Callback(uint8_t new_state);
void FSM_Tick_Callback(void);

// EXTERN VARIABLES

#ifdef __cplusplus
}
#endif

#endif /* __FSM_COMMONS_H */
