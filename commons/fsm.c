#include "fsm.h"

__attribute__((weak)) volatile FSMStatus_t 	g_fsm_status = {
	.state = FSM_START,
	.state_is_running = false,
	.transition_is_running = false,
};

// LOCAL FUNCTIONS DEFINITIONS
//FSM_State_t FSM_Get_State(void);
uint8_t FSM_Get_State(void);
//bool FSM_Set_State(FSM_State_t new_state);
bool FSM_Set_State(uint8_t new_state);
//bool FSM_Activate_State(FSM_State_t new_state);
bool FSM_Activate_State(uint8_t new_state);
//bool FSM_Activate_Transition(FSM_State_t new_transition);
bool FSM_Activate_Transition(uint8_t new_transition);

// FUNCTIONS BODIES
void FSM_Action() 
{
}

//bool FSM_Activate_State(FSM_State_t new_state) 
bool FSM_Activate_State(uint8_t new_state) 
{
	g_fsm_status.state = new_state;
	return true;
}


//bool FSM_Activate_Transition(FSM_State_t new_transition) 
bool FSM_Activate_Transition(uint8_t new_transition) 
{
	g_fsm_status.state = new_transition;
	return true;
}


//FSM_State_t FSM_Get_State(void) 
uint8_t FSM_Get_State(void) 
{
	
	return g_fsm_status.state;
}

//__attribute__((weak)) bool FSM_Set_State_Callback(FSM_State_t new_state) {
__attribute__((weak)) bool FSM_Set_State_Callback(uint8_t new_state) 
{
	return false;
}

__attribute__((weak)) void FSM_START_Callback() 
{
}

__attribute__((weak)) void FSM_INIT_Callback() 
{
}

__attribute__((weak)) void FSM_READY_TO_OPERATE_Callback() 
{
}

__attribute__((weak)) void FSM_OPERATION_ENABLE_Callback() 
{
}

__attribute__((weak)) void FSM_FAULT_REACTION_ACTIVE_Callback() 
{
}

__attribute__((weak)) void FSM_FAULT_Callback() 
{
}

__attribute__((weak)) void FSM_TRANSITION_START_TO_INIT_Callback() 
{
}

__attribute__((weak)) void FSM_TRANSITION_INIT_TO_READY_TO_OPERATE_Callback() 
{
}
__attribute__((weak)) void FSM_TRANSITION_READY_TO_OPERATE_TO_OPERATION_ENABLE_Callback() 
{
}
__attribute__((weak)) void FSM_TRANSITION_OPERATION_ENABLE_TO_READY_TO_OPERATE_Callback() 
{
}
__attribute__((weak)) void FSM_TRANSITION_OPERATION_ENABLE_TO_INIT_Callback() 
{
}
__attribute__((weak)) void FSM_TRANSITION_FAULT_REACTION_ACTIVE_TO_FAULT_Callback() 
{
}
__attribute__((weak)) void FSM_TRANSITION_FAULT_TO_INIT_Callback() 
{
}
__attribute__((weak)) void FSM_Tick_Callback() 
{
	// FSM
	switch (FSM_Get_State()) {
		default:
			FSM_Activate_State(FSM_FAULT_REACTION_ACTIVE);
			break;
	}
}

//bool FSM_Set_State(FSM_State_t new_state) 
bool FSM_Set_State(uint8_t new_state) 
{
	return FSM_Set_State_Callback(new_state);
}

void FSM_Tick() 
{
	switch (FSM_Get_State()) {

		case FSM_START:
			FSM_START_Callback();
			FSM_Activate_Transition(FSM_TRANSITION_START_TO_INIT); // auto
			break;

		case FSM_INIT:
			FSM_INIT_Callback();
			break;

		case FSM_READY_TO_OPERATE:
			FSM_READY_TO_OPERATE_Callback();
			break;

		case FSM_OPERATION_ENABLE:
			FSM_OPERATION_ENABLE_Callback();
			break;

		case FSM_FAULT_REACTION_ACTIVE:
			FSM_FAULT_REACTION_ACTIVE_Callback();
			FSM_Activate_Transition(FSM_TRANSITION_FAULT_REACTION_ACTIVE_TO_FAULT); // auto
			break;

		case FSM_FAULT:
			FSM_FAULT_Callback();
			break;

		case FSM_TRANSITION_START_TO_INIT:
			FSM_TRANSITION_START_TO_INIT_Callback();
			FSM_Activate_State(FSM_INIT);
			break;

		case FSM_TRANSITION_INIT_TO_READY_TO_OPERATE:
			FSM_TRANSITION_INIT_TO_READY_TO_OPERATE_Callback();
			FSM_Activate_State(FSM_READY_TO_OPERATE);
			break;

		case FSM_TRANSITION_READY_TO_OPERATE_TO_OPERATION_ENABLE:
			FSM_TRANSITION_READY_TO_OPERATE_TO_OPERATION_ENABLE_Callback();
			FSM_Activate_State(FSM_OPERATION_ENABLE);
			break;

		case FSM_TRANSITION_OPERATION_ENABLE_TO_READY_TO_OPERATE:
			FSM_TRANSITION_OPERATION_ENABLE_TO_READY_TO_OPERATE_Callback();
			FSM_Activate_State(FSM_READY_TO_OPERATE);
			//motor_stop();
			break;

		case FSM_TRANSITION_OPERATION_ENABLE_TO_INIT:
			FSM_TRANSITION_OPERATION_ENABLE_TO_INIT_Callback();
			FSM_Activate_State(FSM_INIT);
			break;

		case FSM_TRANSITION_FAULT_REACTION_ACTIVE_TO_FAULT:
			FSM_TRANSITION_FAULT_REACTION_ACTIVE_TO_FAULT_Callback();
			FSM_Activate_State(FSM_FAULT);
			break;
		
		case FSM_TRANSITION_FAULT_TO_INIT:
			FSM_TRANSITION_FAULT_TO_INIT_Callback();
			FSM_Activate_State(FSM_INIT);
			break;

		default:
			FSM_Tick_Callback();
			break;
	}
}

