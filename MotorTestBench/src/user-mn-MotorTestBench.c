/****************************************************************************
	[Project] FlexSEA: Flexible & Scalable Electronics Architecture
	[Sub-project] 'flexsea-user' User projects
	Copyright (C) 2016 Dephy, Inc. <http://dephy.com/>

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
*****************************************************************************
	[Lead developper] Luke Mooney, lmooney at dephy dot com.
	[Origin] Based on Jean-Francois Duval's work at the MIT Media Lab
	Biomechatronics research group <http://biomech.media.mit.edu/>
	[Contributors]
*****************************************************************************
	[This file] user-ex-MotorTestBench: User code running on Manage
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2016-12-06 | jfduval | New release
****************************************************************************/

#include "main.h"

#ifdef BOARD_TYPE_FLEXSEA_MANAGE

//****************************************************************************
// Include(s)
//****************************************************************************

#include "../inc/user-mn-MotorTestBench.h"
#include <math.h>
#include "cmd_motor_dto.h"

//****************************************************************************
// Variable(s)
//****************************************************************************

//int mtb_state = -5;

uint8_t mtb_my_control;
int16_t mtb_my_pwm[2] = {0,0};
int16_t mtb_my_cur[2] = {0,0};

int16_t mtb_glob_var_1;
int16_t mtb_glob_var_2;
int16_t mtb_glob_var_3;

int16_t exec_1_pwm_max = 500;
int16_t exec_1_pwm_step = 25;
int16_t exec_1_pwm_counter = 2;
int16_t max_curr_counter = 0;
int16_t max_curr = 8000;

uint8_t motorTbTestFailed = 0;
float exec1ControllerErrorSum = 0;
float exec2ControllerErrorSum = 0;
float torqueCurrentRatio = 0.12;

enum TEST_STATE { WAITING_PLAN, RUNNING_GAIT, RUNNING_CURRENT, FAILED };
enum TEST_STATE motortb_state = WAITING_PLAN;

//****************************************************************************
// Private Function Prototype(s):
//****************************************************************************

void init_testState(void)
{
	motorTbTestFailed = 0;
	motortb_state = WAITING_PLAN;
	exec1ControllerErrorSum = 0;
	exec2ControllerErrorSum = 0;
	torqueCurrentRatio = 0.12;
	int i;
	for(i=0;i<4;i++)
		user_data_1.r[i]=0;
}

//static void MotorTestBench_refresh_values(void);

//****************************************************************************
// Public Function(s)
//****************************************************************************

//Call this function once in main.c, just before the while()
void init_MotorTestBench(void)
{
	mtb_my_control = CTRL_NONE;
	init_testState();
	//mtb_state = -5;
}


/* FSM - Finite State Machines
 * 	executed at 1 kHz
 * */

// State machine which tracks which stage of testing we are in (ready / testing / finished)
void MotorTestBench_fsm_1(void)
{
#if(ACTIVE_PROJECT == PROJECT_MOTORTB)

	/*	States:
	 * 	0 - ready - test has not started yet, wait for plan to signal start of test
	 * 	1 - testing - test has started
	 * 	2 - finished - test has ended
	 * 	other - error codes? TBD
	 * */

	const uint16_t MS_PER_GAIT = 1177; //3000 ms per gait cycle?
	const uint16_t MS_PER_CYCLE = 3000;
	static uint8_t info[2] = {PORT_485_1, PORT_485_1};
	static uint32_t ticks = 0;

	static uint8_t commCounter = 0;
	static uint8_t currentTestCounter = 0;

	static uint8_t startGaitCycle = 0;
	static uint8_t testEx1Current = 0;
	static uint8_t testEx2Current = 0;

	switch(motortb_state)
	{
	case WAITING_PLAN:
		// Check a flag that is set by plan in an interrupt
		// 'user data' value at index 0
		if(motorTbTestFailed)
		{
			motortb_state = FAILED;
		}
		else if(user_data_1.w[0])
		{
			ticks = 0;
			user_data_1.r[0] = user_data_1.w[0];
			user_data_1.w[0] = 0;

			user_data_1.w[3] = 0;
			user_data_1.r[1] = 1;

			motortb_state = RUNNING_GAIT;
		}
		break;

	case RUNNING_GAIT:
		if(motorTbTestFailed)
		{
			motortb_state = FAILED;
			break;
		}

		if(ticks == 0)
		{
			//if ticks is 0, we send a signal to each execute to start a gait cycle.
			startGaitCycle = GAIT_FLAG;
			exec1TestState = GAIT;
			exec2TestState = GAIT;
		}

		ticks++;

		if(ticks > MS_PER_GAIT && exec1TestState == NONE && exec2TestState == NONE)
		{
			exec1ControllerErrorSum = 0;
			exec2ControllerErrorSum = 0;
			torqueCurrentRatio = 0.12;
			motortb_state = RUNNING_CURRENT;
			ticks = 0;
		}

		if(user_data_1.w[3])
		{
			user_data_1.r[3] = user_data_1.w[3];
			user_data_1.w[3] = 0;
			user_data_1.r[1] = 0;
			user_data_1.w[0] = 0;
			motortb_state = WAITING_PLAN;
			ticks = 0;
		}
		break;

	case RUNNING_CURRENT:

		if(ticks == 500)
		{
			testEx1Current = CURRENT_FLAG;
			testEx2Current = CURRENT_FLAG;

			if(currentTestCounter == 0)
			{
				testEx1Current |= CURRENT_UNDER_TEST_FLAG;
				exec1TestState = CURRENT;
			}
			else
			{
				testEx2Current |= CURRENT_UNDER_TEST_FLAG;
				exec2TestState = CURRENT;
			}

			currentTestCounter++;
			currentTestCounter%=2;
		}

		if(ticks > (MS_PER_CYCLE - MS_PER_GAIT))
			if(exec1TestState == NONE && exec2TestState == NONE)
			{
				motortb_state = RUNNING_GAIT;
				ticks = -1; //ticks is unsigned but the point is for it to roll over to 0
			}

		if(user_data_1.w[3])
		{
			user_data_1.r[3] = user_data_1.w[3];
			user_data_1.w[3] = 0;
			user_data_1.r[1] = 0;
			user_data_1.w[0] = 0;
			motortb_state = WAITING_PLAN;
			ticks = 0;
		}

		ticks++;
		break;

	case FAILED:
		user_data_1.r[2] = 666;

		break;

	default:
		break;

	}

	if(!commCounter)
	{
		motor_dto dto;
		dto.ctrl_i = 0;
		dto.ctrl_o = 0;
		dto.gaitCycleFlag = 0;

		dto.controller = 0xFA;
		if(startGaitCycle || testEx1Current || testEx2Current)
		{
			dto.gaitCycleFlag = startGaitCycle | testEx1Current;
		}

		//Send message to execute 1
		info[0] = PORT_485_1;
		tx_cmd_motortb_r(TX_N_DEFAULT, 0, &dto);
		packAndSend(P_AND_S_DEFAULT, FLEXSEA_EXECUTE_1, info, SEND_TO_SLAVE);
		slaves_485_1.xmit.listen = 1;

		dto.gaitCycleFlag = startGaitCycle | testEx2Current;
		//Send message to execute 2
		info[0] = PORT_485_2;
		tx_cmd_motortb_r(TX_N_DEFAULT, 1, &dto);
		packAndSend(P_AND_S_DEFAULT, FLEXSEA_EXECUTE_2, info, SEND_TO_SLAVE);
		slaves_485_2.xmit.listen = 1;

		startGaitCycle = 0;
		testEx1Current = 0;
		testEx2Current = 0;
	}
	commCounter++;
	commCounter %= 5;

	if(user_data_1.w[2])
	{
		user_data_1.w[2] = 0;
		init_testState();
	}

#endif //PROJECT_MOTORTB
}

uint8_t checkForControlFailure(execControllerState_t ctrlState, float* errorSum, float scale)
{
	const float ERROR_SUM_THRESHOLD = 1000;

	float error = (float)(ctrlState.setpoint - ctrlState.actual) / scale;
	error = error > 0 ? error : -1*error;
	(*errorSum) += error;

	if(*errorSum > ERROR_SUM_THRESHOLD) return 1;

	return 0;
}

uint8_t isBatteryBoardInfoValid()
{
	int i;
	for(i = 0; i < 8; i++)
	{
		if(batt1.rawBytes[i] != 0)
			return 1;

	}
	return 0;
}

void MotorTestBench_fsm_2(void)
{
#if(ACTIVE_PROJECT == PROJECT_MOTORTB)

	const int32_t CURRENT_THRESH_UPPER = 3000;
	const int32_t CURRENT_THRESH_LOWER = -1*CURRENT_THRESH_UPPER;

	if(exec1CtrlStateReady)
	{
		exec1CtrlStateReady = 0;

		uint8_t errorSumFailure = checkForControlFailure(exec1ControllerState, &exec1ControllerErrorSum, 8150);
		float newTorqueCurrentRatio = 0.12;

		if(exec1.current != 0 && exec1.current > 100)
		{
			newTorqueCurrentRatio = (float)(exec1ControllerState.actual) / (float)(exec1.current);
		}
	}

	if(!motorTbTestFailed && exec1CtrlStateReady)
	{
		exec1CtrlStateReady = 0;
		if(exec1TestState == GAIT)
		{
			motorTbTestFailed = checkForControlFailure(exec1ControllerState, &exec1ControllerErrorSum, 8150);
			if(motorTbTestFailed)
			{
				motorTbTestFailed *= 2;
				return;
			}

			if(exec1.current != 0 && exec1.current > 100)
			{
				float newRatio = (float)(exec1ControllerState.actual) / (float)(exec1.current);
				torqueCurrentRatio = (95*torqueCurrentRatio + 5*(newRatio))/100;
				motorTbTestFailed = motorTbTestFailed | (torqueCurrentRatio > 0.18 || torqueCurrentRatio < 0.05);
			}
			if(motorTbTestFailed)
			{
				motorTbTestFailed *= 2;
				return;
			}
		}
		else if(exec1TestState == CURRENT)
		{
			motorTbTestFailed = exec1.current > CURRENT_THRESH_UPPER || exec1.current < CURRENT_THRESH_LOWER;
		}

	}
	if(!motorTbTestFailed && exec2CtrlStateReady)
	{
		exec2CtrlStateReady = 0;
		if(exec2TestState == GAIT)
		{
			motorTbTestFailed = checkForControlFailure(exec2ControllerState, &exec2ControllerErrorSum, 1000);
			if(motorTbTestFailed)
			{
				motorTbTestFailed *= 2;
				return;
			}
		}

		else if(exec2TestState == CURRENT)
		{
			motorTbTestFailed = exec2.current > CURRENT_THRESH_UPPER || exec2.current < CURRENT_THRESH_LOWER;
		}
	}

	//check battery voltage
	if(!motorTbTestFailed)
	{
		motorTbTestFailed = isBatteryBoardInfoValid() && (batt1.voltage > 32000 || batt1.voltage < 28000);
	}

	if(motorTbTestFailed)
	{
		motorTbTestFailed *= 20;
	}

	MotorTestBench_refresh_values();
#endif //PROJECT_MOTORTB
}

//****************************************************************************
// Private Function(s)
//****************************************************************************

//Note: 'static' makes them private; they can only called from functions in this
//file. It's safer than making everything global.

static void MotorTestBench_refresh_values(void)
{
	motortb.mnRunning = user_data_1.r[0];
	motortb.mnTestState = motortb_state;
	//motortb.mn1[1] = (int16_t)(((int32_t)motortb.ex1[1]*(int32_t)motortb.ex1[4])/955);

	motortb.ex1[0] = exec1ControllerState.setpoint;
	motortb.ex1[1] = exec1ControllerState.actual;
	motortb.ex1[2] = exec1.current;
	motortb.ex1[3] = exec1.temp;

	motortb.ex2[0] = exec2ControllerState.setpoint;
	motortb.ex2[1] = exec2ControllerState.actual;
	motortb.ex2[2] = exec2.current;
	motortb.ex2[3] = exec2.temp;
}
//That function can be called from the FSM.

#endif 	//BOARD_TYPE_FLEXSEA_MANAGE
