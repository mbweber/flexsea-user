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

#ifdef BOARD_TYPE_FLEXSEA_MANAGE

//****************************************************************************
// Include(s)
//****************************************************************************

#include "../inc/user-mn-MotorTestBench.h"
#include <math.h>

//****************************************************************************
// Variable(s)
//****************************************************************************

int mtb_state = -5;

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

//****************************************************************************
// Private Function Prototype(s):
//****************************************************************************

static void MotorTestBench_refresh_values(void);

//****************************************************************************
// Public Function(s)
//****************************************************************************

//Call this function once in main.c, just before the while()
void init_MotorTestBench(void)
{
	mtb_my_control = CTRL_NONE;
	mtb_state = -5;
}

//Ankle 2-DoF Finite State Machine.
//Call this function in one of the main while time slots.
void MotorTestBench_fsm_1(void)
{
	#if(ACTIVE_PROJECT == PROJECT_MOTORTB)

    static uint32_t time = 0, state_t = 0;

    //Increment time (1 tick = 1ms)
    time++;
    state_t++;

	//Before going to a state we refresh values:
    MotorTestBench_refresh_values();

    if (user_data_1.w[2] == 1)
    {
    	if (mtb_state != 20)
    	{
    		state_t = 0;
    	}
    	mtb_state = 20;
    }
    else if (user_data_1.w[2] == 2)
    {
    	if (mtb_state != 30)
    	{
    		state_t = 0;
    	}
    	mtb_state = 30;
    }
	//Nothing programmed yet...
	switch(mtb_state)
	{

		case -5://Wait for 10 seconds to let everything load

			mtb_my_control = CTRL_OPEN;
			mtb_my_pwm[0] = 0;
			mtb_my_pwm[1] = 0;
			if (state_t>10000 && user_data_1.w[0] != 0)
			{
				mtb_state = 0;
				state_t = 0;
			}

			break;

		case 0:
			mtb_my_control = CTRL_OPEN;
			mtb_my_pwm[0] = state_t/10;
			mtb_my_pwm[1] = state_t/10;

			if (mtb_my_pwm[0]>= exec_1_pwm_counter*exec_1_pwm_step)
			{
				state_t = 0;
				mtb_state = 1;
			}
			break;
		case 1:	//PWM = 100 for 5s

			mtb_my_control = CTRL_OPEN;
			mtb_my_pwm[0] = exec_1_pwm_counter*exec_1_pwm_step;
			mtb_my_pwm[1] = exec_1_pwm_counter*exec_1_pwm_step-state_t/10;

			if (mtb_my_pwm[1]<-mtb_my_pwm[0] || max_curr_counter>10)
			{
				state_t = 0;
				mtb_state = 2;
			}

			if (user_data_1.w[0] == 0)
			{
				state_t = 0;
				mtb_state = 10;
			}

            break;
		case 2:
			mtb_my_control = CTRL_OPEN;
			if (mtb_my_pwm[0]>0)
			{
				mtb_my_pwm[0]--;
			}
			else if (mtb_my_pwm[0]<0)
			{
				mtb_my_pwm[0]++;
			}

			if (mtb_my_pwm[1]>0)
			{
				mtb_my_pwm[1]--;
			}
			else if (mtb_my_pwm[1]<0)
			{
				mtb_my_pwm[1]++;
			}

			if (mtb_my_pwm[0] == 0 && mtb_my_pwm[1] == 0)
			{
				mtb_state = 3;
				state_t = 0;
			}

			break;

		case 3: //measure motor resistance
			mtb_my_control = 6;
			mtb_my_pwm[0] = 0;
			mtb_my_pwm[1] = 0;

			if (state_t>1000)
			{

				state_t = 0;
				if ((exec_1_pwm_counter+1)*exec_1_pwm_step<=exec_1_pwm_max)
				{
					exec_1_pwm_counter++;
				}
				else
				{
					exec_1_pwm_counter = 2;
				}
				if ((exec1.current>-5000 && exec1.current<0)||(exec1.current<5000 && exec1.current>0))
				{
					mtb_state = 11;
				}
				else
				{
					mtb_state = 0;
				}
			}

			if (user_data_1.w[0] == 0)
			{
				state_t = 0;
				mtb_state = 10;
			}
			break;
		case 10:
			mtb_my_control = CTRL_OPEN;
			mtb_my_pwm[0] = 0;
			mtb_my_pwm[1] = 0;
			if (user_data_1.w[0] != 0)
			{
				state_t = 0;
				mtb_state = 0;
				exec_1_pwm_counter = 2;
			}
			break;
		case 11:
			mtb_my_control = CTRL_OPEN;
			mtb_my_pwm[0] = 0;
			mtb_my_pwm[1] = 0;
			if (user_data_1.w[0] == 2)
			{
				state_t = 0;
				mtb_state = 0;
				exec_1_pwm_counter = 2;
			}
			break;
		case 20:
			mtb_my_control = 6;
			mtb_my_pwm[0] = user_data_1.w[3];
			mtb_my_pwm[1] = user_data_1.w[3]/2;
			//applies .481 volts
			//motor current becomes twice the current flowing through the motor

			if (user_data_1.w[2] != 1)
			{

				state_t = 0;
				exec_1_pwm_counter = 2;
				mtb_state = 10;
			}
			break;
		case 30:
			mtb_my_control = CTRL_OPEN;
			mtb_my_pwm[0] = user_data_1.w[3];
			mtb_my_pwm[1] = 0;
			break;
        default:
			//Handle exceptions here
			break;

	}

	#endif	//ACTIVE_PROJECT == PROJECT_MOTORTB
}

//Second state machine for the Ankle project
//Deals with the communication between Manage and 2x Execute, on the same RS-485 bus
//This function is called at 1kHz
void MotorTestBench_fsm_2(void)
{
	#if(ACTIVE_PROJECT == PROJECT_MOTORTB)

	static uint8_t ex_refresh_fsm_state = 0;
	static uint32_t timer = 0;
	uint8_t info[2] = {PORT_485_1, PORT_485_1};

	//This FSM talks to the slaves at 250Hz each
	switch(ex_refresh_fsm_state)
	{
		case 0:		//Power-up

			if(timer < 7000)
			{
				//We wait 7s before sending the first commands
				timer++;
			}
			else
			{
				//Ready to start transmitting
				ex_refresh_fsm_state = 1;
			}

			break;

		case 1:	//Communicating with Execute #1

			info[0] = PORT_485_1;
			tx_cmd_motortb_r(TX_N_DEFAULT, 0, mtb_my_control, mtb_my_cur[0], mtb_my_pwm[0]);
			packAndSend(P_AND_S_DEFAULT, FLEXSEA_EXECUTE_1, info, SEND_TO_SLAVE);

			//slaves_485_1.xmit.listen = 1;	//Legacy - remove once tested
			slaves_485_1.xmit.willListenSoon = 1;	//New version
			ex_refresh_fsm_state++;

			break;

		case 2:

			//Skipping one cycle
			ex_refresh_fsm_state++;

			break;

		case 3:	//Communicating with Execute #2

			info[0] = PORT_485_2;
			tx_cmd_motortb_r(TX_N_DEFAULT, 1, mtb_my_control, mtb_my_cur[1], mtb_my_pwm[1]);
			packAndSend(P_AND_S_DEFAULT, FLEXSEA_EXECUTE_2, info, SEND_TO_SLAVE);

			//slaves_485_2.xmit.listen = 1;	//Legacy - remove once tested
			slaves_485_2.xmit.willListenSoon = 1;	//New version
			ex_refresh_fsm_state++;

			break;

		case 4:

			//Skipping one cycle
			ex_refresh_fsm_state = 1;

			break;
	}

	#endif	//ACTIVE_PROJECT == PROJECT_MOTORTESTBENCH
}

//****************************************************************************
// Private Function(s)
//****************************************************************************

//Note: 'static' makes them private; they can only called from functions in this
//file. It's safer than making everything global.

//
static void MotorTestBench_refresh_values(void)
{
	motortb.mn1[0] = mtb_state;
	motortb.mn1[1] = (int16_t)(((int32_t)motortb.ex1[1]*(int32_t)motortb.ex1[4])/955);

	if ((exec1.current>max_curr || exec1.current<-max_curr) && mtb_state != 3)
	{
		max_curr_counter++;
	}
	else
	{
		max_curr_counter = 0;
	}
}
//That function can be called from the FSM.


#endif 	//BOARD_TYPE_FLEXSEA_MANAGE
