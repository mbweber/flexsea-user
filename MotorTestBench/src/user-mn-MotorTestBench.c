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

	//Nothing programmed yet...
	switch(mtb_state)
	{

		case -5://Wait for 10 seconds to let everything load

			mtb_my_control = CTRL_OPEN;
			mtb_my_pwm[0] = 0;
			mtb_my_pwm[1] = 0;
			if (state_t>10000)
			{
				mtb_state = 1;
			}

			break;

		case 1:	//PWM = 100 for 5s

			if(user_data_1.w[0] <= 200)
			{
				mtb_my_pwm[0] = user_data_1.w[0];
			}
			else
			{
				mtb_my_pwm[0] = 0;
			}

			if (time >= 1000)
			{
				time = 0;
				mtb_state = 2;
			}

            break;

        case 2:	//PWM = 0 for 5s

        	mtb_my_pwm[0] = 0;

			if (time >= 1000)
			{
				time = 0;
				mtb_state = 1;
			}

            break;

        default:
			//Handle exceptions here
			break;
	}

	#endif	//ACTIVE_PROJECT == PROJECT_ANKLE_2DOF
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

			slaves_485_1.xmit.listen = 1;
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

			slaves_485_2.xmit.listen = 1;
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
	//...
}
//That function can be called from the FSM.


#endif 	//BOARD_TYPE_FLEXSEA_MANAGE
