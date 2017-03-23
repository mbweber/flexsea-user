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
	[This file] user-ex-MotorTestBench: User code running on Execute
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2016-12-06 | jfduval | New release
	*
****************************************************************************/

//#include "main.h"
#include <flexsea_board.h>

#ifdef BOARD_TYPE_FLEXSEA_EXECUTE

//****************************************************************************
// Include(s)
//****************************************************************************

#include "../inc/user-ex-MotorTestBench.h"
#include <flexsea_user_structs.h>

//****************************************************************************
// Variable(s)
//****************************************************************************

struct motortb_s my_motortb;

//****************************************************************************
// Private Function Prototype(s):
//****************************************************************************

//****************************************************************************
// Public Function(s)
//****************************************************************************

//Call this function once in main.c, just before the while()
void initMotorTestBench(void)
{
	setBoardID(SLAVE_ID);

    //Controller setup:
    ctrl.active_ctrl = CTRL_OPEN;   //Position controller
    motor_open_speed_1(0);              //0% PWM
	#if(MOTOR_COMMUT == COMMUT_BLOCK)
    Coast_Brake_Write(1);               //Brake (regen)
	#endif

    //Position PID gains - initially 0
    ctrl.position.gain.P_KP = 0;
    ctrl.position.gain.P_KI = 0;

	//User variables:
	motortb.ex1[0] = 0;
	motortb.ex1[1] = 0;
	motortb.ex1[2] = 0;
	motortb.ex1[3] = 0;
	motortb.ex1[4] = 0;
	motortb.ex1[5] = 0;
}

//Knee Finite State Machine.
//Call this function in one of the main while time slots.
void MotorTestBench_fsm(void)
{
    static int state = 0;

    switch (state)
    {
        case 0:
			//...
            break;
	}

	//Code does nothing, everything is happening on Manage
}

//Here's an example function:
void MotorTestBench_refresh_values(void)
{
	motortb.ex1[1] = *exec1.enc_ang_vel;
    motortb.ex1[2] = *exec1.enc_ang;
    motortb.ex1[3] = MOTOR_ORIENTATION * exec1.sine_commut_pwm;
    motortb.ex1[4] = ((strain_read()-31937)*1831)>>13;
}

//****************************************************************************
// Private Function(s)
//****************************************************************************

#endif //BOARD_TYPE_FLEXSEA_EXECUTE
