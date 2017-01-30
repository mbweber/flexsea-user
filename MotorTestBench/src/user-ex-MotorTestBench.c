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

#include "main.h"

#ifdef BOARD_TYPE_FLEXSEA_EXECUTE

//****************************************************************************
// Include(s)
//****************************************************************************
 
#include "../inc/user-ex-MotorTestBench.h"
#include "flexsea_pid_controller.h"
#include "../inc/motor_torque_ff_model.h"

//****************************************************************************
// Variable(s)
//****************************************************************************
 
struct motortb_s my_motortb;
#define CYCLE_LENGTH 1000	
#define START_TEST_TICK_DIFF 500
	
#if(ACTIVE_SUBPROJECT == SUBPROJECT_A)
	
#define P_CONTROLGAINS 75,2,0,10
// define a 1000 long array, defining the position profile for the motor controlled by ex1
// static int32_t positionProfile[] = { .. }; 
#include "../resources/winchPositionProfile.c.resource"
pid_controller positionController;
#endif

//****************************************************************************
// Private Function Prototype(s):
//****************************************************************************  

//****************************************************************************
// Public Function(s)
//****************************************************************************
 
//Call this function once in main.c, just before the while()
void initMotorTestBench(void)
{   
	ctrl.active_ctrl = CTRL_CUSTOM;
	board_id = SLAVE_ID;
	
	#if(MOTOR_COMMUT == COMMUT_BLOCK)
        Coast_Brake_Write(1);               //Brake (regen)
	#endif
    
    #if(ACTIVE_SUBPROJECT == SUBPROJECT_A)
        // initialization code specific to ex 1
        pid_controller_initialize(&positionController, 1024, 50000, 90, 16000);
        pid_controller_settings(&positionController, 0, 1, 1);
		
    #endif
}

enum TEST_FSM_STATE { INITIALIZING, WAITING, START_WIND_UP, WIND_UP, CYCLING, DONE };
static enum TEST_FSM_STATE state = INITIALIZING;

// User finite state machine, implements a tight controller
// Called at 1 kHz
// Call this function in one of the main while time slots.
void MotorTestBench_fsm(void)
{
    static unsigned int ticks = 0;
    /* State represents whether we are running or not 
     * 0 - not running
     * 1 - gait cycle just started / starting
     * 2 - gait cycle in progress
    */
    //#if(ACTIVE_SUBPROJECT == SUBPROJECT_B)
    	static int32_t initialPosition = 0;
		static int32_t positionAtTension = 0;
		static int32_t ang_vel_lp = 0;
		int32_t ticksFromInitial = 0;
    //#endif

	if(ctrl.active_ctrl != CTRL_CUSTOM)
	{
		//Just read this is so that if we switch bakc to custom we don't f ourselves up
		initialPosition = exec1.enc_control_ang;
		state = 4;
	}
	
	switch(state) 
    {

    case INITIALIZING:
        ticks++;
		//Wait 1 second for things to initialize
        if(ticks > 1000)
        {
            ticks = 0;
            state = WAITING;
			
            #if(ACTIVE_SUBPROJECT == SUBPROJECT_A)
                initialPosition = exec1.enc_control_ang;
                pid_controller_setGains(&positionController, 0, 0, 0, 7);
            #endif
        }
        break;

    case WAITING:
        //check the flag sent by manage
		ticksFromInitial = initialPosition - exec1.enc_display;
		if(ticksFromInitial > START_TEST_TICK_DIFF || ticksFromInitial < -1*START_TEST_TICK_DIFF)
		{
			ticks = 0;
			state = START_WIND_UP;
			ang_vel_lp = as5047.filt.vel_cpms;
		}

        break;		
		
    case START_WIND_UP:
        ticks++;
		if(ticks > 50)
		{
			state = WIND_UP;
		}
		motor_open_speed_1(80*PWM_SIGN);
		ang_vel_lp = (95*ang_vel_lp + 5*as5047.filt.vel_cpms)/100;
        
		break;

    case WIND_UP:
		ang_vel_lp = (95*ang_vel_lp + 5*as5047.filt.vel_cpms)/100;
		
		if(ang_vel_lp < 5 && ang_vel_lp > -5)
		{
			positionAtTension = exec1.enc_display;
			state = CYCLING;
			pid_controller_setGains(&positionController, P_CONTROLGAINS);
			ticks = 0;
		}
        break;

	case CYCLING:
		positionController.setpoint = positionAtTension + positionProfile[ticks]/2;
		ticks++;
		ticks %= CYCLE_LENGTH;
		if(0) //check test failure conditions
		{
			state = DONE;
		}
		
		break;
		
	default:
		break;
    }

}

void user_ctrl(void)
{	
	if(ctrl.active_ctrl == CTRL_CUSTOM && state == CYCLING) 
	{		
	    int32_t pwm = 0;
        positionController.controlValue = exec1.enc_control_ang;
        pwm = pid_controller_compute(&positionController);
	    motor_open_speed_1(pwm);	
	}
}

// User fsm controlling motors
void MotorTestBench_fsm2(void)
{

}

//Here's an example function:
void MotorTestBench_refresh_values(void)
{/*
	motortb.ex1[1] = as5047.filt.vel_rpm;
    motortb.ex1[2] = as5047.raw.ang_clks;
    motortb.ex1[3] = exec1.sine_commut_pwm;
    motortb.ex1[4] = ((strain_read()-31937)*1831)>>13;
*/}

//****************************************************************************
// Private Function(s)
//****************************************************************************

#endif //BOARD_TYPE_FLEXSEA_EXECUTE
