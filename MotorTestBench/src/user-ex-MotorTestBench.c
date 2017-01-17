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

//****************************************************************************
// Variable(s)
//****************************************************************************
 
struct motortb_s my_motortb;

#if(ACTIVE_SUBPROJECT == SUBPROJECT_A)

// define a 100 long array, defining the torque profile for the motor controlled by ex2
// static float torqueProfile[] = { .. };
#include "../resources/motorTorqueProfile.c.resource"
pid_controller torqueController;	
	
#endif

//#if(ACTIVE_SUBPROJECT == SUBPROJECT_B)

pid_controller velocityController;

//#endif
    
//****************************************************************************
// Private Function Prototype(s):
//****************************************************************************  
 
//****************************************************************************
// Public Function(s)
//****************************************************************************
 
//Call this function once in main.c, just before the while()
void initMotorTestBench(void)
{   
	ctrl.active_ctrl = CTRL_NONE;
	board_id = SLAVE_ID;
	
	#if(MOTOR_COMMUT == COMMUT_BLOCK)
        Coast_Brake_Write(1);               //Brake (regen)
	#endif
    
    #if(ACTIVE_SUBPROJECT == SUBPROJECT_A)
        // initialization code specific to ex 1
        pid_controller_initialize(&torqueController, 1024, 5000, 90, 8000);
    #endif
    
    //#if(ACTIVE_SUBPROJECT == SUBPROJECT_B)
        // initialization code specific to ex 2
        pid_controller_initialize(&velocityController, 1024, 100000, 90, 40000);
        pid_controller_settings(&velocityController, 1, 1, 1);
		pid_controller_setGains(&velocityController, 2, 15, 0, 9);
    //#endif
    
    int i;
    for(i = 0; i < 6; i++)
    {
        motortb.ex1[i] = 0;
    }
}

int32_t lastEncoderReading = 0;

// User finite state machine, implements a tight controller 
// Called at 1 kHz
// Call this function in one of the main while time slots.
void MotorTestBench_fsm(void)
{
    static unsigned int ticks = 0;
    static uint8_t state = 4;
    /* State represents whether we are running or not 
     * 0 - not running
     * 1 - gait cycle just started / starting
     * 2 - gait cycle in progress
    */
    //#if(ACTIVE_SUBPROJECT == SUBPROJECT_B)
    	static int32_t initialPosition = 0;
    //#endif

	//ctrl.active_ctrl
	
	int32_t maxspeed = 4000;
	int32_t minspeed = -1 * maxspeed;
	
	if(ctrl.active_ctrl != CTRL_CUSTOM)
	{
		ticks = 0;
		lastEncoderReading = exec1.enc_control_ang;
		//Just read this is so that if we switch bakc to custom we don't f ourselves up
		//initialPosition = exec1.enc_control_ang;
	}
	else
	{
		int32_t setpoint = 0;
		int32_t x = ticks / 500;
		int32_t r = ticks % 500; 
		
	    if(x == 0)
	      setpoint = r * maxspeed / 500;      
	    else if(x == 1)
	      setpoint = (500 - r) * maxspeed / 500 ;
	    else if(x == 2)
	      setpoint = r * minspeed / 500;
	    else if(x == 3)
	      setpoint = (500 - r) * minspeed / 500;
		
		velocityController.setpoint = setpoint * sin(4 * PI * ticks / 1000);
		
		ticks++;
		ticks%=2000;
	}
}

// User fsm controlling motors
void MotorTestBench_fsm2(void)
{
	
	if(ctrl.active_ctrl == CTRL_CUSTOM) 
	{
	    int32_t pwm = 0;
		//static int32_t lastRPM = 0;
	    #if(ACTIVE_SUBPROJECT == SUBPROJECT_A)
	        torqueController.controlValue = (((int32_t)(strain_read())-31937)*1831)>>13;
			positionController.controlValue = exec1.enc_control_ang;
			/*
			int32_t scaledAccel = (as5047.raw.vel_rpm - lastRPM) * 20;
			int32_t goal_voltage_mV = (2*torqueController.setpoint- 11*as5047.raw.vel_rpm - scaledAccel); //a generous approximation
			//int32_t goal_voltage_mV = (-11*as5047.filt.vel_rpm); //a generous approximation
			int32_t batVolt_mV = (16*safety_cop.v_vb/3+302)*33;
			int32_t ff = goal_voltage_mV * 1024 / batVolt_mV;
			*/
			pwm = pid_controller_compute(&positionController) + pid_controller_compute(&torqueController);
			//pwm -= ff;
			
	    #endif
		
	    #if(ACTIVE_SUBPROJECT == SUBPROJECT_B)
	       // velocityController.controlValue = (exec1.enc_control_ang - lastEncoderReading);
	        velocityController.controlValue = exec1.enc_control_vel;
			pwm = pid_controller_compute(&velocityController);
	    #endif
	    //lastRPM = as5047.raw.vel_rpm;
	    motor_open_speed_1(pwm);
		
		lastEncoderReading = exec1.enc_control_ang;
	}
}

//Here's an example function:
void MotorTestBench_refresh_values(void)
{
	motortb.ex1[1] = as5047.filt.vel_rpm;
    motortb.ex1[2] = as5047.raw.ang_clks;
    motortb.ex1[3] = exec1.sine_commut_pwm;
    motortb.ex1[4] = ((strain_read()-31937)*1831)>>13;
}

//****************************************************************************
// Private Function(s)
//****************************************************************************

#endif //BOARD_TYPE_FLEXSEA_EXECUTE
