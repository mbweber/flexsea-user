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
#define LENGTH_GAIT_CYCLE_IN_MS 1177	

#if(ACTIVE_SUBPROJECT == SUBPROJECT_A)
	
#define T_CONTROLGAINS 0,5,0,5
//#define T_CONTROLGAINS 0,0,0,7
	// define a 1000 long array, defining the torque profile for the motor controlled by ex2
// static float torqueProfile[] = { .. };
#include "../resources/motorTorqueProfile.c.resource"
pid_controller torqueController;
#endif

#if(ACTIVE_SUBPROJECT == SUBPROJECT_B)
//#define P_CONTROLGAINS 37,1,0,9
#define P_CONTROLGAINS 77,3,0,10
// define a 1000 long array, defining the position profile for the motor controlled by ex1
// static int32_t positionProfile[] = { .. }; 
#include "../resources/motorPositionProfile.c.resource"
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
        pid_controller_initialize(&torqueController, 1024, 5000, 90, 8000);
		pid_controller_settings(&torqueController, 0, 1, 1);
		
//		int j;
//		for(j=0; j<LENGTH_GAIT_CYCLE_IN_MS; j++)
//			torqueProfile[j] = torqueProfile[j] / 8;
    #endif
    
    #if(ACTIVE_SUBPROJECT == SUBPROJECT_B)
        // initialization code specific to ex 2
        pid_controller_initialize(&positionController, 1024, 50000, 90, 16000);
        pid_controller_settings(&positionController, 0, 1, 1);
		
//		int j;
//		for(j=0; j<LENGTH_GAIT_CYCLE_IN_MS; j++)
//			positionProfile[j] = positionProfile[j] / 8;
    #endif
    
    int i;
    for(i = 0; i < 6; i++)
    {
        motortb.ex1[i] = 0;
    }
}

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

	if(ctrl.active_ctrl != CTRL_CUSTOM)
	{
		//Just read this is so that if we switch bakc to custom we don't f ourselves up
		initialPosition = exec1.enc_control_ang;
		state = 4;
	}
	else
	{
		switch(state) 
	    {

	    case 4:
	        ticks++;
			//Wait 1 second for things to initialize
	        if(ticks > 1000)
	        {
	            ticks = 0;
	            state = 0;
				
	            #if(ACTIVE_SUBPROJECT == SUBPROJECT_A)
					pid_controller_setGains(&torqueController, T_CONTROLGAINS);
	                torqueController.setpoint = torqueProfile[0];		
					torqueController.errorSum = 0;
	            #endif

	            #if(ACTIVE_SUBPROJECT == SUBPROJECT_B)
	                initialPosition = exec1.enc_control_ang;
	                positionController.setpoint = initialPosition + positionProfile[0];
	                pid_controller_setGains(&positionController, P_CONTROLGAINS);
	            #endif
	        }
	        break;
	        
	    case 1:
	        // set PID gains to non zero, ie we are actually traversing through the position profile now
	        #if(ACTIVE_SUBPROJECT == SUBPROJECT_A)
	            torqueController.setpoint = torqueProfile[0];
	            pid_controller_setGains(&torqueController, T_CONTROLGAINS);
	        #endif
	        #if(ACTIVE_SUBPROJECT == SUBPROJECT_B)
	            positionController.setpoint = initialPosition + positionProfile[0];
	            pid_controller_setGains(&positionController, P_CONTROLGAINS);
	        #endif
	        ticks = 1;
	        state = 2;
	        break;
	        
	    case 2:
			if(ticks < LENGTH_GAIT_CYCLE_IN_MS)
			{
				#if(ACTIVE_SUBPROJECT == SUBPROJECT_A)
		            torqueController.setpoint = torqueProfile[ticks];
				#endif
		        #if(ACTIVE_SUBPROJECT == SUBPROJECT_B)
		            positionController.setpoint = (initialPosition + positionProfile[ticks]);
				#endif
				ticks++;
			}
			else
			{
			    state = 0;
	            ticks = 0;
			}

	        break;

	    case 0:            
	        //check the flag sent by manage
	        if(motortb_startCycleFlag)
	        {
	            motortb_startCycleFlag = 0;
	            state = 1;

	        }
			#if(ACTIVE_SUBPROJECT == SUBPROJECT_A)
				//torqueController.errorSum = 0;
				torqueController.setpoint = 0;
				torqueController.errorSum = torqueController.errorSum*9/10;
			#endif			

	        break;
	    }	
	}
}

void user_ctrl(void)
{
	static int32_t enc_vel_prev = 0;
	int32_t encVel = 0;
	if(ctrl.active_ctrl == CTRL_CUSTOM) 
	{		
	    int32_t pwm = 0;
		#if(ACTIVE_SUBPROJECT == SUBPROJECT_A)
			encVel = as5047.filt.vel_ctrl_cpms;
			int32_t encAccel = encVel - enc_vel_prev;
			
			volatile int32_t goal_voltage = torqueToVoltage(torqueController.setpoint, encVel, encAccel);
			volatile int32_t bat_volt = ((16*safety_cop.v_vb/3+302)*33)>>7; //battery voltage in mV
			//pwm = duty cycle = goalvoltage / battery voltage
			//however goal voltage's units are in 1000mV / 1024
			//and pwm is scaled so 1024 units = 1 = 100% duty cycle, so
			//(goal_voltage * 1000/1024 / bat_volt) * 1024 / 1 = pwm 
			volatile int32_t ff = goal_voltage*1000/bat_volt;
			ff = goal_voltage*1000/24000;
			
	        torqueController.controlValue = (((int32_t)(strain_read())-31937)*1831)>>13;
			pwm = pid_controller_compute(&torqueController) - ff;
			
	    #endif

	    #if(ACTIVE_SUBPROJECT == SUBPROJECT_B)
	        positionController.controlValue = exec1.enc_control_ang;
	        pwm = pid_controller_compute(&positionController);
	    #endif

	    motor_open_speed_1(pwm);	
	}
	
	enc_vel_prev = encVel;
}

// User fsm controlling motors
void MotorTestBench_fsm2(void)
{
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
