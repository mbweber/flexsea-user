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
 
//****************************************************************************
// Variable(s)
//****************************************************************************
 
struct motortb_s my_motortb;
volatile uint8_t motortb_startCycleFlag = 0;
    
// define a 100 long array, defining the position profile for the motor controlled by ex1
// static float positionProfile[] = { .. }; 
#include "../resources/motorPositionProfile.c.resource"
    
// define a 100 long array, defining the torque profile for the motor controlled by ex2 
// static float torqueProfile[] = { .. };  
#include "../resources/motorTorqueProfile.c.resource"
    
//****************************************************************************
// Private Function Prototype(s):
//****************************************************************************  
 
//****************************************************************************
// Public Function(s)
//****************************************************************************
 
//Call this function once in main.c, just before the while()
void initMotorTestBench(void)
{   
	board_id = SLAVE_ID;
	
	#if(MOTOR_COMMUT == COMMUT_BLOCK)
    Coast_Brake_Write(1);               //Brake (regen)
	#endif
    
    #if(ACTIVE_SUBPROJECT == SUBPROJECT_A)
        // initialization code specific to ex 2
        ctrl.active_ctrl = CTRL_POSITION;   //Position controller
        motor_open_speed_1(0);              //0% PWM
        
        //Position PID gains - initially 0
        ctrl.position.gain.P_KP = 0;
        ctrl.position.gain.P_KI = 0;
        ctrl.position.gain.P_KD = 0;
    #endif
    
    #if(ACTIVE_SUBPROJECT == SUBPROJECT_B)
        // initialization code specific to ex 1
    #endif

    /*
	//User variables:
	motortb.ex1[0] = 0;
	motortb.ex1[1] = 0;
	motortb.ex1[2] = 0;
	motortb.ex1[3] = 0;
	motortb.ex1[4] = 0;
	motortb.ex1[5] = 0;
    */
}

// User finite state machine, implements a tight controller 
// Called at 1 kHz
// Call this function in one of the main while time slots.
void MotorTestBench_fsm(void)
{
    static unsigned int ticks = 0;
    static uint8_t state = 0;
    /* State represents whether we are running or not 
     * 0 - not running
     * 1 - gait cycle just started / starting
     * 2 - gait cycle in progress
    */
    
    if(ctrl.active_ctrl == CTRL_POSITION) 
    {
        unsigned int percentOfGaitCycle;
        switch(state) 
        {
        case 1:
            // set PID gains to non zero, ie we are actually traversing through the position profile now
            ctrl.position.setp = positionProfile[0];
            ticks = 1;
            ctrl.position.gain.P_KP = 90;
            ctrl.position.gain.P_KI = 5;
            ctrl.position.gain.P_KD = 5;
            
            state = 2;
            break;
            
        case 2:
             // using the conversion ticks/30 % 100 means that we complete 1 cycle every 3000 ticks.
             // note the modulo is redundant, ticks should always be less than 3000
            if(ticks < 3000) 
            {
                percentOfGaitCycle = (ticks / 30) % 100;
                ctrl.position.setp = positionProfile[percentOfGaitCycle];
                ticks++;
            }
            else
            {
                state = 0;
            }
            //no break, we transition directly into state 0, as opposed to waiting for another cycle (1ms)
            
        case 0:
            ctrl.position.gain.g0 = 0;
            ctrl.position.gain.g1 = 0;
            ctrl.position.gain.g2 = 0;            
            ticks = 0;
            
            //check the flag sent by manage
            if(motortb_startCycleFlag)
            {
                motortb_startCycleFlag = 0;
                state = 1;
            }
            
            break;
        }
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
