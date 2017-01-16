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

// define a 1000 long array, defining the position profile for the motor controlled by ex1
// static int32_t positionProfile[] = { .. }; 

static int32_t positionProfile[] = {
0,-21,-41,-62,-83,-105,-127,-150,-173,-196,-220,-245,-269,-294,-320,-346,-372,-399,-426,-453,-481,-510,-538,-568,-597,-627,-657,-688,-719,-751,-783,-815,-848,-881,-915,-949,-983,-1018,-1053,-1089,-1125,-1161,-1198,-1235,-1273,-1311,-1349,-1388,-1427,-1466,-1506,-1546,-1587,-1628,-1670,-1712,-1754,-1797,-1840,-1883,-1927,-1971,-2016,-2061,-2106,-2152,-2198,-2244,-2291,-2339,-2386,-2434,-2483,-2532,-2581,-2630,-2680,-2731,-2781,-2833,-2884,-2936,-2988,-3041,-3094,-3147,-3201,-3255,-3310,-3365,-3420,-3476,-3532,-3588,-3645,-3702,-3759,-3817,-3876,-3934,-3993,-4053,-4113,-4173,-4233,-4294,-4355,-4417,-4479,-4541,-4604,-4667,-4731,-4795,-4859,-4924,-4989,-5054,-5120,-5186,-5252,-5319,-5386,-5454,-5522,-5590,-5659,-5728,-5797,-5867,-5937,-6007,-6078,-6149,-6221,-6293,-6365,-6438,-6511,-6584,-6658,-6732,-6806,-6881,-6956,-7032,-7108,-7184,-7261,-7338,-7415,-7493,-7571,-7649,-7728,-7807,-7887,-7966,-8047,-8127,-8208,-8289,-8371,-8453,-8535,-8618,-8701,-8784,-8868,-8952,-9037,-9121,-9206,-9292,-9378,-9464,-9551,-9637,-9725,-9812,-9900,-9989,-10077,-10166,-10256,-10345,-10435,-10526,-10616,-10707,-10799,-10891,-10983,-11075,-11168,-11261,-11355,-11448,-11543,-11637,-11732,-11827,-11923,-12019,-12115,-12211,-12308,-12406,-12503,-12601,-12699,-12798,-12897,-12996,-13096,-13196,-13296,-13397,-13498,-13599,-13701,-13803,-13905,-14008,-14111,-14214,-14318,-14422,-14526,-14631,-14736,-14841,-14947,-15053,-15159,-15266,-15373,-15480,-15587,-15695,-15804,-15912,-16021,-16131,-16240,-16350,-16460,-16571,-16682,-16793,-16904,-17016,-17129,-17241,-17354,-17467,-17581,-17694,-17808,-17923,-18038,-18153,-18268,-18384,-18500,-18616,-18733,-18850,-18967,-19085,-19203,-19321,-19439,-19558,-19677,-19797,-19917,-20037,-20157,-20278,-20399,-20520,-20642,-20764,-20886,-21009,-21132,-21255,-21378,-21502,-21626,-21751,-21875,-22000,-22126,-22251,-22377,-22504,-22630,-22757,-22884,-23011,-23139,-23267,-23396,-23524,-23653,-23782,-23912,-24041,-24172,-24302,-24433,-24564,-24695,-24826,-24958,-25090,-25223,-25355,-25488,-25622,-25755,-25889,-26023,-26157,-26292,-26427,-26562,-26698,-26834,-26970,-27106,-27243,-27380,-27517,-27654,-27792,-27930,-28068,-28207,-28346,-28485,-28624,-28764,-28904,-29044,-29185,-29325,-29466,-29608,-29749,-29891,-30033,-30175,-30318,-30461,-30604,-30747,-30891,-31035,-31179,-31323,-31468,-31613,-31758,-31903,-32049,-32195,-32341,-32487,-32634,-32781,-32928,-33076,-33223,-33371,-33519,-33668,-33816,-33965,-34114,-34264,-34413,-34563,-34713,-34863,-35014,-35165,-35316,-35467,-35618,-35766,-35909,-36043,-36166,-36275,-36366,-36437,-36485,-36507,-36500,-36467,-36409,-36327,-36225,-36103,-35965,-35811,-35645,-35467,-35280,-35084,-34880,-34667,-34446,-34216,-33979,-33733,-33480,-33220,-32953,-32678,-32396,-32108,-31813,-31512,-31204,-30891,-30571,-30246,-29916,-29580,-29239,-28893,-28542,-28187,-27827,-27463,-27095,-26723,-26347,-25968,-25585,-25199,-24809,-24417,-24023,-23625,-23225,-22823,-22419,-22013,-21605,-21195,-20784,-20372,-19958,-19544,-19128,-18712,-18295,-17878,-17461,-17043,-16625,-16208,-15790,-15373,-14957,-14541,-14126,-13712,-13299,-12888,-12477,-12068,-11660,-11254,-10850,-10448,-10047,-9649,-9253,-8860,-8468,-8080,-7693,-7310,-6930,-6552,-6177,-5806,-5438,-5073,-4712,-4354,-3999,-3648,-3301,-2958,-2619,-2284,-1953,-1626,-1303,-985,-671,-361,-56,244,540,830,1117,1398,1674,1945,2211,2472,2728,2978,3223,3463,3697,3925,4148,4366,4577,4783,4983,5177,5365,5547,5722,5892,6055,6212,6363,6507,6645,6776,6900,7017,7123,7220,7304,7375,7430,7470,7493,7496,7480,7446,7396,7332,7256,7171,7078,6980,6879,6776,6674,6573,6474,6375,6277,6181,6085,5991,5898,5805,5714,5624,5534,5446,5359,5273,5188,5104,5021,4938,4857,4777,4698,4620,4543,4467,4392,4317,4244,4172,4101,4030,3961,3892,3825,3758,3693,3628,3564,3501,3439,3378,3318,3259,3200,3143,3086,3031,2976,2922,2869,2816,2765,2714,2665,2616,2567,2520,2474,2428,2383,2339,2296,2253,2211,2170,2130,2091,2052,2014,1977,1940,1905,1869,1835,1801,1769,1736,1705,1674,1644,1614,1585,1557,1529,1502,1476,1450,1425,1400,1377,1353,1331,1308,1287,1266,1245,1226,1206,1187,1169,1151,1134,1118,1101,1086,1070,1056,1041,1028,1014,1001,989,977,965,954,944,933,923,914,905,896,888,880,872,865,858,851,845,839,833,828,823,818,814,810,806,802,799,795,793,790,787,785,783,781,780,778,777,776,775,774,774,773,773,773,773,773,773,773,773,774,774,775,776,776,777,778,779,779,780,781,782,783,784,784,785,786,787,787,788,789,789,789,790,790,790,790,790,790,789,789,788,787,786,785,784,782,780,779,776,774,772,769,766,763,760,756,752,749,744,740,736,731,727,722,717,712,706,701,695,689,683,677,671,665,658,652,645,638,631,624,617,610,602,595,587,579,571,563,555,547,539,531,523,514,506,497,488,480,471,462,453,444,435,426,417,408,399,390,381,371,362,353,343,334,325,315,306,297,287,278,269,259,250,240,231,222,212,203,194,185,176,166,157,148,139,130,121,112,103,95,86,77,69,60,52,44,35,27,19,11,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
};

pid_controller positionController;

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
	ctrl.active_ctrl = CTRL_CUSTOM;
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
        pid_controller_initialize(&positionController, 1024, 50000, 90, 40000);
        pid_controller_settings(&positionController, 0, 1, 1);
    //#endif
    
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
	}
	else
	{
		switch(state) 
	    {

	    case 4:
	        ticks++;
	        if(ticks > 1000)
	        {
	            ticks = 0;
	            state = 0;
				
	            #if(ACTIVE_SUBPROJECT == SUBPROJECT_A)
					pid_controller_setGains(&torqueController, 0, 10, 0, 7);
	                torqueController.setpoint = torqueProfile[0];
					
	                initialPosition = -1*exec1.enc_control_ang;
	                positionController.setpoint = initialPosition - positionProfile[0];
	                pid_controller_setGains(&positionController, 10, 1, 2, 8);				
	            #endif

	            #if(ACTIVE_SUBPROJECT == SUBPROJECT_B)
	                initialPosition = exec1.enc_control_ang;
	                positionController.setpoint = initialPosition + positionProfile[0];
	                pid_controller_setGains(&positionController, 35, 1, 2, 8);
	            #endif
	        }
	        break;
	        
	    case 1:
	        // set PID gains to non zero, ie we are actually traversing through the position profile now
	        #if(ACTIVE_SUBPROJECT == SUBPROJECT_A)
	            torqueController.setpoint = torqueProfile[0];
	            pid_controller_setGains(&torqueController, 0, 10, 0, 7);
				positionController.setpoint = initialPosition - positionProfile[0];
	            pid_controller_setGains(&positionController, 10, 1, 2, 8);
				
	        #endif
	        #if(ACTIVE_SUBPROJECT == SUBPROJECT_B)
	            positionController.setpoint = initialPosition + positionProfile[0];
	            pid_controller_setGains(&positionController, 35, 1, 2, 8);
	        #endif
	        ticks = 1;
	        state = 2;
	        break;
	        
	    case 2:
			
	        #if(ACTIVE_SUBPROJECT == SUBPROJECT_A)
	            torqueController.setpoint = torqueProfile[ticks] >> 2;
				positionController.setpoint = (initialPosition - positionProfile[ticks]) >> 3;
			#endif
	        #if(ACTIVE_SUBPROJECT == SUBPROJECT_B)
	            positionController.setpoint = (initialPosition + positionProfile[ticks]) >> 3;
			#endif

			ticks++;
			if(ticks > 1000) 
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

	        break;
	    }	
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
	        positionController.controlValue = exec1.enc_control_ang;
	        pwm = pid_controller_compute(&positionController);
	    #endif
	    //lastRPM = as5047.raw.vel_rpm;
	    motor_open_speed_1(pwm);	
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
