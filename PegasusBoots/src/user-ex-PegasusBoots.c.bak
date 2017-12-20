/****************************************************************************
	[Project] FlexSEA: Flexible & Scalable Electronics Architecture
	[Sub-project] 'user/PegasusBoots' MIT Biomechatronics Ankle Exoskeleton
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
	[This file] user-ex-PegasusBoots: User code running on Execute
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2016-10-28 | jfduval | New release
	*
****************************************************************************/

#ifdef BOARD_TYPE_FLEXSEA_EXECUTE


//****************************************************************************
// Include(s)
//****************************************************************************

#include <flexsea_board.h>
#include "../inc/user-ex-PegasusBoots.h"
#include "user-ex.h"
#include "control.h"
#include "motor.h"
#include "flexsea_sys_def.h"

//****************************************************************************
// Variable(s)
//****************************************************************************


        
// Per-Subject Parameters
#define SUBJECTMASS 68 //kg
#define LEVERARM (1905) //10*mm, so 145 mm, vertical distance from ankle joint to pulley
#define WPKG 300 //desired peak power, in 100 W/kg
#define PARABOLIC //Leave in for parabolic torque profile, comment out for positive velocity feedback (EXPERIMENTAL)

// System Specific
#define EXO_ID BP2 //Can be WKR, WKL, BP1, BP2
#define DRIVEPULLEY (14) // 14 teeth
#define SPOOLPULLEY (44) // 44 teeth
#define SPOOLRADIUS 40 //10*mm, so 4.0 mm (8mm shaft diameter)
#define STRUTSTIFF 163 //163 Nm/rad is the stiffness so (mom/100)/163*10000

//
    int32 mom_ctrl=0;
    int32 mom_1=0;
    int32 mom_2=0;
    int32 mom_3=0;
    int32 mom_4=0;
    int32 mom_5=0;
    int32 mom_6=0;
    int32 mom_7=0;
    int32 actual_mot_volt=0;
    
// Coefficients and Constants
//pegasus_fsm
int32 DPPF_cutoff = 2000; // [case 0] [-10000 x rad] DF angle from neutral_angle to switch to PF, negative!
int32 parab_delay = 60; // [case 0 and 1], also exo_power (ms)
int32 PPF_fin_angle = 2750; // [case 1] (10000 x rad)
int32 ankle_neutral_angle = 1000;// [case 3] [10000 x rad] ankle angle where there is zero torque
//exo_cpf
int32 exo_cpf_time = 100; // (ms)
//exo_spring
int32 lin_quad_r = 100; //[0 is all quad, 200 is all linear] ratio of linear and quad in DF spring
int32 spring_mag = 800; //[100 * Nm] spring magnitude at 0.4 rad (23 deg) away from neutral_angle (8 Nm)
//exo_power
int32 exo_ppf_time = 200; // (ms)
int32 peak_des_mom = 1250; // Starting parabolic torque goal, 100 x Nm
//exo_swing
int32 exo_swg_time = 150; // (ms)
int32 ankle_swing_angle = 15; //[-100 x rad] angle to let out during swing (negativized)
//position_control
int32 pid_setpos_a = -7500 ;
int32 pid_setpos_b = 5000;
int32 pos_Pgain = 300; // WAS 300
int32 pos_Igain = 0; //WAS 0 //60 worked well if want an I term
//set_mot_torque
int32 torque_Pgain = 1000; // TO BE ADJUSTED
int32 torque_Igain = 30;
//exo_set_volt
int32 deadband = 850;

// Variables
//init_pegasus
int32 exo_trans = 0, enc_per_rad = 0, ref_power = 0, exo_invert = 1;
//exo_refresh_values
int64 exo_mot_pos[4] = {0,0,0,0}, exo_mot_pos_fil[4] = {-10000,-10000,-10000,-10000};
int32 exo_equi_pos = 0;
int32 exo_mot_vel = 0, exo_mot_vel_fil = 0;
int32 ankle_ang = 0;
int64 ankle_angs[4] = {-10000,-10000,-10000,-10000};
int64 ankle_angs_fil[4] = {-10000,-10000,-10000,-10000};
int32 ankle_vel_fil = 0, ankle_ang_fil = 0;
int32 exo_angvel_gyro = 0;
int32 exo_cur = 0, exo_strain = 0;
int32 ankle_mom = 0;
int32 exo_pos_pow = 0;
int32 exo_strain_zero = 0;
int32 exo_batvolt = 0;
int32 exo_temp = 0;
long long exo_time = 0;
long long statetimer = 0;
int32 steptime = 0;
//change_state
int32 exo_state = -3, exo_lastpos = -10000, ankle_last_mom = 0, lastpospow=0;
//exo_zero
int32 counter = 0;
//pegasus_fsm case 0
int32 stanceDFtime = 0;
//exo_power
int32 maxDFang = 0, max_mom = 0, max_PF_power = 0;
//position_control
int32 pos_despos;
int32 pos_Istate = 0;
int32 pos_error;
int32 pos_Pterm;
int32 pos_Iterm;
//set_mot_torque
int32 ankle_des_mom = 0;
int32 torque_Istate = 0;
int32 torque_error;
int32 torque_Pterm;
int32 torque_Iterm;
//exo_set_volt
int32 mot_pwm = 0; // DEPRECATED
int32 mot_volt = 0; // mV

struct exodata_s exo;

// Unused
//int32 exo_pow = 0;
//int32 des_exo_pos_pow = 10000; //1000*W, currently 10 W  NEVER USED
//int32 exo_pos = 0;

//****************************************************************************
// Private Function Prototype(s):
//****************************************************************************

static void exo_refresh_values(void);
static void exo_spring(void);
static void exo_power(void);
static void exo_set_volt(void);
static int8 exo_zero(void);
static void exo_zero_strain(void);
static void change_state(int8);
static void position_control(int32);
static void set_mot_torque(int32);
static void exo_swing(void);
static void exo_cpf(void);
static int64 butt_filter(int64[4], int64[4]);


//****************************************************************************
// Public Function(s)
//****************************************************************************

//Called in user-ex.c, just before the while()
void init_pegasus(void)
{
    //Controller setup:
    setBoardID(SLAVE_ID);
    ctrl.active_ctrl = CTRL_OPEN;   //Open controller
    motor_open_speed_1(0);              //0% PWM
	#if(MOTOR_COMMUT == COMMUT_BLOCK)
    Coast_Brake_Write(1);               //Brake (regen)
	#endif
    
    //Quick algebraic calculations from input variables:
    exo_trans = (SPOOLPULLEY*LEVERARM)/(DRIVEPULLEY*SPOOLRADIUS);//transmission ratio, currently 157.14
	enc_per_rad = 2000*exo_trans/628; //encoder clicks per ankle radian / 100, currently 500.45
	ref_power = SUBJECTMASS*WPKG; // in 1000000 x W
        
    //WKR is only inverted exo (WKL, BP1 and BP2 are non-inverted):
    #if (EXO_ID == WKR)
        exo_invert = -1;
    #else
        exo_invert = 1;
    #endif
    
}

//Pegasus Boot Finite State Machine.
//Call this function in one of the main while time slots.
void pegasus_fsm(void)
{
    
	//Before going to a state we refresh values:
	exo_refresh_values();
	
	switch(exo_state) // wait, wind-up, zero pos, zero strain, cpf, spring df, powerpf, swing, cpf...
	{
        case -3: // wait for user to turn motor
            if (exo_mot_pos[0]>2000)
            {
                change_state(-2);
            }
            break;
        case -2: // determine the zero motor position
            if (exo_zero()==1 && exo_mot_pos[0]>3000)
            {
                counter=500;
                change_state(-1);
            }
            break;
        case -1: // determine the baseline zero-strain reading
			exo_zero_strain();  
            if (exo_mot_pos[0]>=0 && statetimer>100) // second clause just prevents skipping this step
            {
                change_state(0);
            }
            break;
		case 0: // stance phase - get a desired moment from spring equation            
            exo_spring();

            if (ankle_ang<maxDFang) {maxDFang = ankle_ang;} //for logging purposes
            
            if (ankle_ang<-DPPF_cutoff)// && ankle_vel_fil > 0)
            {
                stanceDFtime = statetimer; //records time in stance
                    // set up for the coming powerstroke
        		#ifdef PARABOLIC
                    //apparently delay should be 17.5% of gait cycle? TWEAK
                    parab_delay = steptime*175/1000-30; 
                    if (parab_delay>90)
                        {parab_delay=90;} // max delay is 90
                    else if (parab_delay<20)
                        {parab_delay=20;} // min delay is 20
        			if (max_PF_power>ref_power/100*105) // last step too strong
    				    {peak_des_mom = peak_des_mom-75;} // cool your jets
    				else if (max_PF_power<ref_power/100*95) // last step too weak
                        {peak_des_mom = peak_des_mom+75;} // step it up
        			max_PF_power = 0; // reset PF power tracker
        		#endif
                change_state(1);   
            }
            
            if (ankle_ang>2150) // broken/will never happen, potentially should be a_n_a
            {
                stanceDFtime = statetimer; //records time spent in stance
                change_state(2); 
            }
			break;
		case 1:
			exo_power();
            
            // exit once get to software stop
            if (ankle_ang>PPF_fin_angle)
            {
                change_state(2);
            }

            // or, exit if the parabola is complete
            #ifdef PARABOLIC
                if (statetimer>(exo_ppf_time+parab_delay))
                {
                    change_state(2);
                }
            #endif
            
			break;
        case 2:

			exo_swing();
           
            if ((statetimer>500) || (statetimer>150 && exo_angvel_gyro<0)) // wait for "1 sec" or "0.3 sec and now I'm done swinging forward"
            {
                change_state(0);
            }
           
			break;
        case 3: // cpf until cpf_time(100) cycles and user begins DF
        
			exo_cpf();
            
            //if (statetimer>exo_cpf_time && exo_mot_pos[0]>=ankle_neutral_angle*enc_per_rad/100) 
            if (statetimer>exo_cpf_time+100) 
            {
                change_state(0);
            }
         
			break;
        case 4: // debugging PID
           
            if (counter<1000){
                ankle_des_mom = counter*counter/1000 - counter + 250;
            } else if (counter >=1000){
                ankle_des_mom = -counter*counter/1000 + 3*counter - 1750;
            } 
            set_mot_torque(ankle_des_mom);
            counter=counter+1;
            if (counter == 2000) {
                counter = 0;
            }
            break;
		default:
			//Handle exceptions here
			break;
            
            
	}

    //After picking a state, we update the motor command
    actual_mot_volt = mot_volt;
    
    if (mot_volt > (exo_batvolt-10000)){
        mot_volt = exo_batvolt-10000;
    } else if (mot_volt < (-exo_batvolt+10000)) {
        mot_volt = -exo_batvolt+10000;
    }
    exo_set_volt();
}

//****************************************************************************
// Private Function(s)
//****************************************************************************

static void exo_refresh_values(void) {
        //battery voltage in millivolts
        exo_batvolt = (uint32)safety_cop.v_vb*1942/11+10000;
        exo_temp = (uint32)safety_cop.temperature;
    
	    //update the motor position
        exo_mot_pos[3] = exo_mot_pos[2];    
        exo_mot_pos[2] = exo_mot_pos[1];
        exo_mot_pos[1] = exo_mot_pos[0];
		
		exo_mot_pos[0] = exo_invert*(int32)encoder.count; //motor position in clicks
        exo_mot_pos[0] = exo_mot_pos[0]-exo_equi_pos; //motor position is relative to the post-calibration zero
		
        //filtering the motor position
        int64 tmp_filpos = 0;
        tmp_filpos = butt_filter(exo_mot_pos,exo_mot_pos_fil);
        
        exo_mot_pos_fil[3] = exo_mot_pos_fil[2];    
        exo_mot_pos_fil[2] = exo_mot_pos_fil[1];
        exo_mot_pos_fil[1] = exo_mot_pos_fil[0];
        exo_mot_pos_fil[0] = tmp_filpos; //10^12 

        //Motor velocity[RPM] (P0-P1) * 1000s^-1 *60s/min / 2000clicks/rot
        //exo_mot_vel = (exo_mot_vel+3*(exo_mot_pos[0]-exo_mot_pos[1])*30)/4;
        exo_mot_vel = (exo_mot_pos[0]-exo_mot_pos[3])*333*10/enc_per_rad; //ankle velocity in 1000*rad/sec
        exo_mot_vel_fil = (int32)(int64)((exo_mot_pos_fil[0]-exo_mot_pos_fil[3])*333/enc_per_rad/10000000);   
		
        //enc_per_rad is in [clicks/rad / 100]
        //ankle_ang accounts for the deflection of the struts (currently disabled)
        ankle_ang = exo_mot_pos[0]*100/enc_per_rad;//-ankle_mom*100/STRUTSTIFF; //rad x 10000
        
        ankle_angs[3] = ankle_angs[2]; //x 10^4 rad
        ankle_angs[2] = ankle_angs[1];
        ankle_angs[1] = ankle_angs[0];
        ankle_angs[0] = ankle_ang;
        
        int64 tmp_anklepos = 0;
        tmp_anklepos = butt_filter(ankle_angs,ankle_angs_fil); //x 10^12 rad
        ankle_angs_fil[3] = ankle_angs_fil[2];
        ankle_angs_fil[2] = ankle_angs_fil[1];
        ankle_angs_fil[1] = ankle_angs_fil[0];
        ankle_angs_fil[0] = tmp_anklepos; //x10^12 rad
        
        ankle_vel_fil = (int32)((int64)((ankle_angs_fil[0]-ankle_angs_fil[1])/1000000)); //1000 x rad/sec
        ankle_ang_fil = (int32)((int64)(ankle_angs_fil[0]/100000000));

		exo_angvel_gyro = -imu.gyro.x; //Units? (known that raw value / 16.4 gives degree per second)
        
		//current is at -18.5 mA/bit, variable zero. total string force in [10 x N] (aka decinewtons)
        #if(EXO_ID == WKL)
            exo_cur = -(ctrl.current.actual_val+46)*185/10;
            exo_strain = -strain_read()*410/1000+15656;
            //exo_strain = 15111 + strain_read()*-407/1000 + exo_temp*4181/1000;
        #endif

        #if(EXO_ID == WKR)
            exo_cur = -(ctrl.current.actual_val+10)*185/10;
            exo_strain = -strain_read()*413/1000+15571;
            //exo_strain = 16021 + strain_read()*-415/1000 + exo_temp*-3324/1000;
        #endif

        #if(EXO_ID == BP1)
            exo_cur = -(ctrl.current.actual_val+46)*185/10;
            exo_strain = strain_read()*457/1000-15384;
            //exo_strain = -14723 + strain_read()*456/1000 + exo_temp*-5595/1000;
        #endif

        #if(EXO_ID == BP2)
            exo_cur = -(ctrl.current.actual_val+46)*185/10;
            exo_strain = strain_read()*458/1000-15558;
            //exo_strain = -15458 + strain_read()*458/1000 + exo_temp*-1158/1000;
        #endif

        exo_strain = exo_strain-exo_strain_zero; //accounts for any errors at startup
        
        //LEVERARM is in 10 x mm
        int32 old_ankle_mom;
        old_ankle_mom = ankle_mom;
        ankle_mom = ((LEVERARM/100)*exo_strain)/10; //Applied ankle moment in [100 x Nm] (aka cNm)
        if (ankle_mom<0)
        {ankle_mom = 0;}
        
        mom_ctrl = (ankle_mom+mom_1+mom_2+mom_3+mom_4+mom_5+mom_6+mom_7)/8;
        mom_7 = mom_6;
        mom_6 = mom_5;
        mom_5 = mom_4;
        mom_4 = mom_3;
        mom_3 = mom_2;
        mom_2 = mom_1;
        mom_1 = ankle_mom;
        
        ankle_angs_fil[1] = ankle_angs_fil[0];
        ankle_angs_fil[0] = tmp_anklepos;
        
		//power as instantaneous moment times angular veloctiy, summed over whole plantarflexion
        if (ankle_angs[0]>ankle_angs[1])
        {
            exo_pos_pow = exo_pos_pow+(ankle_angs[0]-ankle_angs[1])/10*(ankle_mom+old_ankle_mom)/20; //10000*J
        }
        
		//increment times after value updates
        exo_time++;
        statetimer++;
        steptime++;
        
        //Update the communication structure
        exo.state = (int8_t)(exo_state); // Mapped to Gyro X
        exo.des_mom = (int16_t)(exo_mot_pos[0]+10000); // Mapped to Gyro Y (REMEMBER TO DIV BY 10 LATER)
        exo.mot_volt = (int16_t)(mot_volt); // Mapped to Gyro Z (REMEMBER TO DIV BY 10 LATER)
        exo.pwm = (int16_t)(pos_despos+10000); //(mot_pwm); // Mapped to Accel X
        exo.max_PF_power = (int16_t)(actual_mot_volt);//(max_PF_power); // Mapped to Accel Y
        exo.ankle_mom = (int16_t)(ankle_mom); // Mapped to Accel Z (REMEMBER TO DIV BY 10 LATER)
        exo.ankle_ang = (int16_t)(ankle_ang); // Mapped to Analog 0
        exo.ankle_vel = (int16_t)(actual_mot_volt); // Mapped to Analog 1
        
        // Unmapped
        exo.time = exo_time; //int32
        exo.shank_angvel = (int16_t)(lastpospow);//(exo_angvel_gyro);
        exo.peak_des_mom = (int16_t)(peak_des_mom)/10;
        exo.mot_curr = (int16_t)(exo_cur)/10;
        exo.bat_volt = (uint8_t)(exo_batvolt);




}

static void change_state(int8 newstate){
    exo_state = newstate;
    statetimer = 0;
    exo_lastpos = exo_mot_pos[0];
    ankle_last_mom = ankle_mom;
    
    max_mom = 0;
    if (newstate == 0)
    {
        lastpospow = exo_pos_pow*10/steptime; //100*JOULES (aka cJ)
        exo_pos_pow = 0;
        steptime = 0; 
        torque_Istate = 0;
    } else if (newstate == 1){
        torque_Istate = 0;
    } else if (newstate == 2 || newstate == 1)
    {
        pos_Istate=0;
    } else if (newstate == 3)
    {
        pos_Istate=0;
    } else if (newstate == 4)
    {
        pos_Istate = 0;
    }
}

static int8 exo_zero(void){
    int32 calibration_mom = 300;
    
    if (statetimer<1000)
    {
        ankle_des_mom = statetimer*calibration_mom/1000;
        //mot_volt = statetimer*1500/1000;
    }
    else 
    {ankle_des_mom = calibration_mom;}//mot_volt=1500;}
    
    set_mot_torque(ankle_des_mom);
    
    if (exo_mot_vel_fil>-100 && exo_mot_vel_fil<100)
    {counter = counter+1;}
    else
    {counter = 0;}
    
    if (counter>2000)
    {
        exo_equi_pos = exo_mot_pos[0]; 
        return 1;
    }
    else
    {return 0;}
}

static void exo_zero_strain(void){
    if (statetimer<200)
    {mot_volt = statetimer*-10000/200;}
    else
    {mot_volt = 1500;}
    
    if (statetimer == 148)
    {exo_strain_zero = exo_strain;}
}

static void exo_cpf(void){
    int32 exo_cpf_pos;
    
    //enc_per_rad is in [clicks/rad /100]
    //ankle_neutral_angle is in [10000 x rad]
    exo_cpf_pos = ankle_neutral_angle*enc_per_rad/100;
    
    if (statetimer<exo_cpf_time)
    {
        pos_despos = exo_lastpos + statetimer*(exo_cpf_pos-exo_lastpos)/(exo_cpf_time);
        //pos_Pgain = 10; // in [10 x mV/click] was 200
    }
    else
    {
        pos_despos = exo_cpf_pos;
        //pos_Pgain = 10; //was 50
    }

    position_control(pos_despos);
}

static void exo_spring(void){
    //ankle_ang_fil and ankle_neutral_angle are in 10000 x rad  
    //ankle_des_mom is in 100 x Nm    
    int32 xf, x0, yf, RR, xank, Aspring, Bspring;
    
    x0 = ankle_neutral_angle;
    xf = x0-4000;
    yf = spring_mag;
    xank = ankle_ang_fil;
    
    RR = yf*lin_quad_r/100;
   
    Aspring = (yf+RR)*1000/(2*(xf-x0));
    Bspring = (yf-(yf+RR)/2)*10000/(xf-x0)*100/(xf-x0);
    
    ankle_des_mom =  Aspring*(xank-x0)/1000+Bspring*(xank-x0)/100*(xank-x0)/10000;
    
    set_mot_torque(ankle_des_mom);   
}

static void exo_power(void){
    int32 exo_ppf_pos;

    //enc_per_rad is in [clicks/rad /100]
    //PPF_fin_angle is in [10000 x rad]
    exo_ppf_pos = PPF_fin_angle*enc_per_rad/100; 
    
    if (statetimer<exo_swg_time)
    {
        pos_despos = exo_lastpos + statetimer*(exo_ppf_pos-exo_lastpos)/(exo_ppf_time);
    }
    else // after swingtime your despos should be at full swgpos anyway
    {
        pos_despos = exo_ppf_pos;
    } 

    position_control(pos_despos);
}

static void exo_swing(void){
    int32 exo_swg_pos;

    //enc_per_rad is in [clicks/rad /100]
    //ankle_swing_angle is in [100 x rad]
    exo_swg_pos = -ankle_swing_angle*enc_per_rad;
    
    if (statetimer<exo_swg_time)
    {
        pos_despos = exo_lastpos + statetimer*(exo_swg_pos-exo_lastpos)/(exo_swg_time);
    }
    else // after swingtime your despos should be at full swgpos anyway
    {
        pos_despos = exo_swg_pos;
    } 

    position_control(pos_despos);
}

static void position_control(int32 despos){
    pos_error = despos - exo_mot_pos[0];
    pos_Istate = pos_Istate + pos_error;
    pos_Pterm = pos_error*pos_Pgain/100;
    pos_Iterm = pos_Istate*pos_Igain/1000;
    mot_volt = pos_Pterm + pos_Iterm;
}

static void set_mot_torque(int32 des_mom){
    //des_mom is the desired ankled moment in [100 x Nm] (aka cNm)
    
    // desired moment is hardlimited at 50 Nm
    if (des_mom>5000)
    {des_mom = 5000;}
    
    // VERSION 3
    
    torque_error = des_mom - mom_ctrl;
    torque_Istate = torque_Istate + torque_error;
    torque_Pterm = torque_error * torque_Pgain/100;
    torque_Iterm = torque_Istate * torque_Igain/1000;
    mot_volt = torque_Pterm + torque_Iterm;     
    
    /* VERSION 2
    int32 des_current = des_mom*4232/exo_trans/10; // outputs current in mA, Kt of motor is 42.32 A/Nm or 423.2 mA/cNm, divided by transmission ratio
    mot_volt = des_current*353/1000 + (exo_mot_pos_fil[0]-exo_mot_pos_fil[3])/100000000*10; //.353 ohm Resistance, 23.63 mV/radps. just needed SOME velocity term (arbitrary gains)
    mot_volt = mot_volt+torque_Pgain*(des_mom-ankle_mom);
    */

    /* VERSION 1 
    //All of this assumes better efficiency than is seen. Can I do better?
    // int32 des_current = des_mom*10/exo_trans*36; //mA (Luke's expression for Current
    int32 des_current = des_mom/exo_trans*423; // outputs current in mA, Kt calculated as 0.023629465 Nm/A or 42.32004427 A/Nm or 423 mA/cNm
    //This means currently hard-limited at 15.5 A! Is that safe? Was 11.3 A
    //back EMF = ankle_vel_fil*trans rad/sec * 60 sec/min * 1 rot/6.28 rad * 1V / 346 RPM
    mot_volt = des_current*353/1000 + (exo_mot_pos[0]-exo_mot_pos[1])*86*50/100; //Previously .386 ohm, now .353 ohm Resistance
    mot_volt = mot_volt+torque_Pgain*(des_mom-ankle_mom);    
    */
}

static void exo_set_volt(){
    
    int32 voltsign = 1;
    int32 abvolt = mot_volt;
    if (abvolt<0)
    {
        voltsign = -1;
        abvolt = -abvolt;
    }

    if (abvolt < 2*deadband && abvolt != 0){
        abvolt = deadband + abvolt/2;
    }
    
    motor_open_speed_1(exo_invert*voltsign*abvolt);	
}

static int64 butt_filter(int64 x[4], int64 y[4]){
    //x is in 10^4
    //y is in 10^12
    
    int64 yy = 0;
    //10 Hz
    //int64 filt_a[4] = {10000, -28744, 27565, -8819}; //a*10^4
    //int64 filt_b[4] = {29146494, 87439483, 87439483, 29146494}; //b*10^12
    
    //30 Hz
    int64 filt_a[4] = {10000, -26236, 23147, -6855}; //a*10^4
    int64 filt_b[4] = {699349650, 2098048950, 2098048950, 699349650}; //b*10^12

    yy = ((filt_b[0]*x[0] + filt_b[1]*x[1] + filt_b[2]*x[2] + filt_b[3]*x[3]) - filt_a[1]*(y[0]) - filt_a[2]*(y[1]) - filt_a[3]*(y[2]))/10000;
    return yy;
}

//******************************************************************************************
#endif //BOARD_TYPE_FLEXSEA_EXECUTE
