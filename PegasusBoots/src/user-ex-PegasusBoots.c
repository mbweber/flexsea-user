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

//Per-Subject Parameters
//#define EXOLEFT //Comment out for the right exo when asymmetric
#define SUBJECTMASS 68 //kg
#define LEVERARM (2000) //10*mm, so 145 mm, vertical distance from ankle joint to pulley
#define PARABOLIC //Leave in for parabolic torque profile, comment out for positive velocity feedback
#define WPKG 300 //desired peak power, in 100 W/kg
    
// Adjustables
int32 ankle_neutral_angle = 500;
int32 DPPF_cutoff = 2000;
int32 G_torque = 15; // TO BE ADJUSTED
int32 exo_swg_time = 200, exo_cpf_time = 75, exo_ppf_time = 200; //ms
int32 parab_delay = 60;
int32 PPF_fin_angle = 2750; //10000 x rad   
    
// System Mechanics
#define DRIVEPULLEY (14) // 14 teeth
#define SPOOLPULLEY (44) // 44 teeth
#define STRUTSTIFF 163 //163 Nm/rad is the stiffness so (mom/100)/163*10000
#define SPOOLRADIUS 40 //10*mm, so 4.0 mm (8mm shaft diameter)
    
//Declarations and Initializations
int32 exo_cur = 0, exo_pow = 0, exo_strain = 0, exo_strain_zero = 0;
int32 exo_angvel = 0, exo_mot_vel = 0, exo_pos = 0, exo_batvolt = 0;
int64 exo_mot_pos[4] = {0,0,0,0};
int64 exo_mot_pos_fil[4] = {-10000,-10000,-10000,-10000};
int64 ankle_angs[4] = {-10000,-10000,-10000,-10000};
int64 ankle_angs_fil[4] = {-10000,-10000,-10000,-10000};
int32 mot_pwm = 0, exo_lastpos = -10000;
int32 mot_volt = 0, exo_equi_pos = 0;
int32 ankle_swing_angle = 18;
int32 counter = 0;
int32 ankle_ang = 0, ankle_mom = 0, ankle_last_mom = 0, ankle_des_mom = 0;
int32 ankle_vel_fil = 0;
int32 maxDFang = 0, stanceDFtime = 0;
int32 max_mom = 0, lin_quad_r = 0, spring_mag = 0;
int32 exo_mot_vel_fil = 0, ankle_ang_fil = 0;
int32 exo_pos_pow = 0, steptime = 0, lastpospow=0;
//int32 des_exo_pos_pow = 10000; //1000*W, currently 10 W  NEVER USED
long long exo_time = 0;
long long statetimer = 0;
int32 exo_state = -3;
int32 clkrad = 0;
int32 exo_trans = 0;
int32 exo_invert = -1; 
int32 exo_des_pos;
int32 peak_des_mom = 1250; // Starting parabolic torque goal, 100 x Nm
int32 max_PF_power = 0;
int32 ref_power = 0;
int32 cycletimer, magnitude;
int32 springy=0;

struct exodata_s exo;

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
static void set_mot_torque(int32);
static void exo_swing(void);
static void exo_cpf(void);
static int64 butt_filter(int64[4], int64[4]);

//****************************************************************************
// Public Function(s)
//****************************************************************************

//Call this function once in main.c, just before the while()
void init_pegasus(void)
{
	setBoardID(SLAVE_ID);

    //Controller setup:
    ctrl.active_ctrl = CTRL_OPEN;   //Open controller
    motor_open_speed_1(0);              //0% PWM
	#if(MOTOR_COMMUT == COMMUT_BLOCK)
    Coast_Brake_Write(1);               //Brake (regen)
	#endif
    
    //Default values for all the variables:
    exo_trans = (SPOOLPULLEY*LEVERARM)/(DRIVEPULLEY*SPOOLRADIUS);//transmission ratio, currently 121.79

	clkrad = 2000*exo_trans/628; //encoder clicks per ankle radian / 100
	
	ref_power = SUBJECTMASS*WPKG; // in 1000000 x W

    
    ctrl.position.gain.P_KP = 200; //[0 is all quad, 200 is all linear] ratio of linear and quad in DF spring
    ctrl.position.gain.P_KI = 800; //[100 * Nm] spring magnitude at 0.4 rad (23 deg) away from neutral_angle (8 Nm)
    ctrl.position.gain.P_KD = 1750; //[-10000 x rad] DF angle from neutral_angle to switch to PF, negative!
    
    ctrl.current.gain.I_KP = 1000; //[10000 x rad] ankle angle where there is zero torque
    ctrl.current.gain.I_KI = 15; //[100 x rad] angle to let out during swing
    
    //While using asymmetric exos:
    #ifdef EXOLEFT
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
            if (exo_zero()==1 && exo_mot_pos[0]>4000)
            {
                exo_equi_pos = exo_mot_pos[0]; 
                change_state(-1);
            }
            break;
        case -1: // determine the baseline zero-strain reading
			exo_zero_strain();  
            if (exo_mot_pos[0]>=0 && statetimer>100)
            {
                change_state(0);
            }
			
            break;
		case 0: // get a desired moment from spring equation
            mot_volt=-exo_mot_pos[0]/8+500;
            
            //exo_spring();
            
           // if (ankle_ang<-DPPF_cutoff)// && ankle_vel_fil > 0)
           // {
            //  stanceDFtime = statetimer; //why?
            //  maxDFang = 1000; //why?
			
        	//		#ifdef PARABOLIC
        	//			if (max_PF_power>ref_power/100*105)
        	//			{peak_des_mom = peak_des_mom-75;}
        	//			else if (max_PF_power<ref_power/100*95)
        	//			{peak_des_mom = peak_des_mom+75;}
        	//			max_PF_power = 0;
                        
             //           parab_delay = steptime*175/1000-30;
             //           if (parab_delay>90)
             //               {parab_delay=90;}
             //           else if (parab_delay<20)
              //              {parab_delay=20;}
                        
        	//		#endif
			  
            //  change_state(1);   
           // }
            
            if (ankle_ang>2150) // broken/will never happen, potentially should be a_n_a
            {
              stanceDFtime = statetimer;//why?
              maxDFang = 1000; // why?
              change_state(2); 
            }
          
			break;
		case 1:
			exo_power();
            
            if (ankle_ang>PPF_fin_angle)
            {
                change_state(2);
            }

            #ifdef PARABOLIC
                if (statetimer>(exo_ppf_time+parab_delay))
                {
                    change_state(2);
                }
            #endif
            
			break;
        case 2:
            
			exo_swing();
           
            if ((statetimer>1000) || (statetimer>150 && exo_angvel<0)) // wait for "1 sec" or "0.3 sec and now I'm done swinging forward"
            {
                change_state(3);
            }
           
			break;
        case 3: // cpf until cpf_time(100) cycles and user begins DF
			exo_cpf();
            
            //if (statetimer>exo_cpf_time && exo_mot_pos[0]>=ankle_neutral_angle*clkrad/100) 
            if (statetimer>exo_cpf_time) 
            {
                change_state(0);
            }
         
			break;
		default:
			//Handle exceptions here
			break;
            
            
	}

    //After picking a state, we update the motor command
    exo_set_volt();
}

//****************************************************************************
// Private Function(s)
//****************************************************************************

static void exo_refresh_values(void)
{
	//update the motor position
        exo_mot_pos[3] = exo_mot_pos[2];    
        exo_mot_pos[2] = exo_mot_pos[1];
        exo_mot_pos[1] = exo_mot_pos[0];
		
		exo_mot_pos[0] = exo_invert*(int32)encoder.count; //motor position in clicks
        exo_mot_pos[0] = exo_mot_pos[0]-exo_equi_pos; //motor position is relative to the post-calibration zero
		
		//Each board has a different current zero, -18.5 mA/bit
        #ifdef EXOLEFT
            exo_cur = -(ctrl.current.actual_val+46)*185/10;
        #else
            exo_cur = -(ctrl.current.actual_val+25)*185/10;
        #endif
		
		exo_angvel = -imu.gyro.x; //Units? (known that raw value / 16.4 gives degree per second)
        
		//filtering the motor position
        int64 tmp_filpos = 0;
        tmp_filpos = butt_filter(exo_mot_pos,exo_mot_pos_fil);
        
        exo_mot_pos_fil[3] = exo_mot_pos_fil[2];    
        exo_mot_pos_fil[2] = exo_mot_pos_fil[1];
        exo_mot_pos_fil[1] = exo_mot_pos_fil[0];
        exo_mot_pos_fil[0] = tmp_filpos; //10^12 
        
		
        //Total string force in [10 x N] (aka decinewtons)
        #ifdef EXOLEFT
            exo_strain = strain_read()*556/1000-18624;
        #else
            //exo_strain = strain_read()*556/1000-18624; //BP1
            exo_strain = strain_read()*475/1000-16163; //BP2
        #endif
        exo_strain = 2*exo_strain-exo_strain_zero; //doubled because sensor calibration only accounts for half force
        
        //LEVERARM is in 10 x mm
        int32 old_ankle_mom;
        old_ankle_mom = ankle_mom;
        ankle_mom = ((LEVERARM/100)*exo_strain)/10; //Applied ankle moment in [100 x Nm] (aka cNm)
        if (ankle_mom<0)
        {ankle_mom = 0;}
        
        //clkrad is in [clicks/rad / 100]
        //ankle_ang accounts for the deflection of the struts (currently disabled)
        ankle_ang = exo_mot_pos[0]*100/clkrad;//-ankle_mom*100/STRUTSTIFF; //rad x 10000
        
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
        
		//power as instantaneous moment times angular veloctiy, summed over whole plantarflexion
        if (ankle_angs[0]>ankle_angs[1])
        {
            exo_pos_pow = exo_pos_pow+(ankle_angs[0]-ankle_angs[1])/10*(ankle_mom+old_ankle_mom)/20; //10000*J
        }
        
        //Motor velocity[RPM] (P0-P1) * 1000s^-1 *60s/min / 2000clicks/rot
        //exo_mot_vel = (exo_mot_vel+3*(exo_mot_pos[0]-exo_mot_pos[1])*30)/4;
        exo_mot_vel = (exo_mot_pos[0]-exo_mot_pos[3])*333*10/clkrad; //ankle velocity in 1000*rad/sec
        exo_mot_vel_fil = (int32)(int64)((exo_mot_pos_fil[0]-exo_mot_pos_fil[3])*333/clkrad/10000000);   

        //Update constants if they are remotely changed                 
        lin_quad_r = ctrl.position.gain.P_KP-100; //since the gain can only be positive
        spring_mag = ctrl.position.gain.P_KI;
        DPPF_cutoff = ctrl.position.gain.P_KD;
        
        ankle_neutral_angle = ctrl.current.gain.I_KP;
        ankle_swing_angle = ctrl.current.gain.I_KI;
        
        
        //battery voltage in millivolts
        //exo_batvolt = (uint32)(((int32)read_analog(5))*815/100-175);    Luke's old voltage calculation, unstable because what is analog 5??
        exo_batvolt = (uint32)safety_cop.v_vb*1942/11+10000;
        
		//increment times after value updates
        exo_time++;
        statetimer++;
        steptime++;
        
        //Update the communication structure
        exo.state = (int8_t)(exo_state);
        exo.time = exo_time; //int32
        exo.shank_angvel = (int16_t)(lastpospow);//(exo_angvel);
        exo.ankle_ang = (int16_t)(ankle_ang);
        exo.ankle_vel = (int16_t)(ankle_vel_fil);
        exo.ankle_mom = (int16_t)(ankle_mom)/10;
        exo.mot_curr = (int16_t)(exo_cur)/10;
        exo.mot_volt = (int16_t)(mot_volt)/10;
        exo.bat_volt = (uint8_t)(exo_batvolt);
        exo.peak_des_mom = (int16_t)(peak_des_mom)/10;
        exo.max_PF_power = (int16_t)(max_PF_power);
        exo.des_mom = (int16_t)(ankle_des_mom)/10;
        exo.pwm = (int16_t)(mot_pwm);
}


//Set the motor voltage to mot_volt
static void exo_set_volt(){   
    int32 voltsign = 1;
    int32 abvolt = mot_volt;
    if (abvolt<0)
    {
        voltsign = -1;
        abvolt = -abvolt;
    }
	
    mot_pwm = voltsign*((abvolt*800)/exo_batvolt+42);
    
	if (mot_pwm>800)
    {
        mot_pwm = 800;
	}
    
    motor_open_speed_1(exo_invert*mot_volt);	
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
	//ankle_ang is in 10000 x rad  
    //ankle_des_mom is in 100 x Nm
	
    if (ankle_ang<maxDFang)
		{maxDFang = ankle_ang;} //for logging purposes
		
	exo_spring(); // get DF-based moment as a start based on spring
	
	#ifdef PARABOLIC

		int32 parab_des_mom = peak_des_mom-4*peak_des_mom*((statetimer-parab_delay)-exo_ppf_time/2)/exo_ppf_time*((statetimer-parab_delay)-exo_ppf_time/2)/exo_ppf_time;

        if (statetimer > parab_delay)
        {
            if (ankle_des_mom < parab_des_mom) 
            {ankle_des_mom = parab_des_mom;}
        }
        
		int32 PF_power = exo_mot_vel/100*ankle_mom/10; // 1000000 x rad/sec x Nm (or, 1000000 x W), will be better filtered and not time sensitive

		if (PF_power > max_PF_power)
		{max_PF_power = PF_power;}

    #else

		int32 posfdbckvel = exo_mot_vel+200; //faster you're walking, more power injected
		
		if (posfdbckvel>0)
		{ankle_des_mom = ankle_des_mom+posfdbckvel*3/4;}
		
		if (ankle_des_mom > max_mom) //if our calculated moment is the greatest we've seen, note that it's the max and use it. otherwise use the max.
		{max_mom = ankle_des_mom;}
		else
		{ankle_des_mom = max_mom;}

    #endif

    set_mot_torque(ankle_des_mom);
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
        
        /*
        if (lastpospow<1200)
        {
            ctrl.position.gain.P_KI += 50;
        }
        else if (lastpospow>1300)
        {
            ctrl.position.gain.P_KI -= 50;
        }
        */
    }
}

static void set_mot_torque(int32 des_mom){
    //des_mom is the desired ankled moment in [100 x Nm] (aka cNm)
    
	// desired moment is hardlimited at 50 Nm
    if (des_mom>5000)
    {des_mom = 5000;}
    
	/* All of this assumes better efficiency than is seen. Can I do better?
	
    // int32 des_current = des_mom*10/exo_trans*36; //mA (Luke's expression for Current
	int32 des_current = des_mom/exo_trans*423; // outputs current in mA, Kt calculated as 0.023629465 Nm/A or 42.32004427 A/Nm or 423 mA/cNm
	//This means currently hard-limited at 15.5 A! Is that safe? Was 11.3 A
	
    //back EMF = ankle_vel_fil*trans rad/sec * 60 sec/min * 1 rot/6.28 rad * 1V / 346 RPM

    mot_volt = des_current*353/1000 + (exo_mot_pos[0]-exo_mot_pos[1])*86*50/100; //Previously .386 ohm, now .353 ohm Resistance
    
    mot_volt = mot_volt+G_torque*(des_mom-ankle_mom);    
	*/
	
	int32 des_current = des_mom*4232/exo_trans/10; // outputs current in mA, Kt of motor is 42.32 A/Nm or 423.2 mA/cNm, divided by transmission ratio

    mot_volt = des_current*353/1000 + (exo_mot_pos_fil[0]-exo_mot_pos_fil[3])/100000000*10; //.353 ohm Resistance, 23.63 mV/radps. just needed SOME velocity term (arbitrary gains)
    
    mot_volt = mot_volt+G_torque*(des_mom-ankle_mom);    
	
	
	
}

static int8 exo_zero(void){
    
    if (statetimer<1000)
    {mot_volt = statetimer*1500/1000;}
    else 
    {mot_volt=1500;}
    
    if (exo_mot_vel_fil>-100 && exo_mot_vel_fil<100)
    {counter = counter+1;}
    else
    {counter = 0;}
    
    if (counter>2000)
    {return 1;}
    else
    {return 0;}
}

static void exo_zero_strain(void){
    if (statetimer<200)
    {mot_volt = -10000;}
    else
    {mot_volt = 2000;}
    
    if (statetimer == 148)
    {exo_strain_zero = exo_strain;}
}

static void exo_swing(void){
    int32 exo_swg_pos, exo_swg_gain;

    //clkrad is in [clicks/rad /100]
    //ankle_swing_angle is in [100 x rad]
    exo_swg_pos = -ankle_swing_angle*clkrad;
    
    if (statetimer<exo_swg_time)
    {
    exo_des_pos = exo_lastpos + statetimer*(exo_swg_pos-exo_lastpos)/(exo_swg_time);
    exo_swg_gain = 10; // in [10 x mV/click] was 100
    }
    else // after swingtime your despos should be at full swgpos anyway
    {
        exo_des_pos = exo_swg_pos;
        exo_swg_gain = 10; //was 50
    } 
    mot_volt = ((exo_des_pos-exo_mot_pos[0])*exo_swg_gain/10);
}

static void exo_cpf(void){
    int32 exo_swg_pos, exo_swg_gain;
    
	//clkrad is in [clicks/rad /100]
    //ankle_neutral_angle is in [10000 x rad]
    exo_swg_pos = ankle_neutral_angle*clkrad/100;
	
    if (statetimer<exo_cpf_time)
    {
    exo_des_pos = exo_lastpos + statetimer*(exo_swg_pos-exo_lastpos)/(exo_cpf_time);
	exo_swg_gain = 10; // in [10 x mV/click] was 200
    }
    else
    {
        exo_des_pos = exo_swg_pos;
		exo_swg_gain = 10; //was 50
    }
    mot_volt = ((exo_des_pos-exo_mot_pos[0])*exo_swg_gain/10);
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
