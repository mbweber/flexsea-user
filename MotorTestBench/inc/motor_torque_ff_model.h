#ifndef MOTOR_TORQUE_FF_MODEL_H
#define MOTOR_TORQUE_FF_MODEL_H

#include "stdint.h"	
	
inline int32_t computeThetaDot(int32_t theta, int32_t theta_prev1, int32_t theta_prev2)
{
	//Second order error
	return (3*theta - 4*theta_prev1 + theta_prev2)/2;
}

inline int32_t computeThetaDoubleDot(int32_t theta, int32_t theta_prev1, int32_t theta_prev2)
{
	//First order error
	return (theta - 2*theta_prev1 + theta_prev2);
}


// k = 0.12
// R = 0.186

/* 	Computes the ideal voltage to apply to the motor in order to achieve the given torque
	considers the given 
*/
inline int32_t torqueToVoltage(int32_t torque, int32_t theta_dot, int32_t theta_double_dot) 
{
	//Goal is to return
	// V = k/R * T + k * theta_dot + J*R/k * theta_double_dot
	// except that would be for SI units.
	// T is in milliNewton metres
	// theta_dot is clicks per millisecond
	// Voltage will be a linear map -1024 to 1024 => -1 to 1 Volts

	// note that we expect theta_dot to usually be pretty small
	// almost def always less than 100

	//below is an approximation after all unit conversions and stuff
	return (40*torque + 152*theta_dot + 118*theta_double_dot)/25;
}


#endif // MOTOR_TORQUE_FF_MODEL_H