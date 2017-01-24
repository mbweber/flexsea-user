// k = 0.12
// R = 0.186


/* 	Computes the ideal voltage to apply to the motor in order to achieve the given torque
	considers the given 

*/
int32_t torqueToVoltage(int32_t torque, int32_t theta_dot) 
{
	//Goal is to return
	// V = k/R * T + k * theta_dot
	// except that would be for SI units.
	// T is in milliNewton metres
	// theta_dot is clicks per millisecond
	// Voltage will be a linear map -1024 to 1024 => -24 to 24 Volts

	// so 24/1024 * V = k / (R * 1000) * T + k * theta_dot * 1000 * 2pi / 16384
	// which yields: V = 0.027527T + 0.3125 * 2pi * theta_dot
	// which is approx V = 11/400 T + 125*2*pi/400 theta_dot
	// which is approx V = 11/400 T + 785/400 theta_dot
	
	// note that we expect theta_dot to usually be pretty small
	// almost def always less than 100

	return (11*torque + 785*theta_dot) / 400;
}