#include "Pid.h"

VPID::VPID(double* in, double* out, double kp, double ki, double kd){
	k[0] = kp;
	k[1] = ki;
	k[2] = kd;
	
	kiError = 0;
	input = in;
	output = out;
}


void VPID::setSetpoint(double newSet){
	setpoint = newSet;
}

void VPID::calculate(){
	double error = *input - setpoint;
	if(error <= 0.19 && error >= -0.19){
		error = 0;
	}
	kiError += k[1] * error;
	if(kiError > max)kiError = max;
	else if(kiError < max * -1)kiError = min * -1;
  
	double tempOutput = k[0] * error + kiError + k[2] * (error - lastError);
        if(tempOutput > max)tempOutput = max;
        else if(tempOutput < max * -1)tempOutput = tempOutput * -1;
	*output = tempOutput;
	lastError = error;
}


