#include "Pid.h"

VPID::VPID(float* in, float* out, float kp, float ki, float kd){
	k[0] = kp;
	k[1] = ki;
	k[2] = kd;
	
	kiError = 0;
	input = in;
	output = out;
}

void VPID::calculatePid(){
	float error = *input - setpoint;
	kiError += k[1] * error;
	if(kiError > max)kiError = max;
	else if(kiError < max * -1)kiError = min * -1;
  
	float tempOutput = k[0] * error + kiError + k[2] * (error - lastError);
        if(tempOutput > max)tempOutput = max;
        else if(tempOutput < max * -1)tempOutput = tempOutput * -1;
	*output = tempOutput;
	lastError = error;
}


