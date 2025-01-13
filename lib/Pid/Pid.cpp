#include "Pid.h"

DPID::DPID(double* in, double* out, double kp, double ki, double kd){
	//Ignoring ki for now
	k[0] = kp;
	k[1] = ki;
	k[2] = kd;
	
	kiError = 0;
	input = in;
	output = out;
}

void DPID::setSetpoint(double newSet){
	setpoint = newSet;
}

void DPID::calculate(){
	double error = *input - setpoint;
    if(error > 180){
        error -= 360;
    }
    else if(error < -180){
        error += 360;
    }

	currentTimeStamp = micros();
	timeElapsed = currentTimeStamp - lastTimeStamp;
	lastTimeStamp = currentTimeStamp;

	double tempOutput = (k[0] * error) + kiError + (k[2] * ((error - lastError) / timeElapsed));

	if(tempOutput > max){
		tempOutput = max;
	}
	else if(tempOutput < max * -1){
		tempOutput = (max * -1);
	}	

	*output = tempOutput;
	lastError = error;
}


