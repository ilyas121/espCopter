#include "MotorController.h"

MotorController::MotorController(Servo* motor){
	motors = motor;
}


void MotorController::attachControllers(double* arr){
	controlSig = arr;
}

void MotorController::loop(){
	motors[0].writeMicroseconds(controlSig[0]);
	motors[1].writeMicroseconds(controlSig[1]);
	motors[2].writeMicroseconds(controlSig[2]);
	motors[3].writeMicroseconds(controlSig[3]);
}

