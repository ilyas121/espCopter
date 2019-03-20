#include "MotorController.h"

MotorController::MotorController(Servo* motor){
	motors = motor;
//	motors[0].writeMicroseconds(2000);
}


void MotorController::attachControllers(VPID** arr){
	roll = arr[0];
	pitch = arr[1];
	yaw = arr[2]; 
}

void MotorController::loop(){
	double output = 0;
	Serial.print("Roll: " + String(*(roll->output)));
	Serial.print("Pitch: " + String(*(pitch->output)));
	Serial.print("Yaw: " + String(*(yaw->output)));
	double values[4] = {1500, 1500, 1500, 1500};
	output = *(roll->output) + 1500;
	motors[0].writeMicroseconds(output);
	motors[1].writeMicroseconds(output);
	motors[3].writeMicroseconds(output);
	motors[4].writeMicroseconds(output);
}

