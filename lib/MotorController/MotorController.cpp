#include "MotorController.h"

MotorController::MotorController(Servo* motor){
	motors = motor;
	motors[0].writeMicroseconds(2000);
}


void MotorController::attachControllers(VPID** arr){
	roll = arr[0];
	pitch = arr[1];
	yaw = arr[2]; 
}

void MotorController::loop(){
	Serial.print("Value: ");
	double values[4] = {1500, 1500, 1500, 1500};
	Serial.println(values[1]);
	motors[0].writeMicroseconds(values[1]);
	motors[1].writeMicroseconds(values[1]);
	motors[2].writeMicroseconds(values[1]);
	motors[3].writeMicroseconds(values[1]);
}

