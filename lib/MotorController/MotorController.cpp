#include "MotorController.h"

MotorController::MotorController(Reciever* inputs, Servo** motor){
	rc = inputs;
	frontRight = *motor[0];
	frontLeft = *motor[1];
	bottomLeft = *motor[2];
	bottomRight = *motor[3];
}

void MotorController::loop(){
	double values[4];
	rc->getData(values);
	Serial.println('IN THE LOOP');
	frontRight.writeMicroseconds(values[0]);
	frontLeft.writeMicroseconds(values[1]);
	bottomLeft.writeMicroseconds(values[2]);
	bottomRight.writeMicroseconds(values[3]);
}
