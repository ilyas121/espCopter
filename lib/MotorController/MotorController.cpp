#include "MotorController.h"

MotorController::MotorController(Reciever* inputs, Servo* motor){
	rc = inputs;
	motors = motor;
}


void MotorController::loop(){
	Serial.println("Getting RC Values");
	double values[] = {2000, 2000, 2000, 2000};
	Serial.println("Getting Values");
	//rc->getData(values);
	Serial.println("IN THE LOOP1");
	//frontRight.writeMicroseconds(1000);
	Serial.println("IN THE LOOP2");
	this->motors[0].writeMicroseconds(2000);
	Serial.println("IN THE LOOP3");
	//bottomLeft.writeMicroseconds(values[2]);
	Serial.println("IN THE LOOP4");
	//bottomRight.writeMicroseconds(values[3]);
}
