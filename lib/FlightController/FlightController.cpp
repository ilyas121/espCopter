#include "FlightController.h"

FlightController::FlightController(Reciever* inputs, Servo* motor){
	rc = inputs;
	motors = motor;
}

void FlightController::loop(){
	Serial.print("Value: ");
	double values[4];
	rc->getData(values);
	Serial.println(values[1]);
	motors[0].writeMicroseconds(values[1]);
	motors[1].writeMicroseconds(values[1]);
	motors[2].writeMicroseconds(values[1]);
	motors[3].writeMicroseconds(values[1]);
}



