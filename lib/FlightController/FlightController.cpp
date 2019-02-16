#include "FlightController.h"

FlightController::FlightController(Reciever* inputs, Servo* motor){
	rc = inputs;
	frontRight = motor[0];
	frontLeft = motor[1];
	bottomLeft = motor[2];
	bottomRight = motor[3];
}

void FlightController::loop(){
	Serial.print("Value: ");
	double values[4];
	rc->getData(values);
	Serial.println(values[1]);
	frontRight.write(values[1]);
}



