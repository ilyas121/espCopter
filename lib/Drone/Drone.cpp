#include "Drone.h"

void Drone::loop() {
	sensor->loop();
	Serial.println("Getting Sensor Data");
	sensor->print();
	Serial.println("Printing Controls Data");
	fastLoop();  
	delay(500);
}

Drone::Drone(MotorController* mc){ 
	sensor = NULL;
}

void Drone::setup() {
      
	sensor = new Imu();
	/* Initialise the sensor */
	if (!bno.begin()) {
		/* There was a problem detecting the BNO055 ... check your connections */
		Serial.print(
				"Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
		delay(1000);
		while (1)
			;
	}

	delay(1000);
	bno.setExtCrystalUse(true);
	sensor->startSensor(&bno);
}

void Drone::fastLoop() {
	controller->loop();
}
