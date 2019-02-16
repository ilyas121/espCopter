#include "Drone.h"

void Drone::loop() {
	if(started == true){
	   if(USE_IMU == true){
		sensor->loop();
		Serial.println("Getting Sensor Data");
		sensor->print();
           }   
		Serial.println("Printing Controls Data");
		fastLoop();  
	}else{
	   started = true;
	   setup();
	}
}

Drone::Drone(MotorController* mc){ 
	sensor = NULL;
}

void Drone::setup() {
        if(USE_IMU == true){ 
	sensor = new Imu();
	/* Initialise the sensor */
	Serial.println("Setting up drone and sensors");	
	if (!bno.begin()) {
		/* There was a problem detecting the BNO055 ... check your connections */
		Serial.print(
				"Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
		delay(1000);
		while (1)
			;
	}else{
		Serial.println("BNO STARTED");
	}

	delay(1000);
	bno.setExtCrystalUse(true);
	sensor->startSensor(&bno);
	}
	Serial.println("IMU IS DONE");
}

void Drone::fastLoop() {
	Serial.println("Starting Fast Loop");
	//`controller->loop();
	Serial.println("End of fast loop"); 
}
