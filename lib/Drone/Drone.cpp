#include "Drone.h"

void Drone::loop() {
	if(started == true){
	   if(USE_IMU == true){
		sensor->loop();
           }   
	   sensor->getData(imuValues);
	   Serial.println("Velocity: " + String(imuValues[3]) + ", "  + String(imuValues[4]) + ", " +  String(imuValues[5]));
	   fastLoop();  
	}
}

Drone::Drone(MotorController* mc, Reciever* r){ 
	sensor = NULL;
	controller = mc;
	rc = r;
	setup();
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
	pidControllers[0] = new VPID(&imuValues[5], &controlY, yK[0], yK[1], yK[2]);
	pidControllers[1] = new VPID(&imuValues[4], &controlZ, zK[0], zK[1], yK[2]);
	pidControllers[2] = new VPID(&imuValues[4], &controlX, xK[0], xK[1], xK[2]);
}

void Drone::fastLoop() {
	controller->loop();
}
