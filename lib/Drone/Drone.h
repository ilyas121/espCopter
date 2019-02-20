#ifndef DRONE_H
#define DRONE_H
#if defined(Arduino_h)
#include <Arduino.h>
#endif
#include "config.h"
#include <ESP32Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "MotorController.h"
#include "Imu.h"
#include "Pid.h"
#include "Reciever.h"
class Drone {
private:
	Imu* sensor;
	Adafruit_BNO055 bno;
        MotorController* controller;	
	Reciever* rc; 
        double started = false;

	double imuValues[12];
	double controlX, controlY, controlZ;
	double setX, setY, setZ;
	double xK[3] = {2, 5, 1};
	double yK[3] = {2, 5, 1};
	double zK[3] = {2, 5, 1};
	
	VPID* pidControllers[3];

	void printAll();
	// This should be run every loop and is internally gated for fast opperation
	void fastLoop();
	// Internal setup function. set up all objects
	void setup();
public:
	Drone(MotorController* mc, Reciever* r);
	// Pulse the loop function from the main thread
	void loop();
};

#endif
