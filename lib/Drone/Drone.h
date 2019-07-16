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

	double rcValues[6];
	double imuValues[12];
	double velControlX, velControlY, velControlZ;
	double output[4] = {0, 0, 0, 0};
	double xK[3] = {0, 0, 0};
	double yK[3] = {12, 0.00, 0};
	double zK[3] = {0, 0.00, 0};
    double velSetpoints[3] = {0, 0, 0};	
	DPID* velControllers[3];
	DPID* posControllers[3];

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
