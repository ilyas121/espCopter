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

	//clock trackers 
	int lastImuUpdate = 0;

	double rcValues[6];
	double imuValues[12];
	double velControlX, velControlY, velControlZ;
	double output[4] = {0, 0, 0, 0};
	double xK[3] = {0.0, 0.0, 0.0}; //Yaw
	double yK[3] = {24.0, 0.0, 0.0}; //Roll
	double zK[3] = {24.0, 0.0, 0.0}; //Pitch
    double velSetpoints[3] = {0, 0, 0};	
	DPID* velControllers[3];
	DPID* posControllers[3];

	void printIO();
	// This should be run every loop and is internally gated for fast opperation
	void fastLoop();
	// Internal setup function. set up all objects
	void setup();
	//This function is to update xK, yK, zK gains
public:
	Drone(MotorController* mc, Reciever* r);
	// Pulse the loop function from the main thread
	void loop();
	void updateGain(double* gains);
	void printGains();
};

#endif
