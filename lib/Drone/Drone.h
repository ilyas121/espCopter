#ifndef DRONE_H
#define DRONE_H
#if defined(Arduino_h)
#include <Arduino.h>
#endif
#include "config.h"
#include <PID_v1.h>
#include <ESP32Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "MotorController.h"
#include "Imu.h"

class Drone {
private:
	Imu* sensor;
	Adafruit_BNO055 bno;
        MotorController* controller;	
	void printAll();
	// This should be run every loop and is internally gated for fast opperation
	void fastLoop();
	// Internal setup function. set up all objects
	void setup();
public:
	Drone(MotorController* mc);
	// Pulse the loop function from the main thread
	void loop();
};

#endif
