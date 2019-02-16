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
class Drone {
private:
	Imu* sensor;
	Adafruit_BNO055 bno;
        MotorController* controller;	
        double started = false;

	double imuX, imuY, imuZ;
	double controlX, controlY, controlZ;
	double setX, setY, setZ;
	double Kp = 2, Ki = 5, Kd = 1;
	double yKp = 2, yKi = 5, yKd = 1;
	
	//PID yPid(&imuY, &controlY, &setY, Kp, Ki, Kd, DIRECT);
	//PID zPid(&imuZ, &controlZ, &setZ, yKp, yKi, yKd, DIRECT);

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
