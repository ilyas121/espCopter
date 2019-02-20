#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>
#include "config.h"
#include <PID_v1.h>
#include <ESP32Servo.h>
#include "Reciever.h"
#include "Adafruit_BNO055.h"
#include "Pid.h"
class MotorController {
public:
	// Motor (servo) objects
	Servo* motors;
	//Motor objects
	MotorController(Servo* motors);
	//Pid values
	double controlX, controlY, controlZ;
	double setX, setY, setZ;
	double xK[3] = {2, 5, 1};
	double yK[3] = {2, 5, 1};
	double zK[3] = {2, 5, 1};
	
	VPID* roll;
	VPID* pitch;
	VPID* yaw;
	// Pulse the loop function from the main thread
	void loop();
	void attachControllers(VPID* arr);
};

#endif

