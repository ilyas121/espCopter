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
	double* controlSig;
	//Motor objects
	MotorController(Servo* motors);
	//Pid values
	// Pulse the loop function from the main thread
	void loop();
	void attachControllers(double* arr);
};

#endif

