#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>
#include "config.h"
#include <PID_v1.h>
#include <ESP32Servo.h>
#include "Reciever.h"
#include "Adafruit_BNO055.h"
class MotorController {
private:
	// Motor (servo) objects
	Servo frontRight;
	Servo frontLeft;
	Servo bottomLeft;
	Servo bottomRight;
	Servo* motors;
	Reciever* rc;
	// The IMU object
	Adafruit_BNO055 bno;
	// Internal setup function. set up all objects
	
public:
	MotorController(Reciever* inputs, Servo* motors);
	// Pulse the loop function from the main thread
	void loop();
};

#endif

