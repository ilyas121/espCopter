#ifndef PID_COMM_H
#define PID_COMM_H

#include <Arduino.h>
#include "config.h"
#include <AsyncUDP.h>
#include "Drone.h"
class PIDComm {

public:
	void setup();
	void loop();
	void sendData();
	void tuneConstants(double* values);
	PIDComm(int p, Drone* drone);

private:
	AsyncUDP udp;
	Drone* pidcontroller;
	int port;
};

#endif

