#ifndef PID_COMM_H
#define PID_COMM_H

#include <Arduino.h>
#include "config.h"
#include <AsyncUDP.h>
#include "Pid.h"
class PIDComm {

public:
	void setup();
	void loop();
	void sendData();
	void tuneConstants();

private:
	AsyncUDP udp;
	DPID* pidcontroller;
	int port;
};

#endif

