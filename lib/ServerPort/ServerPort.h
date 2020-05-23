#ifndef SERVER_PORT_H
#define SERVER_PORT_H

#include <Arduino.h>
#include "config.h"
#include <AsyncUDP.h>
class ServerPort {
public:
	AsyncUDP udp;
	void setup();
	void loop();
	void setPort();
	void sendData();

};

#endif

