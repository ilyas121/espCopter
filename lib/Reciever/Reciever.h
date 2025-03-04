#ifndef RECIEVER_H
#define RECIEVER_H

#include "Arduino.h"

static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

struct RCChannel{
	uint8_t channelPin;
	volatile unsigned long lastRisingEdge;
	volatile double pulseWidth;
};

class Reciever {
	//ISR Globals
	RCChannel channels[6];
	static Reciever* instance;
	static void handleInterrupt(void* arg);

	public:
		Reciever(); 
		void getData(double* buffer);
		void setup();
		void print();
};

#endif
