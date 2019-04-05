#ifndef RECIEVER_H
#define RECIEVER_H

#include "Arduino.h"

class Reciever {
	//ISR Globals
	 double** values;
	public:
		Reciever( double** vals); 
		void getData(double* buffer);
		void print();
};

#endif
