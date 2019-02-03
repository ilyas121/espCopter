#ifndef RECIEVER_H
#define RECIEVER_H

#include "Arduino.h"

class Reciever {
	//ISR Globals
	 double* leftHorizontal;
	 double* rightHorizontal;
	 double* rightVertical;
	 double* leftVertical;
	public:
		Reciever( double* leftH,  double* leftV,  double* rightH,  double* rightV); 
		void getData(double* buffer);
		void print();
};

#endif
