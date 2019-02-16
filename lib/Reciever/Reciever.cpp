#include "Reciever.h"
Reciever::Reciever( double* leftH,  double* leftV,  double* rightH,  double* rightV){
	leftHorizontal = leftH;
	leftVertical = leftV;
	rightHorizontal = rightH;
	rightVertical = rightV;
}

void Reciever::getData(double* buffer){ 
	Serial.println("GETTING DATA");
	buffer[0] = *leftHorizontal;
	Serial.println("GETTING DATA");
	buffer[1] = *leftVertical;
	Serial.println("GETTING DATA");
	buffer[2] = *rightHorizontal;
	Serial.println("GETTING DATA");
	buffer[3] = *rightVertical;
	Serial.println("Values Set");
}

void Reciever::print(){
	Serial.println(*leftHorizontal);
}


