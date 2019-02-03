#include "Reciever.h"
Reciever::Reciever( double* leftH,  double* leftV,  double* rightH,  double* rightV){
	leftHorizontal = leftH;
	leftVertical = leftV;
	rightHorizontal = rightH;
	rightVertical = rightV;
}

void Reciever::getData(double* buffer){ 
	buffer[0] = *leftHorizontal;
	buffer[1] = *leftVertical;
	buffer[2] = *rightHorizontal;
	buffer[3] = *rightVertical;
}

void Reciever::print(){
	Serial.println(*leftHorizontal);
}


