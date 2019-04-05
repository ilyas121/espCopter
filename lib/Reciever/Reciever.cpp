#include "Reciever.h"
Reciever::Reciever( double** vals){
	values = vals;
}

void Reciever::getData(double* buffer){ 
	buffer[0] = *values[0];
	buffer[1] = *values[1];
	buffer[2] = *values[2];
	buffer[3] = *values[3];
	buffer[4] = *values[4];
	buffer[5] = *values[5];
}

void Reciever::print(){
	Serial.print("Value 1: ");
	Serial.println(*values[0]);
	Serial.print("Value 2: ");
	Serial.println(*values[1]);
	Serial.print("Value 3: ");
	Serial.println(*values[2]);
	Serial.print("Value 4: ");
	Serial.println(*values[3]);
	Serial.print("Value 5: ");
	Serial.println(*values[4]);
	Serial.print("Value 6: ");
	Serial.println(*values[5]);
}


