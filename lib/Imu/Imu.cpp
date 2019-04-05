
#include "Imu.h"

void Imu::loop() {
	Serial.println("IMU LOOP");
	if (!started)
		return;
	imu::Vector<3> a;
	imu::Vector<3> v;
	imu::Vector<3> g;
	imu::Vector<3> e;

	
	switch (updateIndex) {
	case (0):
		a = bno->getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
		imuReadings[0] = a.z();
		imuReadings[1] = a.y();
		imuReadings[2] = a.x();
		break;
	case (1):
		v = bno->getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
		imuReadings[3] = v.z();
		imuReadings[4] = v.y();
		imuReadings[5] = v.x();
		break;
	case 2:
		g = bno->getVector(Adafruit_BNO055::VECTOR_GRAVITY);
		imuReadings[6] = g.z();
		imuReadings[7] = g.y();
		imuReadings[8] = g.x();
		break;
	case 3:
		e = bno->getVector(Adafruit_BNO055::VECTOR_EULER);
		imuReadings[9] = e.z();// tilt
		imuReadings[10] = e.y();// elevation
		imuReadings[11] = e.x();// azimuth

	}
	updateIndex++;
	if (updateIndex == 4) {
		updateIndex = 0;
	}
	print();
}

void Imu::startSensor(Adafruit_BNO055 * _bno) {
	bno = _bno;
	started = true;

}

void Imu::getData(double* values){
	for(int i = 0; i < 12; i++){
		values[i] = imuReadings[i];
	}
}

void Imu::print() {
	if (!started)
		return;
}
