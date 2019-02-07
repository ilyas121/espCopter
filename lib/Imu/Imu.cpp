
#include "Imu.h"

void Imu::loop() {
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

void Imu::getData(float* values){
	for(int i = 0; i < 12; i++){
		values[i] = imuReadings[i];
	}
}

void Imu::print() {
	if (!started)
		return;
	Serial.print(
			""
//			+"\r\n Acceleration= "
//			+String(imuReadings[0])+" , "
//			+String(imuReadings[1])+" , "
//			+String(imuReadings[2])+"\r\n Gyro= "
//			+String(imuReadings[3])+" , "
//			+String(imuReadings[4])+" , "
//			+String(imuReadings[5])+"\r\n Gravity= "
//			+String(imuReadings[6])+" , "
//			+String(imuReadings[7])+" , "
//			+String(imuReadings[8])+
					"\r\n Euler= " + String(imuReadings[9]) + " , "
					+ String(imuReadings[10]) + " , "
					+ String(imuReadings[11]) + "\r\n ");
	/* Display calibration status for each sensor. */
	/*
	 uint8_t system, gyro, accel, mag = 0;
	 bno->getCalibration(&system, &gyro, &accel, &mag);
	 Serial.print("\r\n CALIBRATION: Sys=");
	 Serial.print(system, DEC);
	 Serial.print(" Gyro=");
	 Serial.print(gyro, DEC);
	 Serial.print(" Accel=");
	 Serial.print(accel, DEC);
	 Serial.print(" Mag=");
	 Serial.println(mag, DEC);
	 */
}
