#include "Imu.h"

void Imu::loop() {
	imu::Vector<3> a;
	imu::Vector<3> v;
	imu::Vector<3> g;
	imu::Vector<3> e;

	a = bno->getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
	imuReadings[0] = a.x();
	imuReadings[1] = a.y();
	imuReadings[2] = a.z();

	v = bno->getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
	imuReadings[3] = v.x();
	imuReadings[4] = v.y();
	imuReadings[5] = v.z();

	g = bno->getVector(Adafruit_BNO055::VECTOR_GRAVITY);
	imuReadings[6] = g.x();
	imuReadings[7] = g.y();
	imuReadings[8] = g.z();

	e = bno->getVector(Adafruit_BNO055::VECTOR_EULER);
	imuReadings[9] = e.x();// tilt
	imuReadings[10] = e.y();// elevation
	imuReadings[11] = e.z();// azimuth
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
	Serial.println("IMU Data");
	for(int i = 0; i < 12; i++){
		Serial.print(imuReadings[i]);
		Serial.print(", ");
	}
	Serial.println();
}

void Imu::getCalibrationStatus(uint8_t& system, uint8_t& gyro, uint8_t& accel, uint8_t& mag) {
	if (started && bno != nullptr) {
		bno->getCalibration(&system, &gyro, &accel, &mag);
	} else {
		system = gyro = accel = mag = 0;
	}
}

uint8_t Imu::getSystemCalibration() {
	uint8_t system, gyro, accel, mag;
	getCalibrationStatus(system, gyro, accel, mag);
	return system;
}

uint8_t Imu::getGyroCalibration() {
	uint8_t system, gyro, accel, mag;
	getCalibrationStatus(system, gyro, accel, mag);
	return gyro;
}

uint8_t Imu::getAccelCalibration() {
	uint8_t system, gyro, accel, mag;
	getCalibrationStatus(system, gyro, accel, mag);
	return accel;
}

uint8_t Imu::getMagCalibration() {
	uint8_t system, gyro, accel, mag;
	getCalibrationStatus(system, gyro, accel, mag);
	return mag;
}
