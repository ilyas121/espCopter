#ifndef IMU_H

#define IMU_H

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

class Imu{
private:
	Adafruit_BNO055 * bno;
	volatile float imuReadings[12];
	bool started;
	int updateIndex=0;
public:
	void loop();
	void startSensor(Adafruit_BNO055 * _bno);
	void print();
	void getData(double* values);
	void getCalibrationStatus(uint8_t& system, uint8_t& gyro, uint8_t& accel, uint8_t& mag);
	uint8_t getSystemCalibration();
	uint8_t getGyroCalibration();
	uint8_t getAccelCalibration();
	uint8_t getMagCalibration();
};


#endif /* LIBRARIES_MEDIUMKATFIRMWARE_SRC_GETIMU_H_ */
