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
	void getData(float*);
};


#endif /* LIBRARIES_MEDIUMKATFIRMWARE_SRC_GETIMU_H_ */
