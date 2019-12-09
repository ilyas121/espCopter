#include "Drone.h"

void Drone::loop() {
	if(started == true){
	   if(USE_IMU == true){
		sensor->loop();
           }   
	   sensor->getData(imuValues);
	   Serial.println("Euler: " + String(imuValues[9]) + ", "  + String(imuValues[10]) + ", " +  String(imuValues[11]));
	   delay(100);
	   rc->getData(rcValues);
	   //rc->print();
	   fastLoop();  
	}
        delay(100);
	//Serial.println("Pid Z Output: " + String(velControlZ));
	Serial.println("Pid Y Output: " + String(velControlY));
    delay(100);
	//Serial.println("Pid X Output: " + String(velControlX));
}

Drone::Drone(MotorController* mc, Reciever* r){ 
	sensor = NULL;
	controller = mc;
	rc = r;
	setup();
}

void Drone::setup() {
        if(USE_IMU == true){ 
	sensor = new Imu();
	/* Initialise the sensor */
	Serial.println("Setting up drone and sensors");	
	if (!bno.begin()) {
		/* There was a problem detecting the BNO055 ... check your connections */
		Serial.print(
				"Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
		delay(1000);
		while (1)
			;
	}else{
		Serial.println("BNO STARTED");
	}

	delay(1000);
	bno.setExtCrystalUse(true);
	sensor->startSensor(&bno);
	}
	started = true;

	velControlX = 0;
	velControlY = 0;
	velControlZ = 0;

	//Roll = 1 Pitch = 2 Yaw = 0
	velControllers[0] = new DPID(&imuValues[9], &velControlX, xK[0], xK[1], xK[2]);
	velControllers[1] = new DPID(&imuValues[10], &velControlY, yK[0], yK[1], yK[2]);
	velControllers[2] = new DPID(&imuValues[11], &velControlZ, zK[0], zK[1], yK[2]);
	
	posControllers[0] = new DPID(&imuValues[9], &velSetpoints[0], xK[0], xK[1], xK[2]);
	posControllers[1] = new DPID(&imuValues[10], &velSetpoints[1], yK[0], yK[1], yK[2]);
	posControllers[2] = new DPID(&imuValues[11], &velSetpoints[2], zK[0], zK[1], yK[2]);


	
	controller->attachControllers(output);
}

void Drone::fastLoop() { 
//	Serial.println("Getting Data");
	rc->getData(rcValues);
/**	Serial.println("Done getting data");
	Serial.print("Values: ");
	Serial.print(rcValues[0]);
	Serial.print(", ");
	Serial.print(rcValues[1]);
	Serial.print(", ");
	Serial.print(rcValues[2]);
	Serial.print(", ");
	Serial.print(rcValues[3]);
	Serial.println(".");**/
	//Yaw = 0 - Roll = 2 - Pitch = 3  
	rcValues[0] -= 1500;
	rcValues[2] -= 1500;
	rcValues[3] -= 1500;

	rcValues[0] /= 3;
	rcValues[2] /= 3;
	rcValues[3] /= 3; 
/**	
	//Calculating required velocity
	posControllers[0]->calculate();
	posControllers[1]->calculate();
	posControllers[2]->calculate();
**/
	//Set the required velocity
	velControllers[0]->setSetpoint(velSetpoints[0]);
	velControllers[1]->setSetpoint(velSetpoints[1]);
	velControllers[2]->setSetpoint(velSetpoints[2]);

	//Calculating required output signal (thrust added to base throttle) needed 
	velControllers[0]->calculate();
	velControllers[1]->calculate();
	velControllers[2]->calculate();
	if(rcValues[1] < 1030){
        output[0] = 1000;
        output[1] = 1000;
        output[2] = 1000;
        output[3] = 1000;
    }else{
        output[0] = rcValues[1] - velControlY; 
        output[1] = rcValues[1] + velControlY; 
        output[2] = rcValues[1] - velControlY; 
        output[3] = rcValues[1] + velControlY;
    }
	controller->loop();
   /** 
	output[0] = rcValues[1] - velControlY - velControlZ + velControlX; 
	output[1] = rcValues[1] + velControlY - velControlZ - velControlX; 
	output[2] = rcValues[1] - velControlY + velControlZ - velControlX; 
	output[3] = rcValues[1] + velControlY + velControlZ + velControlX; 

	output[0] = rcValues[1] + velControlY + velControlZ - velControlX; //Calculate the pulse for esc 4 (front-left - CW)
	output[1] = rcValues[1] + velControlY - velControlZ + velControlX; //Calculate the pulse for esc 1 (front-right - CCW)
	output[2] = rcValues[1] - velControlY + velControlZ + velControlX; //Calculate the pulse for esc 3 (rear-left - CCW)
	output[3] = rcValues[1] - velControlY - velControlZ - velControlX; //Calculate the pulse for esc 2 (rear-right - CW)
	
    output[0] = rcValues[1] - pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)
	output[1] = rcValues[1] - pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW)
	output[2] = rcValues[1] + pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
	output[3] = rcValues[1] + pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)**/
}
