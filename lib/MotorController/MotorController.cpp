#include "MotorController.h"

MotorController::MotorController(Servo* motor){
	motors = motor;
    lastMotorUpdate = 0;
}


void MotorController::attachControllers(double* arr){
	controlSig = arr;
}

void MotorController::loop(){
    // unsigned long currentMicros = micros();
    
    // if (currentMicros - lastMotorUpdate >= 1000) {
    //     lastMotorUpdate = currentMicros;
        
    if(controlSig[0] > 2000){
        controlSig[0] = 2000;
    }
    if(controlSig[1] > 2000){
        controlSig[1] = 2000;
    }
    if(controlSig[2] > 2000){
        controlSig[2] = 2000;
    }
    if(controlSig[3] > 2000){
        controlSig[3] = 2000;
    }
    motors[0].writeMicroseconds(controlSig[0]);
    motors[1].writeMicroseconds(controlSig[1]);
    motors[2].writeMicroseconds(controlSig[2]);
    motors[3].writeMicroseconds(controlSig[3]);
}

