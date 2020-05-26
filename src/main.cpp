#include "Arduino.h"
#include <ESP32Servo.h>
#include "config.h"
#include "Secrets.h"
#include "Reciever.h"
#include "MotorController.h"
#include "Imu.h"
#include "Drone.h"
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include "ArduinoOTA.h"
#include <WiFiClient.h>
#include <WebServer.h>
#include <Update.h>
#include "PIDComm.h"

// // ---------------------------------------------------------------------------
// //Globals

enum State
{
   ArmESC,
   Idle,
   HomeAllSwitches,
   StartMission
};

State droneState = HomeAllSwitches;
State pastState = HomeAllSwitches;


double pastRh = 0;
double valueRh = 0;

double pastRv = 0;
double valueRv = 0;

double pastLv = 0;
double valueLv = 0;

double pastLh = 0;
double valueLh = 0;

double pastKl = 0;
double valueKl = 0;

double pastKr = 0;
double valueKr = 0;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

double* values[6] = {&valueLh, &valueLv, &valueRh, &valueRv, &valueKl, &valueKr};

//Reciever/comms
Reciever* rc = new Reciever(values);
//Motor system controls
MotorController* flight;
Drone* drone;
PIDComm* com;
int startup = 0;
Servo motA, motB, motC, motD;
Servo motors[4]; 
// ---------------------------------------------------------------------------


// -------------------------------------------------------------------------
// ISR Routines
void changeRh(){
  portENTER_CRITICAL_ISR(&mux);
  double temp = esp_timer_get_time()-pastRh;
  if(digitalRead(25) == LOW){
	if(temp > 950 && temp < 2100){
		valueRh = temp;
  	}
  }else{
	pastRh = esp_timer_get_time();	
  }
  portEXIT_CRITICAL_ISR(&mux);
}
void changeRv(){
  portENTER_CRITICAL_ISR(&mux);
  if(digitalRead(26) == LOW){
  	double temp = micros()-pastRv;
  	if(temp > 950 && temp < 2100){
		  valueRv = temp;
  	}else{
      valueRv = temp;
  	}
  }
  else{
  	pastRv = micros();
  }
  portEXIT_CRITICAL_ISR(&mux);
}
void changeLh(){
  portENTER_CRITICAL_ISR(&mux);
  double temp = esp_timer_get_time() - pastLh;
  if(digitalRead(27) == LOW){
    if(temp > 950 && temp < 2100){
	    valueLh = temp;
    }
  }
  else{
     pastLh = esp_timer_get_time();
  }
  portEXIT_CRITICAL_ISR(&mux);
}
void changeLv(){
  portENTER_CRITICAL_ISR(&mux);
  double temp = esp_timer_get_time() - pastLv;
  if(digitalRead(14) == LOW){
    if(temp > 950 && temp < 2100){
	    valueLv = temp;
    }
  }else{
    pastLv = esp_timer_get_time();
  }
  portEXIT_CRITICAL_ISR(&mux);
}

void changeKl(){
  portENTER_CRITICAL_ISR(&mux);
  double temp = esp_timer_get_time() - pastKl;
  if(digitalRead(34) == LOW){
    if(temp > 950 && temp < 2100){
	    valueKl = temp;
    }
  }else{
    pastKl = esp_timer_get_time();
  }
  portEXIT_CRITICAL_ISR(&mux);
}

void changeKr(){
  portENTER_CRITICAL_ISR(&mux);
  double temp = esp_timer_get_time() - pastKr;
  if(digitalRead(35) == LOW){
    if(temp > 950 && temp < 2100){
	    valueKr = temp;
    }
  }else{
    pastKr = esp_timer_get_time();
  }
  portEXIT_CRITICAL_ISR(&mux);
}
//---------------------------------------------------------------------------

/**
 * Start serial, attach motors, and display the instructions and format
 */
void setup() {
    Serial.begin(115200);
    Serial.println("Booting");
    WiFi.mode(WIFI_STA);
    WiFi.begin(MY_SSID, MY_PASSWORD);
    while (WiFi.waitForConnectResult() != WL_CONNECTED) {
      Serial.println("Connection Failed! Rebooting...");
      delay(5000);
      ESP.restart();
    }

    ArduinoOTA
      .onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
          type = "sketch";
        else // U_SPIFFS
          type = "filesystem";

        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
        Serial.println("Start updating " + type);
      })
      .onEnd([]() {
        Serial.println("\nEnd");
      })
      .onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
      })
      .onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
      });

    ArduinoOTA.begin();

    Serial.println("Ready");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    //////////////////////////
    ///Setup motors
    motA.attach(UPPER_LEFT_MOTOR, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    motB.attach(UPPER_RIGHT_MOTOR, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    motC.attach(LOWER_LEFT_MOTOR, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    motD.attach(LOWER_RIGHT_MOTOR, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
   
    motors[0]= motA;
    motors[1]= motB;
    motors[2]= motC;
    motors[3]= motD;
    flight = new MotorController(motors);
    drone  = new Drone(flight, rc);
    //Attach interrupts for the reciever
    pinMode(25, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(25), changeRh, CHANGE);
    pinMode(26, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(26), changeRv, CHANGE);
    pinMode(27, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(27), changeLh, CHANGE);
    pinMode(14, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(14), changeLv, CHANGE);
    pinMode(34, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(34), changeKl, CHANGE);
    pinMode(35, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(35), changeKr, CHANGE);

    droneState = HomeAllSwitches;
    com = new PIDComm(1234, drone);
}


/**
 * Loop: Read input and execute instruction
 */
void loop() {
    ArduinoOTA.handle();
    switch(droneState){
        case(HomeAllSwitches):
            if(valueKl < 1400 || valueKr < 1400){
                Serial.println("Please home all switches up");
                Serial.println("ValueKl: " + String(valueKl) + "ValueKr" + String(valueKr));
		delay(100);
            }
            else{
                Serial.println("SWITCHES HOMED");
                switch(pastState){
                    case(HomeAllSwitches):
                        droneState = ArmESC;
                        pastState = HomeAllSwitches;
                        break;
                    case(ArmESC):
                        droneState = StartMission;
                        pastState = HomeAllSwitches;
                        break;
                    case(StartMission):
                        break;
                    default:
                        while(1){
                            Serial.println("Don goofed boi, check HomeAllSwitches state machine");
                            delay(100);
                        }
                        break;
                }
            }
            break;
        case(ArmESC):
            Serial.println("ARM ESC");
            if(valueKl < 1400){
                droneState = StartMission;
                pastState = ArmESC;
            }else if(valueKr < 1400){
                droneState = HomeAllSwitches;
                pastState = ArmESC;
                //Start up ESC's 
                motors[0].writeMicroseconds(2000); 
                motors[1].writeMicroseconds(2000); 
                motors[2].writeMicroseconds(2000); 
                motors[3].writeMicroseconds(2000); 
                delay(3000);
                motors[0].writeMicroseconds(1000); 
                motors[1].writeMicroseconds(1000); 
                motors[2].writeMicroseconds(1000); 
                motors[3].writeMicroseconds(1000); 
                delay(2000);
                Serial.println("Setup completed");
                droneState = StartMission;
            }else{
                Serial.println("Choose whether to arm the esc's or not");
                delay(100);
            }
            break;
        case(StartMission):
            //Serial.println("STARTING MISSION");
            pastState = StartMission;
            drone->loop();
            com->loop();
            break;
    }
    
}