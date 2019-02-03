#include <ESP32Servo.h>
#include "config.h"
// ---------------------------------------------------------------------------
Servo motA, motB, motC, motD;
String data;
volatile int testCounter = 0;
volatile int testCounter2 = 0;
//ISR Globals
volatile double rh = 0;
volatile double pastRh = 0;
volatile double currentRh = 0;
volatile double valueRh = 0;

volatile double rv = 0;
volatile double pastRv = 0;
volatile double timeRv = 0;
volatile double valueRv = 0;

volatile double lv = 0;
volatile double pastLv = 0;
volatile double timeLv = 0;
volatile double valueLv = 0;

volatile double lh = 0;
volatile double pastLh = 0;
volatile double timeLh = 0;
volatile double valueLh = 0;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE mux2 = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE mux3 = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE mux4 = portMUX_INITIALIZER_UNLOCKED;
// ---------------------------------------------------------------------------

void IRAM_ATTR fallingRh(){
  portENTER_CRITICAL_ISR(&mux2);
  double temp = esp_timer_get_time()-pastRh;
  if(digitalRead(25) == LOW){
	if(temp > 950 && temp < 2100){
		valueRh = temp;
  	}
  }else{
	pastRh = esp_timer_get_time();	
  }
  portEXIT_CRITICAL_ISR(&mux2);
}
void IRAM_ATTR risingRh(){
  portENTER_CRITICAL_ISR(&mux2);
  pastRh = esp_timer_get_time();
  portEXIT_CRITICAL_ISR(&mux2);
}
void IRAM_ATTR fallingRv(){
  testCounter++;
  portENTER_CRITICAL_ISR(&mux2);
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
  portEXIT_CRITICAL_ISR(&mux2);
}
void IRAM_ATTR risingRv(){
  portENTER_CRITICAL_ISR(&mux2);
  pastRv = esp_timer_get_time();
  portEXIT_CRITICAL_ISR(&mux2);
}
void IRAM_ATTR fallingLh(){
  portENTER_CRITICAL_ISR(&mux2);
  double temp = esp_timer_get_time() - pastLh;
  if(digitalRead(27) == LOW){
    if(temp > 950 && temp < 2100){
	    valueLh = temp;
    }
  }
  else{
     pastLh = esp_timer_get_time();
  }
  portEXIT_CRITICAL_ISR(&mux2);
}
void IRAM_ATTR risingLh(){
  portENTER_CRITICAL_ISR(&mux2);
  pastLh = esp_timer_get_time();
  portEXIT_CRITICAL_ISR(&mux2);
}
void IRAM_ATTR fallingLv(){
  portENTER_CRITICAL_ISR(&mux2);
  double temp = esp_timer_get_time() - pastLv;
  if(digitalRead(14) == LOW){
    if(temp > 950 && temp < 2100){
	    valueLv = temp;
    }
  }else{
    pastLv = esp_timer_get_time();
  }
  portEXIT_CRITICAL_ISR(&mux2);
}
void IRAM_ATTR risingLv(){
  portENTER_CRITICAL_ISR(&mux2);
  pastLv = esp_timer_get_time();
  portEXIT_CRITICAL_ISR(&mux2);
}



/**
 * Start serial, attach motors, and display the instructions and format
 */
void setup() {
    Serial.begin(115200);
    motA.attach(UPPER_LEFT_MOTOR, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    motB.attach(UPPER_RIGHT_MOTOR, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    motC.attach(LOWER_LEFT_MOTOR, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    motD.attach(LOWER_RIGHT_MOTOR, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    giveInstructions();
    //25 26 27 14
    pinMode(25, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(25), fallingRh, CHANGE);


    pinMode(26, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(26), fallingRv, CHANGE);

    pinMode(27, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(27), fallingLh, CHANGE);
    //attachInterrupt(digitalPinToInterrupt(27), risingLh, RISING);

    pinMode(14, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(14), fallingLv, CHANGE);
    //attachInterrupt(digitalPinToInterrupt(14), risingLh, RISING);

    motA.writeMicroseconds(2000); 
    motB.writeMicroseconds(2000); 
    motC.writeMicroseconds(2000); 
    motD.writeMicroseconds(2000); 
    delay(2000);
    motA.writeMicroseconds(1000); 
    motB.writeMicroseconds(1000); 
    motC.writeMicroseconds(1000); 
    motD.writeMicroseconds(1000); 
    delay(1000);
}


/**
 * Loop: Read input and execute instruction
 */
void loop() {
    if (Serial.available()) {
        data = Serial.readString();
        char motor = data.charAt(0);
        int time = data.substring(2).toInt();
        if(motor == 'a'){
            motA.writeMicroseconds(time);    
        }else if(motor == 'b'){
            motB.writeMicroseconds(time);
        }else if(motor == 'c'){
            motC.writeMicroseconds(time);
        }else if(motor == 'd'){
            motD.writeMicroseconds(time);
        }else if(motor == 'f'){
            motA.writeMicroseconds(time); 
            motB.writeMicroseconds(time); 
            motC.writeMicroseconds(time); 
            motD.writeMicroseconds(time); 
        }
    }else{
          motA.writeMicroseconds(valueLv); 
          motB.writeMicroseconds(valueLv); 
          motC.writeMicroseconds(valueLv); 
          motD.writeMicroseconds(valueLv); 
     Serial.print("Values: ");
     Serial.print(valueRh);
     Serial.print(", ");
     Serial.print(valueRv);
     Serial.print(", ");
     Serial.print(valueLh);
     Serial.print(", ");
     Serial.println(valueLv);

     Serial.println(testCounter);
    }

}

/**
 * Give user control format
 */
void giveInstructions()
{  
    Serial.println("INTERFACE RULES:");
    Serial.println("\tMOTOR_LETTER.PULSE_LENGTH");
    Serial.println("\t\tMOTOR_LETTER = {a, b, c, d}");
    Serial.println("\t\tPULSER_LENGTH = [1000,2000]");
}
