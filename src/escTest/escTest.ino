#include <ESP32Servo.h>
#include "config.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* This driver reads raw data from the BNO055

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055();
// ---------------------------------------------------------------------------
Servo motA, motB, motC, motD;
String data;
double lastSampleTime = 0;
double maxAcc = 0;
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
    Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

     /* Initialise the sensor */
    if(!bno.begin())
    {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
    }

  delay(1000);

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(true);

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
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
        }else if(motor == 'g'){
            maxAcc = 0;
        }
    }
      // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  /* Display the floating point data */
  double x = euler.x();
  double y = euler.y();
  double z = euler.z();

  if(x < 0){
    x *= -1;
  }
  if(y < 0){
    y*= -1;
  }
  if(z < 0){
    z*=-1;
  }
  if((x+y+z) > maxAcc){
    maxAcc = x+y+z;
  }
  Serial.println("Max = " + String(maxAcc));
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
