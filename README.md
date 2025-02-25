# espCopter
This is the code for a quadcopter controller using an ESP32 and BNO055. 
The goal is of this project is to write the controller from scatch using ESP-IDF and FreeRTOS. 

More information on setup and building to come soon after stable flight!

![image](https://github.com/user-attachments/assets/daeecef1-2f2f-4daf-8d2e-31bb697c1ca9)

Current Pinout: 

BNO055: 
SDA: 21
SCL: 22
Vin: 5V
GND: GND
RST: 15

ESC: 
Back Left: 19
Back Right: 17
Front Left: 18
Front Right: 16

RC: 
CH1: 25
CH2: 26
CH3: 14
CH4: 27
CH5: 34
CH6: 35

Props: 
Top Left: CCW
Top Right: CW
Bottom Left: CW
Bottom Right: CCW