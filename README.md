# espCopter
This is the code for a quadcopter controller using an ESP32
Build Notes:
- SCL is blue - pin 22 on esp32
- SDA is purple - pin 21 on esp32

To Do list: 
- Figure out which motors are clockwise and which are counterclockwise
    - Done: Red should be the right side!
- Get BNO detected and printing data
    - Also done! Does not need to be level shifted 
- Get receiver working and printing data 
    - Receiver is working for now, not touching it
- Get motors spinning
    - Worked out of the box, not touching it
- Get PID working
    - Right now we're taking the eu
- Get PID working for all axes



Measurement taken: 
- Drone Loop = 650Hz
- With Wifi on Same Core = 620Hz