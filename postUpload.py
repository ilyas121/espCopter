import serial

output = serial.Serial('/dev/ttyUSB0')
ser.flushInput()

while True:
    try:
        ser_bytes = ser.readLine()
        #decoded = float(ser_bytes[0:len(ser_bytes)-2].decode('utf-8'))
        print(decode)
    except:
        print("I give you L's boi'o") 
