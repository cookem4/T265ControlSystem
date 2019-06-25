import serial
import os
import time

ser = serial.Serial('/dev/ttyAMA0',
                    baudrate=9600,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    bytesize=serial.EIGHTBITS
                    )
time.sleep(1)

print("reading")
while 1:
    
    x=ser.readline()
    print(x)
    print(",")
    
