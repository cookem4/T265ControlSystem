###########################################
##########Used for controlling#############
#######cleanflight fligh controller########
###########################################

from Cleanflight_MSP import Cleanflight_MSP
import serial
import os
import time

cleanflightMSP0 = Cleanflight_MSP('/dev/ttyUSB0', 115200)

######################################
#######SUMMARY OF ISSUES##############
######################################
#Data can be sent over UART TX port of pi as if it is connected to an LED the LED blinks
#PC connot receive this data when a USB to TTL converter is used
    #The LED on the USB to TTL converter flahses but no data is recieved in RealTerm on the PC
    #The COM port of the USB to TTL convert shows
#The current UART port is set to be on ttyAMA0
#Transmitting data with MSP in the same way the motion capture program does results in no activity on the flight controller
#Both the pi and the RF receiver have a single wire going from a 3.3V port on the board to the RX pin on the CleanFlight UART
    #The pi gets no response even though the connection is exactly the same
#Using the pi with the pyMultiWii program it is also not able to establish connection with the flight contorller
#*****************The use of USB0 by connectiong with microUSB can be promising***********
#Works with micro usb connection to the flight controller!!!!!!!!!!!!!

ARM = 1600; DISARM = 1000; ANGLE_MODE = 1600; NEUTRAL = 1000;
ZERO_ROLL = 1500; ZERO_PITCH = 1500; ZERO_YAW_RATE = 1500; ZERO_THROTTLE = 1000;
zeroCommands = [ZERO_ROLL, ZERO_PITCH, ZERO_THROTTLE, ZERO_YAW_RATE, ARM, ANGLE_MODE, NEUTRAL, NEUTRAL]
commandsToGo = [] 

HZ=100
disArmCommandsToGoMSP = [ZERO_ROLL, ZERO_PITCH, ZERO_THROTTLE, ZERO_YAW_RATE, DISARM, ANGLE_MODE, NEUTRAL, NEUTRAL] # The 1600 is to enable the "Angle Mode" in clean flight.
armCommandsToGoMSP    = [ZERO_ROLL, ZERO_PITCH, ZERO_THROTTLE, ZERO_YAW_RATE, ARM, ANGLE_MODE, NEUTRAL, NEUTRAL]
dataLength = len(disArmCommandsToGoMSP)
dataBytes = 2*dataLength
direction = '<'
commandsToGo = [ZERO_ROLL, ZERO_PITCH, ZERO_THROTTLE, ZERO_YAW_RATE, ARM, ANGLE_MODE, NEUTRAL, NEUTRAL]
h = 'h'
print("Disarming...")
for i in range(1, 20):  #command for about a second before arming
    cleanflightMSP0.sendMSP(direction, dataBytes, 200, disArmCommandsToGoMSP, direction+ str(dataLength) + h)
    time.sleep(1/HZ)
print("Arming...")
for i in range(1, 20):
    cleanflightMSP0.sendMSP(direction, dataBytes, 200,    armCommandsToGoMSP, direction+ str(dataLength) + h)
    time.sleep(1/HZ)
print("Active...")
#Send throttle data to the copters
i=0
try:
    while (i<200):
        commandsToGo[2] = commandsToGo[2] + 1
        cleanflightMSP0.sendMSP(direction, dataBytes, 200,    commandsToGo, direction+ str(dataLength) + h)
        time.sleep(1/HZ)
        i=i+1
        print("Iteration #" + str(i))
    i=0
    while(i<200):
        commandsToGo[2] = commandsToGo[2] - 1
        cleanflightMSP0.sendMSP(direction, dataBytes, 200,    commandsToGo, direction+ str(dataLength) + h)
        time.sleep(1/HZ)
        i=i+1
        print("Iteration #" + str(i))
except KeyboardInterrupt:
    print("Program Force Stopped")













    
#Try looking at this video:
        #https://www.youtube.com/watch?v=GJuWpBCgQPQ
#ser = serial.Serial('/dev/ttyAMA0',
#                    baudrate=9600,
#                    parity=serial.PARITY_NONE,
#                    stopbits=serial.STOPBITS_ONE,
#                    bytesize=serial.EIGHTBITS,
#                    timeout=3.0
#                    )
#try:
#    print("Writing to serial...")
 #   while(1):
#        ser.write("Testing".encode("utf8"))
#except KeyboardInterrupt:
#    print("Exiting program")
#except Exception as e:
#    print(e)
#finally:
#    ser.close()
#    pass
