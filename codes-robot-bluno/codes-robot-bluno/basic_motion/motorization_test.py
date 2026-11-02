#########################################################################
# CENTRALESUPELEC : ST5 VAC challenge week
#
# This script works with the Arduino programmed wih serial_link.ino
#
#########################################################################

import serial 
import time
import numpy as np
import struct
from utilitaries import *
from arduino import arduino


def MotorizationTest():
    
    steptime = 2 			# duration of each stage
    
    if (1==1):
        print("Moving forward")
        car.Advance(150,150)
        time.sleep(1*steptime)
        car.Stop()
        time.sleep(1)
        
    if (1==1):
        print(" moving backward")
        car.Back(150,150)
        time.sleep(steptime)
        car.Stop()
        time.sleep(1)
    
    if (1==1):
        print(" turning left")
        car.TurnLeft(120,120)
        time.sleep(steptime)
        car.Stop()
        time.sleep(1)
        
        print(" turning right")
        car.TurnRight(120,120)
        time.sleep(steptime)
        car.Stop()
        time.sleep(1)
        

############################################
# Main program
############################################

car = arduino('/dev/ttyACM0')
connected = car.connect()
if (connected==True):
    MotorizationTest()
    car.disconnect()
print ("End of program")


