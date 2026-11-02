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


################################################
# Functions for bidirectional coms with Arduino 
################################################

def DialArduino():
    while True:
        print("") 
        print("Dialogue with Arduino") 
        cma = input("Enter your command (Q to end) ")
        if cma=="Q":
            break
        if cma!='':
            arduino.write(cma.encode('utf-8'))
            #time.sleep(0.01)
            rep = arduino.readline()  		# reading response message
            while rep==b'':					# waiting for a true response
                rep = arduino.readline()
            #print(rep)
            print(rep.decode())
            while arduino.inWaiting()>0 :	# while messages are present in the buffer
                rep = arduino.readline()  	# reading response message
                print(rep.decode())


############################################
# Main program
############################################

#  setting up the serial link with Arduino 
# -----------------------------------------
arduino = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=0.1)
print ("Connecting to Arduino")
time.sleep(2)			# waiting for 2s for the Arduino to be initialised

arduino.write(b'A20')		# requesting connection with full acknowledgement in ASCII
rep = arduino.readline()
#print(rep.decode())
if len(rep.split())>0:
  if rep.split()[0]==b'OK':
    print(rep.decode()) 
    DialArduino()

#  disconnecting Arduino
# ----------------------
arduino.write(b'a')	    # disconnection
arduino.close()         # closing serial link
print ("End of program")

