#########################################################################
# CENTRALESUPELEC : ST5 VAC challenge week
#########################################################################

import serial 
import time
import numpy as np
from utilitaries import *


class arduino:

    def __init__(self,portnr):
        self.serialport = serial.Serial(port=portnr, baudrate=115200, timeout=0.1)
        
    #######################################
    #   connection to Arduino
    #######################################
    def		connect(self):
        if self.serialport.is_open==False:
            self.serialport.open()
        rep=' '   # emptying serial link
        while rep!=b'':
          rep = self.serialport.readline()
        print ("Connecting to Arduino")

        time.sleep(2)			# waiting for 2s for the Arduino to be initialised

        self.serialport.write(b'A22')		# requesting connection with simple 'OK' acknowledgement
        rep = self.serialport.readline()
        if rep.split()[0]==b'OK':
            self.serialport.write(b'I0')
            WaitACK(self.serialport)
            print(rep.decode())
            return True
        else:
            return False
  
        
    #######################################
    #   disconnection
    #######################################
    def		disconnect(self):
        self.serialport.write(b'a')	# disconnection
        self.serialport.close()         # closing serial link

    #######################################
    #   stop motors
    #######################################
    def  Stop(self):
        sendCmdi(self.serialport,b'C',0,0,0,0)

    #######################################
    #   moving forward
    #######################################
    def  Advance(self,c1,c2):
        sendCmdi(self.serialport,b'C',c1,c2,c1,c2)

    #######################################
    #   mise en route des moteurs en sens inverse 
    #######################################
    def  Back(self,c1,c2):
        sendCmdi(self.serialport,b'C',-c1,-c2,-c1,-c2)

    #######################################################
    #   mise en route des moteurs pour tourner à gauche
    #######################################################
    def  TurnLeft(self,c1,c2):
        sendCmdi(self.serialport,b'C',c1,-c2,c1,-c2)

    #######################################################
    #   mise en route des moteurs pour tourner à droite
    #######################################################
    def  TurnRight(self,c1,c2):
        sendCmdi(self.serialport,b'C',-c1,c2,-c1,c2)

    #######################################################
    #   mise en route de la protection infra rouge
    #######################################################
    def  IrOn(self):
        self.serialport.write(b'I1')
        WaitACK(self.serialport)

    #######################################################
    #   recuperation de la vitesse des moteurs
    #######################################################
    def  GetSpeed(self):
        vit1,vit2,vit3,vit4 = retrCmdi(self.serialport,b'P')
        return vit1,vit2,vit3,vit4