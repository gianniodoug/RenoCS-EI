#########################################################################
# CENTRALESUPELEC : ST5 VAC challenge week
#########################################################################

import serial 
import struct

def read_i16(f):
    return struct.unpack('<h', bytearray(f.read(2)))[0]

def read_i32(f):
    return struct.unpack('<l', bytearray(f.read(4)))[0]

def write_i16(f, value):
    f.write(struct.pack('<h', value))

def write_i32(f, value):
    f.write(struct.pack('<l', value))

def WaitACK(arduino):
    rep=b''
    while rep==b'':					# waiting acknowledgement of B2
        rep=arduino.readline()
    print(rep.decode())

def sendCmdi(arduino,cmd,arg1,arg2,arg3,arg4): # send command with 4 ints
    arduino.write(cmd)
    write_i16(arduino, arg1)
    write_i16(arduino, arg2)
    write_i16(arduino, arg3)
    write_i16(arduino, arg4)
    WaitACK(arduino)

def sendCmdl(arduino,cmd,arg1,arg2): # send command with 2 longs
    arduino.write(cmd)
    write_i32(arduino, arg1)
    write_i32(arduino, arg2)
    WaitACK(arduino)

def retrCmdi(arduino,cmd): # retrieve command with 4 ints
    arduino.write(cmd)
    val1=read_i16(arduino)
    val2=read_i16(arduino)
    val3=read_i16(arduino)
    val4=read_i16(arduino)
    return val1,val2,val3,val4
    WaitACK(arduino)

def retrCmdl(arduino,cmd): # retrieve command with 2 longs
    arduino.write(cmd)
    val1=read_i32(arduino)
    val2=read_i32(arduino)
    return val1,val2
    WaitACK(arduino)
