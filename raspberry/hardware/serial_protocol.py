#########################################################################
# CENTRALESUPELEC : ST5 VAC challenge week
#########################################################################

import serial
import struct
import time


class ArduinoStop(Exception):
    """L'Arduino a refusé/annulé un mouvement. cause : 'watchdog', 'obstacle' ou 'muet'."""

    # préfixe de l'acquittement -> cause
    CAUSES = {b'WD': 'watchdog', b'OB': 'obstacle', b'MU': 'muet'}

    def __init__(self, cause):
        super().__init__(f"arrêt Arduino : {cause}")
        self.cause = cause


def check_ack(ack):
    """Lève ArduinoStop si l'acquittement signale un arrêt de sécurité."""
    cause = ArduinoStop.CAUSES.get(ack[:2])
    if cause is not None:
        raise ArduinoStop(cause)
    return ack


def read_i16(f):
    return struct.unpack('<h', bytearray(f.read(2)))[0]

def read_i32(f):
    return struct.unpack('<l', bytearray(f.read(4)))[0]

def write_i16(f, value):
    f.write(struct.pack('<h', value))

def write_i32(f, value):
    f.write(struct.pack('<l', value))

def WaitACK(arduino):
    deadline = time.monotonic() + 0.5
    while time.monotonic() < deadline:
        rep = arduino.readline()
        if rep:
            return rep
    raise TimeoutError("Arduino n'a pas acquitté la commande en 0,5 s")

def sendCmdi(arduino,cmd,arg1,arg2,arg3,arg4): # send command with 4 ints
    arduino.write(cmd)
    write_i16(arduino, arg1)
    write_i16(arduino, arg2)
    write_i16(arduino, arg3)
    write_i16(arduino, arg4)
    return check_ack(WaitACK(arduino))

def sendCmdl(arduino,cmd,arg1,arg2): # send command with 2 longs
    arduino.write(cmd)
    write_i32(arduino, arg1)
    write_i32(arduino, arg2)
    return check_ack(WaitACK(arduino))

def retrCmdi(arduino,cmd): # retrieve command with 4 ints
    arduino.write(cmd)
    val1=read_i16(arduino)
    val2=read_i16(arduino)
    val3=read_i16(arduino)
    val4=read_i16(arduino)
    return val1,val2,val3,val4

def retrCmdl(arduino,cmd): # retrieve command with 2 longs
    arduino.write(cmd)
    val1=read_i32(arduino)
    val2=read_i32(arduino)
    return val1,val2
