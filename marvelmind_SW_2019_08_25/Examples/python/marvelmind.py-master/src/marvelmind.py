#!/usr/bin/env python
#
# marvelmind.py - small class for recieve and parse coordinates from Marvelmind mobile beacon by USB/serial port
# Written by Alexander Rudykh (awesomequality@gmail.com)
#
### Attributes:
#
#   adr - address of mobile beacon (from Dashboard) for data filtering. If it is None, every read data will be appended to buffer.
#       default: None
#
#   tty - serial port device name (physical or USB/virtual). It should be provided as an argument: 
#       /dev/ttyACM0 - typical for Linux / Raspberry Pi
#       /dev/tty.usbmodem1451 - typical for Mac OS X
#
#   baud - baudrate. Should be match to baudrate of hedgehog-beacon
#       default: 9600
#
#   maxvaluescount - maximum count of measurements of coordinates stored in buffer
#       default: 3
#
#   valuesUltrasoundPosition - buffer of measurements
#
#   debug - debug flag which activate console output    
#       default: False
#
#   pause - pause flag. If True, class would not read serial data
#
#   terminationRequired - If True, thread would exit from main loop and stop
#
#
### Methods:
#
#   __init__ (self, tty="/dev/ttyACM0", baud=9600, maxvaluescount=3, debug=False) 
#       constructor
#
#   print_position(self)
#       print last measured data in default format
#
#   position(self)
#       return last measured data in array [x, y, z, timestamp]
#
#   stop(self)
#       stop infinite loop and close port
#
### Needed libraries:
#
# To prevent errors when installing crcmod module used in this script, use the following sequence of commands:
#   sudo apt-get install python-pip
#   sudo apt-get update
#   sudo apt-get install python-dev
#   sudo pip install crcmod
#
###

###
# Changes:
# lastValues -> valuesUltrasoundPosition
# recieveLinearDataCallback -> recieveUltrasoundPositionCallback
# lastImuValues -> valuesImuRawData
# recieveAccelerometerDataCallback -> recieveImuRawDataCallback
# mm and cm -> m
###

import crcmod
import serial
import struct
import collections
import time
from threading import Thread
import math

# import numpy as np
# import marvelmindQuaternion as mq

class MarvelmindHedge (Thread):
    def __init__ (self, adr=None, tty="/dev/ttyACM0", baud=9600, maxvaluescount=3, debug=False, recieveUltrasoundPositionCallback=None, recieveImuRawDataCallback=None, recieveImuDataCallback=None, recieveUltrasoundRawDataCallback=None):
        self.tty = tty  # serial
        self.baud = baud  # baudrate
        self.debug = debug  # debug flag
        self._bufferSerialDeque = collections.deque(maxlen=255)  # serial buffer

        self.valuesUltrasoundPosition = collections.deque([[0]*6]*maxvaluescount, maxlen=maxvaluescount) # ultrasound position buffer
        self.recieveUltrasoundPositionCallback = recieveUltrasoundPositionCallback
        
        self.valuesImuRawData = collections.deque([[0]*10]*maxvaluescount, maxlen=maxvaluescount) # raw imu data buffer
        self.recieveImuRawDataCallback = recieveImuRawDataCallback

        self.valuesImuData = collections.deque([[0]*14]*maxvaluescount, maxlen=maxvaluescount) # processed imu data buffer
        self.recieveImuDataCallback = recieveImuDataCallback

        self.valuesUltrasoundRawData = collections.deque([[0]*5]*maxvaluescount, maxlen=maxvaluescount)
        self.recieveUltrasoundRawDataCallback = recieveUltrasoundRawDataCallback


        self.pause = False
        self.terminationRequired = False
        
        self.distancesUpdated= False;
        
        self.adr = adr
        self.serialPort = None
        Thread.__init__(self)

    def print_position(self):
        if (isinstance(self.position()[1], int)):
            print ("Hedge {:d}: X: {:d} m, Y: {:d} m, Z: {:d} m, Angle: {:d} at time T: {:.2f}".format(self.position()[0], self.position()[1], self.position()[2], self.position()[3], self.position()[4], self.position()[5]/1000.0))
        else:
            print ("Hedge {:d}: X: {:.3f}, Y: {:.3f}, Z: {:.3f}, Angle: {:d} at time T: {:.2f}".format(self.position()[0], self.position()[1], self.position()[2], self.position()[3], self.position()[4], self.position()[5]/1000.0))

    def position(self):
        return list(self.valuesUltrasoundPosition)[-1];
           
    def print_distances(self): 
		self.distancesUpdated= False
		print ("Distances: B{:d}:{:.3f}, B{:d}:{:.3f}, B{:d}:{:.3f}, B{:d}:{:.3f}   at time T: {:.2f}".format(self.distances()[1], self.distances()[2], self.distances()[3], self.distances()[4], self.distances()[5], self.distances()[6], self.distances()[7], self.distances()[8], self.distances()[9]/1000.0))
        
    def distances(self):
        return list(self.valuesUltrasoundRawData)[-1];
    
    def stop(self):
        self.terminationRequired = True
        print ("stopping")

    def run(self):      
        while (not self.terminationRequired):
            if (not self.pause):
                try:
                    if (self.serialPort is None):
                        self.serialPort = serial.Serial(self.tty, self.baud, timeout=3)
                    readChar = self.serialPort.read(1)
                    while (readChar is not None) and (readChar is not '') and (not self.terminationRequired):
                        self._bufferSerialDeque.append(readChar)
                        readChar = self.serialPort.read(1)
                        bufferList = list(self._bufferSerialDeque)
                        
                        strbuf = (b''.join(bufferList))

                        pktHdrOffset = strbuf.find(b'\xff\x47')
                        if (pktHdrOffset >= 0 and len(bufferList) > pktHdrOffset + 4 and pktHdrOffset<220):
#                           print(bufferList)
                            isMmMessageDetected = False
                            isCmMessageDetected = False
                            isRawImuMessageDetected = False
                            isImuMessageDetected = False
                            isDistancesMessageDetected = False
                            pktHdrOffsetCm = strbuf.find(b'\xff\x47\x01\x00')
                            pktHdrOffsetMm = strbuf.find(b'\xff\x47\x11\x00')
                            pktHdrOffsetRawImu = strbuf.find(b'\xff\x47\x03\x00')
                            pktHdrOffsetDistances = strbuf.find(b'\xff\x47\x04\x00')
                            pktHdrOffsetImu = strbuf.find(b'\xff\x47\x05\x00')

                            if (pktHdrOffsetMm!=-1):
                                isMmMessageDetected = True
                                if (self.debug): print ('Message with US-position(mm) was detected')
                            elif (pktHdrOffsetCm!=-1):
                                isCmMessageDetected = True
                                if (self.debug): print ('Message with US-position(cm) was detected')
                            elif (pktHdrOffsetRawImu!=-1):
                                isRawImuMessageDetected = True
                                if (self.debug): print ('Message with raw IMU data was detected')
                            elif (pktHdrOffsetDistances!=-1):
                                isDistancesMessageDetected = True
                                if (self.debug): print ('Message with distances was detected')
                            elif (pktHdrOffsetImu!=-1):
                                isImuMessageDetected = True
                                if (self.debug): print ('Message with processed IMU data was detected')
                            msgLen = ord(bufferList[pktHdrOffset + 4])
                            if (self.debug): print ('Message length: ', msgLen)

                            try:
                                if (len(bufferList) > pktHdrOffset + 4 + msgLen + 2):
                                    usnCRC16 = 0
                                    if (isCmMessageDetected):
                                        usnTimestamp, usnX, usnY, usnZ, usnAdr, usnAngle, usnCRC16 = struct.unpack_from ('<LhhhxBhxxH', strbuf, pktHdrOffset + 5)
                                        usnX = usnX/100.0
                                        usnY = usnY/100.0
                                        usnZ = usnZ/100.0
                                        usnAngle = 0b0000111111111111&usnAngle
                                    elif (isMmMessageDetected):
                                        usnTimestamp, usnX, usnY, usnZ, usnAdr, usnAngle, usnCRC16 = struct.unpack_from ('<LlllxBhxxH', strbuf, pktHdrOffset + 5)
                                        usnX = usnX/1000.0
                                        usnY = usnY/1000.0
                                        usnZ = usnZ/1000.0
                                        usnAngle = 0b0000111111111111&usnAngle
                                    elif (isRawImuMessageDetected):
                                        ax, ay, az, gx, gy, gz, mx, my, mz, timestamp, usnCRC16 = struct.unpack_from ('<hhhhhhhhhxxxxxxLxxxxH', strbuf, pktHdrOffset + 5)
                                    elif (isImuMessageDetected):
                                        x, y, z, qw, qx, qy, qz, vx, vy, vz, ax, ay, az, timestamp, usnCRC16 = struct.unpack_from ('<lllhhhhhhhhhhxxLxxxxH', strbuf, pktHdrOffset + 5)
                                    elif (isDistancesMessageDetected):
                                        HedgeAdr, b1, b1d, b2, b2d, b3, b3d, b4, b4d, timestamp,usnCRC16 = struct.unpack_from ('<BBlxBlxBlxBlxLxxxH', strbuf, pktHdrOffset + 5)

                                    crc16 = crcmod.predefined.Crc('modbus')
                                    crc16.update(strbuf[ pktHdrOffset : pktHdrOffset + msgLen + 5 ])
                                    CRC_calc = int(crc16.hexdigest(), 16)

                                    if CRC_calc == usnCRC16:
                                        if (isMmMessageDetected or isCmMessageDetected):
                                            value = [usnAdr, usnX, usnY, usnZ, usnAngle, usnTimestamp]
                                            if (self.adr == usnAdr or self.adr is None):
                                                self.valuesUltrasoundPosition.append(value)
                                                if (self.recieveUltrasoundPositionCallback is not None):
                                                    self.recieveUltrasoundPositionCallback()
                                        elif (isRawImuMessageDetected):
                                            value = [ax, ay, az, gx, gy, gz, mx, my, mz, timestamp]
                                            self.valuesImuRawData.append(value)
                                            if (self.recieveImuRawDataCallback is not None):
                                                self.recieveImuRawDataCallback()
                                        elif (isDistancesMessageDetected):
                                            value = [HedgeAdr, b1, b1d/1000.0, b2, b2d/1000.0, b3, b3d/1000.0, b4, b4d/1000.0, timestamp]
                                            self.valuesUltrasoundRawData.append(value)
                                            self.distancesUpdated= True
                                            if (self.recieveUltrasoundRawDataCallback is not None):
                                                self.recieveUltrasoundRawDataCallback()
                                        elif (isImuMessageDetected):
                                            value = [x/1000.0, y/1000.0, z/1000.0, qw/10000.0, qx/10000.0, qy/10000.0, qz/10000.0, vx/1000.0, vy/1000.0, vz/1000.0, ax/1000.0,ay/1000.0,az/1000.0, timestamp]
                                            self.valuesImuData.append(value)
                                            if (self.recieveImuDataCallback is not None):
                                                self.recieveImuDataCallback()
                                    else:
                                        if self.debug:
                                            print ('\n*** CRC ERROR')

                                    if pktHdrOffset == -1:
                                        if self.debug:
                                            print ('\n*** ERROR: Marvelmind USNAV beacon packet header not found (check modem board or radio link)')
                                        continue
                                    elif pktHdrOffset >= 0:
                                        if self.debug:
                                            print ('\n>> Found USNAV beacon packet header at offset %d' % pktHdrOffset)
                                    for x in range(0, pktHdrOffset + msgLen + 7):
                                        self._bufferSerialDeque.popleft()
                            except struct.error:
                                print ('smth wrong')
                except OSError:
                    if self.debug:
                        print ('\n*** ERROR: OS error (possibly serial port is not available)')
                    time.sleep(1)
                except serial.SerialException:
                    if self.debug:
                        print ('\n*** ERROR: serial port error (possibly beacon is reset, powered down or in sleep mode). Restarting reading process...')
                    self.serialPort = None
                    time.sleep(1)
            else: 
                time.sleep(1)
    
        if (self.serialPort is not None):
            self.serialPort.close()
