#!/usr/bin/python
import os
import sys
import datetime
import commands
import math
import requests
import serial
import argparse
import logging
import struct


from string import split
from smbus import SMBus
from time import sleep



#class for GPS breakout
from GPS_Controller import GpsController
#class for MinIMU-9 V2 (L3GD20, GYRO)

#from warnings import filterwarnings
#filterwarnings('ignore', category = MySQLdb.Warning)

#set devide ID
deviceID = 1
#enable the GPS socket
os.system('sudo gpsd /dev/ttyAMA0 -F /var/run/gpsd.sock')
sleep(0.5)
#create the gps controller object
gpsc = GpsController()
#start gps controller
gpsc.start()

#access the MySQL databse

def bytes2int(bytes):
    return struct.unpack("B", bytes)[0]

def init_usb(sysnode):
    return os.system("echo disabled > %s/power/wakeup"%sysnode)

def turn_on_usb(sysnode):
    return os.system("echo on > %s/power/level"%sysnode)

def turn_off_usb(sysnode):
    return os.system("echo on > %s/power/level"%sysnode)

def setup_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--device", help="device node",
        default="/dev/ttyUSB0")
    parser.add_argument("-u", "--url", 
        help="POST to this url. If empty, the script will only print out the values")
    parser.add_argument("-l", "--loglevel", help="Log level",
        choices=["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"],
        default="INFO")
    parser.add_argument("-p", "--powersaving", help="Powersaving",
        action="store_true")
    parser.add_argument("-s", "--sysnode", help="System node for the usb - for powersaving",
        default="/sys/bus/usb/devices/usb1")
    return parser.parse_args()


def readGPS():
  #fetch GPS data from Ultimate GPS Breakout
  latitude = gpsc.fix.latitude
  longitude = gpsc.fix.longitude
  timeUtc = gpsc.utc
  timeFix = ''
  if isinstance(gpsc.fix.time, str):
    timeFix = gpsc.fix.time
  altitude2 = gpsc.fix.altitude
  eps = gpsc.fix.eps #speed error estimate in meter/sec			
  epx = gpsc.fix.epx #estimated Longitude error in meters
  epv = gpsc.fix.epv #estimated vertical error in meters
  ept = gpsc.gpsd.fix.ept #estimated timestamp error
  speed = gpsc.fix.speed
  climb = gpsc.fix.climb #climb (positive) or sink (negative rate, meters per second
  track = gpsc.fix.track #course over ground, degrees from true north
  mode = gpsc.fix.mode #NMEA mode: %d, 0=no mode value yet seen, 1=no fix, 2=2D, 3=3D
  satellites = len(gpsc.satellites)
		
  #make sure that the database tables get no input for as long as there is no GPS signal
  if (math.isnan(latitude) or not latitude):
    gpsSignal = 0
  elif (math.isnan(longitude) or not longitude):
    gpsSignal = 0
  elif (timeUtc is None or "nan" in timeUtc or not timeUtc):
    gpsSignal = 0
  elif (math.isnan(altitude2)):
    gpsSignal = 0
  elif (math.isnan(eps)):
    gpsSignal = 0
  elif (math.isnan(epx)):
    gpsSignal = 0
  elif (math.isnan(epv)):
    gpsSignal = 0
  elif (math.isnan(ept)):
    gpsSignal = 0
  elif (math.isnan(speed)):
    gpsSignal = 0
  elif (math.isnan(climb)):
    gpsSignal = 0
  elif (math.isnan(track)):
    gpsSignal = 0
  else:
    gpsSignal = 1

  return {'gpsSignal':gpsSignal, 'latitude':latitude, 'longitude':longitude, 'timeUtc':timeUtc, 'timeFix':timeFix, 'altitude2':altitude2, 'eps':eps, 'epx':epx, 'epv':epv, 'ept':ept, 'speed':speed, 'climb':climb, 'track':track, 'mode':mode, 'satellites':satellites}

def printValues(sysDate, sysTime, latitude, longitude, timeUtc, timeFix, altitude2, eps, epx, epv, ept, speed, climb, track, mode, satellites):

  #print data to screen
  print "Date:           {0}".format(sysDate)
  print "Time:           {0}".format(sysTime)
  print "Latitude:       {0}".format(latitude)
  print "Longitude:      {0}".format(longitude)
  print "Time-Utc:       {0}".format(timeUtc)
  print "Time-Fix:       {0}".format(timeFix)
  print "Altitude-GPS:   {0}".format(altitude2)
  print "EPS:            {0}".format(eps)
  print "EPX:            {0}".format(epx)
  print "EPV:            {0}".format(epv)
  print "EPT:            {0}".format(ept)
  print "Speed:          {0} m/s".format(speed)
  print "Climb:          {0}".format(climb)
  print "Track:          {0}".format(track)
  print "Mode:           {0}".format(mode)
  print "Satellites:     {0}".format(satellites)




if __name__ == '__main__':

    args = setup_args()
    logging.basicConfig(format='%(levelname)s:%(message)s', level=getattr(logging, args.loglevel.upper()))

    if args.powersaving and args.sysnode:
        logging.debug("Init Powersaving")
        logging.debug(init_usb(args.sysnode))
        logging.debug("Turning USB ON")
        logging.debug(turn_on_usb(args.sysnode))

    try:
        with serial.Serial(args.device, baudrate=9600) as ser:

            logging.info("Serial device initialized")
            read_full = False
            pm25 = 0
            pm10 = 0
            data = []
            


            while not read_full:
                if ser.read() == b'\xaa':
                    logging.debug("FIRST HEADER GOOD")
                    # FIRST HEADER IS GOOD
                    if ser.read() == b'\xc0':
                        # SECOND HEADER IS GOOD
                        logging.debug("SECOND HEADER GOOD")
                        for i in range(8):
                            byte = ser.read()
                            data.append(bytes2int(byte))

                        if data[-1] == 171:
                            # END BYTE IS GOOD. DO CRC AND CALCULATE
                            logging.debug("END BYTE GOOD")
                            if data[6] == sum(data[0:6])%256:
                                logging.debug("CRC GOOD")
                            pm25 = (data[0]+data[1]*256)/10
                            pm10 = (data[4]+data[3]*256)/10
                            read_full = True
            if args.url:
                logging.info("Posting to %s" % args.url)

                gps_data = readGPS()
                gpsc.stopController()
                #wait for the thread to finish
                gpsc.join()

                
                r = requests.post(args.url, data={"pm10": pm10, "pm2.5": pm25, "lat": gps_data.get('latitude'), "lon": gps_data.get('longitude'), "alt": gps_data.get('speed'), "spd": gps_data.get('altitude2'), "time": gps_data.get('timeUtc'), "satt": gps_data.get('satellites')})
                logging.debug(r)
            logging.info("PM 10: %s" % pm10)
            logging.info("PM 2.5: %s" % pm25)
            logging.info("Latitude: %s" % gps_data.get('latitude'))
            logging.info("longitude: %s" % gps_data.get('longitude'))
    except serial.SerialException as e:
        logging.critical(e)

    if args.powersaving and args.sysnode:
        logging.debug("Turning USB OFF")
        logging.debug(turn_off_usb())        
