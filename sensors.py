#!/usr/bin/python
import os
import sys
import datetime
import commands
import math
from string import split
from smbus import SMBus
from time import sleep
#class for BMP085 sensor breakout
import Adafruit_BMP.BMP085 as BMP085
#class for GPS breakout
from GPS_Controller import GpsController

deviceID = 1

#enable the GPS socket
os.system('sudo gpsd /dev/ttyAMA0 -F /var/run/gpsd.sock')
sleep(0.5)
#create the gps controller object
gpsc = GpsController()
#start gps controller
gpsc.start()


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


def printValues(sysDate, sysTime, pressure, sealvlPressure, altitude1, temp1C, temp1F, latitude, longitude, timeUtc, timeFix, altitude2, eps, epx, epv, ept, speed, climb, track, mode, satellites, humidity, heading, windDir, temp2C, temp2F, magX, magY, magZ, magGaussX, magGaussY, magGaussZ, accX, accY, accZ, accGX, accGY, accGZ, gyroX, gyroY, gyroZ, gyroDPSX, gyroDPSY, gyroDPSZ, tempRC, tempRF):

  #print data to screen
  print "Date:           {0}".format(sysDate)
  print "Time:           {0}".format(sysTime)
  print "Pressure:       {0} hPa".format(pressure)
  print "SeaLvlPr:       {0} hPa".format(sealvlPressure)
  print "Altitude:       {0} m".format(altitude1)
  print "Temp1B:         {0} C".format(temp1C)
  print "Temp1B:         {0} F".format(temp1F)
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
  print "Humidity:       {0} RH%'".format(humidity) #RH = Relative Humidity
  print "Heading:        {0} deg.".format(heading)
  print "WindDirection:  {0}".format(windDir)
  print "Temp2M:         {0} C".format(temp2C)
  print "Temp2M:         {0} F".format(temp2F)

  try:

    while(True):

      #current Date and Time
      sysDate = datetime.datetime.now().date().strftime('%Y-%m-%d')
      sysTime = datetime.datetime.now().time().strftime('%H:%m:%S')

      printValues(sysDate, sysTime, readBMP085().get('pressure'), readBMP085().get('sealvlPressure'), readBMP085().get('altitude1'), readBMP085().get('temp1C'), readBMP085().get('temp1F'), readGPS().get('latitude'), readGPS().get('longitude'), readGPS().get('timeUtc'), readGPS().get('timeFix'), readGPS().get('altitude2'), readGPS().get('eps'), readGPS().get('epx'), readGPS().get('epv'), readGPS().get('ept'), readGPS().get('speed'), readGPS().get('climb'), readGPS().get('track'), readGPS().get('mode'), readGPS().get('satellites'), readHIH6130().get('humidity'), readMinIMU9V2().get('heading'), readMinIMU9V2().get('windDir'), readMinIMU9V2().get('temp2C'), readMinIMU9V2().get('temp2F'), readMinIMU9V2().get('magX'), readMinIMU9V2().get('magY'), readMinIMU9V2().get('magZ'), readMinIMU9V2().get('magGaussX'), readMinIMU9V2().get('magGaussY'), readMinIMU9V2().get('magGaussZ'), readMinIMU9V2().get('accX'), readMinIMU9V2().get('accY'), readMinIMU9V2().get('accZ'), readMinIMU9V2().get('accGX'), readMinIMU9V2().get('accGY'), readMinIMU9V2().get('accGZ'), readMinIMU9V2().get('gyroX'), readMinIMU9V2().get('gyroY'), readMinIMU9V2().get('gyroZ'), readMinIMU9V2().get('gyroDPSX'), readMinIMU9V2().get('gyroDPSY'), readMinIMU9V2().get('gyroDPSZ'), readOnboardTemp().get('tempRC'), readOnboardTemp().get('tempRF'))

      #only write Dataset to table if the GPS signal is fully valid
      if (readGPS().get('gpsSignal') == 1):
        writeSQL(deviceID, sysDate, sysTime, readBMP085().get('pressure'), readBMP085().get('sealvlPressure'), readBMP085().get('altitude1'), readBMP085().get('temp1C'), readBMP085().get('temp1F'), readGPS().get('latitude'), readGPS().get('longitude'), readGPS().get('timeUtc'), readGPS().get('timeFix'), readGPS().get('altitude2'), readGPS().get('eps'), readGPS().get('epx'), readGPS().get('epv'), readGPS().get('ept'), readGPS().get('speed'), readGPS().get('climb'), readGPS().get('track'), readGPS().get('mode'), readGPS().get('satellites'), readHIH6130().get('humidity'), readMinIMU9V2().get('heading'), readMinIMU9V2().get('windDir'), readMinIMU9V2().get('temp2C'), readMinIMU9V2().get('temp2F'), readMinIMU9V2().get('magX'), readMinIMU9V2().get('magY'), readMinIMU9V2().get('magZ'), readMinIMU9V2().get('magGaussX'), readMinIMU9V2().get('magGaussY'), readMinIMU9V2().get('magGaussZ'), readMinIMU9V2().get('accX'), readMinIMU9V2().get('accY'), readMinIMU9V2().get('accZ'), readMinIMU9V2().get('accGX'), readMinIMU9V2().get('accGY'), readMinIMU9V2().get('accGZ'), readMinIMU9V2().get('gyroX'), readMinIMU9V2().get('gyroY'), readMinIMU9V2().get('gyroZ'), readMinIMU9V2().get('gyroDPSX'), readMinIMU9V2().get('gyroDPSY'), readMinIMU9V2().get('gyroDPSZ'), readOnboardTemp().get('tempRC'), readOnboardTemp().get('tempRF'))
      else:
        print 'No GPS signal, not writing to the database...'
      #wait 5.00 seconds before continuing
      sleep(0.01)

      #clear terminal screen
      sys.stdout.flush()

  #Ctrl C user interrupt
  except KeyboardInterrupt:
    print 'User cancelled...'
 
  #name is not defined
  except NameError as e:
    print 'Name is not defined: %s on Line number: %s' % (e.message.split("'")[1], sys.exc_traceback.tb_lineno)
    raise
  
  #type error
  except TypeError as e:
    print 'Type Error: %s on Line number: %s' % (e, sys.exc_traceback.tb_lineno)
    raise

  #attribute error
  except AttributeError as e:
    print 'Attribute Error: %s on Line number: %s' % (e, sys.exc_traceback.tb_lineno)
    raise

  #unexpected error
  except:
    print 'Unexpected error:', sys.exc_info()[0]
    raise

  finally:

    #current Date and Time
    sysDate = datetime.datetime.now().date().strftime('%Y-%m-%d')
    sysTime = datetime.datetime.now().time().strftime('%H:%m:%S')


  print 'Done'

  sys.exit(0)

if __name__ == '__main__':
  main()
