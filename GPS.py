import pynmea2
import pyproj as proj
import math
import threading
import numpy as np
from time import sleep

import serial
import time



# 0 Reading not used
# 1 is Latitudel
# 2 is longitude
# 3 is FixMode
# 4 HDOP
# 5 Error in meter

class GpsThreadReadings (threading.Thread):
    def __init__(self,GpsReadings):
        super(GpsThreadReadings , self).__init__(name="GPS thread")
        # self.serialCom = serial.Serial(port='/dev/ttyACM0',baudrate=115200)
        self.serialCom = serial.Serial(port='COM3',baudrate=115200)
        self.GpsReadings = GpsReadings
        self.GpsRun = True
        self.timeBefore = 0
        print("GPS Created")
        

    def getGpsReadings(self):
        return self.GpsReadings

    def setDeltaForRover(self,defLat,defLong):
        self.defLat = defLat
        self.defLong = defLong
    def run(self):
        while self.GpsRun:
            #self.GpsReadings[1] = self.GpsReadings[1] - self.defLat*0.5
            #self.GpsReadings[2] = self.GpsReadings[2] - self.defLong*0.5
            #sleep(0.01)
            self.GpsReadings = self.readGPS(self.GpsReadings[0],self.GpsReadings[1],self.GpsReadings[2],self.GpsReadings[3],self.GpsReadings[4],self.GpsReadings[5])

    def calAngle(self,lat1,long1,lat2,long2):
        dy = lat2 - lat1

        dx = math.cos(math.pi/180*lat1)*(long2 - long1)
        angle = math.atan2(dy, dx)*180/math.pi-90
        if angle <-180:
            angle = angle+360
            
        return -angle

    def deg2rad(self,deg):
        return deg * (math.pi/180)

    def getDistanceFromLatLonInKm(self,lat1,lon1,lat2,lon2) :
        R = 6371; # Radius of the earth in km
        dLat = self.deg2rad(lat2-lat1);  # deg2rad below
        dLon = self.deg2rad(lon2-lon1); 
        a = math.sin(dLat/2) * math.sin(dLat/2) + math.cos(self.deg2rad(lat1)) * math.cos(self.deg2rad(lat2)) *  math.sin(dLon/2) * math.sin(dLon/2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a)); 
        d = R * c; # Distance in km
        return d*1000


    def readGPS(self,angle,latAv2,longAv2,FixMode,hdop,error):
        data = self.serialCom.readline()
        GPS_Read = str(data)[2:-5]
        error = 1000000
        try:
            
            msg = pynmea2.parse(GPS_Read)
            
                
            if msg.sentence_type == 'GGA' :
                #print(int(round(time.time() * 1000))-self.timeBefore)
                #latAv1 =  msg.latitude
                #longAv1 =  msg.longitude
                
                latAv2 =  msg.latitude
                longAv2 =  msg.longitude
                FixMode = msg.gps_qual

            if msg.sentence_type == 'GSA' :
                hdop =  msg.hdop
                if FixMode == 0 or FixMode == 1 or FixMode == 2 or FixMode == 6:
                    error = float(float(hdop) * 2.5)
                if FixMode == 4 or FixMode == 5:
                    error = float(float(hdop) *  0.01)
                #distance = self.getDistanceFromLatLonInKm(latAv1,longAv1,latAv2,longAv2)
                

                #print("Distance: %s" % (distance))
                
                #if distance>0.19 :
                #    angle = self.calAngle(latAv1,longAv1,latAv2,longAv2)
                    
                #    latAv2 =latAv1*0.6+0.4*latAv2
                #    longAv2 =longAv1*0.6+0.4*longAv2
                    
                    
                    #print("Angle: %s" % (angle))
                    
                    
                    #plt.scatter(longAv2, latAv2)
                    #plt.pause(0.001)
                #self.timeBefore = int(round(time.time() * 1000))
                
        except:
            pass
        if error < 0.3:
            return [0,latAv2,longAv2,FixMode,hdop,error]
        else:
            if self.getDistanceFromLatLonInKm(latAv2,longAv2,self.GpsReadings[1],self.GpsReadings[2]) > 0.5:
                return [0,latAv2,longAv2,FixMode,hdop,error]
            else:
                return [0,self.GpsReadings[1],self.GpsReadings[2],FixMode,hdop,error]

        #return [angle,latAv2,longAv2]



##plt.axis([31.1790, 31.1792, 30.01407, 30.0142])

##ser = serial.Serial(port='COM3',baudrate=115200)
##i = 0
##GpsReadings = [0,0,0]
##GpsReadings = readGPS(ser,GpsReadings[0],GpsReadings[1],GpsReadings[2])
##print("Angle: %s, latitude: %s,  Longitude: %s" % (GpsReadings[0], GpsReadings[1], GpsReadings[2]))
##plt.show()
##ser.close();


