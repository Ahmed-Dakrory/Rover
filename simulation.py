# -*- coding: utf-8 -*-
"""
Created on Mon Dec  2 14:32:15 2019

@author: Ahmed.Dakrory
"""
import math
import utm

import pyproj
import struct
import time
geodesic = pyproj.Geod(ellps='WGS84')

def convertNumberIntoAsciValue(valueToConvert):
    latLongList = list(str(valueToConvert))
    latLongAsciBytes = []
    for i in range(0,len(latLongList)):
        latLongAsciBytes.append(ord(latLongList[i]))
    return latLongAsciBytes


def ConvertToBytes(data):
    s = struct.pack('>H', data)
    firstByte, secondByte = struct.unpack('>BB', s)
    dataToSend = [firstByte,secondByte]
    return dataToSend



def SendDataOfType(address,data,bus):
    try:
        bus.write_i2c_block_data(address,0,data)
    except:
        pass

def toAnotherRange(OldValue,OldMin,OldMax,NewMin,NewMax):
    NewValue = (((OldValue - OldMin) * (NewMax - NewMin)) / (OldMax - OldMin)) + NewMin
    return NewValue


def sendArrayOfBytes(address,data,bus):
    for i in range(0,len(data)):
        bus.write_byte(address,data[i])



def deg2num(lat_deg, lon_deg, zoom):
  lat_rad = math.radians(lat_deg)
  n = 2.0 ** zoom
  xtile = ((lon_deg + 180.0) / 360.0 * n)
  ytile = ((1.0 - math.log(math.tan(lat_rad) + (1 / math.cos(lat_rad))) / math.pi) / 2.0 * n)
  return (xtile, ytile)


def calAngle(point1_StartGPS,point2_EndDist):
    lat1 = point1_StartGPS[0]
    long1 = point1_StartGPS[1]
    lat2 = point2_EndDist[0]
    long2 = point2_EndDist[1]
    fwd_azimuth,back_azimuth,distance = geodesic.inv(lat1, long1, lat2, long2)

    angle = 90-fwd_azimuth
    if angle > 180:
        angle = angle - 360
    return angle


def getDistanceFromLatLonInMeter(point1,point2) :
    try:
        R = 6371; # Radius of the earth in km
        dLat = deg2rad(point2[0]-point1[0]);  # deg2rad below
        dLon = deg2rad(point2[1]-point1[1]); 
        a = math.sin(dLat/2) * math.sin(dLat/2) + math.cos(deg2rad(point1[0])) * math.cos(deg2rad(point2[0])) *  math.sin(dLon/2) * math.sin(dLon/2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a)); 
        d = R * c; # Distance in km
    except:
        pass
    return d*1000


def deg2rad(deg):
    return deg * (math.pi/180)

def mainLoopForSendTheNeededLengthAndAngle(KpDistance,KpAngle,KpRate,Gps,routingClass,listOfPoints,bus,addr,imu):
    #get the first Point which is the nabour Point to me
    #get the GPS Point Here
    currPointGPS = routingClass.node(Gps.getGpsReadings()[1],Gps.getGpsReadings()[2])
    indexFirstTarget = len(listOfPoints) - 1
    indexSecondTarget = goToNextTargetOrNot(listOfPoints,Gps,indexFirstTarget)
    
    notReachEndPoint = True
    
    indexCurrentTargetPoint = (len(listOfPoints)-1)
    #timeBefore = 0
    while notReachEndPoint:
        

        if imu.Readings !=None and imu.Rates !=None:
            angleRover = imu.Readings['Yaw']
            gyroRover = imu.Rates['gz']
            #print(int(round(time.time() * 1000))-timeBefore)
            #timeBefore = int(round(time.time() * 1000))
        
            [actionDistance, angleAction,actionRate] = calculateControlAction(KpDistance,KpAngle,KpRate,Gps,listOfPoints,indexCurrentTargetPoint,angleRover,gyroRover)
            

            #remove this after set Gps
            #simulateRoverGPS(Gps,listOfPoints,indexCurrentTargetPoint)
            
            indexCurrentTargetPoint = goToNextTargetOrNot(listOfPoints,Gps,indexCurrentTargetPoint)
            notReachEndPoint = checkIfNotReachedEndPoint(indexCurrentTargetPoint)
            
            sendActionsToMicroController(angleRover,gyroRover,actionDistance, angleAction,actionRate,addr,bus)
            print("AngleRover:%f, rate: %f, Distance: %f, AngleAction: %f, GPS: %f, i: %f" %(angleRover,gyroRover,actionDistance, angleAction,Gps.getGpsReadings()[1],indexCurrentTargetPoint))

def sendActionsToMicroController(angleRover,gyroRover, actionDistance, angleAction,actionRate,addr,bus):
    # sendArrayOfBytes(addr,convertNumberIntoAsciValue('#'),bus)
    # sendArrayOfBytes(addr,convertNumberIntoAsciValue(angleRover),bus)
    # sendArrayOfBytes(addr,convertNumberIntoAsciValue('&'),bus)
    # sendArrayOfBytes(addr,convertNumberIntoAsciValue(gyroRover),bus)
    # sendArrayOfBytes(addr,convertNumberIntoAsciValue(':'),bus)
    # sendArrayOfBytes(addr,convertNumberIntoAsciValue(actionDistance),bus)
    # sendArrayOfBytes(addr,convertNumberIntoAsciValue('$'),bus)
    # sendArrayOfBytes(addr,convertNumberIntoAsciValue(angleAction),bus)
    # sendArrayOfBytes(addr,convertNumberIntoAsciValue(';'),bus)
    # sendArrayOfBytes(addr,convertNumberIntoAsciValue(actionRate),bus)
    # sendArrayOfBytes(addr,convertNumberIntoAsciValue('!'),bus)
    
    #Send AngleAction Steering
    TotalAction = angleAction + actionRate #Range = 250+180
    SteeringAngle = int(toAnotherRange(TotalAction,-330,330,0,9000))
    if SteeringAngle>9000:
        SteeringAngle = 9000
    SteeringAngleBytes = ConvertToBytes(SteeringAngle)
    
    #Send SpeedAction
    RobotSpeed = int(toAnotherRange(actionDistance,0,1000,0,6000))
    if RobotSpeed>6000:
        RobotSpeed = 6000
        
    RobotSpeedBytes = ConvertToBytes(RobotSpeed)

    #Send BrakeValue Flag
    BrakeValue = 99
    BrakeValueBytes = (BrakeValue)
    totalPacket = [SteeringAngleBytes[0],SteeringAngleBytes[1],RobotSpeedBytes[0],RobotSpeedBytes[1],BrakeValueBytes]
    SendDataOfType(addr,totalPacket,bus)
    # print("Steering %s,  Speed %s,  BrakeValue %s"%(SteeringAngle,RobotSpeed,BrakeValue))
    
    
def goToNextTargetOrNot(listOfPoints,Gps,indexOfCurrentTarget):
    targetPoint = [listOfPoints[indexOfCurrentTarget][1],listOfPoints[indexOfCurrentTarget][2]]
    #Adjust this Parameter to get the best Performance
    if getDistanceFromLatLonInMeter([Gps.getGpsReadings()[1],Gps.getGpsReadings()[2]],targetPoint) < 0.1:
        del listOfPoints[indexOfCurrentTarget]

    return len(listOfPoints)-1
    
def calculateControlAction(KpDistance,KpAngle,KpRate,Gps,listOfPoints,indexCurrentTargetPoint,angleRover,gyroRover):
    currPointGPS = [Gps.getGpsReadings()[1],Gps.getGpsReadings()[2]]
    currentTarget = [listOfPoints[indexCurrentTargetPoint][1],listOfPoints[indexCurrentTargetPoint][2]]
    distance = getDistanceFromLatLonInMeter(currentTarget,currPointGPS)
    angle = deg2rad(calAngle(currPointGPS,currentTarget))
    
    errorAngle = angle-angleRover
    angleAction = KpAngle * errorAngle
    
    actionRate = KpRate * gyroRover
    
    
    if errorAngle == 0:
        errorAngle = 0.1
    errorDistance = distance/abs(errorAngle)
    
    actionDistance = KpDistance * errorDistance
    
    
    return [actionDistance, angleAction,actionRate]


def simulateRoverGPS(Gps,listOfPoints,indexCurrentTargetPoint):
    
    currPointGPS = [Gps.getGpsReadings()[1],Gps.getGpsReadings()[2]]
    #ThoseWillBeRemoved
    defLat = (currPointGPS[0] - listOfPoints[indexCurrentTargetPoint][1])
    defLong = (currPointGPS[1] - listOfPoints[indexCurrentTargetPoint][2])

    Gps.setDeltaForRover(defLat,defLong)
   

    

    
def checkIfNotReachedEndPoint(indexCurrentTargetPoint):
    if indexCurrentTargetPoint == -1:
       return False
    else:
        return True
   
def mostNabourPointAndDeleteNewPoint(listOfPoints,cPoint):
    indexMin=0
    lengthMin = 99999999999999.99999999

    for n in range(0,len(listOfPoints)):
        listPoint = [listOfPoints[n][1],listOfPoints[n][2]]
        d = getDistanceFromLatLonInMeter(cPoint,listPoint)
        if d<lengthMin:
            lengthMin=d
            indexMin = n
    pointToReturn = [listOfPoints[indexMin][1],listOfPoints[indexMin][2]]
    del listOfPoints[indexMin]
    return [indexMin,lengthMin,pointToReturn]
