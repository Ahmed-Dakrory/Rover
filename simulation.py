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
        bus.write_i2c_block_data(address,255,data)
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

def mainLoopForSendTheNeededLengthAndAngle(KpDistance,KpAngle,KpRate,Gps,routingClass,listOfPoints,bus,addr,imu,serialBus):
    #get the first Point which is the nabour Point to me
    #get the GPS Point Here
    totalPacketBefore =[]
    sendData = True
    currPointGPS = routingClass.node(Gps.getGpsReadings()[1],Gps.getGpsReadings()[2])
    indexFirstTarget = len(listOfPoints) - 1
    # indexFirstTarget = len(listOfPoints)
    indexSecondTarget = goToNextTargetOrNot(listOfPoints,Gps,indexFirstTarget)
    
    notReachEndPoint = True
    
    indexCurrentTargetPoint = (len(listOfPoints)-1)
    #timeBefore = 0
    while notReachEndPoint:
        FixMode = Gps.getGpsReadings()[3]
        ErrorInGps = Gps.getGpsReadings()[5]
        ErrorInGps = int(toAnotherRange(ErrorInGps,0,10,3,253))
        if ErrorInGps > 253:
            ErrorInGps = 253

        if ErrorInGps < 0:
            ErrorInGps = 0
        
        if serialBus.DataExist:
            data = serialBus.Data
            serialBus.DataExist = False
            if data == "FF":
                sendActionsToMicroController(serialBus,totalPacketBefore,0,0,0, 0,0,2,FixMode,addr,bus)
                break
            elif data == "SS":
                sendData = True
            elif data == "BB":
                sendData = False


            

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
            if not notReachEndPoint:
                totalPacket = sendActionsToMicroController(serialBus,totalPacketBefore, 0,0,0, 0,0,254,FixMode,addr,bus)
                print("You Reached")
            elif sendData:
                totalPacket = sendActionsToMicroController(serialBus,totalPacketBefore, angleRover,gyroRover,actionDistance, angleAction,actionRate,ErrorInGps,FixMode,addr,bus)
                totalPacketBefore = totalPacket
                print("AngleRover:%f, rate: %f, Distance: %f, AngleAction: %f, Fix: %f, i: %f, all: %f" %(angleRover,gyroRover,actionDistance, angleAction,Gps.getGpsReadings()[3],indexCurrentTargetPoint,len(listOfPoints)))

def sendActionsToMicroController(serialBus,totalPacketBefore, angleRover,gyroRover, actionDistance, angleAction,actionRate,BrakeValue,FixMode,addr,bus):
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
    RobotSpeed = int(toAnotherRange(actionDistance,0,1000,31000,62000))
    if RobotSpeed>62000:
        RobotSpeed = 62000
    if RobotSpeed<31000:
        RobotSpeed = 31000
        
    RobotSpeedBytes = ConvertToBytes(RobotSpeed)

    #Send BrakeValue Flag
    BrakeValueBytes = (BrakeValue)
    totalPacket = [255,SteeringAngleBytes[0],SteeringAngleBytes[1],RobotSpeedBytes[0],RobotSpeedBytes[1],BrakeValueBytes,FixMode]
    
    comparison = totalPacket == totalPacketBefore
    if not comparison:
        serialBus.SendDataOfType(totalPacket)
        # SendDataOfType(addr,totalPacket,bus)
    return totalPacket
    # print("Steering %s,  Speed %s,  BrakeValue %s"%(SteeringAngle,RobotSpeed,BrakeValue))
    
    
def goToNextTargetOrNot(listOfPoints,Gps,indexOfCurrentTarget):
    targetPoint = [listOfPoints[indexOfCurrentTarget][1],listOfPoints[indexOfCurrentTarget][2]]
    #Adjust this Parameter to get the best Performance
    # print("%s %s %s %s" % (indexOfCurrentTarget,getDistanceFromLatLonInMeter([Gps.getGpsReadings()[1],Gps.getGpsReadings()[2]],targetPoint),targetPoint[0],targetPoint[1]))
    if getDistanceFromLatLonInMeter([Gps.getGpsReadings()[1],Gps.getGpsReadings()[2]],targetPoint) < 0.5:
        del listOfPoints[indexOfCurrentTarget]

    return len(listOfPoints)-1
    
def calculateControlAction(KpDistance,KpAngle,KpRate,Gps,listOfPoints,indexCurrentTargetPoint,angleRover,gyroRover):
    currPointGPS = [Gps.getGpsReadings()[1],Gps.getGpsReadings()[2]]
    currentTarget = [listOfPoints[indexCurrentTargetPoint][1],listOfPoints[indexCurrentTargetPoint][2]]
    distance = getDistanceFromLatLonInMeter(currentTarget,currPointGPS)
    # angle = deg2rad(calAngle(currentTarget,currPointGPS))
    angle = (calAngle(currPointGPS,currentTarget))
    
    errorAngle = angle-angleRover
    angleAction = KpAngle * errorAngle
    # print("%s %s %s   %s %s %s %s" % (indexCurrentTargetPoint,distance,errorAngle,currPointGPS[0],currPointGPS[1],currentTarget[0],currentTarget[1]))
    
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
