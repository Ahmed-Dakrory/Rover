from Routing import RoutingUsage # Import the router
import simulation as sim
import utm
import GPS as gps# Import the router

from time import sleep

from imuDev.MpuRm3100 import IMU



    
    
from smbus import SMBus
import struct

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

KpDistance=1
KpAngle=1
KpRate=1
addr = 8 # bus address
bus = SMBus(1) # indicates /dev/ic2-1


DRDY = 27 #GPIO 27
SSN = 17 #GPIO 17
imu = IMU(SSN,DRDY)

imu.start()

##node2=routingClass.node(51.9284338,4.4893559)
            
GpsData = [0,0,0,0,0,0]

gps = gps.GpsThreadReadings(GpsData)
gps.start()


while True:
    mode = int(input("Press 1 for Gps or 2 for Heading angle or 3 for check angle or 4 for the tracking mode: "))
    if mode == 1:
        while True:
            lat = gps.getGpsReadings()[1]
            lon = gps.getGpsReadings()[2]
            fixMode = gps.getGpsReadings()[3]
            HDOP = gps.getGpsReadings()[4]
            Error = gps.getGpsReadings()[5]
            x , y, Z_N,Ltt = utm.from_latlon(lat, lon)
            print("X: %s ,Y: %s  lat: %s ,lon: %s ,fixMode: %s ,Error: %s" % (x, y,lat,lon,fixMode,Error))
            
            sleep(0.1)
            
    elif mode == 2:
        while True:
            if imu.Readings !=None and imu.Rates !=None:
                angleOld = imu.Readings['Yaw']
                gyroZ = imu.Rates['gz']
                angleNew = int(toAnotherRange(angleOld,-180,180,0,9000))

                # SendDataOfType(addr,angleNew,bus,0)
                # SteeringAngleBytes = ConvertToBytes(angleNew+1)
                # RobotSpeedBytes = ConvertToBytes(angleNew+2)
                # BrakeValueBytes = (95+3)
                # totalPacket = [SteeringAngleBytes[0],SteeringAngleBytes[1],RobotSpeedBytes[0],RobotSpeedBytes[1],BrakeValueBytes]
                # SendDataOfType(addr,totalPacket,bus)
                print("Heading angle: %s     %s    %s" % (angleOld,angleNew,gyroZ))
            
            #sleep(0.1)
            
    elif mode == 3:
        routingClass = RoutingUsage("car")
        #Set This point as a Distination Point

        latDistination = gps.getGpsReadings()[1]
        longDistination = gps.getGpsReadings()[2]
        nodeNeedTOReach=routingClass.node(latDistination , longDistination)

        print(nodeNeedTOReach)
        while True:
            
            
            lat = gps.getGpsReadings()[1]
            lon = gps.getGpsReadings()[2]
           
            GPS_POINT_NOW =routingClass.node(gps.getGpsReadings()[1],gps.getGpsReadings()[2])
            
            angleReqToBe = sim.calAngle(GPS_POINT_NOW,nodeNeedTOReach)
            angleRover = None
            sleep(0.05)
            if imu.Readings !=None:
                angleRover = imu.Readings['Yaw']
                Error = angleReqToBe-angleRover

                print("angleRover: %s ,angleReqToBe: %s, Error: %s" %(angleRover,angleReqToBe,Error))
                #print("X: %s ,Y: %s" %(x,y))
    elif mode == 4:
        nLocations = int(input("Press Number Of Locations: "))
        routingMode = int(input("Press 1 for cycle mode or 2 for car mode: "))

        if routingMode == 1:
            routingClass = RoutingUsage("cycle")
        if routingMode == 2:
            routingClass = RoutingUsage("car")

        nodes = []
        #make Different Routing with multiple routing
        node1=routingClass.node(gps.getGpsReadings()[1],gps.getGpsReadings()[2])
        # node1=routingClass.node(30.0144890,31.1792321)
        print(node1)
        nodes.append(node1)
        for j in range(0,nLocations):
            latNext = float(input("Enter Lat Next Point: "))
            longNext = float(input("Enter Long Next Point: "))
        
            nodeNext=routingClass.node(latNext,longNext)
            #nodeNext=routingClass.node(51.9284338,4.4893559)
            nodes.append(nodeNext)

        nodes.reverse()
        nodesNew = routingClass.arrangeNodesDependsOnLength(nodes)

        queueNodesNewRight = routingClass.getRouteMultiple(nodesNew)

        if queueNodesNewRight == None:
            print("No Path for on of the Paths")
        else:
            f= open("route.osm","w+")


            f.write("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\r\n\r\n<osm version=\"0.6\" generator=\"Overpass API 0.7.55.7 8b86ff77\">\r\n\r\n<note>The data included in this document is from www.openstreetmap.org. The data is made available under ODbL.</note>\r\n\r\n<meta osm_base=\"2019-07-22T12:50:02Z\"/>\r\n\r\n  <bounds minlat=\"30.0059000\" minlon=\"31.1707000\" maxlat=\"30.0271000\" maxlon=\"31.2036000\"/>\r\n")

            for j in range(0,len(queueNodesNewRight)):
                for i in queueNodesNewRight[j][0]:
                    f.write("<node id=\"%d1%d3\" lat=\"%f\" lon=\"%f\" version=\"2\" timestamp=\"2019-05-10T12:30:39Z\" changeset=\"70107889\" uid=\"9535075\" user=\"JacksonWard\"/>\r\n" % (int(i[0]),j,i[1],i[2]))
            

            f.write(" <way id=\"348553335\" version=\"10\" timestamp=\"2019-04-08T04:19:00Z\" changeset=\"99992695\" uid=\"9990114\" user=\"The_Plateau\">\r\n")

            for j in range(0,len(queueNodesNewRight)):
                for i in queueNodesNewRight[j][0]:
                    f.write("<nd ref=\"%d1%d3\"/>\r\n" % (int(i[0]),j))
                
                
            f.write("</way>\r\n\r\n </osm>\r\n")
            f.close()
            listOfPoints = queueNodesNewRight[0][0] 

            sim.mainLoopForSendTheNeededLengthAndAngle(KpDistance,KpAngle,KpRate,gps,routingClass,listOfPoints,bus,addr,imu)

