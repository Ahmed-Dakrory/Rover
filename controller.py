from Routing import RoutingUsage # Import the router
import simulation as sim
import utm
import GPS as gps# Import the router
import smbusRead as smbusReader

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
        bus.write_i2c_block_data(address,0,data)
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

smbusReader = smbusReader.smbusRead(bus,addr)
smbusReader.start()
routingMode = 1 #int(input("Press 1 for cycle mode or 2 for car mode: "))

if routingMode == 1:
    routingClass = RoutingUsage("cycle")
if routingMode == 2:
    routingClass = RoutingUsage("car")
print("Loading the Map")
routingClass.getMap(0,0)

while True:
    print("Lets Start------------------>")
    
    sleep(2)
    nodes = []
    #make Different Routing with multiple routing
    node1=routingClass.node(gps.getGpsReadings()[1],gps.getGpsReadings()[2])
    # node1=routingClass.node(30.0144890,31.1792321)
    print(node1)
    nodes.append(node1)

    while True:
        if smbusReader.DataExist:
            Locations = smbusReader.Data.split(";")
            nLocations = len(Locations)
            print("You Will Go to %s Location" % (nLocations))
            for j in range(0,nLocations):
                posLatLong = Locations[j].split(",")
                print("Lat:%s  , long:%s"%(posLatLong[0],posLatLong[1]))
                latNext = float(posLatLong[0])
                longNext = float(posLatLong[1])
                nodeNext=routingClass.node(latNext,longNext)
                #nodeNext=routingClass.node(51.9284338,4.4893559)
                nodes.append(nodeNext)
            smbusReader.DataExist = False
            break
    # nLocations = int(input("Press Number Of Locations: "))
    
    # for j in range(0,nLocations):
    #     latNext = float(input("Enter Lat Next Point: "))
    #     longNext = float(input("Enter Long Next Point: "))
    
    #     nodeNext=routingClass.node(latNext,longNext)
    #     #nodeNext=routingClass.node(51.9284338,4.4893559)
    #     nodes.append(nodeNext)

    nodes.reverse()
    nodesNew = routingClass.arrangeNodesDependsOnLength(nodes)

    queueNodesNewRight = routingClass.getRouteMultiple(nodesNew)

    if queueNodesNewRight == None:
        print("No Path for on of the Paths")
        sim.sendActionsToMicroController([],0,0,0, 0,0,255,gps.getGpsReadings()[3],addr,bus)
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
        
        print('Finish Routing')
        print('Start Navigation')
        sim.mainLoopForSendTheNeededLengthAndAngle(KpDistance,KpAngle,KpRate,gps,routingClass,listOfPoints,bus,addr,imu,smbusReader)

