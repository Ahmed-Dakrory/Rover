from pyroutelib3 import Router # Import the router

class RoutingUsage:
    def __init__(self,Mode):
        self.router = Router(Mode) # Initialise it
    
    def node(self,lat,longit):
        return [lat,longit]

    def getMap(self,lat,longit):
        self.router.getArea(lat,longit) #Load map for only checking

    def getRouteMultiple(self,nodesNew):
        queueNodesNewRight = []
        for index in range(0,len(nodesNew)-1):
            nodeStart = nodesNew[index]
            nodeEnd = nodesNew[index+1]
            route = self.getTheRouteBetweenTwoNodes(nodeStart[0],nodeStart[1],nodeEnd[0],nodeEnd[1])
            if len(route[0]) == 0 and route[1] > 0:
                return None
            queueNodesNewRight.append(route)
        return queueNodesNewRight
    
    def arrangeNodesDependsOnLength(self,nodes):
        for nodeStartIndex in range(0,len(nodes)-1): 
            lastDistance = self.router.distance(nodes[nodeStartIndex],nodes[nodeStartIndex+1])
            for nodeNowIndex in range(nodeStartIndex+1,len(nodes)):
                theReturnedNodeWhichisNearst=nodes[nodeStartIndex+1]
                nowLength = self.router.distance(nodes[nodeStartIndex],nodes[nodeNowIndex])
                #print(nowLength)
                theReturnedNodeIndex = nodeStartIndex
                if nowLength < lastDistance :
                    theReturnedNodeWhichisNearst = nodes[nodeNowIndex]
                    theReturnedNodeIndex = nodeNowIndex
                    lastDistance = self.router.distance(nodes[nodeStartIndex],nodes[nodeNowIndex])
            ReserveNode = nodes[nodeStartIndex+1]
            nodes[nodeStartIndex+1] = nodes[theReturnedNodeIndex]
            nodes[theReturnedNodeIndex] = ReserveNode
            #print("length: %f"%lastDistance)
        return nodes

    def getTheRouteBetweenTwoNodes(self,lat1,long1,lat2,long2):
        print('Check Points in the maps Stage .....')
        
        
        checkStart = self.router.getPointInBounds(lat1,long1) # Check if the two points in the map


        checkEnd = self.router.getPointInBounds(lat2,long2)
        # print(checkStart)
        # print(checkEnd)

        if checkStart and checkEnd:
            print('Continue Routing Find Near Points Stage .....')
            start = self.router.findNode(lat1,long1) # Find start and end nodes
            end = self.router.findNode(lat2,long2)
            
            print('Continue Routing one Path Stage .....')

    ##        print("start : %s,   Lat: %s,  Lon: %s "% (start,lat1,long1))
    ##        print("end : %s,   Lat: %s,  Lon: %s "% (end,lat2,long2))
            

            status, route = self.router.doRoute(start, end) # Find the route - a list of OSM nodes
        else:
            print('outside Map Range')
            status = 'outOfRange'
            
        if status == 'success':
            print('Continue Routing Successing Stage.....')
            routeLatLons = list(map(self.router.nodeLatLon, route)) # Get actual route coordinates
            
            # list the lat/long
            queueNodes = []
            sumPath = 0
            l = len(route)
            for index, obj in enumerate(route):
                thisElement = route[index]
                newDistance = 0
                if index < l-1 :
                  nextElement = route[index+1]
                  thisElementD = [self.router.nodeLatLon(thisElement)[0],self.router.nodeLatLon(thisElement)[1]]
                  nextElementD = [self.router.nodeLatLon(nextElement)[0],self.router.nodeLatLon(nextElement)[1]]
                  newDistance = self.router.distance(nextElementD,thisElementD)
                  sumPath = sumPath + newDistance
                elif index == l -1:
                  nextElement = route[index-1]

                  
                typeData = self.router.getNodeWay(thisElement,nextElement)
                #get width Depends on the Category
                width = self.router.getRouteWidth(typeData["tag"]["highway"])

                #get width Depends on the way lanes numbers
                NumberOfLanes = typeData["tag"].get("lanes")

                #Const Lanes Width it will be 3 meter
                laneWidth = 3/12742/6*1.1 #Meter
                if NumberOfLanes != None:
                    width = int(NumberOfLanes)*laneWidth

                
                #get width Depends on the way width
                widthUnCalibrated = typeData["tag"].get("width")
                if widthUnCalibrated != None:
                    width = float(widthUnCalibrated)/12742/6*1.1

                nodeNow=self.router.nodeLatLon(thisElement)
                nodeNext=self.router.nodeLatLon(nextElement)
                queueNodes.append([route[index], nodeNow[0],nodeNow[1],width])
                if newDistance > 0.009:
                  newNodesBetween = self.router.getNumberOfNodesBetweenThose(7, nodeNow,nodeNext) 
                  for nodeBet in newNodesBetween:
                    queueNodes.append([str(index)+str(nodeBet[0])+"975", nodeBet[1],nodeBet[2],width])

            #/////////////////////////////////////////////////Shift the Nodes
            
            queueNodesNewRight = []
           
            
            print('Continue Routing Shifting Stage.....')
            for index, obj in enumerate(queueNodes):
               
                lV = len(queueNodes)
                if index < lV-1 :
                    nextElement = [queueNodes[index+1][1],queueNodes[index+1][2]]
                    nextElementId = queueNodes[index+1][0]
                    thisElement = [queueNodes[index][1],queueNodes[index][2]]
                    thisElementId = queueNodes[index][0]
                    newDistance = self.router.distance(thisElement,nextElement)
                elif index == lV -1:
                    nextElement = [queueNodes[index][1],queueNodes[index][2]]
                    thisElement = [queueNodes[index][1],queueNodes[index][2]]
                
                
                newNode = self.router.getLatLongWithNewWidth(queueNodes[index][3],newDistance, thisElement,nextElement)
                queueNodesNewRight.append([queueNodes[index][0], newNode[0],newNode[1],queueNodes[index][3]])
            return [queueNodesNewRight,sumPath]
        else:
            node1=self.node(lat1,long1)
            node2=self.node(lat2,long2)
            return [[],self.router.distance(node1,node2)]
            
