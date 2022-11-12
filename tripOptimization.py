from math import radians, cos, sin, asin, sqrt
import sys

MAX_OCCUPANTS=10
NUM_OF_VEHICLES=5

def distance(lat1, lat2, lon1, lon2):
     
    # The math module contains a function named
    # radians which converts from degrees to radians.
    lon1 = radians(lon1)
    lon2 = radians(lon2)
    lat1 = radians(lat1)
    lat2 = radians(lat2)
      
    # Haversine formula
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
 
    c = 2 * asin(sqrt(a))
    
    # Radius of earth in kilometers. Use 3956 for miles
    r = 6371
      
    # calculate the result
    return(c * r)

def graphGen(nodes):
    graph=[]
    for i in range(len(nodes)):
        temp=[]
        for j in range(len(nodes)):
            temp.append(distance(nodes[i][1],nodes[j][1],nodes[i][2],nodes[j][2]))
        graph.append(temp)
    return graph

class Graph():
 
    def __init__(self, vertices):
        self.V = vertices
        self.graph = [[0 for column in range(vertices)]
                      for row in range(vertices)]
 
    def printSolution(self, dist):
        print("Vertex \tDistance from Source")
        for node in range(self.V):
            print(node, "\t", dist[node])
 
    # A utility function to find the vertex with
    # minimum distance value, from the set of vertices
    # not yet included in shortest path tree
    def minDistance(self, dist, sptSet):
 
        # Initialize minimum distance for next node
        min = sys.maxsize
 
        # Search not nearest vertex not in the
        # shortest path tree
        for u in range(self.V):
            if dist[u] < min and sptSet[u] == False:
                min = dist[u]
                min_index = u
 
        return min_index
 
    # Function that implements Dijkstra's single source
    # shortest path algorithm for a graph represented
    # using adjacency matrix representation
    def dijkstra(self, src,visited):
        dist = [sys.maxsize] * self.V
        dist[src] = 0
        sptSet = [False] * self.V
 
        for cout in range(self.V):
 
            # Pick the minimum distance vertex from
            # the set of vertices not yet processed.
            # x is always equal to src in first iteration
            x = self.minDistance(dist, sptSet)
 
            # Put the minimum distance vertex in the
            # shortest path tree
            sptSet[x] = True
 
            # Update dist value of the adjacent vertices
            # of the picked vertex only if the current
            # distance is greater than new distance and
            # the vertex in not in the shortest path tree
            for y in range(self.V):
                if self.graph[x][y] > 0 and sptSet[y] == False and \
                        dist[y] > dist[x] + self.graph[x][y]:
                    dist[y] = dist[x] + self.graph[x][y]
 
        min=(0,dist[0])
        # print(len(dist))
        # print(min,visited)
        for count,item in enumerate(dist):
            if count in visited:
                continue
            if min[0] in visited:
                min=(count,item)
            if min[1]==0 or (item<min[1] and item!=0):
                min=(count,item)
        return min

def vertDict(people):
    vertDict=[]
    for i in range(len(people)):
        vertDict.append(people[i][0])
    return vertDict


def printTrip(trip):
    # print(trip)
    print(f"Start at Latitude {trip[0][1]} Longitude {trip[0][2]}")
    for i in range(1,len(trip)):
        print(f"Stop {i+1} is at Latitude {trip[i][1]} Longitude {trip[i][2]}")
    pass

if __name__=="__main__":
    start=[0,40.671851, -73.951535]
    raw=[[0,40.671851, -73.951535],[2,40.691638, -73.985181],[2,40.721307, -73.871198],[3,40.680183, -73.834806],[2,40.624963, -74.081998],[3,40.759286, -73.914457]]
    curTrip=0
    lastVisited=0
    trip=[[0,40.671851, -73.951535]]
    tripCount=1
    picked={0}
    while len(picked)!=len(raw):
        # print(curTrip)
        g=Graph(len(raw))
        tempGraph=graphGen(raw)
        g.graph=tempGraph
        toTravel=g.dijkstra(lastVisited,picked)
        if raw[toTravel[0]][0]+curTrip>MAX_OCCUPANTS:
            print(f"Trip number {tripCount}")
            printTrip(trip)
            print()
            tripCount+=1
            trip=[start]
        picked.add(toTravel[0])
        trip.append((raw[toTravel[0]]))
        curTrip+=(raw[toTravel[0]][0])
    print(f"Trip number {tripCount}")
    printTrip(trip)

    # trip.append((raw[lastVisited]))
    # print(f"Trip number {tripCount}")
    
