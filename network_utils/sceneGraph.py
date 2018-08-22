from scipy import misc
from math import floor, sqrt, sin, cos, pi, ceil
import numpy as np
from tree import Tree, TreeSegmention
from pyChristofides import christofides


class sceneGraph:
    def __init__(self, config_file, comm_radius, client_pos, offset=(0, 0)):
        self.map_file         = config_file['map']
        self.map              = misc.imread(self.map_file, 'L')
        self.comm_radius      = int(ceil(sqrt(((comm_radius-2)**2)/2.0)))
        self.graph            = {}
        self.vertice_position = {}
        self.client_pos       = client_pos
        self.offset           = offset
        self.obst_dist        = 20
        self.getMinMax(client_pos)

        #self.createGraph()
        self.createGraphTree(config_file['treefile'])

    def getMinMax(self, dimensions):
        self.min_ = [float('inf'), float('inf')]
        self.max_ = [float('-inf'), float('-inf')]
        for p in  dimensions:
            self.min_[0] = min(self.min_[0], p[0] - self.comm_radius)
            self.min_[1] = min(self.min_[1], p[1] - self.comm_radius)
            self.max_[0] = max(self.max_[0], p[0] + self.comm_radius)
            self.max_[1] = max(self.max_[1], p[1] + self.comm_radius)
        #print(self.min, self.max)


    def verifyObstacleProximity(self, x, y, distance):
        dimensions = self.map.shape
        for angle in np.linspace(0,2*pi, 10):
            xx = int(x + sin(angle)*distance)
            yy = int(y + cos(angle)*distance)

            if(xx > dimensions[0] or xx < 0):
                return False

            if(yy > dimensions[1] or yy < 0):
                return False

            if self.map[xx, yy] < 127:
                 return False

        return True


            


    def isValid(self, position):
        if( not self.verifyObstacleProximity(position[0], position[1], self.obst_dist)):
            return False

        if(position[0] > self.min_[0] and position[0] < self.max_[0]):
            if(position[1] > self.min_[1] and position[1] < self.max_[1]):
                return True

        return False

    def createGraphTree(self, tree_file):

        dimensions = self.map.shape
        self.heigh = dimensions[0]

        self.tree                = Tree(tree_file)
        self.tree_segmentation   = TreeSegmention(self.tree)



        allocation = np.matrix(self.tree_segmentation.doAllocation(self.comm_radius))


        #print('a', self.positions - [[1,2]], [[self.vertice_position[0][0], self.vertice_position[0][1]]])
        positions = np.empty(shape=[0, 2])

        id_ = 0
        for client in self.client_pos:

            client_pos = (client[0], client[1])
            distance = float('inf')
            for i in range(allocation.shape[0]):
                d = sqrt((allocation[i,0]-client[0])**2 + (allocation[i,1]-client[1])**2)
                distance = min(d, distance)

            if(distance < 30):
                continue

            self.vertice_position[id_] = (client[0], client[1])            

            positions = np.append(positions, [[client[0], client[1]]], axis=0)
            self.graph[id_] = []
            id_ += 1
        #print(positions)

        for i in range(allocation.shape[0]):
            self.vertice_position[id_] = (allocation[i,0], allocation[i,1])
            positions = np.append(positions, [[allocation[i,0], allocation[i,1]]], axis=0)
            self.graph[id_] = []
            id_ += 1
        self.positions = positions
        #print(positions)
        comm_radius = int(sqrt(self.comm_radius**2 + self.comm_radius**2))+1


        for id in self.graph:

            #print('teste', [[self.vertice_position[id][0], self.vertice_position[id][1]]])
            distance    = np.sqrt(np.sum(np.power(self.positions - [[self.vertice_position[id][0], self.vertice_position[id][1]]],2), axis=1))
            #print(self.comm_radius)
            connections = np.where(distance <= comm_radius)[0].tolist()
            graph_      = {}

            for conn_id in connections:
                if(conn_id != id):
                    graph_[conn_id] = int(distance[conn_id])

            self.graph[id] = graph_

        print(self.graph)


    def createGraph(self):
        dimensions = self.map.shape
        self.heigh      = dimensions[0]

        node_id = 0
        comm_radius = self.comm_radius

        for x in range(dimensions[0]//comm_radius):
            for y in range(dimensions[1]//comm_radius):
                xx = self.offset[0] + x*comm_radius
                yy = self.offset[1] + y*comm_radius

                if(self.map[xx, yy] < 127) or not self.isValid((xx, yy)):
                    continue
                #print((x*comm_radius, y*comm_radius), 'aass')

                self.graph[node_id]              = []
                self.vertice_position[node_id]   = (yy, xx)
                node_id                         += 1


        #create a matrix containing all the positions
        positions = np.empty(shape=[0, 2])

        for id in self.graph:
            positions = np.append(positions, [[self.vertice_position[id][0], self.vertice_position[id][1]]], axis=0)

        self.positions = positions

        comm_radius = int(sqrt(self.comm_radius**2 + self.comm_radius**2))+1
        for id in self.graph:

            distance    = np.sqrt(np.sum(np.power(positions - [[self.vertice_position[id][0], self.vertice_position[id][1]]],2), axis=1))
            connections = np.where(distance <= comm_radius)[0].tolist()
            graph_      = {}

            for conn_id in connections:
                if(conn_id != id):
                    graph_[conn_id] = distance[conn_id]

            self.graph[id] = graph_

    def getDistanceFromId(self, position, id):
        try:
            return sqrt((self.positions[id][0]-position[0])**2 + (self.positions[id][1] - position[1])**2)
        except:
            print("testing")
            print(self.positions[id][0]-position[0])
            exit()

    def getClosestNode(self, position):
        distance    = np.sqrt(np.sum(np.power(self.positions - [[position[0], position[1]]],2), axis=1))
        #print(self.positions, 'distance')
        id = np.argmin(distance)
        #print(position, self.positions[id])
        return id

    def dijkstra(self, graph, src, dest, result = {}, visited=[],distances={},predecessors={}):
        """ calculates a shortest path tree routed in src
        """    
        # a few sanity checks
        if src not in graph:
            raise TypeError('The root of the shortest path tree cannot be found')
        if dest not in graph:
            raise TypeError('The target of the shortest path cannot be found')    
        # ending condition
        if src == dest:
            # We build the shortest path and display it
            path=[]
            pred=dest
            while pred != None:
                path.append(pred)
                pred=predecessors.get(pred,None)
            result['path'] = path
            result['cost'] = distances[dest]

        else :     
            # if it is the initial  run, initializes the cost
            if not visited: 
                distances[src]=0
            # visit the neighbors
            for neighbor in graph[src] :
                if neighbor not in visited:
                    new_distance = distances[src] + graph[src][neighbor]
                    if new_distance < distances.get(neighbor,float('inf')):
                        distances[neighbor] = new_distance
                        predecessors[neighbor] = src
            # mark as visited
            visited.append(src)
            # now that all neighbors have been visited: recurse                         
            # select the non visited node with lowest distance 'x'
            # run Dijskstra with src='x'
            unvisited={}
            for k in graph:
                if k not in visited:
                    unvisited[k] = distances.get(k,float('inf'))        
            x=min(unvisited, key=unvisited.get)
            self.dijkstra(graph,x,dest,result,visited,distances,predecessors)

    def getDistance(self, id1, id2):
        return sqrt((self.positions[id1][0]-self.positions[id2][0])**2 + (self.positions[id1][1]-self.positions[id2][1])**2)

    def getShortestPath(self, src, dst):
        result = {}
        self.dijkstra(self.graph, src, dst, result, [], {}, {})
        return result['path']

    def createFullConnectedGraph(self, nodes):
        self.full_graph      = {}
        for i in nodes:
            graph_ = {}
            for j in nodes:
                if i == j:
                    continue
                result = {}
                #print('testee', i, j)
                self.dijkstra(self.graph, i, j, result, [], {}, {})
                graph_[j] = result['cost']
            self.full_graph[i] = graph_

        return self.full_graph

    def calcEclideanDist(self, p1, p2):
        return sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2) 

    def calculateTSP(self, nodes):

        if(nodes == []):
            return []

        full_graph  = self.createFullConnectedGraph(nodes)
        #print(full_graph)
        i           = 0
        map_id      = {}
        map_id_node = {}
        n           = len(full_graph)
        dist_graph  = {}
        nodes_      = []


        # for node in nodes:            
        #     nodes_.append(self.vertice_position[node])

        # distances = []
        # for i in range(len(nodes_)):
        #     dist_array = []
        #     for j in range(len(nodes_)):
        #         dist = self.calcEclideanDist(nodes_[i], nodes_[j]) 
        #         dist_array.append(dist)
        #     distances.append(dist_array)

        # TSP = christofides.compute(distances)
        # t = TSP['Christofides_Solution']

        # path = []
        # for p in t:
        #     path.append(nodes[p])

        # #if(len(path) > 1):
        # #    path.append(path[0])
        # return path


        for node in nodes:            
            nodes_.append(self.vertice_position[node])


        for i in range(len(nodes_)):
            for j in range(len(nodes_)):
                if(i == j):
                    continue
                dist_graph[(i,j)] = full_graph[nodes[i]][nodes[j]]

        t = tsp.tsp(nodes_, dist_graph)
        path = []
        for p in t[1]:
            path.append(nodes[p])

        if(len(path) > 1):
            path.append(path[0])
        return path

