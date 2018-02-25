#!/usr/bin/python
import rospy
from network_utils.tree import Tree, TreeSegmention
from scipy.optimize import linear_sum_assignment
from network_utils.network import Network
import numpy as np
import math


#ROS Imports
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion 
from actionlib_msgs.msg import GoalStatusArray
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import MapMetaData
from move_base_msgs.msg import MoveBaseActionGoal

from util import dist_to_segment_alpha, dist_to_segment
from network_utils.rssi_kalman import RSSIKalmanFilter
from network_utils.Routing import Routing
from network_utils.RSSMeasure import RSSMeasure

from enum import IntEnum
import tf
import time

# Idle - Waits to join.
# Move - Moves towards a disconnected client.
# Disconnect - Moves backwards until get connected with the previous backbone.
# Connect- Stays fixed to provide a connection point.
class State(IntEnum):    
    IDLE       = 0
    MOVE       = 1
    DISCONNECT = 2
    CONNECT    = 3

class NeighborState(IntEnum):
    NONE         = 0
    CONNECTED    = 1
    DISCONNECTED = 2
    RECONNECTED  = 3

        
class Robot:
    def __init__(self):

        rospy.init_node('deployment_cefsm', anonymous=True)
        #print("ROBOT")
        self.network             = Network()
        self.network.addCommandCallback(self.receiveNetworkCommand)

        self.initialized         = False
        self.neighbors           = []
        self.neighbor_state      = {}

        ###
        ###         REAL ROBOT(1) OR SIMULATED (0)
        ###
        self.real_robot          = rospy.get_param("~real_robot", False)
        # if(self.real_robot):
        #     self.config_file     = rospy.get_param("~config_file")
        #     self.routing         = Routing('teste4', self.config_file)
        #     self.rss_measure     = RSSMeasure('teste4', self.config_file)



        id                       = rospy.get_param("~id")
        self.lower_threshold     = rospy.get_param("~lower_threshold")
        self.higher_threshold    = rospy.get_param("~higher_threshold")

        self.state = State.IDLE

        self.send_position_time_diff = rospy.get_param("~pose_send_time", 0.1)
        self.tree_file           = rospy.get_param("~tree_file")
        self.radius              = rospy.get_param("~radius", 10)
        self.vote_distance       = 3

        self.map_resolution      = 0.5
        self.height              = 0

        self.send_position_time  = rospy.get_time() 

        self.tree                = Tree(self.tree_file)
        self.clients             = set(self.tree.clients)
        self.initializeClients()
  
        self.robots_ids_start    = len(self.tree.vertices)

        self.id                  = int(id) + self.robots_ids_start
        self.position            = {}
        self.position['id']      = self.id
        self.position['position']= (0.0, 0.0, 0.0)
        self.position['state']   = int(self.state)
        self.position['started'] = 0

        self.closet_client       = -1
        
        self.connected_clients   = set([])
        self.last_connected_clients = set([])
        self.route_to_disconected= {}
        self.disconnected        = -1


        self.ros_id              = self.id - self.robots_ids_start  
        prefix                   = "/robot_"+str(self.ros_id)

        rospy.Subscriber(prefix+"/amcl_pose", PoseWithCovarianceStamped, self.getPose)
  
        self.cancel_pub          = rospy.Publisher(prefix + "/move_base/cancel", GoalID, queue_size=10)
        self.current_goal_id     = 0
        self.goal_pub            = rospy.Publisher(prefix+"/move_base/goal", MoveBaseActionGoal, queue_size=10)
        rospy.Subscriber(prefix+"/move_base/status", GoalStatusArray, self.getStatus)
        rospy.Subscriber("/map_metadata", MapMetaData, self.getMap)

        rospy.Timer(rospy.Duration(0.3), self.simulationMetric)

        self.metric_kalman       = {}
        self.gamma               = 3
        
        if(self.real_robot):
            self.updateRouting()


    def updateRouting(self):
        threading.Timer(1, self.updateMap).start()
        graph = self.createRoutingGraph()
        self.routing.createRoute(graph)


    def initializeClients(self):
        for client in self.clients:
            message = {}
            message['position'] = self.tree.graph_vertex_position[client]
            message['id']       = client
            message['state']    = int(State.CONNECT)
            message['routing']  = []
            self.network.addMessage(message) 

    
    def createRoutingGraph(self):
        graph = {}
        for id in self.network.rcv_data:
            graph[id] = set([])

        graph[self.id] = set([])

        for id in self.network.rcv_data:
            if(self.network.rcv_data[id]['state'] != State.CONNECT):
                continue
            graph[id] = graph[id].union(set(self.network.rcv_data[id]['routing']))
            for neigbor in self.network.rcv_data[id]['routing']:
                graph[neigbor] = graph[neigbor].union(set([id]))

        if(with_my_self):
            graph[self.id] = graph[self.id].union(self.neighbors)

            for neigbor in self.neighbors:            
                graph[neigbor] = graph[neigbor].union(set([self.id]))


        for id in self.network.rcv_data:
            graph[id] = list(graph[id])
        return graph


    def receiveNetworkCommand(self):
        print(self.network.rcv_command)

    def createRoutingTable(self):
        #neighbors_ids, routing = robot.getNeighborsIDs()
        pass

    def logNormalMetric(self, distance, variance):
        if(distance < 1):
            return -40
        return -40 -10*self.gamma*math.log(distance) + np.random.normal(0,math.sqrt(variance),1)[0]


    # def realMetric(self, param):

    #     variance = 1.0
    #     #for the robots
    #     for data_id in self.network.rcv_data:
    #         #if(data_id == self.id):
    #         #    continue
    #         real_distance    = self.getDistance(self.position['position'], self.network.rcv_data[data_id]['position'])*self.map_resolution
    #         real_metric      = self.rss_measure.getMeasurement(data_id)
    #         #simulated_metric = self.logNormalMetric(real_distance, variance) #real_distance + np.random.normal(0,variance,1)[0]
            
    #         if( data_id not in self.metric_kalman):
    #             self.metric_kalman[data_id]   =  RSSIKalmanFilter([-40.0, 2.4], 10.0, variance)

    #         if(real_metric > self.rss_measure.MAX):
    #             self.metric_kalman[data_id].addMeasurement(real_distance, real_metric)


    #     #for the tree
    #     for vertex in self.tree.graph_adj_list:
    #         ##position = (self.tree.graph_vertex_position[vertex][0], self.height - self.tree.graph_vertex_position[vertex][1])
    #         real_distance    = self.getDistance(self.position['position'], position)*self.map_resolution
    #         real_metric      = self.rss_measure.getMeasurement(vertex)

    #         if( vertex not in self.metric_kalman):
    #             self.metric_kalman[vertex] =  RSSIKalmanFilter([-40.0, 2.4], 10.0, variance)
    #         if(real_metric > self.rss_measure.MAX):
    #             self.metric_kalman[vertex].addMeasurement(real_distance, real_metric)


    def simulationMetric(self, param):

        variance = 1.0
        #for the robots
        for data_id in self.network.rcv_data:
            #if(data_id == self.id):
            #    continue
            real_distance    = self.getDistance(self.position['position'], self.network.rcv_data[data_id]['position'])*self.map_resolution
            simulated_metric = self.logNormalMetric(real_distance, variance) #real_distance + np.random.normal(0,variance,1)[0]
            

            #0-time, 1-realposition, 2-neighposition, 3-real_distance, 4-simulated_metric
            #self.metric_measurements[data_id] = (rospy.get_time(), self.position['position'], self.network.rcv_data[data_id]['position'], real_distance, simulated_metric)
            if( data_id not in self.metric_kalman):
                self.metric_kalman[data_id]   =  RSSIKalmanFilter([-40.0, 2.4], 10.0, variance)


            m, P = self.metric_kalman[data_id].getResult()
            #print(m)

            self.metric_kalman[data_id].addMeasurement(real_distance, simulated_metric)


        #for the tree
        for vertex in self.tree.graph_adj_list:
            real_distance    = self.getDistance(self.position['position'], self.tree.graph_vertex_position[vertex])*self.map_resolution
            simulated_metric = self.logNormalMetric(real_distance, variance)#real_distance + np.random.normal(0,variance,1)[0]
            #self.metric_measurements[vertex] = (rospy.get_time(), self.position['position'], self.tree.graph_vertex_position[vertex], real_distance, simulated_metric)

            if( vertex not in self.metric_kalman):
                self.metric_kalman[vertex] =  RSSIKalmanFilter([-40.0, 2.4], 10.0, variance)

            self.metric_kalman[vertex].addMeasurement(real_distance, simulated_metric)

    def getStatus(self, Status):

        if(len(Status.status_list) > 0):
            self.status = Status.status_list[0].status
            #self.current_goal_id = Status.status_list[0].goal_id

    def getMap(self, MapData):

        self.map_resolution = MapData.resolution
        self.height         = MapData.height


    def sendDeployment(self, deployment_position):


        #print(deployment_position)
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = deployment_position[0]*self.map_resolution
        pose.pose.position.y = (self.height - deployment_position[1])*self.map_resolution

        print(pose.pose.position)

        #for debug
        self.position['destination'] = (deployment_position[0], deployment_position[1])

        print(pose.pose.position.x, pose.pose.position.y)
        q = tf.transformations.quaternion_from_euler(0, 0, 0)
        pose.pose.orientation = Quaternion(*q)


        goal_id = GoalID()
        goal_id.id = str(self.current_goal_id)

        self.current_goal_id +=1


        goal = MoveBaseActionGoal()
        goal.goal_id = goal_id
        goal.goal.target_pose = pose


        self.goal_pub.publish(goal)



    def getPose(self, Pose):
        orientation = (
            Pose.pose.pose.orientation.x,
            Pose.pose.pose.orientation.y,
            Pose.pose.pose.orientation.z,
            Pose.pose.pose.orientation.w)
        orientation_euler = tf.transformations.euler_from_quaternion(orientation)

        self.position['position'] = (Pose.pose.pose.position.x/self.map_resolution, self.height- Pose.pose.pose.position.y/self.map_resolution, orientation_euler[2])
        
        if(self.state == State.CONNECT):
            self.position['routing']  = self.neighbors
        else:
            self.position['routing']  = []

        self.position['state']    = int(self.state)
        #print(self.position)
        #print(self.position['position'])
        #avoid to flood the network with messages
        if(rospy.get_time() - self.send_position_time > self.send_position_time_diff or not self.initialized):
            self.network.sendMessage(self.position)
            self.send_position_time__ = rospy.get_time()

        self.initialized = True
                
 
    def getDistance(self, p1, p2):
        return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

    def distance_metric(self, id):
        p0       = self.network.rcv_data[id]['position']
        p1       = self.position['position']
        distance = self.getDistance(p0, p1)*self.map_resolution
        #print('distance ', distance, 'id ', id)
        if(id in self.metric_kalman):
            return self.metric_kalman[id].getMetricValue(distance)
        else:
            return -1000

    def defineNeighbors(self):
        neighbors = []
        for id in self.network.rcv_data:
            if(id == self.id):
                continue
            if(not id in self.neighbor_state):
                self.neighbor_state[id] = NeighborState.DISCONNECTED

            #print(self.distance_metric(id), self.lower_threshold, self.higher_threshold)
            if(self.neighbor_state[id] == NeighborState.CONNECTED):
                if(self.distance_metric(id) < self.lower_threshold and self.state == State.MOVE):
                    self.neighbor_state[id] = NeighborState.DISCONNECTED


            if(self.neighbor_state[id] == NeighborState.DISCONNECTED):
                if(self.distance_metric(id) > self.higher_threshold and self.network.rcv_data[id]['state'] == State.CONNECT):
                    self.neighbor_state[id] = NeighborState.CONNECTED

            if(self.neighbor_state[id] == NeighborState.CONNECTED):
                neighbors.append(id)
            #print('dsin' , self.distance_metric(id), self.neighbor_state[id])

        self.neighbors = neighbors

    def defineGraph(self, with_my_self=True):
        graph = {}
        for id in self.network.rcv_data:
            graph[id] = set([])

        graph[self.id] = set([])

        for id in self.network.rcv_data:
            if(self.network.rcv_data[id]['state'] != State.CONNECT):
                continue
            graph[id] = graph[id].union(set(self.network.rcv_data[id]['routing']))
            for neigbor in self.network.rcv_data[id]['routing']:
                graph[neigbor] = graph[neigbor].union(set([id]))

        if(with_my_self):
            graph[self.id] = graph[self.id].union(self.neighbors)

            for neigbor in self.neighbors:            
                graph[neigbor] = graph[neigbor].union(set([self.id]))


        for id in self.network.rcv_data:
            graph[id] = list(graph[id])
        return graph

    def searchGraph(self, graph, next_id, visited, client_list):
        clients = set([])
        for id in graph[next_id]:
            if(not id in visited):

                new_clients = self.searchGraph(graph, id, visited.union(set([id])), client_list)
                clients = clients.union(new_clients)
                if(id in self.clients):
                    clients = clients.union(set([id]))
                    #print("new connection", clients)
                    client_list[id] = visited



        return clients

    def precomputeGraph(self):

        if(self.state == State.MOVE):
            self.route_to_disconected = self.neighbors

        self.defineNeighbors()
        self.graph = self.defineGraph()
        #print(self.graph)
        clients_connections = {}
        
        connected_clients = self.searchGraph(self.graph, self.id, set([]), clients_connections)
        print('connected clients', connected_clients)
        
        self.clients_connections = clients_connections

        if(self.state == State.MOVE):
            if(self.last_connected_clients - connected_clients == set([])):
                self.last_connected_clients = connected_clients

        if(self.connected_clients - connected_clients == set([])):
            self.connected_clients = connected_clients

        else:
            #self.last_connected_clients = self.connected_clients
            self.connected_clients = connected_clients



    ##
    ##          Defining the actions
    ##
    # Move - Moves to the closest disconnected client through the Steiner tree.
    # Move Backwards - Moves towards a node to rejoin the network.
    # Stall - Stands still

    def getClosestDisconnected(self):
        disconnected_clients = self.clients - self.connected_clients
        #print('disconnected ', disconnected_clients, self.connected_clients)
        min_id   = -1
        min_dist = float('inf')
        for client in disconnected_clients:
            dist     = self.getDistance(self.position['position'], self.tree.graph_vertex_position[client])
            if(dist < min_dist):
                min_id = client
                min_dist = dist
        return min_id


    def Move(self):
        #get closest disconnected client
        closet_client = self.getClosestDisconnected()
        #print(closet_client)
        if(closet_client != self.closet_client):
            self.closet_client = closet_client
            if(closet_client >= 0):
                self.sendDeployment(self.tree.graph_vertex_position[closet_client])
                self.position['started'] = 1

    def MoveBackwards(self):
        disconnected = list(self.last_connected_clients - self.connected_clients)
        print('discon: ', disconnected, ' ', self.disconnected, 'last', self.last_connected_clients, self.connected_clients)
        if( disconnected != [] and self.disconnected != disconnected[0]):
            self.disconnected = disconnected[0]
            self.sendDeployment(self.tree.graph_vertex_position[self.disconnected])
        if(disconnected == []):
            self.disconnected = -1


    def Stall(self):
        goal = GoalID()
        goal.id = str(self.current_goal_id-1)
        print('stall', self.current_goal_id-1)
        self.cancel_pub.publish(goal)

    ##  
    ##  defining the events
    ##
       
    # All clients connected- Verifies that the network is fully connected.
    # Disconnected to client - Some client lost network connection.

    # Reconnected to client - A client reconnected to network.
    # Belongs to solution - There will be network partition if robot disjoin the network.
    # Route to client changed - A client reconnected to network and its route changed.
    # Win vote - Win a competition for connect state
    # Lose vote Lose a competition for connect state


    def AllClientsConnected(self):
        print('connected ', self.connected_clients)
        if self.clients == self.connected_clients:
            return True
        return False

    def DisconnectedToClient(self):
        if((self.last_connected_clients - self.connected_clients) != set([])):
            print("disconnect", self.last_connected_clients, self.connected_clients, self.clients_connections)
            #self.disconnected = list(self.last_connected_clients - self.connected_clients)[0]

            return True
        return False

    def ReconnectedToClient(self):
        print('reconnected ', self.last_connected_clients, self.connected_clients)
        if(self.last_connected_clients - self.connected_clients == set([])):
            return True
        return False
        
    def BelongsToSolution(self):
        graph = self.defineGraph(False)
        clients_connections = {}
        connected_clients = self.searchGraph(graph, list(self.clients)[0], set([]), clients_connections)

        connected_clients = connected_clients.union(set([list(self.clients)[0]]))
        if(self.connected_clients == connected_clients):
            return False
        return True


    def RouteToClientChanged(self):
        print("route change", self.route_to_disconected, self.neighbors)
        if(self.route_to_disconected == self.neighbors):
            return False
        return True


    def WinVote(self):
        for id in self.network.rcv_data:
            if(self.network.rcv_data[id]['state'] == State.DISCONNECT or self.network.rcv_data[id]['state'] == State.MOVE):
                if(id != self.id):
                    if(self.getDistance(self.position['position'], self.network.rcv_data[id]['position'])*self.map_resolution < self.vote_distance):
                        if(self.id > id):
                            return False
        return True

        
    def LoseVote(self):
        return not self.WinVote()


    def CEFSM(self):
        self.precomputeGraph() #muda estado da classe
        print('State ', self.state, self.ros_id )
        if self.state == State.IDLE:
            if(not self.AllClientsConnected()):
                self.state = State.MOVE


        elif self.state == State.MOVE:
            self.Move()

            print('allclient', self.AllClientsConnected(),'win', self.WinVote(), 'belong', self.BelongsToSolution())
            if(self.DisconnectedToClient()):#muda estado da classe
                self.MoveBackwards()
                self.state = State.DISCONNECT 

            elif(self.AllClientsConnected() and self.WinVote() and self.BelongsToSolution()):
                self.Stall()
                self.state = State.CONNECT

            elif(self.AllClientsConnected() and not self.BelongsToSolution()):
                self.Stall()
                self.state = State.IDLE

        elif self.state == State.CONNECT:
            pass

        elif self.state == State.DISCONNECT:
            self.MoveBackwards()
            if(self.ReconnectedToClient() and (self.RouteToClientChanged() or self.LoseVote())):
                #TODO: update neigbor state
                self.state = State.MOVE 

            elif(self.ReconnectedToClient() and self.WinVote()):
                self.Stall()
                self.state = State.CONNECT






 
        
if __name__ == "__main__":
    robot = Robot()
    time.sleep(20 + robot.ros_id*20)
    rate = rospy.Rate(25.0)
    while not rospy.is_shutdown():
        now = rospy.get_rostime()
        #print('send')
        robot.CEFSM()
        rate.sleep()
        #print(now-rospy.get_rostime())
