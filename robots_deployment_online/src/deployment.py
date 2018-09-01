#!/usr/bin/python
import rospy
from network_utils.tree import Tree, TreeSegmention
from scipy.optimize import linear_sum_assignment
from network_utils.network import Network
import numpy as np
import math
import yaml
from math import sqrt


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
from network_utils.rssmeasure_v2 import RSSMeasure
from enum import IntEnum

import time
import tf

from network_utils.bspline import BSpline
import os
import subprocess as sub

class COMMANDS(IntEnum):    
    SETINITALPOSE         = 0
    STARTDEPLOYMENT       = 1
    EXECCOMMAND           = 2


        
class Robot:
    def __init__(self):
        rospy.init_node('robot_deployment', anonymous=True)
        print("INITIALIZING ROBOT ....")
        ####
        ####  ALGORITHM PARAMETERS
        ####

        self.alpha          = 0.45*(1.0/100.0) #0.5*(1.0/100.0)#1.0    ## the importance of following the objective
        self.beta           = 1.8  #2.0               ## the importance of following the path
        self.robot_center_y = 0.6                     ## how fast the robot turns
        self.dead_velocity  = 0.15


        ###
        ###         REAL ROBOT(1) OR SIMULATED (0)
        ###
        self.real_robot          = rospy.get_param("~real_robot", 0)
        self.config_file         = rospy.get_param("~config_file")
        self.config              = self.readConfig(self.config_file)

        self.real_robot          = not self.config['configs']['simulation'] 

        if(self.real_robot):        	
            self.routing         = Routing('teste4', self.config_file, 'ra0')
            self.rss_measure     = RSSMeasure('teste4', self.config_file)
            id                   = self.routing.getID()
        else:
       	    id                   = rospy.get_param("~id")


       	#configure network
        self.network             = Network(id=-1, broadcast_addr = self.config['configs']['broadcast_address'], port = self.config['configs']['algorithm_port'])
        self.network.addCommandCallback(self.receiveNetworkCommand)

        
        self.log_rss             = False

        self.neigh_0             = -1
        self.neigh_1             = -1
        
        self.allocation_id       = -1        
        self.initialized         = False
        self.high_level_distance = 3
        self.sent_goal           = 0

        self.covariance          = np.matrix([[0.0, 0.0], [0.0, 0.0]])

        self.send_position_time_diff = rospy.get_param("~pose_send_time", 0.1)
        self.tree_file           = self.config['configs']['treefile']
        self.radius              = rospy.get_param("~ray", 120)


        self.map_resolution      = 0.5
        self.height              = 529

        self.send_position_time  = rospy.get_time() 

        self.tree                = Tree(self.tree_file)
        self.tree_segmentation   = TreeSegmention(self.tree)
        self.tree_segmentation_segments = []


        self.robots_ids_start    = len(self.tree.vertices)


        if(self.real_robot):
        	self.id              = int(id) - 1
        else:
        	self.id              = int(id) + self.robots_ids_start


        self.position            = {}
        self.position['id']      = self.id
        self.position['position']= (0.0, 0.0, 0.0)
        self.position['started'] = 0
        self.position['diff']    = -1
        self.position['s_size']  = 10000
        self.position['radius']  = 1

        self.solution_size       = 10000
        
        self.metric_measurements = {}

        self.num_robots          = 0
        self.deployment_position = []

        self.status              = -1
        self.allocation_position = -1
        self.started_control     = False
        self.max_linear_vel      = 0.4

        self.ros_id              = self.id - self.robots_ids_start

        if(self.real_robot):
            prefix               = rospy.get_param("~prefix")
        else:
            prefix               = "/robot_"+str(self.ros_id)

        self.map_info = False
        rospy.Subscriber("/map_metadata", MapMetaData, self.getMap)
        while(not self.map_info and not rospy.is_shutdown()):
            time.sleep(0.5)

        rospy.Subscriber(prefix+"/amcl_pose", PoseWithCovarianceStamped, self.getPose)
        self.vel_pub             = rospy.Publisher(prefix + "/cmd_vel", Twist, queue_size=10)

        self.cancel_pub          = rospy.Publisher(prefix + "/move_base/cancel", GoalID, queue_size=10)
        self.current_goal_id     = 0
        self.goal_pub            = rospy.Publisher(prefix + "/move_base/goal", MoveBaseActionGoal, queue_size=10)

        rospy.Subscriber(prefix + "/move_base/status", GoalStatusArray, self.getStatus)
        

        while (self.initialized == False and not rospy.is_shutdown()):
            time.sleep(0.5)

        self.start_real = True
        if(self.real_robot):
            self.start_real = False
            self.log_rss    = True
            self.rss_measure.addCallback(self.realMetricCallback)
        else:
            rospy.Timer(rospy.Duration(0.3), self.simulationMetric)


        rospy.Timer(rospy.Duration(1.0), self.pose_callback)



        self.send_deployment     = 0
        self.send_deployment_time_diff = 4.0

        self.metric_kalman       = {}
        self.gamma               = 4



        #if(self.real_robot):
        #    self.updateRouting()


    def pose_callback(self, param):
        self.network.sendMessage(self.position)
        #print('send pose')


    def readConfig(self, config_file):
        with open(config_file, 'r') as stream:
            return yaml.load(stream)


    def getClientsIDs(self):
        ids = self.tree.clients
        positions = []
        for id_ in ids:
            positions.append(self.tree.graph_vertex_position[id_])
        return positions, ids

    def updateRouting(self):
        threading.Timer(1, self.updateMap).start()
        graph = self.createRoutingGraph()
        self.routing.createRoute(graph)

        
    def receiveNetworkCommand(self, command):
        print(command)
        if (command['command'] == COMMANDS.STARTDEPLOYMENT):
            self.start_real = True


    def createRoutingTable():
        neighbors_ids = self.getNeighborsIDs()

    def logNormalMetric(self, distance, variance):
        if(distance < 1):
            return -40
        return -40 -10*self.gamma*math.log10(distance) + np.random.normal(0,math.sqrt(variance),1)[0]

    def realMetricCallback(self, data_id, rss):

        if(not self.start_real or self.id == data_id):
            return

        m_var = 4.0
        if( data_id not in self.metric_kalman):
            self.metric_kalman[data_id]   =  RSSIKalmanFilter(self.id, [-40.0, 3.5], 0.1, 10.0, m_var, self.log_rss)

        position = self.getPositionByID(data_id)

        real_distance    = self.getDistance(self.position['position'], position)*self.map_resolution
        real_metric    = rss


        x = abs(self.position['position'][0] - position[0])*self.map_resolution
        y = abs(self.position['position'][1] - position[1])*self.map_resolution
        gamma = self.metric_kalman[data_id].getGamma()

        d = np.matrix([[10*x*gamma/(x**2 + y**2), 10*y*gamma/(x**2 + y**2)]])
        measurement_var = np.dot(np.dot(d,self.covariance),d.T)[0,0] + m_var

        self.metric_kalman[data_id].setMeasurmentVar(measurement_var)
        self.metric_kalman[data_id].addMeasurement(real_distance, real_metric)


    def simulationMetric(self, param):

        variance = 2.0
        #for the robots
        data_ids = self.network.getDataIds()
        for data_id in data_ids:
            #if(data_id == self.id):
            #    continue
            real_distance    = self.getDistance(self.position['position'], self.network.getData(data_id)['position'])*self.map_resolution
            simulated_metric = self.logNormalMetric(real_distance, variance) #real_distance + np.random.normal(0,variance,1)[0]
            

            #0-time, 1-realposition, 2-neighposition, 3-real_distance, 4-simulated_metric
            #self.metric_measurements[data_id] = (rospy.get_time(), self.position['position'], self.network.rcv_data[data_id]['position'], real_distance, simulated_metric)
            if( data_id not in self.metric_kalman):
                self.metric_kalman[data_id]   =  RSSIKalmanFilter(self.id, [-40.0, 4], .1 ,.1, variance, self.log_rss)


            m, P = self.metric_kalman[data_id].getResult()
            #print(m)
            if(real_distance > 1.0):
                #caculate the variance
                x = abs(self.position['position'][0] - self.network.getData(data_id)['position'][0])*self.map_resolution
                y = abs(self.position['position'][1] - self.network.getData(data_id)['position'][1])*self.map_resolution

                d = np.matrix([[x, y]])
                measurement_var = np.dot(np.dot(d,self.covariance),d.T)[0,0] + variance
                self.metric_kalman[data_id].setMeasurmentVar(measurement_var)

                self.metric_kalman[data_id].addMeasurement(real_distance, simulated_metric)


        #for the tree
        for vertex in self.tree.graph_adj_list:
            real_distance    = self.getDistance(self.position['position'], self.tree.graph_vertex_position[vertex])*self.map_resolution
            simulated_metric = self.logNormalMetric(real_distance, variance)#real_distance + np.random.normal(0,variance,1)[0]
            #self.metric_measurements[vertex] = (rospy.get_time(), self.position['position'], self.tree.graph_vertex_position[vertex], real_distance, simulated_metric)

            if( vertex not in self.metric_kalman):
                self.metric_kalman[vertex] =  RSSIKalmanFilter(self.id, [-40.0, 4], .1, .1, variance, self.log_rss)

            
            if(real_distance > 1.0):
                #caculate the variance
                x = abs(self.position['position'][0] - self.tree.graph_vertex_position[vertex][0])*self.map_resolution
                y = abs(self.position['position'][1] - self.tree.graph_vertex_position[vertex][1])*self.map_resolution

                d = np.matrix([[x, y]])
                measurement_var = np.dot(np.dot(d,self.covariance),d.T)[0,0] + variance
                self.metric_kalman[vertex].setMeasurmentVar(measurement_var)

                self.metric_kalman[vertex].addMeasurement(real_distance, simulated_metric)


    def createRoutingGraph(self):
        graph = {}
        data_ids = self.network.getDataIds()
        for id in data_ids:
            graph[id] = set([])

        graph[self.id] = set([])

        for id in data_ids:
            graph[id] = graph[id].union(set(self.network.getData(id)['routing']))
            for neigbor in self.network.getData(id)['routing']:
                graph[neigbor] = graph[neigbor].union(set([id]))

        if(with_my_self):
            graph[self.id] = graph[self.id].union(self.neighbors)

            for neigbor in self.neighbors:            
                graph[neigbor] = graph[neigbor].union(set([self.id]))


        for id in data_ids:
            graph[id] = list(graph[id])
        return graph


    def getStatus(self, Status):

        if(len(Status.status_list) > 0):
            self.status = Status.status_list[0].status

    def getMap(self, MapData):

        self.map_resolution = MapData.resolution
        self.height         = MapData.height
        #print(MapData.height, 'teste')
        self.map_info = True

    def Stall(self):
        goal = GoalID()
        goal.id = str(self.current_goal_id-1)
        print('stall', self.current_goal_id-1)
        self.cancel_pub.publish(goal)

    def sendDeployment(self, deployment_position):

        if(rospy.get_time() - self.send_deployment < self.send_deployment_time_diff):
            return
        self.send_deployment = rospy.get_time()
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

        #the variance for the kalman filter
        xx = Pose.pose.covariance[0]
        xy = Pose.pose.covariance[1]
        yx = Pose.pose.covariance[6]
        yy = Pose.pose.covariance[7]

        self.covariance = np.matrix([[xx, xy], [yx, yy]])

        #print(Pose.pose.pose.position.x, Pose.pose.pose.position.y, self.map_resolution, self.height)
        self.position['position'] = (Pose.pose.pose.position.x/self.map_resolution, self.height- Pose.pose.pose.position.y/self.map_resolution, orientation_euler[2])
        
        if(self.neigh_0 < 0 or self.neigh_1 < 0):
            self.position['routing']  = []
        else:
            self.position['routing']  = [self.neigh_0, self.neigh_1]

        #print(self.position['position'])
        #avoid to flood the network with messages
        #print('send pose', self.position)
        if(rospy.get_time() - self.send_position_time > self.send_position_time_diff or not self.initialized):
            self.network.sendMessage(self.position)
            
            self.send_position_time__ = rospy.get_time()

        self.initialized = True
                
               
    def getMetricDistance(self, p1, p2):
        return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)


    def getDistance(self, p1, p2):
        return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)



    #The current robot is always allocated in the first position
    def getAllRobotsPositions(self, allocation, inverse = False):


        robot_positions = []
        robot_ids = []
        robot_positions.append(self.position['position'])
        robot_ids.append(self.id)
        #print('ids teste 0')
        data_ids = self.network.getDataIds()
        for data_id in data_ids:
            if(data_id < 0):
                continue

            #print(data_id, 'teste1')

            if(data_id != self.id and allocation == []):
                position = self.network.getData(data_id)['position']
                robot_positions.append(position)
                robot_ids.append(data_id)
            elif(data_id != self.id and data_id in allocation and not inverse):
                position = self.network.getData(data_id)['position']
                robot_positions.append(position)
                robot_ids.append(data_id)
            elif(data_id != self.id and inverse): #data_id not in allocation
                position = self.network.getData(data_id)['position']
                robot_positions.append(position)
                robot_ids.append(data_id)
            #print(data_id, 'teste3')

        return robot_positions, robot_ids
  
    def getSegmentation(self):
        if(not self.tree_segmentation_segments == []):
            return self.tree_segmentation_segments, self.splines
        self.tree_segmentation.segmentation_search([], [])
        #print(tree_segmentation.segmentaion_paths)
        segmentation = self.tree_segmentation.evaluate_segmentation(self.radius)
        self.tree_segmentation_segments = segmentation

        #create the spline
        self.splines = {}
        i = 0
        for segment in segmentation:
            x = []
            y = []
            for index in segment:
                p = self.tree.graph_vertex_position[index]
                x.append(p[0])
                y.append(p[1])
                #print('teste', self.height)
            #print(x, y)
            self.splines[i] = BSpline(x,y, 10.0)
            i += 1

        return segmentation, self.splines


    def getTreeAllocationPerSegment(self):

        ##define the number of robots per segment
        segmentation, splines = self.getSegmentation()
        n_robots = {}
        allocation_sum = []
        sum_robots = 0
        i = 0
        for segment in segmentation:
            cost = self.tree_segmentation.get_path_cost(segment)
            n_robots[i] = math.floor(cost/self.radius)
            sum_robots += n_robots[i]

            allocation_sum.append(sum_robots)
            i = i+1

        self.solution_size = sum_robots
        allocation = {}
        for j in range(len(segmentation)):
            allocation[j] = []
        for r in range(int(sum_robots)):
            for j in range(len(segmentation)):
                if(r < allocation_sum[j]):
                    allocation[j].append(r+self.robots_ids_start)
                    break


        for alloc_id in allocation:
            if(self.id in allocation[alloc_id]):
                self.allocation_position = allocation[alloc_id].index(self.id)
                break


        return allocation, segmentation, splines




    def getClosetPointToTree(self):

        #TODO: make it less cost
        allocation, segmentation, splines = self.getTreeAllocationPerSegment()

        for alloc_id in allocation:
            if(self.id in allocation[alloc_id]):
                self.allocation_id = alloc_id
                break

        allocated_segment = segmentation[self.allocation_id]

        #get the closest point in each segment
        r = self.position['position']

        #gets the closest point for each segment
        closest_points = []     
        closest_points_segments = []
        for i in range(1,len(allocated_segment)):
            p = self.tree.graph_vertex_position[allocated_segment[i-1]]
            q = self.tree.graph_vertex_position[allocated_segment[i]]

            closest_point = dist_to_segment(p, q, r)
            closest_points.append(closest_point)

            closest_points_segments.append((allocated_segment[i-1], allocated_segment[i]))

        #determine the closest point
        min_distance = float('inf')
        closest_point = []
        closest_point_seg_allocation = []
        closest_point_segment = []


        #gets the closest point from all segments
        for i in range(len(closest_points)):
            distance = self.getDistance(closest_points[i], r)
            if(min_distance > distance):
                min_distance = distance
                closest_point = closest_points[i]
                closest_point_segment = closest_points_segments[i]

        return closest_point, closest_point_segment, allocated_segment


    def getAllocationAlpha(self):
        allocation, segmentation, splines = self.getTreeAllocationPerSegment()
        for alloc_id in allocation:
            if(self.id in allocation[alloc_id]):
                self.allocation_id = alloc_id
                break
       
        p = self.tree.graph_vertex_position[segmentation[self.allocation_id][0]]
        q = self.tree.graph_vertex_position[segmentation[self.allocation_id][-1]]

        position = self.position['position']

        alpha = dist_to_segment_alpha(p, q, position)
       
        return alpha

    def getPathSize(self, path):
        path_size = 0
        for i in range(1,len(path)):
            p1        = self.tree.graph_vertex_position[path[i-1]]
            p2        = self.tree.graph_vertex_position[path[i]]
            path_size += self.getDistance(p1, p2)

        return path_size


    def getCenterOfPath(self):
        allocation, segmentation, splines = self.getTreeAllocationPerSegment()
        for alloc_id in allocation:
            if(self.id in allocation[alloc_id]):
                self.allocation_id = alloc_id
                break
        allocation_path = segmentation[self.allocation_id]
        half_path_size = self.getPathSize(allocation_path)/2.0

        path_size = 0
        for i in range(1,len(allocation_path)):
            p1        = self.tree.graph_vertex_position[allocation_path[i-1]]
            p2        = self.tree.graph_vertex_position[allocation_path[i]]
            path_size += self.getDistance(p1, p2)
            if(path_size > half_path_size):
                return((p1[0] + p2[0])/2, (p1[1]+p2[1])/2)
                continue


    def isAllocated(self):

        allocation, segmentation, splines = self.getTreeAllocationPerSegment()
        
        for alloc_id in allocation:
            if(self.id in allocation[alloc_id]):
                return True
        return False

    def getClosetPointToPath(self):
        
        #TODO: make it less cost
        allocation, segmentation, splines = self.getTreeAllocationPerSegment()

        for alloc_id in allocation:
            if(self.id in allocation[alloc_id]):
                self.allocation_id = alloc_id
                break


        allocation_path = segmentation[self.allocation_id]
        #print("Allocation Path", allocation_path, allocation)


        #get the closest point in each segment
        r = self.position['position']
        closest_points = []
        for i in range(1,len(allocation_path)):
            p = self.tree.graph_vertex_position[allocation_path[i-1]]
            q = self.tree.graph_vertex_position[allocation_path[i]]

            closest_point = dist_to_segment(p, q, r)
            closest_points.append(closest_point)
   
        #determine the closest point
        min_distance = float('inf')
        closest_point = []

        #from all segments, gets the closest point
        for i in range(len(closest_points)):
            distance = self.getDistance(closest_points[i], r)
            if(min_distance > distance):
                min_distance = distance
                closest_point = closest_points[i]

        return closest_point

    def robotAsObstacles(self, robot_size = 0.70, force_multiplier = 1):

        robots,_ = self.getAllRobotsPositions([])
        resulting_direction = (0, 0)

        #get the distances
        for robot in robots[1:]:
            distance = self.getDistance(robot, robots[0])*self.map_resolution
            direction = ((robots[0][0]-robot[0])*self.map_resolution,(robots[0][1]-robot[1])*self.map_resolution)
            size = self.getDistance((0,0), direction) + 0.001
            direction = (direction[0]/size, direction[1]/size)

            if(distance < robot_size):
                force = (robot_size - distance)*force_multiplier
                resulting_direction = (resulting_direction[0] +force*direction[0], resulting_direction[1] +force*direction[1])

            #print("DIRECTION", distance)
        return resulting_direction


    def getNeighbors(self):

        #get all robots that are from the same allocation
        #TODO: make it less cost
        allocation, segmentation, splines = self.getTreeAllocationPerSegment()
        #print(allocation)
        #get my allocation

        for alloc_id in allocation:
            if(self.id in allocation[alloc_id]):
                self.allocation_id = alloc_id
                break



        #get all robots that are in the allocation
        positions,_ = self.getAllRobotsPositions(allocation[self.allocation_id])

        p = self.tree.graph_vertex_position[segmentation[self.allocation_id][0]]
        q = self.tree.graph_vertex_position[segmentation[self.allocation_id][-1]]


        positions.append(p)
        positions.append(q)


        alphas = []
        for position in positions:

            alpha = dist_to_segment_alpha(p, q, position)
            alphas.append(alpha)


        #get the closet element from the position 0 alpha // self
        #negative closest and positive closest

        neighbors = []
        #position 0 is the self robot
        self_alpha = alphas[0]
        alphas = np.asfarray(alphas[1:]) - self_alpha


        positive_closest = alphas.copy()
        positive_closest[positive_closest < 0] = np.inf    
        
        negative_closest = alphas.copy()
        negative_closest[negative_closest >= 0] = -np.inf

        if(not np.isinf(positive_closest.min()) and not np.isinf(negative_closest.max())):
            neighbors.append(positive_closest.argmin() + 1)
            neighbors.append(negative_closest.argmax() + 1)

        elif(not np.isinf(positive_closest.min())):
            neighbors = positive_closest.argsort()[0:2]

        else:
            neighbors = negative_closest.argsort()[-2:]

        
        neighbor_positions = []
        neighbor_positions.append(positions[neighbors[0]])
        neighbor_positions.append(positions[neighbors[1]])


        return neighbor_positions

    def isTreeInternalNode(self, id_):
        if not id_ in self.tree.clients and id_ < self.robots_ids_start:
            return True
        return False


    def getNeighborsIDs(self):

        #get all robots that are from the same allocation
        #TODO: make it less costy
        
        allocation, segmentation, splines = self.getTreeAllocationPerSegment()

        #get my allocation
        for alloc_id in allocation:
            if(self.id in allocation[alloc_id]):
                self.allocation_id = alloc_id
                break

        #print(allocation, self.allocation_id)
        #get all robots that are in the allocation
        positions, ids = self.getAllRobotsPositions(allocation[self.allocation_id])
        


        p = self.tree.graph_vertex_position[segmentation[self.allocation_id][0]]
        q = self.tree.graph_vertex_position[segmentation[self.allocation_id][-1]]

        #append the clients ids to the allocation list
        ids.append(segmentation[self.allocation_id][0])
        ids.append(segmentation[self.allocation_id][-1])
        #append the clients positions to the positions list
        positions.append(p)
        positions.append(q)



        alphas = []
        for position in positions:

            alpha = dist_to_segment_alpha(p, q, position)
            alphas.append(alpha)


        #get the closet element from the position 0 alpha // self
        #negative closest and positive closest

        neighbors = []
        #position 0 is the self robot
        self_alpha = alphas[0]
        alphas = np.asfarray(alphas[1:]) - self_alpha
        #print(alphas, self_alpha)
        #guarantee that the robot start the deployment only inside its path
        if(self_alpha < 0.05):
            neigh_0 = self.id
            neigh_1 = segmentation[self.allocation_id][-1]
            self.neigh_0 = neigh_0
            self.neigh_1 = neigh_1
            return [neigh_0, neigh_1] 

        if(self_alpha > 0.95):
            neigh_0 = segmentation[self.allocation_id][0]
            neigh_1 = self.id
            self.neigh_0 = neigh_0
            self.neigh_1 = neigh_1
            return [neigh_0, neigh_1] 


        positive_closest = alphas.copy()
        positive_closest[positive_closest < 0] = np.inf    
        
        negative_closest = alphas.copy()
        negative_closest[negative_closest > 0] = -np.inf
        

        if(not np.isinf(positive_closest.min()) and not np.isinf(negative_closest.max())):
            #print('0')
            neighbors.append(positive_closest.argmin() + 1) # +1 -> the element 0 was desconsidered -> alphas = alphas[1:]
            neighbors.append(negative_closest.argmax() + 1)

        elif(not np.isinf(positive_closest.min())):
            inverse = -1          
            neigh = positive_closest.argsort()[0:2] 
            neighbors.append(neigh[1] + 1) # +1 -> the element 0 was desconsidered -> alphas = alphas[1:]
            neighbors.append(neigh[0] + 1)

        else:
            inverse = -1    
            neigh = negative_closest.argsort()[0:2] 
            neighbors.append(neigh[0] + 1) # +1 -> the element 0 was desconsidered -> alphas = alphas[1:]
            neighbors.append(neigh[1] + 1)


        neigh_0 = ids[neighbors[0]]
        neigh_1 = ids[neighbors[1]]
        #print(neigh_0, neigh_1, 'neig')
       

        #get the closest from the other group - positive
        if(self.isTreeInternalNode(ids[neighbors[0]])):
            positions, ids_local = self.getAllRobotsPositions(allocation[self.allocation_id], True)

            selected_position = self.tree.graph_vertex_position[ids[neighbors[0]]]
            self_alpha   = dist_to_segment_alpha(p, q, positions[0])
            max_distance = float('inf')
            max_index    = -1
            index = 1

            for position in positions[1:]:
                alpha    = dist_to_segment_alpha(p, q, position)
                distance = self.getDistance( position , selected_position)
                if((alpha - self_alpha) > 0 and distance < self.radius):
                    if(distance < max_distance):
                        max_distance = distance
                        max_index    = index
                index += 1

            if(max_index >= 0):               
                neigh_0 = ids_local[max_index]

        #get the closest from the other group - negative
        if(self.isTreeInternalNode(ids[neighbors[1]])):

            positions, ids_local = self.getAllRobotsPositions(allocation[self.allocation_id], True)

            selected_position = self.tree.graph_vertex_position[ids[neighbors[1]]]
            self_alpha   = dist_to_segment_alpha(p, q, positions[0])
            max_distance = float('inf')
            max_index    = -1
            index = 1

            for position in positions[1:]:
                alpha    = dist_to_segment_alpha(p, q, position)
                distance = self.getDistance( position , selected_position)

                if((alpha - self_alpha) < 0 and distance < self.radius):
                    if(distance < max_distance):
                        max_distance = distance
                        max_index    = index

                index += 1
            if(max_index >= 0):
                neigh_1 = ids_local[max_index]




       


        # ##
        # ##  IF the neigbor selected is not a client or a robot from the segment allocation
        # ##
        # if(neighbors[0] < self.robots_ids_start or neighbors[1] < self.robots_ids_start):
        #     #defining the alphas for all robots that are not from my segment
        #     positions, ids = self.getAllRobotsPositions(allocation[self.allocation_id], True)

        #     alphas = []
        #     for position in positions:

        #         alpha = dist_to_segment_alpha(p, q, position)
        #         alphas.append(alpha)

        #     self_alpha = alphas[0]
        #     alphas = np.asfarray(alphas[1:]) - self_alpha
            
        #     if(neighbors[0] < self.robots_ids_start):
        #         positive_closest = alphas.copy()
        #         positive_closest[positive_closest < 0] = np.inf
        #         if(not np.isinf(positive_closest.min())):
        #             neighbors[0] = neighbors.append(positive_closest.argmin() + 1)
        #             neigh_0 = ids[neighbors[0]]

        #     if(neighbors[1] < self.robots_ids_start):
        #         negative_closest = alphas.copy()
        #         negative_closest[negative_closest >= 0] = -np.inf
        #         if(not np.isinf(negative_closest.max())):
        #             neighbors[1] = neighbors.append(negative_closest.argmax() + 1)
        #             neigh_1 = ids[neighbors[1]]

        #save the niegbors in the instance        
        self.neigh_0 = neigh_0
        self.neigh_1 = neigh_1


        return [neigh_0, neigh_1] 


    def highLevelControl(self):

        #print('status 22', self.status)
        if(not self.isAllocated()):
            return 
        #print("control")
        closest_point = self.getClosetPointToPath()
        r = self.position['position']


        dist_to_segmentation = self.getDistance(r, closest_point)*self.map_resolution
        
        alpha = self.getAllocationAlpha()
        #  
        #  Chose the high level navigation or the gradient allocation
        #
        #print('info', dist_to_segmentation, alpha)

        if((dist_to_segmentation > self.high_level_distance or (alpha < 0.1 or alpha > 0.90)) and not self.started_control):
            if(self.status == 1 or self.status == 0):
                return
            #print('control 1')
            goal_ = self.getCenterOfPath()
            goal = (goal_[0], goal_[1])
            #goal = (19.695165209372934, 10.23384885215893)
            #print("Goal ", goal, 'status ', self.status)
            self.sendDeployment(goal)
        
            #print(self.position['destination'])
        else:
            self.started_control = True
            #print('status', self.status)
            if(self.status == 1 or self.status == 0):
                self.Stall()
            else:
                #print('control 2')
        self.control_nonholonomic()

       
        self.position['started'] = 1

    def getDistanceByID(self, id):
        r = self.position['position']
        p = self.getPositionByID(id)

        distance = self.getDistance(r, p)*self.map_resolution

        return self.metric_kalman[id].getMetricValue(distance)


    def getPositionByID(self, id):
        #print(self.network.rcv_data)
        ##Client or tree junctions id
        if(id < self.robots_ids_start):
            position = self.tree.graph_vertex_position[id]
            return position
        else:
            try:
                return self.network.getData(id)['position']
            except:
                return (0,0)


    def verifyMetricOnNeighbors(self, neighbors_ids):
        print(self.network.getDataIds())
        return (neighbors_ids[0] in self.metric_kalman) and (neighbors_ids[1] in self.metric_kalman)      


    def control_holonomic(self):
        r = self.position['position']

        neighbors_ids = self.getNeighborsIDs()

        if( not self.verifyMetricOnNeighbors(neighbors_ids) ):
            return

        #print(self.id, neighbors_ids)

        #neighbors_positions = robot.getNeighbors()
        neighbors_positions = [ self.getPositionByID(neighbors_ids[0]), self.getPositionByID(neighbors_ids[1])]

        #neighbors_positions = self.getNeighbors()
        neighbor_1_distance = -self.getDistanceByID(neighbors_ids[0])
        neighbor_2_distance = -self.getDistanceByID(neighbors_ids[1])
        
        t, closest_point = self.splines[self.allocation_id].getClosestPoint(r[0], r[1])
        
        tangent = (0, 0)
        #print('out', self.allocation_id)
        if(t > 0):
            tangent      = self.splines[self.allocation_id].getDerivative(t)    
            tangent      = (tangent[0], -tangent[1])
            m            =  (neighbor_1_distance-neighbor_2_distance)
            tangent      = (tangent[0]*m, tangent[1]*m)
           
            #print('teste', abs(neighbor_1_distance-neighbor_2_distance))
        closest_point_path = closest_point
        path_distance = self.getDistance(r, closest_point_path)*self.map_resolution
        path_direction = (((closest_point_path[0] - r[0])*self.map_resolution), -(closest_point_path[1] - r[1])*self.map_resolution)
        
        alpha = 0.5*(1.0/100.0)#1.0
        beta  = 2.0#2.0
        final_direction = (alpha*tangent[0] + beta*path_direction[0], alpha*tangent[1] + beta*path_direction[1])
            
        ##
        ##  Coverting to non-holonomic
        ##
        robot_angle = -r[2]

        cmd_vel = Twist()
        cmd_vel.linear.x = final_direction[0]*math.cos(robot_angle) - final_direction[1]*math.sin(robot_angle)
        cmd_vel.linear.y = final_direction[0]*math.sin(robot_angle) + final_direction[1]*math.cos(robot_angle)        
        cmd_vel.angular.z = 0

        self.vel_pub.publish(cmd_vel)

        ##info
        self.position['diff'] = neighbor_1_distance - neighbor_2_distance
        self.position['s_size']  = self.solution_size
        self.position['radius']  = self.radius*self.map_resolution

    def truncateLinearVelocity(self, velocity):
        return velocity if abs(velocity) < self.max_linear_vel else (velocity/abs(velocity))*self.max_linear_vel

    def control_nonholonomic(self):


        r = self.position['position']
        neighbors_ids = self.getNeighborsIDs()
        if( not self.verifyMetricOnNeighbors(neighbors_ids) ):
            return        

        #neighbors_positions = robot.getNeighbors()
        neighbors_positions = [ self.getPositionByID(neighbors_ids[0]), self.getPositionByID(neighbors_ids[1])]

        #neighbors_positions = self.getNeighbors()
        neighbor_1_distance = -self.getDistanceByID(neighbors_ids[0])
        neighbor_2_distance = -self.getDistanceByID(neighbors_ids[1])

        #print('running', self.id, neighbors_ids, neighbor_1_distance, neighbor_2_distance)

        t, closest_point = self.splines[self.allocation_id].getClosestPoint(r[0], r[1])
        
        tangent = (0, 0)
        #print('out', self.allocation_id)
        if(t > 0):
            tangent      = self.splines[self.allocation_id].getDerivative(t)    
            tangent      = (tangent[0], -tangent[1])
            m            =  (neighbor_1_distance-neighbor_2_distance)
            tangent      = (tangent[0]*m, tangent[1]*m)
           
            #print('teste', abs(neighbor_1_distance-neighbor_2_distance))
        closest_point_path = closest_point
        path_distance = self.getDistance(r, closest_point_path)*self.map_resolution
        path_direction = (((closest_point_path[0] - r[0])*self.map_resolution), -(closest_point_path[1] - r[1])*self.map_resolution)
        
        alpha = self.alpha
        beta  = self.beta
        final_direction = (alpha*tangent[0] + beta*path_direction[0], alpha*tangent[1] + beta*path_direction[1])
        robot_angle = r[2]
    

        theta  = self.robot_center_y*(final_direction[1]*math.cos(robot_angle) - final_direction[0]*math.sin(robot_angle))
        linear = final_direction[0]*math.cos(robot_angle) + final_direction[1]*math.sin(robot_angle)
    
        cmd_vel = Twist()
        cmd_vel.linear.x = self.truncateLinearVelocity(linear)#final_direction[0]*math.cos(robot_angle) - final_direction[1]*math.sin(robot_angle)
        cmd_vel.linear.y = 0#final_direction[0]*math.sin(robot_angle) + final_direction[1]*math.cos(robot_angle)
        
        cmd_vel.angular.z = theta
        print('control', sqrt(final_direction[0]**2 +  final_direction[1]**2))

        if(sqrt(final_direction[0]**2 +  final_direction[1]**2) > self.dead_velocity):
            self.vel_pub.publish(cmd_vel)

        self.position['diff'] = neighbor_1_distance - neighbor_2_distance
        self.position['s_size']  = self.solution_size
        self.position['radius']  = self.radius*self.map_resolution

        #print('control')



        
if __name__ == "__main__":
    robot = Robot()

    rate = rospy.Rate(25.0)

    while(not robot.start_real and not rospy.is_shutdown()):
        rate.sleep()


    while not rospy.is_shutdown():
        now = rospy.get_rostime()
        #print('send')
        robot.highLevelControl()
        rate.sleep()
        #print(now-rospy.get_rostime())

    sub.Popen(('kill', '-9', str(os.getpid())))
